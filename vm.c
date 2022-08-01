#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/termios.h>
#include <sys/mman.h>

int running;

// Memory storage
#define MEMORY_MAX (1 << 16)
uint16_t memory[MEMORY_MAX];  // 16-bit uint -> 2^16 -> 65536 locations

// Registers
enum Registers {
    // 8 general purpose regs
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,   // program counter
    R_COND, // condition flag
    R_COUNT
};

uint16_t reg[R_COUNT];

enum ConditionFlags {
    FL_POS = 1 << 0,
    FL_ZRO = 1 << 1,
    FL_NEG = 1 << 2,
};


// Instruction set
enum OpCodes {
    OP_BR = 0, // 0000 branch
    OP_ADD,    // 0001 add
    OP_LD,     // 0010 load
    OP_ST,     // 0011 store
    OP_JSR,    // 0100 jump register
    OP_AND,    // 0101 bitwise and
    OP_LDR,    // 0110 load register
    OP_STR,    // 0111 store register
    OP_RTI,    // 1000 unused
    OP_NOT,    // 1001 bitwise not
    OP_LDI,    // 1010 load indirect
    OP_STI,    // 1011 store indirect
    OP_JMP,    // 1100 jump
    OP_RES,    // 1101 reserved (unused)
    OP_LEA,    // 1110 load effective address
    OP_TRAP    // 1111 execute trap
};

// Trap codes
enum TrapCodes {
    TRAP_GETC = 0x20,  // get character from keyboard, not echoed to the terminal
    TRAP_OUT = 0x21,   // output a character
    TRAP_PUTS = 0x22,  // output a word string
    TRAP_IN = 0x23,    // get character from keyboard, echoed onto the terminal
    TRAP_PUTSP = 0x24, // output a byte string
    TRAP_HALT = 0x25   // halt the program
};

enum MemoryMappedRegisters {
    MR_KBSR = 0xFE00, // keyboard status
    MR_KBDR = 0xFE02  // keyboard data
};


/*
 * Input buffering helpers
 */
struct termios original_tio;

void disable_input_buffering() {
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

uint16_t check_key() {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

void handle_interrupt(int signal) {
    restore_input_buffering();
    printf("\n");
    exit(-2);
}

/*
 * With memory mapped registers, instead of accessing memory array directly,
 * memory is accessed through getters and setters.
*/
void mem_write(uint16_t address, uint16_t val) {
    memory[address] = val;
}

uint16_t mem_read(uint16_t address) {
    if (address == MR_KBSR) {
        if (check_key()) {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = getchar();
        } else {
            memory[MR_KBSR] = 0;
        }
    }
    return memory[address];
}


/*
 * Used to sign-extend values to match a 16-bit unsigned int
 */
uint16_t sextend(uint16_t x, int bit_count) {
    if ((x >> (bit_count - 1)) & 1)
        x |= (0xFFFF << bit_count);
    return x;
}

/*
 * Used to swap byte order (big-endian -> little-endian)
 */
uint16_t swap16(uint16_t val) {
    return (val << 8) | (val >> 8);
}


void read_image_file(FILE *file) {
    uint16_t origin;
    fread(&origin, sizeof(origin), 1, file);
    origin = swap16(origin); // first 16 bits specify where in memory the program starts

    /* maximum file size so we only need one fread */
    uint16_t max_read = MEMORY_MAX - origin;
    uint16_t *p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    while (read-- > 0) {
        *p = swap16(*p); // swap to little-endian
        ++p;
    }
}

int read_image(const char *img_path) {
    FILE *file = fopen(img_path, "rb");
    if (!file) { return 0; };
    read_image_file(file);
    fclose(file);
    return 1;
}

/*
 * Whenever writing to a register, update the condition code to indicate the sign of the contents stored.
 */
void update_flags(uint16_t r) {
    if (reg[r] == 0) {
        reg[R_COND] = FL_ZRO;
    } else if (reg[r] >> 15) { // a 1 in the left-most bit means it's negative
        reg[R_COND] = FL_NEG;
    } else {
        reg[R_COND] = FL_POS;
    }
}

/*
 * ADD – takes the sum of two values and stores it in DR. 5th bit indicates mode:
 * - register mode: both operands are read from registers
 * - immediate mode: first operand from register, second from instruction (max 5 bits that must be sign-extended)
 */
void add(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t r1 = (instr >> 6) & 0x7;
    uint16_t is_imm = (instr >> 5) & 0x1;

    if (is_imm) {
        uint16_t imm5 = sextend(instr & 0x1F, 5);
        reg[r0] = reg[r1] + imm5;
    } else {
        uint16_t r2 = instr & 0x7;
        reg[r0] = reg[r1] + reg[r2];
    }
    update_flags(r0);
}

/*
 * AND – two values are bitwise ANDed and the result is stored in DR. 5th bit indicates mode (register/imm.)
 */
void and(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t r1 = (instr >> 6) & 0x7;
    uint16_t is_imm = (instr >> 5) & 0x1;

    if (is_imm) {
        uint16_t imm5 = sextend(instr & 0x1F, 5);
        reg[r0] = reg[r1] & imm5;
    } else {
        uint16_t r2 = instr & 0x7;
        reg[r0] = reg[r1] & reg[r2];
    }
    update_flags(r0);
}

/*
 * NOT – a value from SR is bitwise NOTed and stored in DR.
 */
void not(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t r1 = (instr >> 6) & 0x7;

    reg[r0] = ~reg[r1];
    update_flags(r0);
}

/*
 * BR – Condition codes specified by the state of bits 11, 10, 9 (n, z, p) are tested.
 * If bit 11 is on, N is tested. If bit 10 is on, Z is tested etc.
 * If any of the condition codes tested is set,
 * the program branches to the location specified by adding the sign-extended PCoffset9 field to the incremented PC
 */
void branch(uint16_t instr) {
    uint16_t pc_offset_9 = sextend(instr & 0x1FF, 9);
    uint16_t cond_flag = (instr >> 9) & 0x7;
    if (cond_flag & reg[R_COND]) {
        reg[R_PC] += pc_offset_9;
    }
}

/*
 * JMP – Unconditionally jumps to the location specified by the contents of the base register (bits [8:6]).
 * Also handles the return (RET) instruction which is a special case of jump where r1 is 7.
 */
void jump(uint16_t instr) {
    uint16_t r1 = (instr >> 6) & 0x7;
    reg[R_PC] = reg[r1];
}

/*
 * JSR/JSRR – The incremented PC is saved in r7 (reference to calling routine).
 * The PC is then loaded with the address of the instructions of the subroutine. It will immediately jump to that address.
 * The 11th bit determines addressing mode - whether to use JSR or JSRR:
 * – Bit 11 is set (JSR): uses an 11 bit offset (sign-extended bits [10:0]) to the incremented PC to get the address to jump to.
 * - Bit 11 is clear (JSRR): reads from the base register to get the address to jump to.
 */
void jump_reg(uint16_t instr) {
    uint16_t pc_relative = (instr >> 11) & 1;
    reg[R_R7] = reg[R_PC]; // save return address (aka. "linking")

    if (pc_relative) {
        uint16_t pc_offset = sextend(instr & 0x7FF, 11);
        reg[R_PC] += pc_offset;
    } else {
        uint16_t r1 = (instr >> 6) & 0x7;
        reg[R_PC] = reg[r1];
    }
}

/*
 * LD – Calculates an address by adding sign-extended bits [8:0] to the incremented PC.
 * The contents at this address are loaded into DR.
 */
void load(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sextend(instr & 0x1FF, 9);
    reg[r0] = mem_read(reg[R_PC] + pc_offset);
    update_flags(r0);
}

/*
 * LDR – Loads base with an offset. Address is calculated by adding sign-extended bits [5:0] to the contents
 * of the register specified in bits [8:6]. Contents at this address are loaded into DR.
 */
void load_register(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t r1 = (instr >> 6) & 0x7;
    uint16_t offset = sextend(instr & 0x3F, 6);
    reg[r0] = mem_read(reg[r1] + offset);
    update_flags(r0);
}

/*
 * LEA – Address is computed by sign-extending bits [8:0] and adding this value to the incremented PC.
 * Address is loaded into DR.
 */
void load_effective_address(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sextend(instr & 0x1FF, 9);
    reg[r0] = reg[R_PC] + pc_offset;
    update_flags(r0);
}

/*
 * ST – Sign-extended bits [8:0] are added to the incremented PC to get the address.
 * The contents of the register specified by SR are the stored at this address.
 */
void store(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sextend(instr & 0x1FF, 9);
    mem_write(reg[R_PC] + pc_offset, reg[r0]);
}

/*
 * STI – Similar to ST except that what's stored in memory at the address obtained by
 * adding sign-extended bits [8:0] to the incremented PC is another address at which the data in SR is stored.
 */
void store_indirect(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t pc_offset = sextend(instr & 0x1FF, 9);
    mem_write(mem_read(reg[R_PC] + pc_offset), reg[r0]);
}

/*
 * STR – The contents of the register specified by SR are stored in the memory location
 * whose address is computed by sign-extending bits [5:0] to 16 bits
 * and adding this value to the contents of the register specified by bits [8:6].
 */
void store_register(uint16_t instr) {
    uint16_t r0 = (instr >> 9) & 0x7;
    uint16_t r1 = (instr >> 6) & 0x7;
    uint16_t offset = sextend(instr & 0x3F, 6);
    mem_write(reg[r1] + offset, reg[r0]);
}

/*
 * LOAD INDIRECT – Loads a value from memory into a register
 */
void load_indirect(uint16_t instr) {
    uint16_t dr = (instr >> 9) & 0x7;
    uint16_t pc_offset_9 = sextend(instr & 0x1FF, 9);
    /* add pc_offset to the current PC, look at that memory location to get the final address */
    reg[dr] = mem_read(mem_read(reg[R_PC] + pc_offset_9));
    update_flags(dr);
}

/*
 * Trap routine implementations
 */
void trap(uint16_t instr) {
    reg[R_R7] = reg[R_PC];

    switch (instr & 0xFF) {
        case TRAP_GETC: {
            /* read a single ASCII char */
            reg[R_R0] = (uint16_t) getchar();
            update_flags(R_R0);
            break;
        }

        case TRAP_OUT: {
            putc((char) reg[R_R0], stdout);
            fflush(stdout);
            break;
        }
        case TRAP_PUTS: {
            uint16_t *c = memory + reg[R_R0]; // one char per word
            while (*c) {
                putc((char) *c, stdout);
                ++c;
            }
            fflush(stdout);
            break;
        }
        case TRAP_IN: {
            printf("Enter a character: ");
            char c = getchar();
            putc(c, stdout);
            fflush(stdout);
            reg[R_R0] = (uint16_t) c;
            update_flags(R_R0);
            break;
        }
        case TRAP_PUTSP: {
            /* one char per byte (two bytes per word)
               here we need to swap back to big-endian format */
            uint16_t *c = memory + reg[R_R0];
            while (*c) {
                char char1 = (*c) & 0xFF;
                putc(char1, stdout);
                char char2 = (*c) >> 8;
                if (char2) putc(char2, stdout);
                ++c;
            }
            fflush(stdout);
            break;
        }
        case TRAP_HALT: {
            puts("HALT");
            fflush(stdout);
            running = 0;
            break;
        }
    }
}

int main(int argc, const char *argv[]) {
    // Load arguments
    if (argc < 2) {
        // show usage string
        printf("lc3 [image-file1] ...\n");
        exit(2);
    }

    for (int j = 1; j < argc; ++j) {
        if (!read_image(argv[j])) {
            printf("failed to load image: %s\n", argv[j]);
            exit(1);
        }
    }

    // Setup
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();

    reg[R_COND] = FL_ZRO;

    uint16_t PC_START = 0x3000; // set a default position for PC
    reg[R_PC] = PC_START;

    int running = 1;
    while (running) {
        uint16_t instr = mem_read(reg[R_PC]++); // PC is incremented after instruction is loaded
        enum OpCodes op = instr >> 12;

        switch (op) {
            case OP_ADD:
                add(instr);
                break;
            case OP_AND:
                and(instr);
                break;
            case OP_NOT:
                not(instr);
                break;
            case OP_BR:
                branch(instr);
                break;
            case OP_JMP:
                jump(instr);
                break;
            case OP_JSR:
                jump_reg(instr);
                break;
            case OP_LD:
                load(instr);
                break;
            case OP_LDI:
                load_indirect(instr);
                break;
            case OP_LDR:
                load_register(instr);
                break;
            case OP_LEA:
                load_effective_address(instr);
                break;
            case OP_ST:
                store(instr);
                break;
            case OP_STI:
                store_indirect(instr);
                break;
            case OP_STR:
                store_register(instr);
                break;
            case OP_TRAP:
                trap(instr);
                break;
            case OP_RES:
            case OP_RTI:
            default:
                break;
        }
    }
    restore_input_buffering();
}