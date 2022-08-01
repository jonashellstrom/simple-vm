## Simple VM
Basic LC-3 virtual machine implemented in C that can run object files. 
It features:
- 128 KB of memory
  - LC3 instructions are 16 bits each, so memory is implemented as an array of `uint16_t`.
  - 65,536 memory locations (each 16 bits)
- 10 registers
  - 8 general purpose registers (`R0-R7`)
  - 1 program counter register (`PC`)
  - 1 condition flag register (`COND`)
- 16 opcodes

### To run
```bash
gcc vm.c -o vm
./vm some_obj_file.obj
```