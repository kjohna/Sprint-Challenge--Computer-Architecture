#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // to use sleep function for debugging
#include "cpu.h"

#define DATA_LEN 6

// helpers to read and write cpu's ram
unsigned int cpu_ram_read(struct cpu *cpu, int index)
{
  return cpu->ram[index];
};

void cpu_ram_write(struct cpu *cpu, int index, unsigned char value)
{
  cpu->ram[index] = value;
};

/**
 * Load the binary bytes from a .ls8 source file into a RAM array
 */
void cpu_load(struct cpu *cpu, char *prog_file)
{
  // something less hard-coded:
  // loads program (from prog_file) into memory
  FILE *fp;
  char line[1024];
  unsigned char ram_i = 0;

  fp = fopen(prog_file, "r");

  if (fp == NULL)
  {
    fprintf(stderr, "File not found.\n");
    exit(1);
  };

  while (fgets(line, 1024, fp) != NULL)
  {
    char *end_ptr;
    unsigned char val = strtoul(line, &end_ptr, 2);
    if (end_ptr != line)
    {
      // printf("cpu->ram write val: %02x\n", val);
      // printf("cpu->ram write add: %d\n", ram_i);
      cpu_ram_write(cpu, ram_i, val);
      ram_i++;
    }
  };

  fclose(fp);
  // return ram_i - 0b1 == end of program
  // (mod2 arithmetic)
  // printf("end of prog: %d\n", ram_i - 0b1);
}

/**
 * ALU
 */
void alu(struct cpu *cpu, enum alu_op op, unsigned char regA, unsigned char regB)
{
  switch (op)
  {
  case ALU_MUL:
    // printf("ALU: multiplies %d x %d\n", regA, regB);
    cpu->gp_registers[regA] = cpu->gp_registers[regA] * cpu->gp_registers[regB];
    break;

  case ALU_ADD:
    // printf("ALU: adds %d + %d\n", regA, regB);
    cpu->gp_registers[regA] = cpu->gp_registers[regA] + cpu->gp_registers[regB];
    break;

  case ALU_CMP:
    // sets flag register with result
    // flag register: 0000 0LGE
    // reset flag register first
    cpu->fl = 0;
    if (cpu->gp_registers[regA] == cpu->gp_registers[regB])
    {
      // set E flag to 1
      cpu->fl = 0x01;
    }
    else if (cpu->gp_registers[regA] < cpu->gp_registers[regB])
    {
      // set L flag to 1
      cpu->fl = 0x04;
    }
    else
    {
      // A > B
      // set G flag to 1
      cpu->fl = 0x02;
    }
    // printf("ALU > CMP: cpu->fl = %02x\n", cpu->fl);
    break;

  case ALU_INC:
    // printf("ALU > INC: %02x\n", cpu->gp_registers[regA]);
    cpu->gp_registers[regA]++;
    // printf("ALU > INC: %02x\n", cpu->gp_registers[regA]);
    break;

  case ALU_DEC:
    // printf("ALU > DEC: %02x\n", cpu->gp_registers[regA]);
    cpu->gp_registers[regA]--;
    // printf("ALU > DEC: %02x\n", cpu->gp_registers[regA]);
    break;

  default:
    printf("Bad ALU op.");
    break;
  }
}

/**
 * Run the CPU
 */
void cpu_run(struct cpu *cpu)
{
  int running = 1; // True until we get a HLT instruction
  int oper_count, operands[2];
  unsigned char instruction;
  unsigned char moves_pc = 0;

  while (running)
  {
    // TODO
    // 1. Get the value of the current instruction (in address PC).
    instruction = cpu_ram_read(cpu, cpu->pc);
    // 2. Figure out how many operands this next instruction requires
    oper_count = instruction >> 6;
    // 2a. Figure out if this instruction moves pc
    moves_pc = instruction & 0b00010000;
    // 3. Get the appropriate value(s) of the operands following this instruction
    for (unsigned char i = 0; i < oper_count; i++)
    {
      operands[i] = cpu_ram_read(cpu, cpu->pc + i + 1);
    }
    // what's it up to?
    // printf("pc: %02x\n", cpu->pc);
    // printf("instr: %02x\n", instruction);
    // printf("oper_count: %d\n", oper_count);
    // 4. switch() over it to decide on a course of action.
    // 5. Do whatever the instruction should do according to the spec.
    switch (instruction)
    {
    case HLT:
      // Halt the CPU (and exit the emulator).
      // printf(">> HLT command received. Exiting.\n");
      running = 0;
      break;

    case LDI:
      // Set the value of a register to an integer.
      // printf(">> LDI command received, loading register %02x with %02x.\n", operands[0], operands[1]);
      cpu->gp_registers[operands[0]] = operands[1];
      break;

    case PRN:
      // Print to the console the decimal integer value that is stored in the given register.
      // printf(">> PRN received, register %d\n", operands[0]);
      printf("%d\n", cpu->gp_registers[operands[0]]);
      break;

    case MUL:
      // *This is an instruction handled by the ALU.*
      // Multiply the values in two registers together and store the result in registerA.
      alu(cpu, 0, operands[0], operands[1]);
      break;

    case PUSH:
      // Push the value in the given register on the stack.
      cpu->gp_registers[7]--; // decr stack pointer
      cpu->ram[cpu->gp_registers[7]] = cpu->gp_registers[operands[0]];
      // printf(">> PUSH: value %02x to address %02x\n", cpu->gp_registers[operands[0]], cpu->gp_registers[7]);
      break;

    case POP:
      // Pop the value at the top of the stack into the given register.
      // printf(">> POP: value %02x at address %02x to register %02x\n", cpu->ram[cpu->gp_registers[7]], cpu->gp_registers[7], operands[0]);
      cpu->gp_registers[operands[0]] = cpu->ram[cpu->gp_registers[7]];
      cpu->gp_registers[7]++; // incr stack pointer
      break;

    case CALL:
      // Calls a subroutine (function) at the address stored in the register.
      // 1) Push address of the instruction directly after `CALL` to the stack. (add 2 since call has 1 operand)
      cpu->gp_registers[7]--; // decr stack pointer
      cpu->ram[cpu->gp_registers[7]] = cpu->pc + 2;
      // printf(">> CALL: push pc of next instr (%02x)\n", cpu->pc + 1);
      // 2) Set PC to the address stored in the register
      // cast to int first (pc is an int)
      int tmp = cpu->gp_registers[operands[0]];
      cpu->pc = tmp;
      // printf(">> CALL: move pc to: %02x\n", cpu->pc);
      // sleep(1);
      break;

    case RET:
      // Return from subroutine:
      // Pop the value from the top of the stack and store it in the `PC`.
      cpu->pc = cpu->ram[cpu->gp_registers[7]];
      cpu->gp_registers[7]++; // incr stack pointer
      // printf(">> RET: move pc to: %02x\n", cpu->pc);
      // sleep(1);
      break;

    case ADD:
      // *This is an instruction handled by the ALU.*
      // Add the value in two registers and store the result in registerA.
      alu(cpu, 1, operands[0], operands[1]);
      break;

    case CMP:
      //  *This is an instruction handled by the ALU.*
      // Compare the values in two registers.
      alu(cpu, 2, operands[0], operands[1]);
      break;

    case JEQ:
      // If `equal` flag is set (true), jump to the address stored in the given register.
      // printf(">> JEQ: cpu->fl = %02x.\n", cpu->fl);
      if (cpu->fl == 0x01)
      {
        cpu->pc = cpu->gp_registers[operands[0]];
      }
      else
      {
        // if not jumping, unset moves_pc so that pc increments properly
        moves_pc = 0;
      }
      // sleep(1);
      break;

    case LD:
      // Loads registerA (operands[0]) with the value at the memory address stored in registerB (operands[1]).
      cpu->gp_registers[operands[0]] = cpu->ram[cpu->gp_registers[operands[1]]];
      break;

    case PRA:
      // Print alpha character value stored in the given register.
      printf("%c", cpu->gp_registers[operands[0]]);
      break;

    case INC:
      // *This is an instruction handled by the ALU.*
      // Increment (add 1 to) the value in the given register.
      alu(cpu, 3, operands[0], operands[1]);
      break;

    case DEC:
      // *This is an instruction handled by the ALU.*
      // Decrement (subtract 1 from) the value in the given register.
      alu(cpu, 4, operands[0], operands[1]);
      break;

    case JMP:
      // Jump: Set the `PC` to the address stored in the given register.
      // printf(">> JMP to %02x\n", cpu->gp_registers[operands[0]]);
      cpu->pc = cpu->gp_registers[operands[0]];
      break;

    default:
      printf(">> Unknown command: %02x\n", instruction);
      break;
    }
    // 6. Move the PC to the next instruction.
    // IFF the instruction didn't move it already
    if (!moves_pc)
    {
      cpu->pc = cpu->pc + 1 + oper_count;
      // printf("Move PC.\n");
    }
    // printf("PC: %02x\n", cpu->pc);
  }
}

/**
 * Initialize a CPU struct
 */
void cpu_init(struct cpu *cpu)
{
  // Initialize the PC and other special registers
  cpu->pc = 0;
  cpu->ir = 0;
  cpu->mar = 0;
  cpu->mdr = 0;
  cpu->fl = 0;
  // R7 = SP, points at F4 when stack is empty
  cpu->gp_registers[7] = 0xF4;
}
