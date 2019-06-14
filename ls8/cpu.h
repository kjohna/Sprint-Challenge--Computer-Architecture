#ifndef _CPU_H_
#define _CPU_H_

// Holds all information about the CPU
struct cpu
{
  // TODO
  // PC
  unsigned char pc;
  // registers (array)
  unsigned char ir, mar, mdr, fl;
  unsigned char gp_registers[8];
  // ram (array)
  unsigned char ram[256];
};

// ALU operations
enum alu_op
{
  ALU_MUL,
  ALU_ADD,
  ALU_CMP,
  ALU_INC,
  ALU_DEC
  // Add more here
};

// Instructions

// These use binary literals. If these aren't available with your compiler, hex
// literals should be used.
#define HLT 0b00000001  // 01
#define LDI 0b10000010  // 82, 2 operands
#define PRN 0b01000111  // 47, 1 operand
#define MUL 0b10100010  // A2, 2 operands
#define PUSH 0b01000101 // 45, 1 operand
#define POP 0b01000110  // 46, 1 operand
#define CALL 0b01010000 // 50, 1 operand
#define RET 0b00010001  // 11
#define ADD 0b10100000  // A0, 2 operands
#define CMP 0b10100111  // A7, 2 operands
#define JEQ 0b01010101  // 55, 1 operand
#define LD 0b10000011   // 83, 2 operands
#define PRA 0b01001000  // 48, 1 operand
#define INC 0b01100101  // 65, 1 operand
#define DEC 0b01100110  // 66, 1 operand
#define JMP 0b01010100  // 54, 1 operand
// TODO: more instructions here. These can be used in cpu_run().

// Function declarations

extern void cpu_load(struct cpu *cpu, char *prog_file);
extern void cpu_init(struct cpu *cpu);
extern void cpu_run(struct cpu *cpu);

#endif
