#include <stdio.h>
#include <stdlib.h>
#include "cpu.h"

/**
 * Main
 */
int main(int argc, char **argv)
{
  // take file with program to run as args
  if (argc != 2)
  {
    fprintf(stderr, "Usage: $ ./ls8 program.ls8\n");
    exit(1);
  }
  printf("Run program: %s...\n", argv[1]);

  struct cpu cpu;

  cpu_init(&cpu);
  cpu_load(&cpu, argv[1]);
  cpu_run(&cpu);

  return 0;
}