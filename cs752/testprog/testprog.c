#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define CACHE_SIZE 128 * 1024 * 1024 // Size of the cache in bytes
#define BLOCK_SIZE 64   // Size of each cache block in bytes

// #define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while (0)
#endif

void empty_function(char c) {
  c++;
  const uint8_t x = 10;
  c += x;
}

int main(int argc, char *argv[]) {
  // Allocate a large block of memory
  // size_t mem_size = 2*CACHE_SIZE * BLOCK_SIZE;
  size_t mem_size = CACHE_SIZE;
  char* memory = (char*)malloc(mem_size);

  // Write to conflicting addresses in the cache
  char* address1 = memory + (0 * CACHE_SIZE); // Address that maps to cache block 0
  char* address2 = memory + (1 * CACHE_SIZE); // Address that also maps to cache block 0
  *address1 = 'A';
  *address2 = 'B';

  // Debugging output to show the writes
  DEBUG_PRINT("Wrote '%c' to address %p\n", *address1, address1);
  DEBUG_PRINT("Wrote '%c' to address %p\n", *address2, address2);

  // Read the 2 bytes that were written in the same order
  char* readAddress1 = memory + (0 * CACHE_SIZE);
  char* readAddress2 = memory + (1 * CACHE_SIZE);
  char byte1 = *readAddress1;
  char byte2 = *readAddress2;

  // Print the bytes
  DEBUG_PRINT("Read '%c' from address %p\n", byte1, readAddress1);
  DEBUG_PRINT("Read '%c' from address %p\n", byte2, readAddress2);


  const int iterations = 110000;
  char* offset_memory = memory + (mem_size / 2);

  // Write to each address location in the allocated memory
  for (int i = 0; i < iterations; i+=4) {
    // Calculate the starting address of the cache block
    char* address = memory + i;

    // Write to the address
    // *address = (char)((unsigned long long)address % BLOCK_SIZE);
    *address = 'E';
    *(address+1) = 'F';
    *(address+2) = 'G';
    *(address+3) = 'H';

    // Debugging output to show the write
    // DEBUG_PRINT("Wrote '0x%02x' to address %p\n", (unsigned char)*address, (void*)address);
  }

  for (int i = 0; i < iterations; i++) {
    // Calculate the starting address of the cache block
    char* address = offset_memory + i;

    // Write to the address
    // *address = (char)((unsigned long long)address % BLOCK_SIZE);
    *address = 'B';

    // Debugging output to show the write
    // DEBUG_PRINT("Wrote '0x%02x' to address %p\n", (unsigned char)*address, (void*)address);
  }

  // Read from each address location in the allocated memory
  for (int i = 0; i < iterations; i++) {
    // Calculate the starting address of the cache block
    char* address = memory + i;

    // Read from the address
    char byte = *address;

    // Debugging output to show the read
    DEBUG_PRINT("Read '0x%02x' from address %p\n", (unsigned char)byte, (void*)address);
    byte = byte;
    byte++;
    empty_function(byte);
  }

  for (int i = 0; i < iterations; i++) {
    // Calculate the starting address of the cache block
    char* address = offset_memory + i;

    // Read from the address
    char byte = *address;

    // Debugging output to show the read
    DEBUG_PRINT("Read '0x%02x' from address %p\n", (unsigned char)byte, (void*)address);
    byte = byte;
    byte++;
    empty_function(byte);
  }

  // Free the allocated memory
  free(memory);

  return 0;
}
