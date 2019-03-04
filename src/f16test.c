#include <stdio.h>
#include <stdint.h>
#include "float16.h"

/*
// IEEE 754 binary16 format

1111 1100 0000 0000
5432 1098 7654 3210
SEEE EEMM MMMM MMMM

S = Sign bit - 0=positive, 1=negative
E = Exponent, bias = 15
M = Mantissa / significand, 10 bits


// IEEE 754 binary32 format

3322 2222 2222 1111 1111 1100 0000 0000
1098 7654 3210 9876 5432 1098 7654 3210
SEEE EEEE EMMM MMMM MMMM MMMM MMMM MMMM

S = Sign bit - 0=positive, 1=negative
E = Exponent, bias = 127
M = Mantissa / significand, 23 bits

*/

int main(int argc, char* argv[])
{
   float in = 22.9;
   float16_t out = 0;
   void* ptr = &in;
   
   uint32_t inint = *(uint32_t*)ptr;
   
   printf(" Input:  %f  [%02X %02X %02X %02X]\n", in, (uint8_t)(inint>>24), (uint8_t)(inint>>16), (uint8_t)(inint>>8), (uint8_t)inint);
   printf(" F32: sign=%d, exp=%d, mantissa=%d\n", (inint & 0x80000000)?1:0, 0xFF & (inint>>23), inint & 0x7FFFFF);
   
   
   out = F32toF16(in);
   
   printf(" Output: [%02X %02X]\n", out>>8, (uint8_t)out);
   printf(" F16: sign=%d, exp=%d, mantissa=%d\n", (out & 0x8000)?1:0, 0x1F & (out>>10), inint & 0x3FF);

   out = float16(in);
   printf(" Output: [%02X %02X]\n", out>>8, (uint8_t)out);
   printf(" F16: sign=%d, exp=%d, mantissa=%d\n", (out & 0x8000)?1:0, 0x1F & (out>>10), inint & 0x3FF);

   in = float32(out);
   inint = *(uint32_t*)ptr;
   
   printf(" Regurgitated Input:  %f  [%02X %02X %02X %02X]\n", in, (uint8_t)(inint>>24), (uint8_t)(inint>>16), (uint8_t)(inint>>8), (uint8_t)inint);
   printf(" F32: sign=%d, exp=%d, mantissa=%d\n", (inint & 0x80000000)?1:0, 0xFF & (inint>>23), inint & 0x7FFFFF);


   return 0;
}
