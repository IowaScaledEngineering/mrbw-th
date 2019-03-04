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


float16_t F32toF16(float i)
{
	float16_t f16;
	void* ptr = &i;
	uint32_t k = *((uint32_t*)ptr);
	uint8_t sign = (k & 0x80000000)?1:0;
	uint8_t rawExponent = 0xFF & ((k & 0x7F800000)>>23);

	// This adds the implicit high bit unless it's a sub-normal number
	uint32_t rawSignificand = (k & 0x007FFFFF);

	f16 = 0;

	// The sign is (almost) certainly getting saved
	f16 |= sign?0x8000:0;

	if (0 == rawExponent)
	{
		// It's either zero or a sub-normal
		// Either way it's effectively an underflow upon conversion, so just return a zero
		return f16;

	} else if (127 == rawExponent) {
		// Infinity - and beyond!
		if (0 == rawSignificand) // Infinity
			f16 |= 0x7C00;
		else // sNaN and qNaN aren't consistent, just go with one of them
			f16 |= 0x7E00;
		return f16;

	} else {
		// It's a normal value
		// Calculate the new exponent - float32 is biased at 127, float16 biased at 15
		int16_t newExponent = (int16_t)rawExponent - 127 + 15;
		uint16_t f16s = 0;

		if (newExponent > 0x1E)
		{
			// Overflow, set to inf
			f16 |= 0x7C00;
			return f16;
		} else if (newExponent <= 0) {
			// Underflow
			if ((14 - newExponent) <= 24)
			{
				// Can be recovered as a subnormal
				// Add the implied high bit
				rawSignificand |= 0x00800000;
				f16s = rawSignificand >> (14 - newExponent);
				if ( (rawSignificand >> (13 - newExponent)) & 0x00000001u)
					f16s++;
			}
			f16 |= (0x03FF & f16s);
		} else {
			// Sanity check on exponent is good
			f16 = (newExponent & 0x001E) << 10;
			// Truncate and round significand
			f16 |= (rawSignificand >> 13) + (rawSignificand & 0x1000)?1:0;
		}
	}
	return(f16);
}
