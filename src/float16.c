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


float float32(float16_t h)
{
  int s = h & 0x8000;
  int e = (h & 0x7c00) >> 10;
  int m = h & 0x03ff;
  uint32_t x = 0;
	void* ptr;
	
  s <<= 16;
  if (e == 31) {
    // infinity or NAN
    e = 255 << 23;
    m <<= 13;
    x = s | e | m;
  } else if (e > 0) {
    // normalized
    e = e + (127 - 15);
    e <<= 23;
    m <<= 13;
    x = s | e | m;
  } else if (m == 0) {
    // zero
    x = s;
  } else {
    // subnormal, value is m times 2^-24
    float f = ((float) m);
    ptr = &f;
    x = s | (*(uint32_t*)ptr - (24 << 23));
  }
  ptr = &x;
  
  return(*((float*)ptr));
}


float16_t float16(float f)
{
	void* ptr = &f;
	uint32_t k = *((uint32_t*)ptr);
	float16_t h = 0;

  int e = (k >> 23) & 0x000000ff;
  int s = (k >> 16) & 0x00008000;
  int m = k & 0x007fffff;

  e = e - 127;
  if (e == 128) {
    // infinity or NAN; preserve the leading bits of mantissa
    // because they tell whether it's a signaling or quiet NAN
    h = s | (31 << 10) | (m >> 13);
  } else if (e > 15) {
    // overflow to infinity
    h = s | (31 << 10);
  } else if (e > -15) {
    // normalized case
    if ((m & 0x00003fff) == 0x00001000) {
      // tie, round down to even
      h = s | ((e+15) << 10) | (m >> 13);
    } else {
      // all non-ties, and tie round up to even
      //   (note that a mantissa of all 1's will round up to all 0's with
      //   the exponent being increased by 1, which is exactly what we want;
      //   for example, "0.5-epsilon" rounds up to 0.5, and 65535.0 rounds
      //   up to infinity.)
      h = s | ((e+15) << 10) + ((m + 0x00001000) >> 13);
    }
  } else if (e > -25) {
    // convert to subnormal
    m |= 0x00800000; // restore the implied bit
    e = -14 - e; // shift count
    m >>= e; // M now in position but 2^13 too big
    if ((m & 0x00003fff) == 0x00001000) {
      // tie round down to even
    } else {
      // all non-ties, and tie round up to even
      m += (1 << 12); // m += 0x00001000
    }
    m >>= 13;
    h = s | m;
  } else {
    // zero, or underflow
    h = s;
  }
  return h;
};
