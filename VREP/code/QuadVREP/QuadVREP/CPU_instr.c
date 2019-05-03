#include "stdint.h"

short __CLZ(uint32_t value) {
	int i;
	for (i = 0; i < 32; i++) {
		if (value & (1 << (31 - i))) {
			break;
		}
	}
	return i;
}