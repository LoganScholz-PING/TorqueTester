#include "printBinary.h"

void print_binary(byte val)
{
	byte i = 0;
	for (i = 0; i < 8; i++)
	{
		if (val & (0x01 << (7 - i)))
		{
			Serial.print("1");
		}
		else
		{
			Serial.print("0");
		}
		if (i == 3) Serial.print("_");
	}
}