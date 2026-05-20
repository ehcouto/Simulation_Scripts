


#include "C_Extensions.h"
#include "MciZeroCross_prm.h"


#define PORTB   1
#define INPUT_PULLDOWN 5

sint32 Gpio__PinRead(sint32 port, sint32 pin);
void Gpio__PinUpdate(sint32 input);
void Gpio__PinConfig(sint32 port, sint32 pin, sint32 type);