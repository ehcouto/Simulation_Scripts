#include "stdafx.h"  //Remove it at embedeed code!



#include "Gpio.h"

sint32 Gpio_Input;


void Gpio__PinUpdate(sint32 input)
{
	Gpio_Input = input;
}


sint32 Gpio__PinRead(sint32 port, sint32 pin)
{
	return(Gpio_Input);
}


void Gpio__PinConfig(sint32 port, sint32 pin, sint32 type)
{

}