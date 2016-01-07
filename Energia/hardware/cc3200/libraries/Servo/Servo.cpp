/*
 Servo.cpp - Interrupt driven Servo library for the CC3200 using 16 bit timers
 Copyright (c) 2009 Michael Margolis.  All right reserved.
 
 2014: Modified by Robert Wessels for the CC3200

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


#include "Servo.h"

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/debug.h>
#include <driverlib/gpio.h>
#include <driverlib/pin.h>
#include <driverlib/prcm.h>
#include <driverlib/rom_map.h>
#include <driverlib/rom.h>
#include <driverlib/timer.h>

#include <stdio.h>
#include <stdlib.h>

/** variables and functions common to all Servo instances **/

volatile unsigned long ticksPerMicrosecond;  // Holds the calculated value
unsigned int servoAssignedMask;
static servo_t servos[SERVOS_PER_TIMER];
unsigned int remainderPulseWidth;
volatile int currentServo;
bool servoInitialized = false;

// Calculate the new period remainder
static void calculatePeriodRemainder(void)
{
	unsigned long servoPeriodSum = 0;
	for (int i = 0; i < SERVOS_PER_TIMER; i++){
		servoPeriodSum += servos[i].pulse_width;
	}
	remainderPulseWidth = REFRESH_INTERVAL - servoPeriodSum;
}

void initServo(void) {

	// Initialize global variables
	ticksPerMicrosecond = 0;
	servoAssignedMask = 0;
	remainderPulseWidth = 0;
	currentServo = 0;

	for(int i = 0; i < SERVOS_PER_TIMER; i++)
	{
		servos[i].pin_number = 0;
		servos[i].pulse_width = DEFAULT_SERVO_PULSE_WIDTH;
		servos[i].enabled = false;
	}

	calculatePeriodRemainder();

	// Enable Timer
	MAP_PRCMPeripheralClkEnable(SERVO_TIMER_PERIPH, PRCM_RUN_MODE_CLK);

	// Reset the Timer
	MAP_PRCMPeripheralReset(SERVO_TIMER_PERIPH);

	// Register Timer Interrupt
	TimerIntRegister(SERVO_TIMER, SERVO_TIMER_AB, ServoIntHandler);

	// Configure the Timer
	MAP_TimerConfigure(SERVO_TIMER, SERVO_TIME_CFG);
	MAP_TimerPrescaleSet(SERVO_TIMER, TIMER_B, 39);
	MAP_TimerPrescaleMatchSet(SERVO_TIMER, SERVO_TIMER_AB, 39);

	// Calculate the number of timer counts/microsecond
	ticksPerMicrosecond = (F_CPU / 40) / 1000000;

	// Initially load the timer with 20ms interval time
	MAP_TimerLoadSet(SERVO_TIMER, SERVO_TIMER_AB, ticksPerMicrosecond * REFRESH_INTERVAL);

	// Setup the interrupt for the TIMERA0B timeout.
	MAP_TimerIntEnable(SERVO_TIMER, SERVO_TIMER_TRIGGER);

	// Enable the timer.
	MAP_TimerEnable(SERVO_TIMER, SERVO_TIMER_AB);
}

/** end of static functions **/

/*
 * When a new servo is created:
 * Initialize the servo module if it has not been initialized already.
 * Add the servo to the assigned servos mask with a new index.
 */
Servo::Servo()
{
	this->index = INVALID_SERVO;

	// Look for a free servo index.
	for (int i = 0; i < SERVOS_PER_TIMER; i++)
	{
		if (((servoAssignedMask >> i) & 1) == 0)
		{
			// Save the index for this instance of Servo.
			this->index = i;

			// Mark the spot in the mask.
			servoAssignedMask |= (1 << i);

			// Stop searching for free slots.
			break;
		}
	}
}

//! Write a pulse width of the given number of microseconds to the Servo's pin
void Servo::writeMicroseconds(int value)
{
	if(value < this->min) value = this->min;
	if(value > this->max) value = this->max;

	servos[this->index].pulse_width = value;

	calculatePeriodRemainder();
}

//! Write a pulse width of the given degrees (if in the appropriate range to be degrees)
//! or of the specified number of microseconds (if in the appropriate range to be microseconds)
void Servo::write(int value)
{
	// treat values less than the min pulse width as angles in degrees (valid values in microseconds are handled as microseconds)
	if(value < MIN_SERVO_PULSE_WIDTH)
	{
		if(value < 0) value = 0;
		if(value > 180) value = 180;

		value = map(value, 0, 180, this->min,  this->max);
	}
	this->writeMicroseconds(value);
}

//! Returns the current pulse width of the Servo's signal, in microseconds
int Servo::readMicroseconds()
{
	return servos[this->index].pulse_width;
}

//! Returns the current position of the Servo, in degrees
int Servo::read() // return the value as degrees
{
  return  map( this->readMicroseconds()+1, this->min, this->max, 0, 180);
}

//! Attach the Servo to the given pin (and, if specified, with the given range of legal pulse widths)
unsigned int Servo::attach(unsigned int pin, int min, int max)
{
	// If the module has not been initialized
	if(!servoInitialized)
	{
		// Initialize it.
		initServo();
		// It has been initialized, prevent further calls to initServo().
		servoInitialized = true;
	}

	this->min = min;
	this->max = max;

	servos[this->index].pin_number = pin;

	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);

	calculatePeriodRemainder();

	servos[this->index].enabled = true;

	return this->index;
}

//! Detach the Servo from its pin
void Servo::detach()
{
	// Disable, clean up
	servos[this->index].enabled = false;
	servos[this->index].pulse_width = DEFAULT_SERVO_PULSE_WIDTH;
	calculatePeriodRemainder();

	digitalWrite(servos[this->index].pin_number, LOW);
}

//! ISR for generating the pulse widths
void ServoIntHandler(void)
{

	// Clear the timer interrupt.
	MAP_TimerIntClear(SERVO_TIMER, SERVO_TIMER_TRIGGER);

	// Get the pulse width value for the current servo from the array
	// and reload the timer with the new pulse width count value
	// if we have already serviced all servos (currentServo = MAX_SERVO_NO)
	// then this value should be the 20ms period value
	if(currentServo < SERVOS_PER_TIMER)
	{
		MAP_TimerLoadSet(SERVO_TIMER, SERVO_TIMER_AB, ticksPerMicrosecond * servos[currentServo].pulse_width);
	}
	else
	{
		MAP_TimerLoadSet(SERVO_TIMER, SERVO_TIMER_AB, ticksPerMicrosecond * remainderPulseWidth);
	}

	// End the servo pulse set previously (if any)
	if(currentServo > 0)  // If not the 1st Servo....
	{
		if (servos[currentServo - 1].enabled)
		{
			digitalWrite(servos[currentServo - 1].pin_number, LOW);
		}
	}

	// Set the current servo pin HIGH
	if(currentServo < SERVOS_PER_TIMER)
	{
		if (servos[currentServo - 1].enabled)
		{
			digitalWrite(servos[currentServo].pin_number, HIGH);
		}
		currentServo++;  // Advance to next servo for processing next time
	}
	else
	{
		currentServo = 0; // Start all over again
	}
}
