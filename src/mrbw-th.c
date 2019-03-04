/*************************************************************************
Title:    MRBW-TH Firmware
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2019 Nathan Holmes & Michael Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "mrbee.h"
#include "avr-i2c-master.h"
#include "float16.h"

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;
uint8_t num_avg = 1;
uint16_t pkt_period = 20;

void enableExternal33()
{
	PORTC &= ~_BV(PC2);
}

void disableExternal33()
{
	PORTC |= _BV(PC2);
}

void enableXB3()
{
	PORTD &= ~_BV(PD4);
}

void disableXB3()
{
	PORTD |= _BV(PD4);
}

uint8_t isXbeeEnabled()
{
	return ( (PORTD & _BV(PD4))?0:1 );
}

void createVersionPacket(uint8_t destAddr, uint8_t *buf)
{
	buf[MRBUS_PKT_DEST] = destAddr;
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_LEN] = 9;
	buf[MRBUS_PKT_TYPE] = 'v';
	// Software Revision
	buf[6]  = 'T';
	buf[7]  = 'H'; // Software Revision
	buf[8]  = '3';
}


void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	uint8_t rssi;

	if (0 == mrbeePktQueuePop(&mrbeeRxQueue, rxBuffer, sizeof(rxBuffer), &rssi))
		return;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		else if ( (MRBUS_EE_DEVICE_UPDATE_L == rxBuffer[6]) || (MRBUS_EE_DEVICE_UPDATE_H == rxBuffer[6]) )
		{
			pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
		}
		goto PktIgnore;
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		createVersionPacket(rxBuffer[MRBUS_PKT_SRC], txBuffer);
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
}

volatile uint16_t busVoltage=0;
volatile uint8_t busVoltageCount=0;

void initializeADC()
{
	busVoltage = 0;
	busVoltageCount = 0;

	ACSR = _BV(ACD);

	// Setup ADC for bus voltage monitoring
	ADMUX  = 0x47;  // AVCC reference, ADC0 starting channel
	ADCSRA = _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC
}


ISR(ADC_vect)
{
	busVoltage += ADC;
	if (++busVoltageCount >=8)
	{
		// Disable ADC
		ADCSRA &= ~(_BV(ADEN) | _BV(ADIE) | _BV(ADATE));	
	}
}

void startADCConversion()
{
	busVoltage = 0;
	busVoltageCount = 0;
	// Enable ADC and ISR, clear interrupt flag
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF) | _BV(ADATE);
}

uint8_t isADCDone()
{
	return ((busVoltageCount>=8)?1:0);
}

volatile uint8_t sht3x_read_countdown=0;

#define SHT3X_I2C_ADDR        (0x44<<1)

// Sensirion polynomial: x^8 + x^5 + x^4 + 1
#define CRC_POLYNOMIAL 0x31

uint8_t sht3x_crc_calculate(uint16_t sensorVal)
{
	uint8_t crc = 0;
	uint8_t bit;

	crc ^= (sensorVal >> 8);

	for(bit=8; bit > 0; bit--)
	{
		if (crc & 0x80)
			crc = (crc << 1) ^ CRC_POLYNOMIAL;
		else
			crc = (crc << 1);
	}

	crc ^= (sensorVal & 0xFF);

	for(bit=8; bit > 0; bit--)
	{
		if (crc & 0x80)
			crc = (crc << 1) ^ CRC_POLYNOMIAL;
		else
			crc = (crc << 1);
	}

	return crc;
}

void sht3x_start_conversion()
{
	uint8_t msgBuf[3];
	memset(msgBuf, 0, sizeof(msgBuf));    
	msgBuf[0] = SHT3X_I2C_ADDR;
	msgBuf[1] = 0x24; // No clock stretching
	msgBuf[2] = 0x00; // High accuracy/repeatability
	i2c_transmit(msgBuf, 3, 1);
	while(i2c_busy());
	sht3x_read_countdown = 2; // In 100mS increments
}

uint8_t sht3x_read_value(float* tempCelsius, float* relativeHumidity)
{
	uint8_t msgBuf[7];
	uint16_t tmpVal=0;
	uint8_t fail = 0;
	memset(msgBuf, 0, sizeof(msgBuf));

	msgBuf[0] = SHT3X_I2C_ADDR | 0x01;  // Read address
	i2c_transmit(msgBuf, 7, 0);
	i2c_receive(msgBuf, 7);

	tmpVal = ((((uint16_t)msgBuf[1]))<<8) | ((uint16_t)msgBuf[2]);
	if (1 || msgBuf[3] == sht3x_crc_calculate(tmpVal))
	{
		// Checksum works out
		// Tc = -45 + 175 * (Sensor / 65535)
		*tempCelsius = -45.0 + ( ((float)tmpVal) / (65535.0 / 175.0) );
	} else {
		*tempCelsius = 0;
		fail |= 0x01;
	}

	tmpVal = ((((uint16_t)msgBuf[4]))<<8) | ((uint16_t)msgBuf[5]);
	if (1 || msgBuf[6] == sht3x_crc_calculate(tmpVal))
	{
		// Checksum works out
		// RH = 100 * (Sensor / 65535)
		*relativeHumidity = ((float)tmpVal) / 655.35;
	} else {
		*relativeHumidity = 0;
		fail |= 0x02;
	}
		
	return(fail);
}

volatile uint8_t wdt_tripped=0;

ISR(WDT_vect) 
{
	wdt_tripped=1;  // set global volatile variable
}


uint16_t system_sleep(uint16_t sleep_decisecs)
{
	uint16_t slept = 0;

	while(slept < sleep_decisecs)
	{
		uint16_t remaining_sleep = sleep_decisecs - slept;
		uint8_t planned_sleep = 80;
		uint8_t wdtcsr_bits = _BV(WDIF) | _BV(WDIE);

		if (remaining_sleep == 1)
		{
			wdtcsr_bits |= _BV(WDP1) | _BV(WDP0);
			planned_sleep = 1;
		}
		else if (remaining_sleep <= 3)
		{
			wdtcsr_bits |= _BV(WDP2);
			planned_sleep = 3;
		}
		else if (remaining_sleep <= 5)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP0);
			planned_sleep = 5;
		}
		else if (remaining_sleep <= 10)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1);
			planned_sleep = 10;
		}
		else if (remaining_sleep <= 20)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
			planned_sleep = 20;
		}
		else if (remaining_sleep <= 40)
		{
			wdtcsr_bits |= _BV(WDP3);
			planned_sleep = 40;
		}
		else
		{
			wdtcsr_bits |= _BV(WDP3) | _BV(WDP0);
			planned_sleep = 80;
		}

		// Procedure to reset watchdog and set it into interrupt mode only

		cli();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // set the type of sleep mode to use
		sleep_enable();                           // enable sleep mode
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = wdtcsr_bits;

		sei();

		wdt_tripped = 0;
		// Wrap this in a loop, so we go back to sleep unless the WDT woke us up
		while (0 == wdt_tripped)
			sleep_cpu();

		wdt_reset();
		WDTCSR |= _BV(WDIE); // Restore WDT interrupt mode

		slept += planned_sleep;
	}

	sleep_disable();

	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	MCUSR &= ~(_BV(WDRF));
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();

	return(slept);
}


// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint16_t decisecs;
volatile uint8_t xbeeIdleCountdown = 0;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0x6C; // Appropriate for 11.0592 MHz
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	static uint8_t ticks = 0;
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;

#ifdef SHT3X
		if (sht3x_read_countdown)
			sht3x_read_countdown--;
#endif

		if (xbeeIdleCountdown)
			xbeeIdleCountdown--;

	}
}

// End of 100Hz timer

typedef enum
{
	TH_STATE_IDLE          = 0x00,
	TH_STATE_TRIGGER       = 0x10,
	TH_STATE_WAIT          = 0x20,
	TH_STATE_READ          = 0x30,
	TH_STATE_SEND_PACKET   = 0x40,
	TH_STATE_XMIT_WAIT     = 0x50,
	TH_STATE_WAKE          = 0x60,
	TH_STATE_SLEEP         = 0x70
} THState;


void init(void)
{
	// Kill watchdog
	MCUSR = 0;
	wdt_enable(WDTO_1S);
	wdt_reset();

	DDRB = 0;
	PORTB = 0xFF;

	DDRC = _BV(PC2); // External 3.3V enable pin	
	PORTC = 0xFF;
	PORTC &= ~(_BV(PC2));

	// Set sleep pin to output; drive low
	DDRD = _BV(PD4) | _BV(PD2);
	PORTD = 0xFF;

	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);

	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	// Initialize MRBus packet update interval from EEPROM
	pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);

	enableExternal33();
	i2c_master_init();

	initializeADC();

	enableXB3();
}


int main(void)
{
	THState thState = TH_STATE_IDLE;
	
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();
	
	sei();

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
			PktHandler();

		switch(thState)
		{
			case TH_STATE_IDLE:
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					if (decisecs >= pkt_period)
					{
							decisecs = 0;
							thState = TH_STATE_TRIGGER;
					}
				}
				break;

			case TH_STATE_TRIGGER:
				startADCConversion();
				sht3x_start_conversion();
				thState = TH_STATE_WAIT;
				break;

			case TH_STATE_WAIT:
				if (isADCDone() && 0 == sht3x_read_countdown)
					thState = TH_STATE_READ;
				break;

			case TH_STATE_READ:
				{
					uint8_t mrbusTxBuffer[MRBUS_BUFFER_SIZE];
					float temperature = 0.0;
					float humidity = 0.0;
					float16_t k;
					memset(mrbusTxBuffer, 0, sizeof(mrbusTxBuffer));

					sht3x_read_value(&temperature, &humidity);

					mrbusTxBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
					mrbusTxBuffer[MRBUS_PKT_DEST] = 0xFF;
					mrbusTxBuffer[MRBUS_PKT_LEN] = 10;
					mrbusTxBuffer[5] = 'S';
					k = F32toF16(temperature);
					mrbusTxBuffer[6] = (uint8_t)(k>>8);
					mrbusTxBuffer[7] = (uint8_t)k;
					mrbusTxBuffer[8] = (uint8_t)humidity;
					mrbusTxBuffer[9] = busVoltage / 248;  // 1023 = 3.3V, therefore 1023 * 8 / 33 = 248
					

					mrbusPktQueuePush(&mrbeeTxQueue, mrbusTxBuffer, mrbusTxBuffer[MRBUS_PKT_LEN]);
				}
				thState = TH_STATE_SEND_PACKET;
				break;

			case TH_STATE_SEND_PACKET:
				// Transmit any pending MRBus packets
				while (mrbusPktQueueDepth(&mrbeeTxQueue))
				{
					if (!isXbeeEnabled())
					{
						enableXB3(); // Wake the XBee at this point - can take up to 6mS to start up
						_delay_ms(6);
					}
					mrbeeTransmit();
				}

				// Wait for XBee TX ISR to finish
				if (!mrbeeTxActive())
				{
					xbeeIdleCountdown = 3;
					thState = TH_STATE_XMIT_WAIT;
				}
				break;

			case TH_STATE_XMIT_WAIT:
				if (0 == xbeeIdleCountdown)
					thState = TH_STATE_SLEEP;
				break;

			case TH_STATE_WAKE:
				thState = TH_STATE_IDLE;
				break;

			case TH_STATE_SLEEP:
				{
					uint16_t decisecsCopy;

					disableXB3();

					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						decisecsCopy = decisecs;
					}

					//PRR |= _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRSPI) | _BV(PRADC);

					decisecsCopy += system_sleep(pkt_period - decisecsCopy);

					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						 decisecs = decisecsCopy;
					}

					thState = TH_STATE_WAKE;
				}
				break;

			default:
				thState = TH_STATE_IDLE;
				break;
		}
	}
}



