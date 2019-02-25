/*************************************************************************
Title:    MRBus AVR Template
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2012 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
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

#include "mrbus.h"

#include "mrbee.h"

#define TH_STATE_IDLE          0x00
#define TH_STATE_TRIGGER       0x10 
#define TH_STATE_WAIT          0x20
#define TH_STATE_READ          0x30
#define TH_STATE_SEND_PACKET   0x40
#define TH_STATE_XMIT_WAIT     0x50
#define TH_STATE_WAKE          0x60

#define TH_EE_NUM_AVG          0x10

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;
uint8_t th_state = TH_STATE_IDLE;
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
	PORTD &= ~_BV(PD7);
}

void disableXB3()
{
	PORTD |= _BV(PD7);
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

ISR(ADC_vect)
{
	busVoltage += ADC;
	if (++busVoltageCount >=8)
	{
		// Disable ADC
		ADCSRA &= ~(_BV(ADEN) | _BV(ADIE));	
	}
}


#ifdef SHT3X
#include "avr-i2c-master.h"

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
	i2c_transmit(msgBuf, 1, 0);
	while(i2c_busy());
	sht3x_read_countdown = 2; // In 100mS increments
}

uint8_t sht3x_read_value(float* kelvinTemp, float* relativeHumidity)
{
	uint8_t msgBuf[7];
	uint16_t tmpVal=0;
	uint8_t fail = 0;
	memset(msgBuf, 0, sizeof(msgBuf));

	msgBuf[0] = SHT3X_I2C_ADDR | 0x01;  // Read address
	i2c_transmit(msgBuf, 7, 0);
	i2c_receive(msgBuf, 7);

	tmpVal = ((((uint16_t)msgBuf[1]))<<8) | ((uint16_t)msgBuf[2]);
	if (msgBuf[3] == sht3x_crc_calculate(tmpVal))
	{
		// Checksum works out
		// Tc = -45 + 175 * (Sensor / 65535)
		*kelvinTemp = -45.0 + ( ((float)tmpVal) / (65535.0 / 175.0) );
	} else {
		*kelvinTemp = 0;
		fail |= 0x01;
	}

	tmpVal = ((((uint16_t)msgBuf[4]))<<8) | ((uint16_t)msgBuf[5]);
	if (msgBuf[6] == sht3x_crc_calculate(tmpVal))
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

#endif


#ifdef LOWPOWER

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

#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	MCUSR &= ~(_BV(WDRF));
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	return(slept);
}

#endif


// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	//OCR0A = 0xC2;
	OCR0A = 0x6C; // Appropriate for 11.0592 MHz
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;

#ifdef SHT3X
		if (sht3x_read_countdown)
			sht3x_read_countdown--;
#endif
	}
}

// End of 100Hz timer



void init(void)
{
	// Kill watchdog
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	// Set sleep pin to output; drive low
	DDRD = _BV(PD7);
	PORTD = 0x7F;

	DDRB = 0;
	PORTB = 0xFF;

	DDRC = _BV(PC2); // External 3.3V enable pin	
	PORTC = 0xFF;

	ACSR = _BV(ACD);
#if defined SHT3X
	i2c_master_init();
#endif
	
	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);

	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	// Initialize MRBus packet update interval from EEPROM
	pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);

	// Initialize averaging value from EEPROM
#if defined SHT3X
	num_avg = 1;
#else
	num_avg = eeprom_read_byte((uint8_t*)TH_EE_NUM_AVG);
#endif
	
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	disableExternal33();
}


int main(void)
{
	float kelvinTemp = 0.0; // Expressed in 1/16ths K, base of 273.15 K
	float relHumidity = 0.0; // Expressed in 1/10ths % RH
	uint8_t count=0;
	uint16_t decisecs_snapshot;

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
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
			PktHandler();

		switch(th_state)
		{
			case TH_STATE_IDLE:
				ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
				{
					if (decisecs >= pkt_period)
					{
						th_state = TH_STATE_TRIGGER;
						count = 0;
						decisecs = 0;
					}
				}
				break;

			case TH_STATE_TRIGGER:
	#if defined SHT3X
				sht3x_start_conversion();
	#elif defined DUMMY_SENSOR
				// Do nothing
	#endif
				// Trigger an ADC conversion
				busVoltage = 0;
				busVoltageCount = 0;
				ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
				th_state = TH_STATE_WAIT;
				break;

			case TH_STATE_WAIT:
				{
					uint8_t conversionComplete = 0;

#if defined SHT3X
					if (0 == sht3x_read_countdown)
						conversionComplete = 1;
#elif defined DUMMY_SENSOR
					// Just wait a while
					_delay_us(200);
					conversionComplete = 1;
#endif
					if (conversionComplete && !(ADCSRA & _BV(ADEN)) )
					{
						PORTC &= ~_BV(PC5);
						th_state = TH_STATE_READ;
					}
				}
				break;

			case TH_STATE_READ:
				{
					relHumidity = 0.0; // Expressed in 1/10ths % RH		

		#if defined SHT3X
					sht3x_read_value(&kelvinTemp, &relHumidity);
		#elif defined DUMMY_SENSOR
					// Tada!  Both the temperature and humity are going to be 42
					kelvinTemp = 42;
					relHumidity = 42;
		#endif
					// Div by 8, as we use 8 samples
					busVoltage = busVoltage >> 3;  

					if(++count >= num_avg)
					{
						th_state = TH_STATE_SEND_PACKET;
						count = 0;
					} else {
						th_state = TH_STATE_TRIGGER;
					}
				}
				break;

			case TH_STATE_SEND_PACKET:
				{
					uint8_t mrbusTxBuffer[MRBUS_BUFFER_SIZE];
					memset(mrbusTxBuffer, 0, sizeof(mrbusTxBuffer));

					mrbusTxBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
					mrbusTxBuffer[MRBUS_PKT_DEST] = 0xFF;
					mrbusTxBuffer[MRBUS_PKT_LEN] = 12;
					mrbusTxBuffer[5] = 'S';
					mrbusTxBuffer[6] = 0;  // Status byte.  Lower three bits are sensor type, 000 = DHT11/DHT22/RHT03
/*					mrbusTxBuffer[7] = (kelvinTemp >> 8);
					mrbusTxBuffer[8] = kelvinTemp & 0xff;
					mrbusTxBuffer[9] = (relHumidity >> 8);
					mrbusTxBuffer[10] = relHumidity & 0xff;*/

					mrbusTxBuffer[11] = (VINDIV * VDD * (uint32_t)busVoltage) / 1024;  // VINDIV is reciprocal of VIN divider ratio.  VDD is in decivolts

					mrbusPktQueuePush(&mrbeeTxQueue, mrbusTxBuffer, mrbusTxBuffer[MRBUS_PKT_LEN]);
				}
#ifdef LOWPOWER
				th_state = TH_STATE_XMIT_WAIT;
#else
				th_state = TH_STATE_IDLE;
#endif
				break;


			case TH_STATE_XMIT_WAIT:
				/*
				if(mrbus_state & (MRBUS_TX_BUF_ACTIVE | MRBUS_TX_PKT_READY)))
				{
					uint8_t i=0;
					// FIXME: this is crap
					// Need to actually see when we're done sending out of the XBEE
					// Also, do this as a loop, in case we're on a short watchdog
					for (i=0; i<25; i++)
					{
						wdt_reset();
						_delay_ms(10);
					}
					
					// Grab snapshot of decisecs to make sure it doesn't advance between
					// the comparison and the subtraction, causing a "negative" sleep value.
					ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
					{
						decisecs_snapshot = decisecs;
					}

					if(decisecs_snapshot < pkt_period)
					{
						// Not yet at pkt_period, sleep for remaining time
						decisecs += system_sleep(pkt_period - decisecs_snapshot);
					}

					th_state = TH_STATE_WAKE;
				}*/
				// This block is total crap and needs to be rewritten
				break;

			case TH_STATE_WAKE:
				// FIXME:  Do wakeup stuff
				th_state = TH_STATE_IDLE;
				break;

			default:
				// No idea why we're here, go back to idle
				th_state = TH_STATE_IDLE;
				break;
		}


		// Handle any MRBus packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
		{
			PktHandler();
		}

		// Transmit any pending MRBus packets
		while (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			// Unsleep the XBee
			enableXB3();
			wdt_reset();
			mrbeeTransmit();
		}

		// Wait for XBee TX ISR to finish
		do {wdt_reset();} while (bit_is_set(UCSR0B, UDRIE0));

		// Handle any MRBus packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
		{
			PktHandler();
		}


		// Sleep the XBee
		disableXB3();
	}
}



