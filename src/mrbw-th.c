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
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#ifdef MRBEE
// If wireless, redefine the common variables and functions
#include "mrbee.h"
#define mrbus_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define MRBUS_TX_PKT_READY MRBEE_TX_PKT_READY
#define MRBUS_RX_PKT_READY MRBEE_RX_PKT_READY
#define mrbux_rx_buffer mrbee_rx_buffer
#define mrbus_tx_buffer mrbee_tx_buffer
#define mrbus_state mrbee_state
#define mrbusInit mrbeeInit
#define mrbusPacketTransmit mrbeePacketTransmit
#endif

#include "mrbus.h"

extern uint8_t mrbus_activity;
extern uint8_t mrbus_rx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_tx_buffer[MRBUS_BUFFER_SIZE];
extern uint8_t mrbus_state;

#define TH_STATE_IDLE          0x00
#define TH_STATE_TRIGGER       0x10 
#define TH_STATE_WAIT          0x20
#define TH_STATE_READ          0x30
#define TH_STATE_SEND_PACKET   0x40

#define TH_EE_NUM_AVG          0x10

uint8_t mrbus_dev_addr = 0;
uint8_t th_state = TH_STATE_IDLE;
uint8_t num_avg = 1;
uint16_t pkt_period = 20;

// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint8_t decisecs;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
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
	}
}

// End of 100Hz timer

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (mrbus_rx_buffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != mrbus_rx_buffer[MRBUS_PKT_DEST] && mrbus_dev_addr != mrbus_rx_buffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<mrbus_rx_buffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, mrbus_rx_buffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != mrbus_rx_buffer[MRBUS_PKT_CRC_L]))
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
	
	if ('A' == mrbus_rx_buffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 6;
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'a';
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	} 
	else if ('W' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6], mrbus_rx_buffer[7]);
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = mrbus_rx_buffer[7];
		if (MRBUS_EE_DEVICE_ADDR == mrbus_rx_buffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		else if ( (MRBUS_EE_DEVICE_UPDATE_L == mrbus_rx_buffer[6]) || (MRBUS_EE_DEVICE_UPDATE_H == mrbus_rx_buffer[6]) )
		{
		    pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
#if defined DHT11 || defined DHT22
			pkt_period = max(pkt_period, 20);
#endif
		}
		else if (TH_EE_NUM_AVG == mrbus_rx_buffer[6])
		{
			num_avg = eeprom_read_byte((uint8_t*)TH_EE_NUM_AVG);			
#if defined DHT11 || defined DHT22
			num_avg = 1;
#endif
		}
		mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	
	}
	else if ('R' == mrbus_rx_buffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		mrbus_tx_buffer[MRBUS_PKT_DEST] = mrbus_rx_buffer[MRBUS_PKT_SRC];
		mrbus_tx_buffer[MRBUS_PKT_LEN] = 8;			
		mrbus_tx_buffer[MRBUS_PKT_TYPE] = 'r';
		mrbus_tx_buffer[6] = mrbus_rx_buffer[6];
		mrbus_tx_buffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)mrbus_rx_buffer[6]);			
		mrbus_state |= MRBUS_TX_PKT_READY;
		goto PktIgnore;
	}

	// FIXME:  Insert code here to handle incoming packets specific
	// to the device.

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	mrbus_state &= (~MRBUS_RX_PKT_READY);
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

#if defined DHT11 || defined DHT22

volatile uint8_t dht11_bitnum=0;
volatile uint8_t dht11_data[5];
volatile uint8_t dht11_read_complete=3;

void initializeDHT11Timer()
{
	// Set up timer 1 for 100Hz interrupts
	TCNT2 = 0;
	OCR2A = 0xC2;
	TCCR2A = 0;
	TCCR2B = _BV(CS21);
}

ISR(PCINT1_vect)
{
	// This is used by the DHT11 code to read asynchronously to the main loop
	PORTC |= _BV(PC5);
	if(!(PINC & _BV(PC3)))
	{
		uint8_t timerval = TCNT2;
		// It was the falling edge.
		// 26-28ms in the timer indicates that it's a 0, 70ms indicates a 1
		// 0.7265625uS per count
		// A logic zero will be between 27 and 48
		// A logic one will be between 83 and 110
		
		TCNT2 = 0;
		if (timerval > 70 && timerval < 120)
		{
			// It's a one, put one in the correct place
			dht11_data[dht11_bitnum / 8] |= 1<<(7-(dht11_bitnum % 8));
		
		}
		dht11_bitnum++;
	}
	else
	{
		// It's the rising edge, indicating that we're at the beginning of a clock period
		// Reset the timer
		TCNT2 = 0;
		TIMSK2 |= _BV(TOIE2);
	}
	PORTC &= ~_BV(PC5);

	if(dht11_bitnum >= 40)
	{
		// Shut down the PCINT11 interrupt
		PCIFR |= _BV(PCIF1);
		PCICR &= ~_BV(PCIE1);
		TIMSK2 &= ~_BV(TOIE2);

		if ((dht11_data[0] + dht11_data[1] + dht11_data[2] + dht11_data[3]) == dht11_data[4])
			// We're done, indicate complete and shut down the interrupt
			dht11_read_complete = 1;
		else
			dht11_read_complete = 2; // Indicate failure - checksums didn't match

	}
}


ISR(TIMER2_OVF_vect)
{
	// We overflowed, meaning the DHT11 stopped responding correctly
	dht11_read_complete = 2;

	// Shut down Timer 2
	TIMSK2 &= ~_BV(TOIE2);
	PORTC &= ~_BV(PC5);	
	// Shut down the PCINT11 interrupt
	PCIFR |= _BV(PCIF1);
	PCICR &= ~_BV(PCIE1);
}

void dht11_start_conversion()
{
	uint8_t countdown = 20;
	// Get triggery
	// Set DHT11 data wire to ground for 20ms

	initializeDHT11Timer();

	dht11_bitnum=0;
	memset((void*)dht11_data, 0, sizeof(dht11_data));
	dht11_read_complete=0;
	
	DDRC |= _BV(PC3);	
	while (countdown-- != 0)
	{
		_delay_ms(1);
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();			
	}
	
	// Clear all the timer interrupts - chances are we've overflowed
	//  since the last time we started
	TCNT2 = 0;
	TIFR2 |= 0x07;
	// Clear the timer counter and enable the interrupt
	TIMSK2 |= _BV(TOIE2);

	DDRC &= ~_BV(PC3);
	TCNT2 = 0;

	// 20-40uS high after AVR releases the line
	// Wait for the low for up to 186uS
	while ((PINC & _BV(PC3)) && 0 == dht11_read_complete);
	
	// Failure, the DHT11 didn't respond, return
	if (0 != dht11_read_complete)
	{
		dht11_read_complete = 0x42;
		return;
	}

	TCNT2 = 0;
	// After going high for 20-40uS, the DHT11 should pull it low 
	// for 80uS, then high again for 80uS
	// Wait up to 186 uS for the high.
	while (!(PINC & _BV(PC3)) && 0 == dht11_read_complete);

	// Failure, the DHT11 didn't respond, return
	if (0 != dht11_bitnum)
	{
		dht11_read_complete = 0x62;
		return;
	}

	// 20-40uS high after AVR releases the line
	// Wait for the low for up to 186uS
	TCNT2 = 0;
	while ((PINC & _BV(PC3)) && 0 == dht11_read_complete);

	TCNT2 = 0;
	// Failure, the DHT11 didn't respond, return
	if (0 != dht11_read_complete)
	{
		dht11_read_complete = 0x82;
		return;
	}

	// If we're here, we're on the low ahead of the first bit.  
	// Enable pin change interrupt and go!
	PCMSK1 = _BV(PCINT11);
	PCIFR |= _BV(PCIF1);
	PCICR |= _BV(PCIE1);
}

#endif


void init(void)
{
	DDRC &= ~_BV(PC3);
	PORTC &= ~(_BV(PC3) | _BV(PC5));
	DDRC |= _BV(PC5);
	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);

	// Initialize MRBus packet update interval from EEPROM
	pkt_period = ((eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H) << 8) & 0xFF00) | (eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) & 0x00FF);
#if defined DHT11 || defined DHT22
	pkt_period = max(pkt_period, 20);
#endif

	// Initialize averaging value from EEPROM
#if defined DHT11 || defined DHT22
	num_avg = 1;
#else
	num_avg = eeprom_read_byte((uint8_t*)TH_EE_NUM_AVG);
#endif
	
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = 0x36;  // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC
}


int main(void)
{
	uint16_t kelvinTemp = 4370; // Expressed in 1/16ths K, base of 273.15 K
	uint16_t relHumidity = 0; // Expressed in 1/10ths % RH
	uint8_t count=0;
#if defined DHT11 || defined DHT22
	uint8_t dht11_powerup_lockout = 1;
#else
	uint8_t dht11_powerup_lockout = 0;
#endif

	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusInit();

	sei();	

	while (1)
	{
#ifdef MRBEE
		mrbeePoll();
#endif
		// Handle any packets that may have come in
		if (mrbus_state & MRBUS_RX_PKT_READY)
			PktHandler();

		// Notes of warning - DANGER, WILL ROBINSON!  DANGER!
		// DHT11/DHT22/RHT03 must *NOT* be accessed within the first second of being powered up
		// DHT11/DHT22/RHT03 must *NOT* be read more than once every 2 seconds
		// Both of these restrictions come from the datasheet

		if (dht11_powerup_lockout && decisecs > 12)
			dht11_powerup_lockout = 0;

		if (!dht11_powerup_lockout && decisecs >= pkt_period)
		{
			if(TH_STATE_IDLE == th_state)
				th_state = TH_STATE_TRIGGER;

			decisecs = 0;
		}

		if (TH_STATE_TRIGGER == th_state)
		{
#if defined DHT11 || defined DHT22
			dht11_start_conversion();
#endif
			// Trigger an ADC conversion
			busVoltage = 0;
			busVoltageCount = 0;
			ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
			th_state = TH_STATE_WAIT;
		}
		else if (TH_STATE_WAIT == th_state)
		{
			uint8_t conversionComplete = 0;

#if defined DHT11 || defined DHT22
			if (dht11_read_complete)
				conversionComplete = 1;
#endif

			if (conversionComplete && !(ADCSRA & _BV(ADEN)) )
//			if (conversionComplete)
				th_state = TH_STATE_READ;

		}
		else if (TH_STATE_READ == th_state)
		{
			uint16_t temp;

			kelvinTemp = 4370; // Expressed in 1/16ths K, base of 273.15 K
			relHumidity = 0; // Expressed in 1/10ths % RH		


			// This is where things get ugly
			// For the DHT11 data bytes:
			//   0: integer humidity %
			//   1: unused, reads as 0
			//   2: integer temperature in C, (high bit indicates negative temp)
			//   3: unused, reads as 0
			//   4: checksum (literally a sum of the first four bytes)
			// For the DHT22/RHT03 data bytes:
			//   0: high 8 bits of humidity, in 1/10ths of %
			//   1: low 8 bits of humidity, in 1/10ths of %
			//   2: high 8 bits of temp in C, in 1/10ths of degrees (high bit indicates negative temp)
			//   3: low 8 bits of temp in C, in 1/10ths of degrees
			//   4: checksum (literally a sum of the first four bytes)
			// There's no systemic way to tell these things apart, so we have to be configured one way or the other

#ifdef DHT11
			relHumidity = (uint16_t)dht11_data[0] * 2;
			temp = (uint16_t)(dht11_data[2] & 0x7F);
			temp <<= 4;
			if (dht11_data[2] & 0x80)
				kelvinTemp -= temp;
			else
				kelvinTemp += temp;

#elif defined DHT22
			relHumidity = (((uint16_t)dht11_data[0])<<8) + (uint16_t)dht11_data[1];
			temp = ((uint16_t)((dht11_data[2] & 0x7F))<<8)+(uint16_t)dht11_data[3];
			temp *= 8;
			temp /= 5;
			
			relHumidity /= 5;
			
			if (dht11_data[2] & 0x80)
				kelvinTemp -= temp;
			else
				kelvinTemp += temp;
#endif
			// Div by 8, as we use 8 samples
			busVoltage = busVoltage >> 3;  
			
			//At this point, we're at (Vbus/6) / 5 * 1024
			//So multiply by 300, divide by 1024, or multiply by 75 and divide by 256
			busVoltage = ((uint32_t)busVoltage * 75) >> 8;

			count++;
			if(count >= num_avg)
			{
				th_state = TH_STATE_SEND_PACKET;
				count = 0;
			} else {
				th_state = TH_STATE_TRIGGER;
			}
		}
		else if(th_state == TH_STATE_SEND_PACKET)
		{
			mrbus_tx_buffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbus_tx_buffer[MRBUS_PKT_DEST] = 0xFF;
			mrbus_tx_buffer[MRBUS_PKT_LEN] = 11;
			mrbus_tx_buffer[5] = 'S';
			mrbus_tx_buffer[6] = 0;  // Status byte.  Lower three bits are sensor type, 000 = DHT11/DHT22/RHT03
			mrbus_tx_buffer[7] = (kelvinTemp >> 8);
			mrbus_tx_buffer[8] = kelvinTemp & 0xff;
			mrbus_tx_buffer[9] = relHumidity & 0xff;
			mrbus_tx_buffer[10] = (uint8_t)busVoltage;
			
//			mrbus_tx_buffer[6] = dht11_data[0];  // Status byte.  Lower three bits are sensor type, 000 = DHT11/DHT22/RHT03
//			mrbus_tx_buffer[7] = dht11_data[1];
//			mrbus_tx_buffer[8] = dht11_data[2];
//			mrbus_tx_buffer[9] = dht11_data[3];
//			mrbus_tx_buffer[10] = dht11_read_complete;

			mrbus_state |= MRBUS_TX_PKT_READY;

			th_state = TH_STATE_IDLE;
		}

		// If we have a packet to be transmitted, try to send it here
		while(mrbus_state & MRBUS_TX_PKT_READY)
		{
			uint8_t bus_countdown;

			// Even while we're sitting here trying to transmit, keep handling
			// any packets we're receiving so that we keep up with the current state of the
			// bus.  Obviously things that request a response cannot go, since the transmit
			// buffer is full.
			if (mrbus_state & MRBUS_RX_PKT_READY)
				PktHandler();


			if (0 == mrbusPacketTransmit())
			{
				mrbus_state &= ~(MRBUS_TX_PKT_READY);
				break;
			}

#ifndef MRBEE
			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			bus_countdown = 20;
			while (bus_countdown-- > 0 && MRBUS_ACTIVITY_RX_COMPLETE != mrbus_activity)
			{
				//clrwdt();
				_delay_ms(1);
				if (mrbus_state & MRBUS_RX_PKT_READY) 
					PktHandler();
			}
#endif
		}
	}
}



