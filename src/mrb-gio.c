/*************************************************************************
Title:    MRBus GIO Basic Firmware
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2013 Nathan Holmes

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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "mrbus.h"

uint8_t mrbus_dev_addr = 0;
uint8_t pkt_count = 0;

// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;


volatile uint8_t status=0;
#define STATUS_READ_INPUTS 0x01

/*
  Configuration addresses:
  0x10 - Operating mode
     0 - Simple, all inputs
     1 - Simple, all outputs
     2 - Simple, 8 inputs, 8 outputs
    20 - Complex configuration
    
  0x11 - Simple mode input source node address (for nodes with simple mode outputs only)
  0x12 - Pullups for inputs 0-7
  0x13 - Pullups for inputs 8-15

  0x20 - 

*/

#define MRBGIO_EE_OP_MODE          0x10
#define MRBGIO_EE_SIMPLE_SRC_ADDR  0x11
#define MRBGIO_EE_IN0_7_PULLUPS    0x12
#define MRBGIO_EE_IN8_15_PULLUPS   0x13

#define MRBGIO_EE_CPLX_DDR_0       0x20
#define MRBGIO_EE_CPLX_DDR_1       0x21

#define MRBGIO_EE_FILTER_ADDR      0x30
#define MRBGIO_EE_FILTER_PKT       0x40
#define MRBGIO_EE_FILTER_BITBYTE   0x50
#define MRBGIO_EE_FILTER_SUBTYPE   0x60


#define MRBGIO_OP_MODE_IN16        0x00
#define MRBGIO_OP_MODE_OUT16       0x01
#define MRBGIO_OP_MODE_IN8OUT8     0x02
#define MRBGIO_OP_MODE_COMPLEX     0x20


uint8_t io_ddr[2] = {0,0}; // 1=output, 0=input
uint8_t io_pullups[2] = {0xFF,0xFF}; // 1=on, 0=off
uint8_t io_output[2] = {0,0};
uint8_t io_input[2] = {0,0};
uint8_t operatingMode = 0;
uint8_t io_srcAddr = 0x00;

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

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
	if (ticks & 0x01)
		status |= STATUS_READ_INPUTS;

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
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	
	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
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
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE] && 0xFF != rxBuffer[MRBUS_PKT_DEST]) 
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
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
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
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 16;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = MRBUS_VERSION_WIRED;
		txBuffer[7]  = 0; // Software Revision
		txBuffer[8]  = 0; // Software Revision
		txBuffer[9]  = 0; // Software Revision
		txBuffer[10]  = 0; // Hardware Major Revision
		txBuffer[11]  = 0; // Hardware Minor Revision
		txBuffer[12] = 'T';
		txBuffer[13] = 'M';
		txBuffer[14] = 'P';
		txBuffer[15] = 'L';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
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
	
	switch(operatingMode)
	{
		case MRBGIO_OP_MODE_OUT16:
			// If it didn't come from our source node, get out.
			if (io_srcAddr == rxBuffer[MRBUS_PKT_SRC] && 'S' == rxBuffer[MRBUS_PKT_TYPE] && rxBuffer[MRBUS_PKT_LEN] >= 8)
			{
				io_output[1] = rxBuffer[6];
				io_output[0] = rxBuffer[7];
			}
			break;

		case MRBGIO_OP_MODE_IN8OUT8:
			// If it didn't come from our source node, get out.
			if (io_srcAddr == rxBuffer[MRBUS_PKT_SRC] && 'S' == rxBuffer[MRBUS_PKT_TYPE] && rxBuffer[MRBUS_PKT_LEN] >= 8)
			{
				io_output[1] = rxBuffer[7];
			}
			break;	
			

		case MRBGIO_OP_MODE_COMPLEX:
			// Complex mode can key any output from any bit on the bus using the typical src/type/bit/byte scheme
			/* BITBYTE is computed as follows:
				x = bit = 0-7
				y = byte = byte in data stream (6 is first data byte)
				xxxyyyy
			*/
	
			for (i=0; i<(MRBGIO_EE_FILTER_PKT - MRBGIO_EE_FILTER_ADDR); i++)
			{
				if (rxBuffer[MRBUS_PKT_SRC] == eeprom_read_byte((uint8_t*)(i+MRBGIO_EE_FILTER_ADDR)))
				{
					if (rxBuffer[MRBUS_PKT_TYPE] == eeprom_read_byte((uint8_t*)(i+MRBGIO_EE_FILTER_PKT)))
					{
						uint8_t byteNum = eeprom_read_byte((uint8_t*)(i+MRBGIO_EE_FILTER_BITBYTE));
						uint8_t bitNum = (byteNum>>5) & 0x07;
						byteNum &= 0x1F;
	
						if (rxBuffer[byteNum] & (1<<bitNum))
							io_output[i/8] |= 1<<(i%8);
						else
							io_output[i/8] &= ~(1<<(i%8));
					}
				}
			}
		
			break;
			
		default:
			break;
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

void init(void)
{
	// FIXME:  Do any initialization you need to do here.
	
	// Clear watchdog (in the case of an 'X' packet reset)
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

	pkt_count = 0;

	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);
	update_decisecs = min(100, update_decisecs);
}

void initGIO()
{
	// Get Operating Mode
	operatingMode = eeprom_read_byte((uint8_t*)MRBGIO_EE_OP_MODE);
	if (0xFF == operatingMode)
	{
		eeprom_write_byte((uint8_t*)MRBGIO_EE_OP_MODE, MRBGIO_OP_MODE_IN16);
		operatingMode = eeprom_read_byte((uint8_t*)MRBGIO_EE_OP_MODE);
	}

	switch(operatingMode)
	{
		case MRBGIO_OP_MODE_IN16:
			io_ddr[0] = 0;
			io_ddr[1] = 0;
			break;
		case MRBGIO_OP_MODE_OUT16:
			io_ddr[0] = 0xFF;
			io_ddr[1] = 0xFF;
			io_srcAddr = eeprom_read_byte((uint8_t*)MRBGIO_EE_SIMPLE_SRC_ADDR);
			break;
		
		case MRBGIO_OP_MODE_IN8OUT8:
			io_ddr[0] = 0;
			io_ddr[1] = 0xFF;
			io_srcAddr = eeprom_read_byte((uint8_t*)MRBGIO_EE_SIMPLE_SRC_ADDR);
			break;		

		case MRBGIO_OP_MODE_COMPLEX:
			io_ddr[0] = eeprom_read_byte((uint8_t*)MRBGIO_EE_CPLX_DDR_0);
			io_ddr[1] = eeprom_read_byte((uint8_t*)MRBGIO_EE_CPLX_DDR_1);
			break;
	
	}
	
	// Mask pullups by whatever pins are inputs
	io_pullups[0] = ~(io_ddr[0]) & eeprom_read_byte((uint8_t*)MRBGIO_EE_IN0_7_PULLUPS);
	io_pullups[1] = ~(io_ddr[1]) & eeprom_read_byte((uint8_t*)MRBGIO_EE_IN8_15_PULLUPS);
	
	DDRB = (DDRB & 0xE0) | (io_ddr[0] & 0x1F);  // IO0-IO4
	DDRD = (DDRD & 0x07) | ((io_ddr[0]>>2) & 0x38) | ((io_ddr[1]<<6) & 0xC0); //IO5-IO9
	DDRC = (DDRC & 0xC0) | ((io_ddr[1]>>2) & 0x3F); // IO10-IO15
}

// IO0  - PB0
// IO1  - PB1
// IO2  - PB2
// IO3  - PB3
// IO4  - PB4
// IO5  - PD3
// IO6  - PD4
// IO7  - PD5
// IO8  - PD6
// IO9  - PD7
// IO10 - PC0
// IO11 - PC1
// IO12 - PC2
// IO13 - PC3
// IO14 - PC4
// IO15 - PC5

uint8_t debounce_inputs()
{
	uint8_t io_rawInputLow = (~io_ddr[0]) & ((PINB & 0x1F) | ((PIND<<2) & 0xE0)); // Low bits
	uint8_t io_rawInputHigh = (~io_ddr[1]) & (((PIND>>6) & 0x03) | ((PINC<<2) & 0xFC)); // High bits
	uint8_t delta0 = io_rawInputLow ^ io_input[0];
	uint8_t delta1 = io_rawInputHigh ^ io_input[1];
	static uint8_t clock_A0=0, clock_A1=0, clock_B0=0, clock_B1=0, changes0, changes1;

	clock_A0 ^= clock_B0;                     //Increment the counters
	clock_B0  = ~clock_B0;
	clock_A0 &= delta0;                       //Reset the counters if no changes
	clock_B0 &= delta0;                       //were detected.
	changes0 = ~((~delta0) | clock_A0 | clock_B0);
	io_input[0] ^= changes0;

	clock_A1 ^= clock_B1;                     //Increment the counters
	clock_B1  = ~clock_B1;
	clock_A1 &= delta1;                       //Reset the counters if no changes
	clock_B1 &= delta1;                       //were detected.
	changes1 = ~((~delta1) | clock_A1 | clock_B1);
	io_input[1] ^= changes1;

	return(0 != (changes0 | changes1)?1:0);
}


int main(void)
{
	uint8_t changed = 0;

	// Application initialization
	init();

	initGIO();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();

	sei();	

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
		
		// Write outputs	
		PORTB = (PORTB & 0xE0) | (io_output[0] & 0x1F) | (io_pullups[0] & 0x1F);  // IO0-IO4
		PORTD = (PORTD & 0x07) | ((io_output[0]>>2) & 0x38) | ((io_output[1]<<6) & 0xC0) | ((io_pullups[0]>>2) & 0x38) | ((io_pullups[1]<<6) & 0xC0); //IO5-IO9
		PORTC = (PORTC & 0xC0) | ((io_output[1]>>2) & 0x3F) | ((io_pullups[1]>>2) & 0x3F); // IO10-IO15
	
		if (status & STATUS_READ_INPUTS)
		{
			changed |= debounce_inputs()?1:0;
			status &= ~STATUS_READ_INPUTS;
		}
		
		if (decisecs >= update_decisecs)
		{
			changed |= 1;
			decisecs = 0;
		}
		
		
		if (changed && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];		
		
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 8;
			txBuffer[5] = 'S';
			txBuffer[6] = (io_output[1] & io_ddr[1]) | (io_input[1] & ~(io_ddr[1]));
			txBuffer[7] = (io_output[0] & io_ddr[0]) | (io_input[0] & ~(io_ddr[0]));
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			changed = 0;
		}	

		if (mrbusPktQueueDepth(&mrbusTxQueue))
		{
			uint8_t fail = mrbusTransmit();

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit to avoid hammering the bus
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			if (fail)
			{
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
			}
		}
	}
}



