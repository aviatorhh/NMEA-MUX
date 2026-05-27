#include<Arduino.h>
#include "canbus.h"

CAN::CAN() {}

CAN::CAN(uint8_t cs_pin) {
    this->cs_pin = cs_pin;
}

void CAN::init(uint8_t cs_pin, bool do_reset) {

	// SPIMasterInit();

	if (do_reset) {
		reset(cs_pin);  // coming out in config mode
	}

#if defined (__SCP) || (__AVR_ATmega32U4__ )	// 8 MHz, 250kb/s
	// CAN::writeRegister(REG_CNF1, 0x01, cs_pin);
	// CAN::writeRegister(REG_CNF2, 0xb1, cs_pin);
	// CAN::writeRegister(REG_CNF3, 0x05, cs_pin);
    	writeRegister(REG_CNF1, 0x41, cs_pin);
	writeRegister(REG_CNF2, 0xfb, cs_pin);
	writeRegister(REG_CNF3, 0x46, cs_pin);
#else

#if F_CPU == 20000000
	// 250baud @ 20MHz
	writeRegister(REG_CNF1, 0x41, cs_pin);
	writeRegister(REG_CNF2, 0xfb, cs_pin);
	writeRegister(REG_CNF3, 0x46, cs_pin);
#elif F_CPU == 16000000
	// 250baud @ 16MHz
	writeRegister(REG_CNF1, 0x41, cs_pin);
	writeRegister(REG_CNF2, 0xf1, cs_pin);
	writeRegister(REG_CNF3, 0x45, cs_pin);
#else
#error CAN speed setup incomplete, check frequency
#endif
#endif
	CAN::writeRegister(REG_CANINTE, FLAG_RXnIE(1) | FLAG_RXnIE(0), cs_pin);// | 
		//FLAG_TXnIF(0) | FLAG_TXnIF(1) | FLAG_TXnIF(2), cs_pin);    // Receive Buffer Full and TX Buffer empty Interrupt Enable bits
	CAN::writeRegister(REG_BFPCTRL, 0x00, cs_pin);     // Do not use any pin functions
	CAN::writeRegister(REG_TXRTSCTRL, 0x00, cs_pin);   // Do not use any pin functions
	CAN::writeRegister(REG_RXBnCTRL(0), FLAG_RXM1 | FLAG_RXM0 | FLAG_BUKT, cs_pin);    // Turns mask/filters off; receives any message, NO rollover
	CAN::writeRegister(REG_RXBnCTRL(1), FLAG_RXM1 | FLAG_RXM0, cs_pin);    // Turns mask/filters off; receives any message
	// normal mode
	CAN::writeRegister(REG_CANCTRL, 0x00, cs_pin);

    // TX buffer default priorities
    CAN::writeRegister(REG_TXBnCTRL(0), FLAG_TXP(0) | FLAG_TXP(1), cs_pin);
    CAN::writeRegister(REG_TXBnCTRL(1), FLAG_TXP(1), cs_pin);
    CAN::writeRegister(REG_TXBnCTRL(2), FLAG_TXP(0), cs_pin);

	while ((CAN::readRegister(REG_CANSTAT, cs_pin) & 0b11100000) != 0x00);

}

void CAN::SPIMasterInit() {
	// enable SPI as master
	SPCR = _BV(SPE) | _BV(MSTR);// | _BV(CPOL) | _BV(CPHA);

#if defined(SPI_FOSC_2)
    SPCR |= _BV(SPI2X); // fosc/2
#elif defined(SPI_FOSC_4)
#elif defined(SPI_FOSC_8)
    SPSR |= _BV(SPI2X); 
    SPCR |= _BV(SPR0); // fosc/8
#elif defined(SPI_FOSC_16)
    SPCR |= _BV(SPR0); // fosc/16
#elif defined(SPI_FOSC_32)
    SPSR |= _BV(SPI2X);
    SPCR |= _BV(SPR1); // fosc/32
#elif defined(SPI_FOSC_128)
    SPCR |= _BV(SPR1) | _BV(SPR0); // fosc/128
#else
    SPCR |= _BV(SPR1); // fosc/64
#endif

	// set pin modes
	DDR_SPI |= _BV(DD_MOSI) | _BV(DD_SCK);
}

/**
 * Mask 1 SID
 * Mask 2 EID
 * Filter 1 and 2 SID
 * Filter 3 to 6 EID
 */
void CAN::setMasksAndFilters(uint32_t *mask, uint32_t *filter, uint8_t f_len, uint8_t cs_pin) {

     // config mode
	CAN::writeRegister(REG_CANCTRL, 0x80, cs_pin);
	while ((CAN::readRegister(REG_CANSTAT, cs_pin) & 0b11100000) != 0x80);

    // Enable filtering for buffer 0
	CAN::modifyRegister(REG_RXBnCTRL(0), 0b01100000, 0, cs_pin);
	
    // Enable filtering for buffer 1
	CAN::modifyRegister(REG_RXBnCTRL(1), 0b01100000, 0, cs_pin);

    // masks
	// Mask 1 for RXB0
	// Mask 2 for RXB1
    uint8_t n = 0;
    CAN::writeRegister(REG_RXMnSIDH(n), mask[n] >> 3, cs_pin);
    CAN::writeRegister(REG_RXMnSIDL(n), (mask[n] << 5), cs_pin);
    n++;    
    CAN::writeRegister(REG_RXMnSIDH(n), (mask[n] >> 21) & 0xff, cs_pin);
    CAN::writeRegister(REG_RXMnSIDL(n), ((((mask[n] >> 18) & 0x03) << 5) | FLAG_EXIDE | ((mask[n] >> 16) & 0x03)) & 0xff, cs_pin);

    CAN::writeRegister(REG_RXMnEID8(n), (mask[n] >> 8) & 0xff, cs_pin);
    CAN::writeRegister(REG_RXMnEID0(n), mask[n] & 0xff, cs_pin);
    // filter
	// Therefore filter 1 and 2 RXB0
	// Therefore filter 3, 4, 5 and 6 for RXB1
    for (n = 0; n < f_len; n++) {
        if (n < 2) {
	        CAN::writeRegister(REG_RXFnSIDH(n), (filter[n] >> 3), cs_pin);
	        CAN::writeRegister(REG_RXFnSIDL(n), filter[n] << 5, cs_pin);
        } else {
            CAN::writeRegister(REG_RXFnSIDH(n), (filter[n] >> 21) & 0xff, cs_pin);
            CAN::writeRegister(REG_RXFnSIDL(n), ((((filter[n] >> 18) & 0x03) << 5) | FLAG_EXIDE | ((filter[n] >> 16) & 0x03)) & 0xff, cs_pin);
            CAN::writeRegister(REG_RXFnEID8(n), (filter[n] >> 8) & 0xff, cs_pin);
            CAN::writeRegister(REG_RXFnEID0(n), filter[n] & 0xff, cs_pin);
        }
	}

    // normal mode
	CAN::writeRegister(REG_CANCTRL, 0x00, cs_pin);
	while ((CAN::readRegister(REG_CANSTAT, cs_pin) & 0b11100000) != 0x00);
}

void CAN::sleep(bool bus_awake, uint8_t cs_pin) {

	// sleep mode
	CAN::modifyRegister(REG_CANCTRL, 0b11100000, 0b00100000, this->cs_pin);
	while ((CAN::readRegister(REG_CANSTAT, this->cs_pin) & 0b11100000) != 0b00100000);
	if (bus_awake) {
		this->modifyRegister(REG_CANINTE, 0b01000000, 0b01000000, cs_pin);	// enable wake int
	} else {
		this->modifyRegister(REG_CANINTE, 0b01000000, 0b00000000, cs_pin);	// disable wake int
	}
	this->modifyRegister(REG_CANINTF, 0b01000000, 0b00000000, cs_pin); // clear flag
}
void CAN::wakeup(uint8_t cs_pin) {
	// active wakeup command
	this->modifyRegister(REG_CANINTF, 0b01000000, 0b01000000, cs_pin);
	// normal mode
	CAN::writeRegister(REG_CANCTRL, 0x00, this->cs_pin);
	while ((CAN::readRegister(REG_CANSTAT, this->cs_pin) & 0b11100000) != 0x00);	

	this->modifyRegister(REG_CANINTE, 0b01000000, 0b00000000, cs_pin);	// disable wake int
	this->modifyRegister(REG_CANINTF, 0b01000000, 0b00000000, cs_pin); // clear flag
}

char CAN::SPIMasterTransmit(char cData) {
	SPDR = cData;
	
	while(!(SPSR & _BV(SPIF))) {};
	
	return SPDR;
}
void CAN::writeRegister(uint8_t address, uint8_t value, uint8_t cs_pin) {
	CS_LOW(cs_pin);
	CAN::SPIMasterTransmit(WRITE);
	CAN::SPIMasterTransmit(address);
	CAN::SPIMasterTransmit(value);
	CS_HIGH(cs_pin);
}
uint8_t CAN::readRegister(uint8_t address, uint8_t cs_pin) {
	uint8_t value;

	CS_LOW(cs_pin);
	CAN::SPIMasterTransmit(READ);
	CAN::SPIMasterTransmit(address);
	value = CAN::SPIMasterTransmit(0xff);
	CS_HIGH(cs_pin);

	return value;
}

/**
 * After doing this reset, the controller comes out
 * in configuration mode.
 */
void CAN::reset(uint8_t cs_pin) {
	CS_LOW(cs_pin);
	CAN::SPIMasterTransmit(RESET);
	CS_HIGH(cs_pin);
	//_delay_ms(50); // Just in case
}

void CAN::sendMessageWait(msg_t& msg, const uint8_t tx_buffer, const uint8_t cs_pin) {
    while (CAN::readRegister(REG_TXBnCTRL(tx_buffer), cs_pin) & _BV(TXREQ)) {};
    sendMessage(msg, cs_pin);
}
/**
 * Send out a message through a selected buffer
 */
void CAN::sendMessage(msg_t& msg, const uint8_t buffer_nr, const uint8_t cs_pin) {
	const uint8_t data_len = msg.DLC;
	if (msg.REQ) {
		msg.DLC |= 0x40;	// set RTR flag
	}

	// We need to disable the interrupt to be able to transfer data 
	// to the MCP2515 without disturbances
#if defined (__AVR_ATmega32U4__ )
	EIMSK &= ~_BV(INT6);
#elif defined (DUAL_CAN)		
	EIMSK &= ~(_BV(INT0) | _BV(INT1));
#elif defined (__AVR_ATmega328P__ )
	EIMSK &= ~_BV(INT0);
#endif
    
    if (msg.priority) {
	    CAN::modifyRegister(REG_TXBnCTRL(buffer_nr), 0b00000011, msg.priority, cs_pin); // clear some error flags
    }

	CS_LOW(cs_pin);

	// Fill up the TX buffer
	CAN::SPIMasterTransmit(LOAD_TX_BUFFER(buffer_nr));

	// Address (SID)
	if (msg.IDE) {
		CAN::SPIMasterTransmit(msg.SID >> 21);
		CAN::SPIMasterTransmit((((msg.SID >> 18) & 0x07) << 5) | FLAG_EXIDE | ((msg.SID >> 16) & 0x03));
		CAN::SPIMasterTransmit((msg.SID >> 8) & 0xff);
		CAN::SPIMasterTransmit(msg.SID & 0xff);
	} else {
		CAN::SPIMasterTransmit(msg.SID >> 3);
		CAN::SPIMasterTransmit(msg.SID << 5);
		CAN::SPIMasterTransmit(0xff);
		CAN::SPIMasterTransmit(0xff);
	}

	// DLC / data length
	CAN::SPIMasterTransmit(msg.DLC);

	// Transfer data to buffer
	for (uint8_t i = 0; i < data_len; i++) {
    	CAN::SPIMasterTransmit(msg.data[i]);
    }
	CS_HIGH(cs_pin);
	// Shoot for transmission
	CS_LOW(cs_pin);
    CAN::SPIMasterTransmit(RTS + _BV(buffer_nr)); // RTS TXn
    CS_HIGH(cs_pin);

	// Enable interrupt again
#if defined (__AVR_ATmega32U4__ )
	EIMSK |= _BV(INT6);
#elif defined (DUAL_CAN)	
	EIMSK |= _BV(INT0) | _BV(INT1);
#elif defined (__AVR_ATmega328P__ )
	EIMSK |= _BV(INT0);
#endif
}

/**
 * Autonomous function that waits for a free buffer
 */
uint8_t CAN::sendMessage(msg_t& msg, const uint8_t cs_pin) {

	uint8_t buffer_nr = 0;
#if defined (__AVR_ATmega32U4__)
	EIMSK &= ~_BV(INT6);
#elif defined (DUAL_CAN)
	EIMSK &= ~(_BV(INT0) | _BV(INT1));
#elif defined (__AVR_ATmega328P__)
	EIMSK &= ~_BV(INT0);
#endif
	// Which of the three buffers is free? Choose it then
	while (CAN::readRegister(REG_TXBnCTRL(buffer_nr), cs_pin) & _BV(TXREQ)) {//};
		buffer_nr++;
		if (CAN::readRegister(REG_TXBnCTRL(buffer_nr), cs_pin) & _BV(TXREQ)) {
			buffer_nr++;
			if (CAN::readRegister(REG_TXBnCTRL(buffer_nr), cs_pin) & _BV(TXREQ)) {
				buffer_nr = 0;
				continue;
			} else {
				break;
			}
		} else {
			break;
		}
	} 
	CAN::sendMessage(msg, buffer_nr, cs_pin);
	return buffer_nr;
}

void CAN::stop(uint8_t cs_pin) {
	reset(cs_pin);

	// disable SPI as master
	SPCR &= ~(_BV(SPE) | _BV(MSTR));
	// reset pin modes
	DDR_SPI &= ~(_BV(DD_MOSI) | _BV(DD_SCK));
}

void CAN::read(uint8_t n, msg_t& msg, uint8_t cs_pin) {

	CS_LOW(cs_pin);
	// Beginning the read at the first buffer address
	CAN::SPIMasterTransmit(READ_RX_BUFFER(n));
	msg.SID = (CAN::SPIMasterTransmit(0xff)) << 3 & 0x07f8;
    const uint32_t val = CAN::SPIMasterTransmit(0xff);
    msg.IDE = val & 0b00001000;
	msg.SID += ((val >> 5) & 0b00000111);
    if (msg.IDE) {
        msg.SID = msg.SID << 18;
        msg.SID += (val & 0b11) << 16;
        msg.SID += CAN::SPIMasterTransmit(0xff) << 8;
        msg.SID += CAN::SPIMasterTransmit(0xff);
    } else {
        msg.REQ = val & 0b00010000;
        CAN::SPIMasterTransmit(0xff);	// just filling for sequencing
        CAN::SPIMasterTransmit(0xff);	// just filling for sequencing
    }
	// fetch DLC and RTR
	msg.DLC = CAN::SPIMasterTransmit(0xff) & 0b01001111;
    if (msg.IDE) {
        msg.REQ = msg.DLC >> 6;	// RTR for extended frame?
    }
    msg.DLC &= 0b1111;	// Filter out RTR flag

	// Transfer data from buffer
	for (uint8_t i = 0; i < msg.DLC; i++) {
    	msg.data[i] = CAN::SPIMasterTransmit(0xff);
	}
	// Clearing of the interrupt flag is done automatically here now
	CS_HIGH(cs_pin);
	// modifyRegister(REG_CANINTF, _BV(n), 0, cs_pin);
}

/**
 * Modify single bitts in a register. Set bits in the mask determines the bit
 * that can be changed by the value.
 */
void CAN::modifyRegister(uint8_t address, uint8_t mask, uint8_t value, uint8_t cs_pin) {
	CS_LOW(cs_pin);
	CAN::SPIMasterTransmit(BIT_MODIFY);
	CAN::SPIMasterTransmit(address);
	CAN::SPIMasterTransmit(mask);
	CAN::SPIMasterTransmit(value);
	CS_HIGH(cs_pin);
}

/**
 * Fetching the status register
 */
uint8_t CAN::readStatus(uint8_t cs_pin) {
	uint8_t value;
	CS_LOW(cs_pin);
	CAN::SPIMasterTransmit(READ_STATUS);
	value = CAN::SPIMasterTransmit(0xff);
	CS_HIGH(cs_pin);
	return value;
}
uint8_t CAN::readRxStatus(uint8_t cs_pin) {
	uint8_t value;
	CS_LOW(cs_pin);
	CAN::SPIMasterTransmit(READ_RX_STATUS);
	value = CAN::SPIMasterTransmit(0xff);
	CS_HIGH(cs_pin);
	return value;
}