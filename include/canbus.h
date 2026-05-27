/*
 * canbus.h
 *
 *  Created on: Sun Jan 22 22:44:32 2023
 *      Author: nicholas
 */
#ifndef CANBUS_H_
#define CANBUS_H_

#include<stdint.h>

/* #ifndef MCP_CAN_CS_PIN
#define MCP_CAN_CS_PIN PIN_PB1     	// Pin to use for the CAN controller chip select
#endif */
#ifndef MCP_CAN_INT_PIN
#define MCP_CAN_INT_PIN PIN_PD2     	// Pin to use for the CAN controller interrupt
#endif

#define CAN_ID_MASK 0x4ff
#define CAN_ID_ENG 0x00a

// These variables are used by the CAN interrupt
extern volatile bool has_can;   // has a CAN message arrived?
extern uint8_t buf[8];   // CAN message payload data
// extern volatile bool eid;
extern bool has_id;
extern bool rtr;
extern volatile uint8_t buf_ee[8];   // CAN message payload data for eeprom
extern uint8_t len; // global variable for package length
// extern volatile uint32_t canId; // CAN ID of message
// extern volatile uint8_t dlc;   // data length of message
// extern volatile uint8_t canintf;

#define REG_CANCTRL                0x0f
#define REG_CANSTAT                0x0e
#define REG_CNF3                   0x28
#define REG_CNF2                   0x29
#define REG_CNF1                   0x2a
#define BUKT                        2


#define CAN_ID_MSG                 0x64

#define REG_TXBnCTRL(n)            (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n)            (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n)            (0x32 + (n * 0x10))
#define REG_TXBnEID8(n)            (0x33 + (n * 0x10))
#define REG_TXBnEID0(n)            (0x34 + (n * 0x10))
#define REG_TXBnDLC(n)             (0x35 + (n * 0x10))
#define REG_TXBnD0(n)              (0x36 + (n * 0x10))

#define FLAG_RXnIF(n)              (0x01 << n)

#define REG_RXBnSIDH(n)            (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n)            (0x62 + (n * 0x10))
#define REG_RXBnDLC(n)             (0x65 + (n * 0x10))
#define REG_RXBnD0(n)              (0x66 + (n * 0x10))

#define REG_RXFnSIDH(n)             n < 3 ? n * 4 : (n-3) * 4 + 0x10 
#define REG_RXFnSIDL(n)             n < 3 ? n * 4 + 1 : (n-3) * 4 + 0x11
#define REG_RXFnEID8(n)             n < 3 ? 0x02 + n * 4 : 0x12 + (n - 3) * 4
#define REG_RXFnEID0(n)             n < 3 ? 0x03 + n * 4 : 0x13 + (n - 3) * 4

#define REG_RXMnSIDH(n)            0x20 + n * 4
#define REG_RXMnSIDL(n)            0x21 + n * 4 
#define REG_RXMnEID8(n)            0x22 + n * 4
#define REG_RXMnEID0(n)            0x23 + n * 4

#define REG_RXBnEID8(n)            (0x63 + (n * 0x10))
#define REG_RXBnEID0(n)            (0x64 + (n * 0x10))

#define REG_CANINTE                0x2b
#define REG_CANINTF                0x2c

#define FLAG_RXnIE(n)              (0x01 << n)
#define FLAG_RXnIF(n)              (0x01 << n)
#define FLAG_TXnIF(n)              (0x04 << n)

#define FLAG_TXP(n)                (0x01 << n)

#define REG_BFPCTRL                0x0c
#define REG_TXRTSCTRL              0x0d
#define REG_EFLG                   0x2d

#define FLAG_BUKT                  0x04
#define FLAG_RXM0                  0x20
#define FLAG_RXM1                  0x40

#define REG_RXBnCTRL(n)             0x60 + n * 0x10

#define FLAG_IDE                   0x08
#define FLAG_SRR                   0x10
#define FLAG_RTR                   0x40
#define FLAG_EXIDE                 0x08
#define RTS                        0x80
#define TXREQ                      0x03
#define READ_STATUS                0x0a
#define RESET                      0xc0
#define BIT_MODIFY                 0x05
#define READ                       0x03
#define WRITE                      0x02
#define READ_RX_STATUS             0xb0


#define LOAD_TX_BUFFER(n)          (0x40 + n * 2)
#define READ_RX_BUFFER(n)          (0x90 + n * 4)

#if defined( __AVR_ATmega328P__ )
#define DDR_SPI                     DDRB
#define DD_MOSI                     PB3
#define DD_SCK                      PB5
/* #if defined(__SCP)
#define DD_CS 								PB2
#else
#define DD_CS 								PB1
#endif
 */
#elif defined( __AVR_ATmega1284P__ )
#define DDR_SPI                     DDRB
#define DD_MOSI                     PB5
#define DD_SCK                      PB7
#define DD_SS                       PB4
// #define DD_CS                       PB3
#define MCP_RESET_PIN               PA2
#elif defined (__AVR_ATmega32U4__)  
#define DDR_SPI                     DDRB
#define DD_MOSI                     PB2
#define DD_SCK                      PB1
// #define DD_CS                       PB6

#endif

#ifndef DD_PORT
#define DD_PORT                     PORTB
#endif

//#ifndef DD_CS
//#define DD_CS   PB1
//#endif

#define CS_LOW(pin)                      DD_PORT &= ~_BV(pin);
#define CS_HIGH(pin)                     DD_PORT |=  _BV(pin);

typedef struct msg {
    uint32_t SID;
    uint8_t DLC;
    uint8_t REQ;
    uint8_t IDE;
    uint8_t data[8];
    uint8_t priority;
} msg_t;

class CAN {
    public:
        CAN();
        CAN(uint8_t cs_pin);
        static void init(uint8_t cs_pin, bool do_reset);
        static void reset(uint8_t cs_pin);
        static void SPIMasterInit();
        static void setMasksAndFilters(uint32_t *mask, uint32_t *filter, uint8_t f_len, uint8_t cs_pin);
        static uint8_t sendMessage(msg_t& msg, const uint8_t cs_pin);
        static void sendMessage(msg_t& msg, const uint8_t tx_buffer, const uint8_t cs_pin);
        static void sendMessageWait(msg_t& msg, const uint8_t tx_buffer, const uint8_t cs_pin);
        static void stop(uint8_t cs_pin);
        static void read(uint8_t n, msg_t& msg, uint8_t cs_pin);
        static uint8_t readRxStatus(uint8_t cs_pin);
        static uint8_t readStatus(uint8_t cs_pin);
        static char SPIMasterTransmit(char cData);
        static uint8_t readRegister(uint8_t address, uint8_t cs_pin);
        static void writeRegister(uint8_t address, uint8_t value, uint8_t cs_pin);
        static void modifyRegister(uint8_t address, uint8_t mask, uint8_t value, uint8_t cs_pin);

        void sleep(bool bus_awake, uint8_t cs_pin);
        void wakeup(uint8_t cs_pin);

    private:
        uint8_t cs_pin;
       
        
        
        
};





#endif /* CANBUS_H_ */