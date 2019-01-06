/*
 * Copyright (c) 2016 by Stefan van der Linden <spvdlinden@gmail.com>
 *
 * DWire: a library to provide full hardware-driven I2C functionality
 * to the TI MSP432 family of microcontrollers. It is possible to use
 * this library in Energia (the Arduino port for MSP microcontrollers)
 * or in other toolchains.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
 *
 */

#include <stdint.h>

#ifndef DWIRE_DWIRE_H_
#define DWIRE_DWIRE_H_

// Similar for the roles
#define BUS_ROLE_MASTER 0
#define BUS_ROLE_SLAVE 1

// define I2C speed
#define STANDARD 0
#define FAST     1
#define FASTPLUS 2

// Default buffer size in bytes
#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 512


#define TIMEOUTLIMIT 0xFFFF

#include <driverlib.h>

/* Device specific includes */
#include <dwire_pins.h>

extern "C" 
{
	extern void EUSCIB2_IRQHandler( void );
}

/* Main class definition */
class DWire 
{
private:

    uint32_t delayCycles;
	/* TX buffer pointers */
	uint8_t * pTxBuffer;
    volatile uint16_t * pTxBufferIndex;
    volatile uint16_t * pTxBufferSize;

	/* RX buffer pointers */
    uint8_t * pRxBuffer;
    uint16_t * pRxBufferIndex;
    uint16_t * pRxBufferSize;
    uint16_t * pRxBufferUnprocessed;

    volatile bool requestDone;
    volatile bool sendStop;
    volatile bool gotNAK;

	/* MSP specific modules */
    uint_fast32_t module;
    uint32_t intModule;
    uint_fast8_t modulePort;
    uint_fast16_t modulePins;
    uint_fast16_t moduleSCL;
    
    /* Internal states */
    eUSCI_I2C_MasterConfig config;
    uint8_t mode;
    uint8_t slaveAddress;
    uint8_t busRole;
    uint32_t timeout;
    
    void (*user_onRequest)( void );
    void (*user_onReceive)( uint8_t );

    void _initMain( void );
    void _initMaster( const eUSCI_I2C_MasterConfig * );
    void _initSlave( void );
    void _setSlaveAddress( uint_fast8_t );
    void _I2CDelay( void );
    void _resetBus( void );

public:
    /* Constructors */
    DWire( );
    ~DWire( void );

    /* MASTER specific */
    void begin( );
    void setStandardMode( );
    void setFastMode( );
    void setFastModePlus( );

    void beginTransmission( uint_fast8_t );
    void write( uint_fast8_t );
    bool endTransmission( void );
    bool endTransmission( bool );

    uint16_t requestFrom( uint_fast8_t, uint_fast16_t );

    /* SLAVE specific */
    void begin( uint8_t );

    uint8_t read( void );

    void onRequest( void (*)( void ) );
    void onReceive( void (*)( uint8_t ) );

    /* Miscellaneous */
    bool isMaster( void );

    /* Internal */
    void _handleReceive( uint8_t * );
    void _handleRequestSlave( void );
    void _finishRequest( bool );
    bool _isSendStop( ) { return sendStop; }

    //@mohammed: added functions for compatibility with BNO080 library

    uint16_t available(void);
};

#endif /* DWIRE_DWIRE_H_ */
