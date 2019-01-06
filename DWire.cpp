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

#include <DWire.h>

/**** MACROs ****/

/**
 * Create the buffers for the specified module number (e.g. M = 0 for EUSCI_B0_BASE)
 */
uint8_t EUSCIB2_txBuffer[TX_BUFFER_SIZE];
uint16_t EUSCIB2_txBufferIndex = 0;
uint16_t EUSCIB2_txBufferSize = 0;
uint8_t EUSCIB2_rxBuffer[RX_BUFFER_SIZE];
uint16_t EUSCIB2_rxBufferIndex = 0;
uint16_t EUSCIB2_rxBufferSize = 0;
uint16_t EUSCIB2_rxBufferUnprocessed = 0;


/**** GLOBAL VARIABLES ****/

/* A reference list of DWire DWire_instances */
DWire * DWire_instance;

/**** CONSTRUCTORS ****/
DWire::DWire( ) 
{
	// set default settings
    this->module = EUSCI_B2_BASE;
    this->mode = STANDARD;
}

DWire::~DWire( ) 
{
	/* Reset the module */
    I2C_disableModule( module );
    
    /* Deregister from the moduleMap */

    DWire_instance = 0;


}

/**** PUBLIC METHODS ****/

void DWire::begin( ) 
{
    // Initialising the given module as a master
    busRole = BUS_ROLE_MASTER;
    slaveAddress = 0;
    _initMain( );

    // calculate the number of iterations of a loop to generate
    // a delay based on clock speed
    // this is needed to handle NACKs in a way that is independent
    // of CPU speed and OS (Energia or not)
	delayCycles = CS_getMCLK( ) * 12 / 7905857;
	
	/* Set the EUSCI configuration */
	config.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;	// SMCLK Clock Source
	config.i2cClk = CS_getSMCLK( );							// Get the SMCLK clock frequency
	config.byteCounterThreshold = 0;							// No byte counter threshold
	config.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;		// No Autostop
	
    if (mode == FAST) 
    {
    	config.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
    	
        _initMaster( &config );
		// accommodate a delay of at least ~30us (~68us measured)
        delayCycles = delayCycles * 4;
    } 
    else if(mode == FASTPLUS) 
    {
    	config.dataRate = EUSCI_B_I2C_SET_DATA_RATE_1MBPS;
    	
        _initMaster( &config );
        // accommodate a delay of ~12us (~16us measured)
    } 
    else 
    {
    	config.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    	
        _initMaster( &config );
        // accommodate a delay of at least ~120us (~130 us measured)
        delayCycles = delayCycles * 10;
    }
}

void DWire::setStandardMode( ) 
{
    this->mode = STANDARD;
}

void DWire::setFastMode( ) 
{
    this->mode = FAST;
}

void DWire::setFastModePlus( ) 
{
    this->mode = FASTPLUS;
}

void DWire::begin( uint8_t address ) 
{
    // Initialising the given module as a slave
    busRole = BUS_ROLE_SLAVE;
    slaveAddress = address;

    _initMain( );

    _initSlave( );
}

/**
 * Begin a transmission as a master
 */
void DWire::beginTransmission( uint_fast8_t slaveAddress ) 
{
    // Starting a transmission as a master to the slave at slaveAddress
    if (busRole != BUS_ROLE_MASTER)
        return;

    // Wait in case a previous message is still being sent
    timeout = TIMEOUTLIMIT;
    while ((*pTxBufferIndex > 0) & timeout)
        timeout--;
        
    if (!timeout) 
    {
        /* If we can't start the transmission, then reset everything */
        _resetBus( );
    }

    if (slaveAddress != this->slaveAddress)
        _setSlaveAddress( slaveAddress );
}

/**
 * Write a single byte
 */
void DWire::write( uint_fast8_t dataByte ) 
{
    // Add data to the tx buffer
    pTxBuffer[*pTxBufferIndex] = dataByte;
    (*pTxBufferIndex)++;
}

bool DWire::endTransmission( void ) 
{
    return endTransmission( true );
}

/**
 * End the transmission and transmit the tx buffer's contents over the bus
 * it returns false if succesful
 */
bool DWire::endTransmission( bool sendStop ) 
{
    // return, if there is nothing to transmit
    if (!*pTxBufferIndex) 
    {
        return true;
    }

    // Wait until any ongoing (incoming) transmissions are finished
    timeout = TIMEOUTLIMIT;
    while ( I2C_masterIsStopSent( module ) == EUSCI_B_I2C_SENDING_STOP
            && timeout)
        timeout--;

    if (!timeout) 
    {
        /* If we can't start the transmission, then reset everything */
        _resetBus( );
        return true;
    }

    this->sendStop = sendStop;
    gotNAK = false;

    // Clear the interrupt flags and enable
    I2C_clearInterruptFlag( module,
    EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT );

    I2C_enableInterrupt( module,
    EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT );

    // Set the master into transmit mode
    I2C_setMode( module, EUSCI_B_I2C_TRANSMIT_MODE );

    // Send the start condition and initial byte
    (*pTxBufferSize) = *pTxBufferIndex;

    // Send the first byte, triggering the TX interrupt
    I2C_masterSendMultiByteStartWithTimeout( module, pTxBuffer[0],
    TIMEOUTLIMIT );

    // make sure the transmitter buffer has been flushed
    timeout = TIMEOUTLIMIT;
    while (*pTxBufferIndex && timeout)
        timeout--;

    if (!timeout) 
    {
        _resetBus( );
        return true;
    }

    if (gotNAK) 
    {
        _I2CDelay( );
        I2C_masterReceiveMultiByteStop( module );
    }
    return gotNAK;
}

/**
 * Request data from a SLAVE as a MASTER
 */
uint16_t DWire::requestFrom( uint_fast8_t slaveAddress, uint_fast16_t numBytes )
{
    // No point of doing anything else if there we're not a MASTER
    if (busRole != BUS_ROLE_MASTER)
        return 0;

    // still something to send? Flush the TX buffer but do not send a STOP
    if (*pTxBufferIndex > 0) 
    {
    	// this is a repeated start: no point in trying to receive if we fail finishing the transmission
        if (endTransmission( false ))
        {
        	return 0;
        }
    } 
    else 
    {
        // Wait until any request is finished
        timeout = TIMEOUTLIMIT;
        while ( I2C_masterIsStopSent( module ) == EUSCI_B_I2C_SENDING_STOP
                && timeout)
            timeout--;
    }

    if (!timeout) 
    {
        /* If we get a timeout, then reset everything */
        _resetBus( );
        return 0;
    }

    // Re-initialise the rx buffer
    // and make sure we never request 1 byte only
    // this is an anomalous behaviour of the MSP432 related to the double
    // buffering of I2C. This is a workaround.
    if (numBytes == 1) 
    {
        *pRxBufferSize = 2;
    } else 
    {
        *pRxBufferSize = numBytes;
    }
    *pRxBufferIndex = 0;

    // Configure the correct slave
    I2C_setSlaveAddress( module, slaveAddress );
    this->slaveAddress = slaveAddress;

    I2C_clearInterruptFlag( module,
    EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT );
    I2C_enableInterrupt( module,
    EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT );

    // Set the master into receive mode
    I2C_setMode( module, EUSCI_B_I2C_RECEIVE_MODE );

    // Initialize the flag showing the status of the request
    requestDone = false;
    gotNAK = false;

    // Send the START
    I2C_masterReceiveStart( module );

    // Wait until the request is done
    timeout = TIMEOUTLIMIT;
    while (!requestDone && timeout)
        timeout--;

    if (!timeout)
    {
        /* If we get a timeout, then reset everything */
        _resetBus( );
        return 0;
    }

    if (gotNAK) 
    {
        _I2CDelay( );
        I2C_masterReceiveMultiByteStop( module );
        return 0;
    } 
    else 
    {
        if (numBytes == 1)
        {
            return --(*pRxBufferSize);
        }
        else
        {
            return *pRxBufferSize;
        }
    }
}

/**
 * Reads a single byte from the rx buffer
 */
uint8_t DWire::read( void ) 
{
    // Return a 0 if there is nothing to read or if the index is out of bounds
    if ((*pRxBufferSize == 0) || (*pRxBufferIndex >= *pRxBufferSize) )
    {
    	*pRxBufferSize = 0;
        return 0;
    }
    // return the next byte and increment the index
    // bounds checking is done at the next iteration of read
    (*pRxBufferUnprocessed)--;
    return pRxBuffer[(*pRxBufferIndex)++];
}

/**
 * Register the user's interrupt handler
 */
void DWire::onRequest( void (*islHandle)( void ) ) 
{
    user_onRequest = islHandle;
}

/**
 * Register the interrupt handler
 * The argument contains the number of bytes received
 */
void DWire::onReceive( void (*islHandle)( uint8_t ) ) 
{
    user_onReceive = islHandle;
}

/**
 * Returns true if the module is configured as a master
 */
bool DWire::isMaster( void ) 
{
	return busRole == BUS_ROLE_MASTER;
}

/**** PRIVATE METHODS ****/

/**
 * The main initialisation method to setup pins and interrupts
 */
void DWire::_initMain( void ) 
{
    requestDone = false;
    sendStop = true;

    DWire_instance = this;

    pTxBuffer = EUSCIB2_txBuffer;
    pTxBufferIndex = &EUSCIB2_txBufferIndex;
    pTxBufferSize = &EUSCIB2_txBufferSize;

    pRxBuffer = EUSCIB2_rxBuffer;
    pRxBufferIndex = &EUSCIB2_rxBufferIndex;
    pRxBufferSize = &EUSCIB2_rxBufferSize;

    pRxBufferUnprocessed = &EUSCIB2_rxBufferUnprocessed;

    modulePort = EUSCI_B2_PORT;
    modulePins = EUSCI_B2_PINS;
	moduleSCL = EUSCI_B2_SCL;

    intModule = INT_EUSCIB2;

    I2C_registerInterrupt( module, EUSCIB2_IRQHandler );
 
    
    // Initialise the receiver buffer and related variables
    *pTxBufferIndex = 0;
    *pRxBufferIndex = 0;
    *pTxBufferSize = 0;
    *pRxBufferSize = 0;
    *pRxBufferUnprocessed = 0;
}

/**
 * Called to set the eUSCI module in 'master' mode
 */
void DWire::_initMaster( const eUSCI_I2C_MasterConfig * i2cConfig ) 
{
    // Initialise the pins
    GPIO_setAsPeripheralModuleFunctionInputPin( modulePort, modulePins,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // Initializing I2C Master to SMCLK with no autostop
    I2C_initMaster( module, i2cConfig );

    // Specify slave address
    I2C_setSlaveAddress( module, slaveAddress );

    // Set Master in transmit mode
    I2C_setMode( module, EUSCI_B_I2C_TRANSMIT_MODE );

    // Enable I2C Module to start operations
    I2C_enableModule( module );

    // Clear the interrupt flag
    I2C_clearInterruptFlag( module,
            EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT
                    + EUSCI_B_I2C_RECEIVE_INTERRUPT0 );

    // Register the interrupts on the correct module
    Interrupt_enableInterrupt( intModule );
    Interrupt_setPriority(intModule, 0);
    Interrupt_enableMaster( );
}

void DWire::_initSlave( void ) 
{
    // Init the pins
    GPIO_setAsPeripheralModuleFunctionInputPin( modulePort, modulePins,
    GPIO_PRIMARY_MODULE_FUNCTION );

    // initialise driverlib
    I2C_initSlave( module, slaveAddress, EUSCI_B_I2C_OWN_ADDRESS_OFFSET0,
    EUSCI_B_I2C_OWN_ADDRESS_ENABLE );

    // Enable the module and enable interrupts
    I2C_enableModule( module );
    I2C_clearInterruptFlag( module,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_STOP_INTERRUPT
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT0 | EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT );
    I2C_enableInterrupt( module,
            EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_STOP_INTERRUPT
                    | EUSCI_B_I2C_TRANSMIT_INTERRUPT0 | EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT );

    /* Enable the clock low timeout */
    EUSCI_B_CMSIS( module )->CTLW1 = (EUSCI_B_CMSIS( module )->CTLW1
            & ~EUSCI_B_CTLW1_CLTO_MASK) | 0xC0;

    Interrupt_enableInterrupt( intModule );
    Interrupt_enableMaster( );
}

/**
 * Re-set the slave address (the target address when master or the slave's address when slave)
 */
void DWire::_setSlaveAddress( uint_fast8_t newAddress ) 
{
    slaveAddress = newAddress;
    I2C_setSlaveAddress( module, newAddress );
}

/**
 * Handle a request ISL as a slave
 */
void DWire::_handleRequestSlave( void ) 
{
    // Check whether a user interrupt has been set
    if ( !user_onRequest )
        return;

    // If no message has been set, then call the user interrupt to set
    if (!(*pTxBufferIndex)) 
    {
        user_onRequest( );

        *pTxBufferSize = *pTxBufferIndex - 1;
        *pTxBufferIndex = 0;
    }

    // If we've transmitted the entire message, then reset the tx buffer
    if (*pTxBufferIndex > *pTxBufferSize) 
    {
        *pTxBufferIndex = 0;
        *pTxBufferSize = 0;
    } 
    else 
    {
        // Transmit a byte
        I2C_slavePutData( module, pTxBuffer[*pTxBufferIndex] );
        (*pTxBufferIndex)++;
    }
}

/**
 * Internal process handling the rx buffers, and calling the user's interrupt handles
 */
void DWire::_handleReceive( uint8_t * rxBuffer )
{
    // No need to do anything if there is no handler registered
    if (!user_onReceive)
        return;

    // reset the RX buffer index to prepare the readout
    *pRxBufferIndex = 0;

	// call the user-defined receive handler
    user_onReceive( *pRxBufferSize );
}

void DWire::_finishRequest( bool success ) 
{
	// reset the RX buffer index to prepare the readout
	*pRxBufferIndex = 0;
	// mark the transaction as failed
    gotNAK = !success;
    // unlock the main thread
    requestDone = true;
}

void DWire::_I2CDelay( void ) 
{
    // delay for 1.5 byte-times and send the stop
    // this is needed because the MSP432 ignores any
    // stop if the byte is being received / transmitted

    for (int i = 0; i < delayCycles; i++) 
	{
		__no_operation();
	}
}

void DWire::_resetBus( void ) 
{
    /* Reset buffers */
    *pTxBufferIndex = 0;
    *pTxBufferSize = 0;
    *pRxBufferIndex = 0;
    *pRxBufferSize = 0;

    *pRxBufferUnprocessed = 0;

    /* Reset the module */
    I2C_disableModule( module );

    /* Perform bus clear according to I2C-bus Specification and User Manual 
     * (UM10204) section 3.1.16 
     */
    if (this->isMaster( )) 
    {
        GPIO_setOutputLowOnPin( modulePort, moduleSCL );
        for (uint_fast8_t i = 0; i < 9; i++) 
        {
            GPIO_setAsOutputPin( modulePort, moduleSCL );
            this->_I2CDelay( );
            GPIO_setAsInputPin( modulePort, moduleSCL );
            this->_I2CDelay( );
        }
        GPIO_setAsPeripheralModuleFunctionInputPin( modulePort,
                moduleSCL, GPIO_PRIMARY_MODULE_FUNCTION );
    }

    /* Re-enable the module */
    I2C_enableModule( module );
}

uint16_t DWire::available(void) {
    return *pRxBufferUnprocessed;
}

extern "C" {

void EUSCIB2_IRQHandler(void)
{
        uint_fast16_t status;
        status = I2C_getEnabledInterruptStatus(EUSCI_B2_BASE);
        I2C_clearInterruptFlag(EUSCI_B2_BASE, status);

        /* Get a reference to the correct instance */
        /* if it is null, ignore the interrupt */
        DWire * instance = DWire_instance;
        if ( !instance )
        {
            /* Disable all interrupts if the handler was not registered */
            I2C_disableInterrupt(EUSCI_B2_BASE, 0xFFFF);
            return;
        }

        /* Handle a NAK */
        if ( status & EUSCI_B_I2C_NAK_INTERRUPT )
        {
            /* Disable all other interrupts */
            I2C_disableInterrupt(EUSCI_B2_BASE,
                    EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_TRANSMIT_INTERRUPT0
                            | EUSCI_B_I2C_NAK_INTERRUPT);

            EUSCIB2_txBufferIndex = 0;
            EUSCIB2_rxBufferSize = 0;
            /* Mark the request as done and failed */
            instance->_finishRequest(false);
        }

        /* Check for clock low interrupt: if it is low for too long, then reset the I2C peripheral */
        if( status & EUSCI_B_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT)
        {
            ResetCtl_initiateHardReset();
        }

        /* RXIFG */
        /* Triggered when data has been received */
        if ( status & EUSCI_B_I2C_RECEIVE_INTERRUPT0 )
        {
            /* If we're a master, then we're handling the slave response after/during a request */
            if ( instance->isMaster( ) )
            {
            /* do range checking around this block to avoid possible errors */
                EUSCIB2_rxBuffer[EUSCIB2_rxBufferIndex] =
                I2C_masterReceiveMultiByteNext(EUSCI_B2_BASE);
                EUSCIB2_rxBufferIndex++;
                EUSCIB2_rxBufferUnprocessed++;


                /* if we only need to read 1 more byte, start sending a stop */
                if ( EUSCIB2_rxBufferIndex == EUSCIB2_rxBufferSize - 1 )
                {
                    I2C_masterReceiveMultiByteStop(EUSCI_B2_BASE);
                }

                if ( EUSCIB2_rxBufferIndex >= EUSCIB2_rxBufferSize )
                {
                    /* Disable the RX interrupt */
                    I2C_disableInterrupt(EUSCI_B2_BASE,
                    EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT);
                    /* Mark the request as done and succesful */
                    instance->_finishRequest(true);
                }
                /* If we're a slave, then we're receiving data from the master */
            } else
            {
                EUSCIB2_rxBuffer[EUSCIB2_rxBufferIndex] = I2C_slaveGetData(
                        EUSCI_B2_BASE);
                EUSCIB2_rxBufferIndex++;
            }
        }

        /* As master: triggered when a byte has been transmitted */
        if ( status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0 )
        {
            /* If the module is setup as a master, then we're transmitting data */
            if ( instance->isMaster( ) )
            {
                if ( EUSCIB2_txBufferIndex == 1 )
                {
                    /* Send a STOP condition if required */
                    if ( instance->_isSendStop( ) )
                    {
                        I2C_masterSendMultiByteStop(EUSCI_B2_BASE);
                    }
                    /* Disable the TX interrupt */
                    I2C_disableInterrupt(EUSCI_B2_BASE,
                    EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);
                    EUSCIB2_txBufferIndex--;

                } else if ( EUSCIB2_txBufferIndex > 1 )
                {
                    /* If we still have data left in the buffer, then transmit that */
                    I2C_masterSendMultiByteNext(EUSCI_B2_BASE,
                            EUSCIB2_txBuffer[(EUSCIB2_txBufferSize)
                                    - (EUSCIB2_txBufferIndex) + 1]);
                    EUSCIB2_txBufferIndex--;
                }
                /* If we're a slave, then we're handling a request from the master */
            } else
            {
                instance->_handleRequestSlave( );
            }
        }

        /* STPIFG: Called when a STOP is received */
        if ( status & EUSCI_B_I2C_STOP_INTERRUPT )
        {
            if ( EUSCIB2_txBufferIndex != 0 && !instance->isMaster( ) )
            {
                EUSCIB2_rxBufferIndex = 0;
                EUSCIB2_rxBufferSize = 0;
            } else if ( EUSCIB2_rxBufferIndex != 0 )
            {
                instance->_handleReceive(EUSCIB2_rxBuffer);
            }
        }
    }

}
