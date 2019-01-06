/*
 * dwire_msp432p401r.h
 *
 *  Created on: 30 May 2016
 *      Author: Stefan van der Linden
 */

#ifndef INCLUDE_DWIRE_MSP432P401R_H_
#define INCLUDE_DWIRE_MSP432P401R_H_

#define EUSCI_B0_PORT GPIO_PORT_P1
#define EUSCI_B0_SDA GPIO_PIN6
#define EUSCI_B0_SCL GPIO_PIN7
#define EUSCI_B0_PINS (EUSCI_B0_SDA | EUSCI_B0_SCL)

#define EUSCI_B1_PORT GPIO_PORT_P6
#define EUSCI_B1_SDA GPIO_PIN4
#define EUSCI_B1_SCL GPIO_PIN5
#define EUSCI_B1_PINS (EUSCI_B1_SDA | EUSCI_B1_SCL)

#define EUSCI_B2_PORT GPIO_PORT_P3
#define EUSCI_B2_SDA GPIO_PIN6
#define EUSCI_B2_SCL GPIO_PIN7
#define EUSCI_B2_PINS (EUSCI_B2_SDA | EUSCI_B2_SCL)

#define EUSCI_B3_PORT GPIO_PORT_P6
#define EUSCI_B3_SDA GPIO_PIN6
#define EUSCI_B3_SCL GPIO_PIN7
#define EUSCI_B3_PINS (EUSCI_B3_SDA | EUSCI_B3_SCL)

#endif /* INCLUDE_DWIRE_MSP432P401R_H_ */
