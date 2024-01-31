 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Ehab
 ******************************************************************************/

#ifndef PORT_CFG_H_
#define PORT_CFG_H_

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION            (1U)
#define PORT_CFG_SW_MINOR_VERSION            (0U)
#define PORT_CFG_SW_PATCH_VERSION            (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION    (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION    (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION    (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)

/*The number of specified PortPins in this PortContainer*/
#define Port_Number_Of_Port_Pins             (Port_PinType)43

/*The number of configuration modes*/
#define Port_Number_Of_Modes                 (uint8)11

/*Pre-processor switch to enable / disable the use of the function Port_SetPinDirection()*/
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

/*Pre-processor switch to enable / disable the use of the function Port_SetPinMode()*/
#define PORT_SET_PIN_MODE_API                (STD_ON)

/*Port Number of the port pin*/
#define PORT_NUM_LED1                        (Port_PortNum)PORTF
#define PORT_NUM_SW1                         (Port_PortNum)PORTF

/*Pin Number of the port pin*/
#define PIN_NUM_LED1                         (Port_PinNum)PIN1
#define PIN_NUM_SW1                          (Port_PinNum)PIN4

/*The initial direction of the pin (IN or OUT)*/
#define PORT_PIN_DIRECTION_LED1              (Port_PinDirectionType)PORT_PIN_OUT
#define PORT_PIN_DIRECTION_SW1               (Port_PinDirectionType)PORT_PIN_IN

/*The internal resistor type of the pin*/
#define PORT_INTERNAL_RESISTOR_LED1          (Port_InternalResistor)INTERNAL_RESISTOR_OFF
#define PORT_INTERNAL_RESISTOR_SW1          (Port_InternalResistor)INTERNAL_RESISTOR_PULL_UP

/*Port Pin Level value from Port pin list*/
#define PORT_PIN_LEVEL_VALUE_LED1            (Port_PinLevelValue)PORT_PIN_LEVEL_LOW
#define PORT_PIN_LEVEL_VALUE_SW1            (Port_PinLevelValue)PORT_PIN_LEVEL_HIGH

/*Port pin mode from mode list for use with Port_Init() function*/
#define PORT_PIN_INITIAL_MODE_LED1           (Port_PinMode)PORT_PIN_MODE_DIO
#define PORT_PIN_INITIAL_MODE_SW1            (Port_PinMode)PORT_PIN_MODE_DIO

/*Port pin mode from mode list*/
#define PORT_PIN_MODE_LED1                   (Port_PinMode)PORT_PIN_MODE_DIO
#define PORT_PIN_MODE_SW1                    (Port_PinMode)PORT_PIN_MODE_DIO

/*Parameter to indicate if the direction is changeable on a port pin during runtime*/
#define PORT_PIN_DIRECTION_CHANGEABLE_LED1   (Port_PinDirectionChangeable)DIRECTION_CHANGEABLE_OFF
#define PORT_PIN_DIRECTION_CHANGEABLE_SW1    (Port_PinDirectionChangeable)DIRECTION_CHANGEABLE_OFF

/*Parameter to indicate if the mode is changeable on a port pin during runtime*/
#define PORT_PIN_MODE_CHANGEABLE_LED1        (Port_PinModeChangeable)MODE_CHANGEABLE_OFF
#define PORT_PIN_MODE_CHANGEABLE_SW1         (Port_PinModeChangeable)MODE_CHANGEABLE_OFF

/* ID of used pins */
#define PORT_PIN_ID_LED1                     (Port_PinType)39
#define PORT_PIN_ID_SW1                      (Port_PinType)42

#endif /* PORT_CFG_H_ */
