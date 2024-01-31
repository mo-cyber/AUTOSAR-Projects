 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Ehab
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Dio Module Id */
#define PORT_MODULE_ID    (130U)

/* Dio Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* Service ID for Port Init */
#define PORT_INIT_SID                   (uint8)0x00

/* Service ID for Port Set Pin Direction */
#define PORT_SET_PIN_DIRECTION_SID      (uint8)0x01

/* Service ID for Port Refresh Port Direction */
#define PORT_REFRESH_PORT_DIRECTION_SID (uint8)0x02

/* Service ID for Port Get Version Info */
#define PORT_GET_VERSION_INFO_SID       (uint8)0x03

/* Service ID for Port Set Pin Mode */
#define PORT_SET_PIN_MODE_SID           (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* DET code to report Invalid Port Pin ID requested */
#define PORT_E_PARAM_PIN                (uint8)0x0A

/* DET code to report Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE   (uint8)0x0B

/* DET code to report API Port_Init service */
#define PORT_E_PARAM_CONFIG             (uint8)0x0C

/* DET code to report API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE       (uint8)0x0D
#define PORT_E_MODE_UNCHANGEABLE        (uint8)0x0E

/* DET code to report API service called without module initialization */
#define PORT_E_UNINIT                   (uint8)0x0F

/* DET code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER            (uint8)0x10

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Data type for the symbolic name of a port pin */
typedef uint8 Port_PinType;

/* Different port pin modes */
typedef uint8 Port_PinModeType;

/* Possible directions of a port pin */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Enum to hold internal resistor type for PIN */
typedef enum
{
    INTERNAL_RESISTOR_OFF,INTERNAL_RESISTOR_PULL_UP,INTERNAL_RESISTOR_PULL_DOWN
}Port_InternalResistor;

/* Enum to hold initial pin level */
typedef enum
{
    PORT_PIN_LEVEL_LOW,PORT_PIN_LEVEL_HIGH
}Port_PinLevelValue;

/* Enum to hold pin modes */
typedef enum
{
    PORT_PIN_MODE_ADC,PORT_PIN_MODE_CAN,PORT_PIN_MODE_DIO,PORT_PIN_MODE_DIO_GPT,PORT_PIN_MODE_DIO_WDG, \
    PORT_PIN_MODE_ICU,PORT_PIN_MODE_PWM,PORT_PIN_MODE_SPI,PORT_PIN_MODE_UART,PORT_PIN_MODE_I2C,PORT_PIN_MODE_USB
}Port_PinMode;

/* Enum to hold the port number */
typedef enum
{
    PORTA,PORTB,PORTC,PORTD,PORTE,PORTF
}Port_PortNum;

/* Enum to hold the pin number */
typedef enum
{
    PIN0,PIN1,PIN2,PIN3,PIN4,PIN5,PIN6,PIN7,PIN8
}Port_PinNum;

/* Enum to hold the direction change ability of the pin during run time */
typedef enum
{
    DIRECTION_CHANGEABLE_OFF,DIRECTION_CHANGEABLE_ON
}Port_PinDirectionChangeable;

/* Enum to hold the mode change ability of the pin during run time */
typedef enum
{
    MODE_CHANGEABLE_OFF,MODE_CHANGEABLE_ON
}Port_PinModeChangeable;

/* Structure to configure each individual PIN */
typedef struct 
{
    /* the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5 */
    Port_PortNum port_num;
    /* the number of the pin in the PORT */
    Port_PinNum pin_num;
    /* the direction of pin --> INPUT or OUTPUT */
    Port_PinDirectionType direction;
    /* the internal resistor --> Disable, Pull up or Pull down */
    Port_InternalResistor resistor;
    /* the initial output or input value of the pin --> High or Low*/
    Port_PinLevelValue initial_value;
    /* the initial mode of the pin --> DIO, ADC, CAN, ... */
    Port_PinMode initial_mode;
    /* additional mode of the pin if the pin mode changeable during runtime */
    Port_PinMode mode;
    /* the direction of pin changeable during runtime or not */
    Port_PinDirectionChangeable direction_changeable;
    /* the mode of pin changeable during runtime or not */
    Port_PinModeChangeable mode_changeable;

}Port_ConfigPin;

/* Type of the external data structure containing the initialization data for this module */
typedef struct Port_ConfigType
{
    Port_ConfigPin Pins[Port_Number_Of_Port_Pins];
} Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* Initializes the Port Driver module */
void Port_Init(const Port_ConfigType* ConfigPtr);

/* Sets the port pin direction */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction);
#endif

/* Refreshes port direction */
void Port_RefreshPortDirection(void);

/* Returns the version information of this module */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif

/* Sets the port pin mode */
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode);
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
