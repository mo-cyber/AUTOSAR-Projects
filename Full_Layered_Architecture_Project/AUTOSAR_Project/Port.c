 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Ehab
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigPin * Port_Pins = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module
************************************************************************************/

void Port_Init(const Port_ConfigType* ConfigPtr)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check if the input configuration pointer is not a NULL_PTR */
    if (NULL_PTR == ConfigPtr)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
                        PORT_E_PARAM_CONFIG);
    }
    else
#endif
    {
        /* Declare the iterator of the for loop - the index of the configuration array of structures */
        Port_PinType pin_id;

        /* point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures */
        Port_Pins = ConfigPtr->Pins;                   /* address of the first Pins structure --> Pins[0] */
        for(pin_id = 0; pin_id < Port_Number_Of_Port_Pins; pin_id++)
        {
            volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

            switch(Port_Pins[pin_id].port_num)
            {
            case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
            break;
            case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
            break;
            case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
            break;
            case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
            break;
            case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
            break;
            case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
            break;
            }
            if( ((Port_Pins[pin_id].port_num == PORTD) && (Port_Pins[pin_id].pin_num == PIN7)) || ((Port_Pins[pin_id].port_num == PORTF) && (Port_Pins[pin_id].pin_num == PIN0)) ) /* PD7 or PF0 */
            {
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_Pins[pin_id].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
            else if( (Port_Pins[pin_id].port_num == PORTC) && (Port_Pins[pin_id].pin_num <= PIN3) ) /* PC0 to PC3 */
            {
                /* Do Nothing ...  this is the JTAG pins */
            }
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }
            if(Port_Pins[pin_id].direction == PORT_PIN_OUT)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[pin_id].pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                if(Port_Pins[pin_id].initial_value == PORT_PIN_LEVEL_HIGH)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_Pins[pin_id].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_Pins[pin_id].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                }
            }
            else if(Port_Pins[pin_id].direction == PORT_PIN_IN)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[pin_id].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

                if(Port_Pins[pin_id].resistor == INTERNAL_RESISTOR_PULL_UP)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_Pins[pin_id].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                }
                else if(Port_Pins[pin_id].resistor == INTERNAL_RESISTOR_PULL_DOWN)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_Pins[pin_id].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_Pins[pin_id].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_Pins[pin_id].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                }
            }
            else
            {
                /* Do Nothing */
            }
            if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_ADC)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[pin_id].pin_num);      /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[pin_id].pin_num);     /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[pin_id].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[pin_id].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
            }
            if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[pin_id].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[pin_id].pin_num * 4));     /* Clear the PMCx bits for this pin */
            }
            else
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[pin_id].pin_num);               /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[pin_id].pin_num * 4));     /* Clear the PMCx bits for this pin */
                /* Checking the possible modes on every allowed pin of every port to set the PMCx bits
                 * to select the configured peripheral function for each pin */
                if(Port_Pins[pin_id].port_num == PORTA)
                {
                    switch(Port_Pins[pin_id].pin_num)
                    {
                    case PIN0:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN1:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN2:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN3:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN4:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN5:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN6:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN7:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    }
                }
                else if(Port_Pins[pin_id].port_num == PORTB)
                {
                    switch(Port_Pins[pin_id].pin_num)
                    {
                    case PIN0:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN1:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN2:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN3:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN4:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN5:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN6:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN7:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    }
                }
                else if(Port_Pins[pin_id].port_num == PORTC)
                {
                    switch(Port_Pins[pin_id].pin_num)
                    {
                    case PIN4:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN5:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN6:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_USB)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN7:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_USB)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    }
                }
                else if(Port_Pins[pin_id].port_num == PORTD)
                {
                    switch(Port_Pins[pin_id].pin_num)
                    {
                    case PIN0:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN1:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN2:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_USB)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN3:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_USB)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN4:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN5:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN6:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN7:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_WDG)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    }
                }
                else if(Port_Pins[pin_id].port_num == PORTE)
                {
                    switch(Port_Pins[pin_id].pin_num)
                    {
                    case PIN0:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN1:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN4:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN5:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_I2C)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    }
                }
                else if(Port_Pins[pin_id].port_num == PORTF)
                {
                    switch(Port_Pins[pin_id].pin_num)
                    {
                    case PIN0:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN1:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_UART)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN2:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN3:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_SPI)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_CAN)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_PWM)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    case PIN4:
                        if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_DIO_GPT)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else if(Port_Pins[pin_id].initial_mode == PORT_PIN_MODE_USB)
                        {
                            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[pin_id].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                        }
                        else
                        {
                            /* Do Nothing */
                        }
                        break;
                    }
                }
                else
                {
                    /* Do Nothing */
                }
            }
        }
        Port_Status = PORT_INITIALIZED;   /* Set the module state to initialized */
    }
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin    -    Port Pin ID number
*                  Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
************************************************************************************/

#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{
    boolean error = FALSE;
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin is within the valid range */
    if (Port_Number_Of_Port_Pins <= Pin)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin direction is changeable */
    if (DIRECTION_CHANGEABLE_OFF == Port_Pins[Pin].direction_changeable)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif
    /* In-case there are no errors */
    if(FALSE == error)
    {

        switch(Port_Pins[Pin].port_num)
        {
        case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
        break;
        case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        break;
        case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        break;
        case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        break;
        case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        break;
        case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        break;
        }

        if(Direction == PORT_PIN_OUT)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[Pin].pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        }
        else if(Direction == PORT_PIN_IN)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[Pin].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        }
        else
        {
            /* Do Nothing */
        }
    }
    else
    {
        /* No Action Required */
    }
}
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction
************************************************************************************/

void Port_RefreshPortDirection(void)
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
    }
    else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    {
        /* Declare the iterator of the for loop - the index of the configuration array of structures */
        Port_PinType pin_id;

        /* Looping on all pins configuration */
        for(pin_id = 0; pin_id < Port_Number_Of_Port_Pins; pin_id++)
        {
            /* Refresh port direction only on unchangeable direction pins */
            if(DIRECTION_CHANGEABLE_OFF == Port_Pins[pin_id].direction_changeable)
            {
                switch(Port_Pins[pin_id].port_num)
                {
                case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                break;
                case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                break;
                case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                break;
                case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                break;
                case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                break;
                case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                break;
                }

                if(Port_Pins[pin_id].direction == PORT_PIN_OUT)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[pin_id].pin_num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                }
                else if(Port_Pins[pin_id].direction == PORT_PIN_IN)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_Pins[pin_id].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                }
                else
                {
                    /* Do Nothing */
                }
            }
            else
            {
                /* Do Nothing */
            }
        }
    }
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): VersionInfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Function to get the version information of this module.
************************************************************************************/

#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if(NULL_PTR == versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin  -  Port Pin ID number
*                  Mode -  New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode
************************************************************************************/

#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{
    boolean error = FALSE;
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if the Driver is initialized before using this function */
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin is within the valid range */
    if (Port_Number_Of_Port_Pins <= Pin)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin direction is changeable */
    if (MODE_CHANGEABLE_OFF == Port_Pins[Pin].mode_changeable)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the required mode is a valid mode */
    if (Port_Number_Of_Modes <= Mode)
    {

        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif
    /* In-case there are no errors */
    if(FALSE == error)
    {
        switch(Port_Pins[Pin].port_num)
        {
        case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
        break;
        case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        break;
        case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        break;
        case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        break;
        case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        break;
        case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        break;
        }

        if(Mode == PORT_PIN_MODE_ADC)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[Pin].pin_num);      /* Set the corresponding bit in the GPIOAMSEL register to enable analog functionality on this pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[Pin].pin_num);     /* Clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_Pins[Pin].pin_num);      /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_Pins[Pin].pin_num);         /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        }
        if(Mode == PORT_PIN_MODE_DIO)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[Pin].pin_num);             /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[Pin].pin_num * 4));     /* Clear the PMCx bits for this pin */
        }
        else
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_Pins[Pin].pin_num);               /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Pins[Pin].pin_num * 4));     /* Clear the PMCx bits for this pin */
            /* Checking the possible modes on every allowed pin of every port to set the PMCx bits
             * to select the configured peripheral function for each pin */
            if(Port_Pins[Pin].port_num == PORTA)
            {
                switch(Port_Pins[Pin].pin_num)
                {
                case PIN0:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN1:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN2:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN3:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN4:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN5:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN6:
                    if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN7:
                    if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                }
            }
            else if(Port_Pins[Pin].port_num == PORTB)
            {
                switch(Port_Pins[Pin].pin_num)
                {
                case PIN0:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN1:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN2:
                    if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN3:
                    if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN4:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN5:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN6:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN7:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                }
            }
            else if(Port_Pins[Pin].port_num == PORTC)
            {
                switch(Port_Pins[Pin].pin_num)
                {
                case PIN4:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN5:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN6:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_USB)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN7:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_USB)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                }
            }
            else if(Port_Pins[Pin].port_num == PORTD)
            {
                switch(Port_Pins[Pin].pin_num)
                {
                case PIN0:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN1:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN2:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_USB)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN3:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_USB)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN4:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN5:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN6:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN7:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_WDG)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                }
            }
            else if(Port_Pins[Pin].port_num == PORTE)
            {
                switch(Port_Pins[Pin].pin_num)
                {
                case PIN0:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN1:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN4:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN5:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_I2C)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                }
            }
            else if(Port_Pins[Pin].port_num == PORTF)
            {
                switch(Port_Pins[Pin].pin_num)
                {
                case PIN0:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN1:
                    if(Mode == PORT_PIN_MODE_UART)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN2:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN3:
                    if(Mode == PORT_PIN_MODE_SPI)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_CAN)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_PWM)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                case PIN4:
                    if(Mode == PORT_PIN_MODE_DIO_GPT)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000007 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else if(Mode == PORT_PIN_MODE_USB)
                    {
                        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_Pins[Pin].pin_num * 4));     /* Selecting the desired peripheral function using the PMCx bits for this pin */
                    }
                    else
                    {
                        /* Do Nothing */
                    }
                    break;
                }
            }
            else
            {
                /* Do Nothing */
            }
        }
    }
    else
    {
        /* No Action Required */
    }
}
#endif
