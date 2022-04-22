 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

#include "Common_Macros.h"
#include "Std_Types.h"
#define PORT_VENDOR_ID    (1000U)

/* Dio Module Id */
#define PORT_MODULE_ID    (120U)

/* Dio Instance Id */
#define PORT_INSTANCE_ID  (0U)
/*odule Version 1.0.0
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


#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)
#include "Port_Cfg.h"

/*******************************************************************************
 *                              Module Definitions                             *
 *******************************************************************************/

/* GPIO Registers base addresses */
#define GPIO_PORTA_BASE_ADDRESS           0x40004000
#define GPIO_PORTB_BASE_ADDRESS           0x40005000
#define GPIO_PORTC_BASE_ADDRESS           0x40006000
#define GPIO_PORTD_BASE_ADDRESS           0x40007000
#define GPIO_PORTE_BASE_ADDRESS           0x40024000
#define GPIO_PORTF_BASE_ADDRESS           0x40025000

/* GPIO Registers offset addresses */
#define PORT_DATA_REG_OFFSET              0x3FC
#define PORT_DIR_REG_OFFSET               0x400
#define PORT_ALT_FUNC_REG_OFFSET          0x420
#define PORT_PULL_UP_REG_OFFSET           0x510
#define PORT_PULL_DOWN_REG_OFFSET         0x514
#define PORT_DIGITAL_ENABLE_REG_OFFSET    0x51C
#define PORT_LOCK_REG_OFFSET              0x520
#define PORT_COMMIT_REG_OFFSET            0x524
#define PORT_ANALOG_MODE_SEL_REG_OFFSET   0x528
#define PORT_CTL_REG_OFFSET               0x52C
/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
#define PORT_INIT_SID                   (uint8)0x00
#define PORT_SETPINDIRECTION_SID        (uint8)0x01
#define PORT_REFRESHPORTDIRECTION__SID  (uint8)0x02
#define PORT_GETVERSIONINFO_SID         (uint8)0x03
#define PORT_SETPINMODE_SID             (uint8)0x04
   
   
   /*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
   
#define PORT_E_PARAM_PIN                (uint8)0x0A
#define PORT_E_DIRECTION_UNCHANGEABLE   (uint8)0x0b
#define PORT_E_PARAM_CONFIG             (uint8)0x0c
#define PORT_E_PARAM_INVALID_MODE       (uint8)0x0d
#define PORT_E_MODE_UNCHANGEABLE        (uint8)0x0e
#define PORT_E_UNINIT                   (uint8)0x0f  
#define PORT_E_PARAM_POINTER            (uint8)0x10
   
   
   
   
   
/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

typedef uint8 Port_PinType;

typedef uint8 Port_PinModeType;


typedef uint8 Port_PortType;  
   
   
/* Description: Enum to hold PIN direction */
typedef enum
{
    INPUT,OUTPUT
}Port_PinDirection;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;


typedef enum
{
  ENABLED,DISABLED
}Port_PinDirection_changable;


typedef enum
{
    ENABLLED,DISABLLED
}Port_PinMode_changable;






/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 */
typedef struct 
{   
    Port_PortType port_num; 
    Port_PinType pin_num; 
    
    Port_PinDirection direction;
    Port_InternalResistor resistor;
    Port_PinModeType mode;
    uint8 initial_value;
    Port_PinDirection_changable state_d;
    Port_PinMode_changable state_m;
    
}Port_ConfigSpecs;

typedef struct Port_ConfigType
{
 Port_ConfigSpecs pins[PORT_CONFIGURED_CHANNLES ];
}Port_ConfigType;


/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr );

#if (PORT_SET_PIN_DIRECTION_API==STD_ON)

void Port_SetPinDirection(Port_PinType Pin,Port_PinDirection Direction);


#endif

void Port_RefreshPortDirection(void);



#if (PORT_VERSION_INFO_API==STD_ON)

void Port_GetVersionInfo(void)

#endif

void Set_SetPinMode(Port_PinType Pin,Port_PinModeType Mode);


extern const Port_ConfigType Port_Configuration;




#endif /* PORT_H */
