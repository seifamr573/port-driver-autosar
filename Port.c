 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Tarek
 ******************************************************************************/
#include "Port.h"
#include "Std_Types.h"
#include "Platform_Types.h"

#include "tm4c123gh6pm_registers.h"
   #if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif
   
STATIC const Port_ConfigSpecs* Port_Channel= NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;
volatile uint32 * PortGpio_Ptr = NULL_PTR;
/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x10
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initialize the PORT driver.
************************************************************************************/

void Port_Init(const Port_ConfigType* ConfigPtr )
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
		/*
		 * Set the module state to initialized and point to the PB configuration structure using a global pointer.
		 * This global pointer is global to be used by other functions to read the PB configuration structures
		 */
		Port_Status       = PORT_INITIALIZED;
		Port_Channel = ConfigPtr->pins; /* address of the first Channels structure --> Channels[0] */
                 volatile uint32 delay = 0;
                 SYSCTL_REGCGC2_REG |= 0x0000003f;
                  delay = SYSCTL_REGCGC2_REG;
	}
        
        for(int i=0  ;i< PORT_CONFIGURED_CHANNLES   ;i++){
          switch((Port_Channel+i)->port_num){
            
          case PORTA: PortGpio_Ptr = (volatile uint32 *) GPIO_PORTA_BASE_ADDRESS  ;
            break;
            
            
            case PORTB: PortGpio_Ptr = (volatile uint32 *) GPIO_PORTB_BASE_ADDRESS  ;
            break;
            
            case PORTC: PortGpio_Ptr = (volatile uint32 *) GPIO_PORTC_BASE_ADDRESS ; 
              
            break;
            
            
            case PORTD: PortGpio_Ptr = (volatile uint32 *) GPIO_PORTD_BASE_ADDRESS;  
            break;
            
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          }
          
          //checking if the pin need to be unlocked
            if( (((Port_Channel+i)->port_num == PORTD) && ((Port_Channel+i)->pin_num == PIN7)) || (((Port_Channel+i)->port_num == PORTF) && (Port_Channel+i)->pin_num == PIN0))  /* PD7 or PF0 */
    {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , (Port_Channel+i)->pin_num );  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
    }
    else if( ((Port_Channel+i)->port_num==PORTC) && ((Port_Channel+i)->pin_num <= PIN3) ) /* PC0 to PC3 */
    {
        /* Do Nothing ...  this is the JTAG pins */
        continue;
    }
    else
    {
        /* Do Nothing ... No need to unlock the commit register for this pin */
    }
          
          
          
          
          
          //adjusting the mood for the pins         
          
          if((Port_Channel+i)->mode==DIO){
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , (Port_Channel+i)->pin_num );             /* Disable Alternative function for this pin*/
            
           *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << ((Port_Channel+i)->pin_num * 4));     /* Clear the PMCx bits for this pin */
          
          
          }
          else {
          /* Put the mode number in the PCTL register for the current pin Clearing the 4 bits and then insert the mode number */
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_Channel+i)->pin_num * 4);
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((Port_Channel+i)->mode << (Port_Channel+i)->pin_num* 4);
            
          
          
          
}


    //check if the anlog mode should be activated or not   
if((Port_Channel+i)->mode!=ADC){

   

 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Port_Channel+i)->pin_num );   

CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Port_Channel+i)->pin_num );  

}
else{



SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , (Port_Channel+i)->pin_num );   

CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , (Port_Channel+i)->pin_num );   

}

//adjusting the direction and the intial values  of the pin

if((Port_Channel+i)->direction==OUTPUT){

SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET ) , (Port_Channel+i)->pin_num );   

if((Port_Channel+i)->initial_value==0){

  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , (Port_Channel+i)->pin_num);  //intial value
}
else{

SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , (Port_Channel+i)->pin_num);

}
}
else{

CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET ) , (Port_Channel+i)->pin_num );


if((Port_Channel+i)->resistor==OFF){
  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , (Port_Channel+i)->pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , (Port_Channel+i)->pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
  
  
}
else if((Port_Channel+i)->resistor==PULL_UP){
  
 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , (Port_Channel+i)->pin_num);
  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , (Port_Channel+i)->pin_num);
}
else{
  
 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , (Port_Channel+i)->pin_num);
CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , (Port_Channel+i)->pin_num);  

}

}

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        }
        
        
}



/***********************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in):  Pin,Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: set the port pin direction .
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API==STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirection Direction){
  boolean error = FALSE;

  #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINDIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
	
	if ((Port_Channel+Pin)->state_d==DISABLED)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINDIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        if(Pin>=PORT_CONFIGURED_CHANNLES){
          
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINDIRECTION_SID, PORT_E_PARAM_PIN  );
		error = TRUE;
        
        }
        else{
        }
#endif
  if(FALSE == error)
	{
          if(Direction==OUTPUT){
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET ) , Pin );   
          
          }
          else{
           CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr +PORT_DIR_REG_OFFSET ) , Pin);   
          
          
          }
          
          
	}
	else
	{
		/* No Action Required */
	}
        
        
        
        
        
        
        
        
     
  

}



#endif



//=============each bit ( set) in the element of array of mode show the modes avaliable for this pin========== 
// i know the array should be port_cfg file but it gives me a linker error when i put it there 




char Mode_arr[ PORT_CONFIGURED_CHANNLES ]={
0x0103,/* PIN0 */
  0x0103,/* PIN1 */
  0x0005,/* PIN2 */
  0x0005,/* PIN3 */
  0x0005,/* PIN4 */
  0x0005,/* PIN5 */
  0x0029,/* PIN6 */
  0x0029,/* PIN7 */
  0x0083,/* PIN8 */
  0x0083,/* PIN9 */
  0x0089,/* PIN10 */
  0x0089,/* PIN11 */
  0x0195,/* PIN12 */
  0x0195,/* PIN13*/
  0x0095,/* PIN14 */
  0x0095,/* PIN15 */
  0x01D7,/* PIN16 */
  0x01D7,/* PIN17 */
  0x01C3,/* PIN18 */
  0x0183,/* PIN19 */
  0x00BF,/* PIN20 */
  0x00BF,/* PIN21 */
  0x0197,/* PIN22 */
  0x01C7,/* PIN23 */
  0x0083,/* PIN24 */
  0x0083,/* PIN25 */
  0x00D3,/* PIN26 */
  0x01C3,/* PIN27 */
  0x0003,/* PIN28 */
  0x0003,/* PIN29 */
  0x0001,/* PIN30 */
  0x0001,/* PIN31 */
  0x013B,/* PIN32 */
  0x013B,/* PIN33 */
  0x03EF,/* PIN34 */
  0x42E7,/* PIN35 */
  0x40B5,/* PIN36 */
  0x40AD,/* PIN37 */
  0x01E1,/* PIN38 */



  
  
  
  





  
  
  
  
  








};






//=================================================================================================================================================

/***********************************************************************************
* Service Name: Set_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in):  Pin,Mode
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: set the port pin mode .
************************************************************************************/



 void  Set_SetPinMode(Port_PinType Pin,Port_PinModeType Mode){
        
        
        boolean error = FALSE;

  #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINMODE_SID , PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
	
	if ((Port_Channel+Pin)->state_m==DISABLLED)
	{

		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINMODE_SID, PORT_E_MODE_UNCHANGEABLE);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        if(Pin>=PORT_CONFIGURED_CHANNLES){
          
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINMODE_SID, PORT_E_PARAM_PIN  );
		error = TRUE;
        
        }
        else{
        }
        
        if(Mode_arr[Pin] & (1<<Mode)){
          
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINMODE_SID, PORT_E_PARAM_INVALID_MODE );
		error = TRUE;
        
        }
        else{
        }
#endif
  if(FALSE == error)
	{        
		(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << Pin* 4));
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (Mode << Pin* 4);
	}
	else
	{
		/* No Action Required */
	}
        
        
        
        
        
        
        
        
     
  
        
        
        
        
        }




/***********************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in):  None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function refresh all the unchangable pins .
************************************************************************************/
void Port_RefreshPortDirection(void)
{
  boolean error = FALSE;
 #if (PORT_DEV_ERROR_DETECT == STD_ON)
  /* check if the port driver is not initialized*/
  if (Port_Status == PORT_NOT_INITIALIZED)
  {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				PORT_SETPINMODE_SID , PORT_E_UNINIT);
		error = TRUE;
  } 
 #endif
  if(error == FALSE)
  {
   for(int i=0 ; i<PORT_CONFIGURED_CHANNLES  ; i++)
   {
     if( (Port_Channel+i)->state_d == DISABLED) 
     {
       switch((Port_Channel+i)->port_num)
       {
        case PORTA : PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
        break ;
        case PORTB : PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
        break ;
        case PORTC : PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
        break ;
        case PORTD : PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
        break ;
        case PORTE : PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
        break ;
        case PORTF : PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
        break ;
       }
        /* the output of previuos block is pointer to the base address of the pin */
        if((Port_Channel+i)->direction == INPUT ) 
           {
             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , (Port_Channel+i)->pin_num);
           }
        else 
           {
             SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , (Port_Channel+i)->pin_num);     
           }
     }
     else 
     {
       /* Do nothing */
     }
   } /* end of for loop */
  }
  else 
  {
    /* Do nothing */
  }
}

/***********************************************************************************
* Service Name: void Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in):  None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: return the version information for this module .
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
				PORT_GET_VERSION_INFO_SID,PORT_E_PARAM_POINTER);
	}
	else
#endif /* (DIO_DEV_ERROR_DETECT == STD_ON) */
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



