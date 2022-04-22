#ifndef PORT_CFG_H
#define PORT_CFG_H


#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)
 
/*Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

#define PORT_VERSION_INFO_API               (STD_OFF)

#define PORT_CONFIGURED_CHANNLES              (39U)


#define PortConf_LED1_PORT_NUM                5 /* PORTF */
#define PortConf_SW1_PORT_NUM                 5 /* PORTF */

/* DIO Configured Channel ID's */
#define PortConf_LED1_CHANNEL_NUM             1 /* Pin 1 in PORTF */
#define PortConf_SW1_CHANNEL_NUM              4 /* Pin 4 in PORTF */


//=======defines of pins and ports==========================
#ifndef PORTA
#define PORTA 0
#endif

#ifndef PORTB
#define PORTB 1
#endif

#ifndef PORTC
#define PORTC 2
#endif

#ifndef PORTD
#define PORTD 3
#endif


#ifndef PORTE
#define PORTE 4
#endif


#ifndef PORTF
#define PORTF 5
#endif


#define PIN0 0

#define PIN1 1

#define PIN2 2

#define PIN3 3

#define PIN4 4

#define PIN5 5


#define PIN6 6

#define PIN7 7



//=============================

// DEFINES FOR THE MODES OF PINS

#define DIO             0
#define UART            1
#define SPI             2
#define I2C             3
#define PWM             4
#define PH              5
#define ID              6
#define TIMER           7
#define WATCHDOG        8
#define CAN             9
#define ADC             10

















#endif