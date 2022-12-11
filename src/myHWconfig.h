/************************************************************
 * In this File everything which is related to the 
 * Hardware Design is defined
 * - I2C Interface
 * - Interupt Pin
 ************************************************************/ 
#ifndef _MYHWCONFIG_H_
#define _MYHWCONFIG_H_

/************************************************************
 * I2C 
 * - 800kHz
 * - GPIO Routing
 ************************************************************/ 
#define I2CSPEED         800000  
#define I2C_SDA            12                          // GPIO of SDA Signal
#define I2C_CLK            15                          // GPIO of CLK Signal

/************************************************************
 * Interupt Pin 
 ************************************************************/ 
#define INT_PIN            19                          // Attach Interuptto this PIN

#endif // _MYHWCONFIG_H_
