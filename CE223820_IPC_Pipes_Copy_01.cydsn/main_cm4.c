/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.10
*
* Description: This example demonstrates how to use the IPC to implement a 
*              message pipe in PSoC 6 MCU. The pipe is used as a method to 
*              send messages between the CPUs.
*
* Related Document: CE223820.pdf
*
* Hardware Dependency: CY8CKIT-062 PSoC 6 Pioneer kit
*
******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation.
******************************************************************************
* This software, including source code, documentation and related materials
* ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and 
* foreign), United States copyright laws and international treaty provisions. 
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the 
* Cypress source code and derivative works for the sole purpose of creating 
* custom software in support of licensee product, such licensee product to be
* used only in conjunction with Cypress's integrated circuit as specified in the
* applicable agreement. Any reproduction, modification, translation, compilation,
* or representation of this Software except as specified above is prohibited 
* without the express written permission of Cypress.
* 
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use
* of Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use as critical components in any products 
* where a malfunction or failure may reasonably be expected to result in 
* significant injury or death ("ACTIVE Risk Product"). By including Cypress's 
* product in a ACTIVE Risk Product, the manufacturer of such system or application
* assumes all risk of such use and in doing so indemnifies Cypress against all
* liability. Use of this Software may be limited by and subject to the applicable
* Cypress software license agreement.
*****************************************************************************/
#include "project.h"
#include "ipc_def.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define SDA_pin (P13_0)
#define SCL_pin (P13_1)

#define SCL_port 		(GPIO_PRT13)
#define SCL_pin_num		((uint8_t)1u)
#define SDA_port 		(GPIO_PRT13)
#define SDA_pin_num		((uint8_t)0u)
#define SDA_pin_mask	(0x01)

#define SDA_HIGH_SCL_HIGH   (0b00000011)
#define SDA_HIGH_SCL_LOW    (0b00000001)
#define SDA_LOW_SCL_LOW     (0b00000000)
#define SDA_LOW_SCL_HIGH    (0b00000010)


/****************************************************************************
*            Global Variables
*****************************************************************************/
//volatile bool rdyToProcess = false;      /* Ready to process flag           */
ipc_msg_t ipcMsgForCM0 = {               /* IPC structure to be sent to CM0 */
    .clientId = IPC_CM4_TO_CM0_CLIENT_ID,
    .userCode = 0,
    .intrMask = CY_SYS_CYPIPE_INTR_MASK,
    .buffer   = {0}
};
//static char stringToProcess[IPC_BUFFER_SIZE] = {0}; /* String to process    */


uint8_t buffer [256] = "";
uint8_t *buffer_ptr, *print_ptr;

uint8_t output [256] = "";
uint8_t *out_ptr;

volatile uint8_t i = 1, j11 = 0;

/****************************************************************************
*            Prototype Functions
*****************************************************************************/
void CM4_MessageCallback();
void I2C_Read();

/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   Main function of Cortex-M4. Parse the string message received by CM0 and
*   process the operation. Send a message back to CM0 with the result.
*
*******************************************************************************/
int main(void)
{
    
    __enable_irq(); /* Enable global interrupts. */
    
    /* Register the Message Callback */
    Cy_IPC_Pipe_RegisterCallback(CY_IPC_EP_CYPIPE_ADDR,
                                 CM4_MessageCallback,
                                 IPC_CM0_TO_CM4_CLIENT_ID);    

    buffer_ptr = ipcMsgForCM0.buffer;
    print_ptr = buffer;
    for(;;)
    {
        I2C_Read();
            
    }
}

/*******************************************************************************
* Function Name: CM4_MessageCallback()
********************************************************************************
* Summary:
*   Callback function that is executed when a string message is received from 
*   CM0. Copy the string message to a local array, which is processed in the
*   main loop.
*
* Parameters:
*   msg: IPC message received
*
*******************************************************************************/
void CM4_MessageCallback()
{
    
    
}

void I2C_Read()
{
    uint8_t s;
    
    bool done = false; 

    // Expect both SCL and SDA to be high
    while ( (GPIO_PRT_IN(SDA_port) & SDA_HIGH_SCL_HIGH) != SDA_HIGH_SCL_HIGH) ;
    // both SLC and SDA high at this point

    // Looking for START condition. Ie SDA transitioning from 
    // high to low while SLC is high.
    while ( (GPIO_PRT_IN(SDA_port) & SDA_HIGH_SCL_HIGH) != SDA_LOW_SCL_HIGH)
    {
        if (i == 0)
        {
            *(buffer_ptr) = '\0';
            Cy_IPC_Pipe_SendMessage(CY_IPC_EP_CYPIPE_CM0_ADDR,
                                    CY_IPC_EP_CYPIPE_CM4_ADDR,
                                    (void *) &ipcMsgForCM0, 0);
            
            buffer_ptr = ipcMsgForCM0.buffer;            
            i = 1;
        }
   
    }
    *(buffer_ptr ++) = 'S';

    // wait for SCL low
    while (((GPIO_PRT_IN(SDA_port) & SDA_LOW_SCL_HIGH)) != 0) ;
    while (!done) {

        // wait for SCL low
        while ( ((GPIO_PRT_IN(SDA_port) & SDA_LOW_SCL_HIGH)) != 0) ;

        // Wait for SCL to transition high
        while ( ((GPIO_PRT_IN(SDA_port) & SDA_LOW_SCL_HIGH)) == 0) ;

        // Sample SDA at the transition point
        s = (GPIO_PRT_IN(SDA_port) & SDA_HIGH_SCL_LOW);
        *(buffer_ptr ++) = (s == 0 ? '0' : '1');

        // Wait for SCL to transition low while looking 
        // for start or stop condition. A START or STOP
        // means the previous bit isn't a data bit. So will
        // write START, STOP condition into the same memory slot
        if (s == 0) {
            while ( (GPIO_PRT_IN(SDA_port) & SDA_LOW_SCL_HIGH) != 0 && (!done)) {
                if ( ((GPIO_PRT_IN(SDA_port) & SDA_HIGH_SCL_LOW)) != 0) {
                    // detected STOP condition
                    *(buffer_ptr - 1) = 'H';
                    *(buffer_ptr ++) = '\r';
                    *(buffer_ptr ++) = '\n';
                    i = 0;
                    done = true; 
                }
            }
        } 
    } 
}


/* [] END OF FILE */
