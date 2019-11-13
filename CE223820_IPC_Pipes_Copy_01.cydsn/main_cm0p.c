/******************************************************************************
* File Name: main_cm0p.c
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

/****************************************************************************
*            Global Variables
*****************************************************************************/
char *msgToPrint = NULL;                /* Message to be printed over UART  */
ipc_msg_t ipcMsgForCM4 = {              /* IPC structure to be sent to CM4  */
    .clientId = IPC_CM0_TO_CM4_CLIENT_ID,
    .userCode = 0,
    .intrMask = CY_SYS_CYPIPE_INTR_MASK,
    .buffer   = {0}
};
volatile bool rdyToRecvMsg = true;      /* Ready to receive message flag    */

/****************************************************************************
*            Prototype Functions
*****************************************************************************/
void CM0_MessageCallback(uint32_t *msg);
void CM0_ReleaseCallback(void);

/*******************************************************************************
* Function Name: main()
********************************************************************************
* Summary:
*   Main function of Cortex-M0. Handle the UART communication. 
*
*******************************************************************************/
int main(void)
{
    
    __enable_irq(); /* Enable global interrupts. */
    
    /* Initialize the UART block */
    Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    
    /* Enable the interrupt handler for UART */
//    Cy_SysInt_Init(&UART_SCB_IRQ_cfg, &UART_Interrupt);
//    NVIC_EnableIRQ(UART_SCB_IRQ_cfg.intrSrc);

    /* Enable the UART block */
    Cy_SCB_UART_Enable(UART_HW);
    
    /* Register callback to handle response from CM4 */
    Cy_IPC_Pipe_RegisterCallback(CY_IPC_EP_CYPIPE_ADDR,
                                 CM0_MessageCallback,
                                 CY_IPC_EP_CYPIPE_CM4_ADDR);
    
    /* Enable CM4.  CY_CORTEX_M4_APPL_ADDR must be updated if CM4 memory layout is changed. */
    Cy_SysEnableCM4(CY_CORTEX_M4_APPL_ADDR); 

    for(;;)
    {
        /* Check if any message to print */
        if (msgToPrint)
        {
            /* Send response */
            Cy_SCB_UART_PutString(UART_HW, msgToPrint);
            Cy_SCB_UART_PutString(UART_HW, "\r\n");
            /* Clear pointer */
            msgToPrint = NULL;
        }
        
    }
}

/*******************************************************************************
* Function Name: CM4_MessageCallback()
********************************************************************************
* Summary:
*   Callback function that is executed when a string message is received from 
*   CM4. 
*
* Parameters:
*   msg: IPC message received
*
*******************************************************************************/
void CM0_MessageCallback(uint32_t *msg)
{
    ipc_msg_t *ipcMsgFromCM4;
    
    if (msg != NULL)
    {
        /* Cast the message received to the IPC structure */
        ipcMsgFromCM4 = (ipc_msg_t *) msg;
        
        /* Extract the message to be print over UART */
        msgToPrint = (char *) ipcMsgFromCM4->buffer;
        //Cy_SCB_UART_PutString(UART_HW, (char *) ipcMsgFromCM4->buffer);
        
    }
}