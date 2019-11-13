/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See Readme.md
*
*
*******************************************************************************
* (c) 2019, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
//#include "cy_retarget_io.h"

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

#define UART_BAUDRATE0		(115200)
#define UART_BAUDRATE1		(230400)
#define UART_BAUDRATE2		(460800)
#define UART_BAUDRATE3		(921600)

#define UART_BAUDRATE		(UART_BAUDRATE0)

uint8_t buffer [256] = "";
uint8_t *buffer_ptr, *print_ptr;

uint8_t output [256] = "";
uint8_t *out_ptr;

cyhal_uart_t cy_retarget_io_uart_obj;


cy_rslt_t cy_retarget_io_init1(cyhal_gpio_t tx, cyhal_gpio_t rx, uint32_t baudrate)
{
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0,
    };

    cy_rslt_t result = cyhal_uart_init(&cy_retarget_io_uart_obj, tx, rx, NULL, &uart_config);

    if (result == CY_RSLT_SUCCESS)
    {
        result = cyhal_uart_set_baud(&cy_retarget_io_uart_obj, baudrate, NULL);
    }

    return result;
}



void I2C_Read ()
{

    uint8_t s;
    bool done = false;

    lookForStart:

    // Expect both SCL and SDA to be high
    while ( (GPIO_PRT_IN(SDA_port) & SDA_HIGH_SCL_HIGH) != SDA_HIGH_SCL_HIGH) ;
    // both SLC and SDA high at this point

    // Looking for START condition. Ie SDA transitioning from 
    // high to low while SLC is high.
    while ( (GPIO_PRT_IN(SDA_port) & SDA_HIGH_SCL_HIGH) != SDA_LOW_SCL_HIGH)
    {
    	// Both are high
    	if (print_ptr < buffer_ptr)
    	{
    		cyhal_uart_putc(&cy_retarget_io_uart_obj, *(print_ptr ++));
    	}
    	else
    	{
    		buffer_ptr = buffer;
    		print_ptr = buffer;
    	}
    }
    *(buffer_ptr ++) = 'S';

    // wait for SCL low
    while ( ((GPIO_PRT_IN(SDA_port) & SDA_LOW_SCL_HIGH)) != 0) ;

    lookForData:

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
                    *(buffer_ptr ++) = ';';
                    *(buffer_ptr ++) = '\r';
                    *(buffer_ptr ++) = '\n';
                    //buffer_ptr ++;
                    done = true; 
                }
            }
        } 
    } 
}

void I2C_Process()
{
    if ((buffer_ptr - buffer) > 0u)
   	{
        *(buffer_ptr) = '\0';
		printf("Data = %s\r\n", buffer);
        buffer_ptr = buffer;
    }
}

int main(void)
{
    cy_rslt_t result;

    buffer_ptr = buffer;
    print_ptr = buffer;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retargeting standard IO to the debug UART port */
    cy_retarget_io_init1(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, UART_BAUDRATE);

    printf("\r\nI2C Sniffer!!\r\n");

    cyhal_gpio_init((cyhal_gpio_t) SDA_pin, CYHAL_GPIO_DIR_INPUT,
    		CYHAL_GPIO_DRIVE_PULLUP, 1U); // Pull up for SDA

    cyhal_gpio_init((cyhal_gpio_t)SCL_pin, CYHAL_GPIO_DIR_INPUT,
    		CYHAL_GPIO_DRIVE_PULLUP, 1U); // Pull up for SCL

    __enable_irq();

    
    for(;;)
    {
        I2C_Read();
    }
}

/* [] END OF FILE */
