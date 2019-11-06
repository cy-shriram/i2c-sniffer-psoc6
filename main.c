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
    		buffer_ptr = &buffer;
    		print_ptr = &buffer;
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
                    *(buffer_ptr ++) = '\r';
                    *(buffer_ptr ++) = '\n';
                    //buffer_ptr ++;
                    done = true; 
                }
            }
        } 
        /*
        else {
            while ( ((GPIO_PRT_IN(SDA_port) & SDA_LOW_SCL_HIGH)) != 0) {
                if ( ((GPIO_PRT_IN(SDA_port) & SDA_HIGH_SCL_LOW)) == 0) {
                    // detected START condition
                    *(buffer_ptr -1) = 'D';
                    goto lookForData;
                }
            }
        }
        */
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

    buffer_ptr = &buffer;
    print_ptr = &buffer;

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
                    CYHAL_GPIO_DRIVE_NONE, 1U);

    cyhal_gpio_init((cyhal_gpio_t)SCL_pin, CYHAL_GPIO_DIR_INPUT,
                    CYHAL_GPIO_DRIVE_NONE, 1U);

    __enable_irq();

    
    for(;;)
    {
        I2C_Read();
        //I2C_Process();
    }
}



/* [] END OF FILE */

// /******************************************************************************
// * File Name:   main.c
// *
// * Description: This is the source code for the Empty PSoC6 Application
// *              for ModusToolbox.
// *
// * Related Document: See Readme.md
// *
// *
// *******************************************************************************
// * (c) 2019, Cypress Semiconductor Corporation. All rights reserved.
// *******************************************************************************
// * This software, including source code, documentation and related materials
// * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
// * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
// * protection (United States and foreign), United States copyright laws and
// * international treaty provisions. Therefore, you may use this Software only
// * as provided in the license agreement accompanying the software package from
// * which you obtained this Software ("EULA").
// *
// * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
// * non-transferable license to copy, modify, and compile the Software source
// * code solely for use in connection with Cypress's integrated circuit products.
// * Any reproduction, modification, translation, compilation, or representation
// * of this Software except as specified above is prohibited without the express
// * written permission of Cypress.
// *
// * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
// * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
// * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
// * reserves the right to make changes to the Software without notice. Cypress
// * does not assume any liability arising out of the application or use of the
// * Software or any product or circuit described in the Software. Cypress does
// * not authorize its products for use in any products where a malfunction or
// * failure of the Cypress product may reasonably be expected to result in
// * significant property damage, injury or death ("High Risk Product"). By
// * including Cypress's product in a High Risk Product, the manufacturer of such
// * system or application assumes all risk of such use and in doing so agrees to
// * indemnify Cypress against all liability.
// *******************************************************************************/

// #include "cy_pdl.h"
// #include "cyhal.h"
// #include "cybsp.h"
// #include "cy_retarget_io.h"

// #define SDA_pin (P13_0)
// #define SCL_pin (P13_1)

// #define SCL_port 		(GPIO_PRT13)
// #define SCL_pin_num		((uint8_t)1u)
// #define SDA_port 		(GPIO_PRT13)
// #define SDA_pin_num		((uint8_t)0u)
// #define SDA_pin_mask	(0x01)

// #define SDA_INTERRUPT_PRIORITY      (7u)
// #define SCL_INTERRUPT_PRIORITY      (6u)
// #define TIMER_INTERRUPT_PRIORITY    (8u)
// #define TIMER_COMPARE_VALUE			(250000U)
// #define TIMER_PERIOD_VALUE			(0x10000000)

// bool process = false, start = false, sample = false, interrupt = false;

// static void sda_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
// static void scl_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

// static void timer_callback(void *handler_arg, cyhal_gpio_event_t event);

// uint8_t buffer [256] = "";
// uint8_t *buffer_ptr;

// uint8_t output [256] = "";
// uint8_t *out_ptr;

// cyhal_timer_t process_data_timer;

// int main(void)
// {
//     cy_rslt_t result;

//     buffer_ptr = &buffer;

//     cyhal_timer_cfg_t timer_config = {true, (cyhal_timer_direction_t)CYHAL_TIMER_DIR_UP, true, TIMER_PERIOD_VALUE, TIMER_COMPARE_VALUE, 0 };

//     /* Initialize the device and board peripherals */
//     result = cybsp_init() ;
//     if (result != CY_RSLT_SUCCESS)
//     {
//         CY_ASSERT(0);
//     }

//     /* Initialize retargeting standard IO to the debug UART port */
//     cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
//                         CY_RETARGET_IO_BAUDRATE);

//     printf("\r\nI2C Sniffer!!\r\n");

//     cyhal_gpio_init((cyhal_gpio_t) SDA_pin, CYHAL_GPIO_DIR_INPUT,
//                     CYHAL_GPIO_DRIVE_NONE, 1U);

//     cyhal_gpio_init((cyhal_gpio_t)SCL_pin, CYHAL_GPIO_DIR_INPUT,
//                     CYHAL_GPIO_DRIVE_NONE, 1U);
//     cyhal_gpio_register_callback((cyhal_gpio_t) SCL_pin, 
//                                  scl_interrupt_handler, NULL);

//     // Init timer
//     cyhal_timer_init(&process_data_timer, (cyhal_gpio_t) CYHAL_NC_PIN_VALUE, NULL);
//     cyhal_timer_configure(&process_data_timer, &timer_config);
//     cyhal_timer_register_callback(&process_data_timer, (cyhal_timer_event_callback_t)timer_callback, NULL);
//     cyhal_timer_enable_event(&process_data_timer, CYHAL_TIMER_IRQ_CAPTURE_COMPARE,
//                                  TIMER_INTERRUPT_PRIORITY, true);
//     cyhal_timer_start(&process_data_timer);

//     // Wait for both SDA and SCL to be high
//     //while (!(cyhal_gpio_read((cyhal_gpio_t)SCL_pin) && cyhal_gpio_read((cyhal_gpio_t)SDA_pin)));

//     cyhal_gpio_enable_event((cyhal_gpio_t)SCL_pin, CYHAL_GPIO_IRQ_BOTH, 
//                                  SCL_INTERRUPT_PRIORITY, true);

//     __enable_irq();


//     for(;;)
//     {
//         if (process)
//         {
//             // To do display data
//         	process = false;
//         	if ((buffer_ptr - buffer) > 0u)
//         	{
// 				*(buffer_ptr) = '\0';
// 				printf("Data = %s\r\n", buffer);

// //				uint8_t * temp = buffer;
// //				out_ptr = output;
// //				if (*temp >= 'A')
// //				{
// //					temp ++;
// //				}
// //				do
// //				{
// //					uint32_t sda_rise = *(temp++), sda_fall = (*(temp++) - 'A' + '0');
// //					int difference = sda_fall - sda_rise;
// //					if (difference > 0)
// //					{
// //						// SDA goes from LOW to HIGH when SCL is HIGH => Stop condition
// //						*(out_ptr++) = 'H';
// //					}
// //					else if (difference < 0)
// //					{
// //						// SDA goes from HIGH to LOW when SCL is HIGH => Start condition
// //						*(out_ptr++) = 'S';
// //					}
// //					else
// //					{
// //						// Data
// //						*(out_ptr++) = sda_fall;
// //					}
// //
// //				} while (temp < buffer_ptr-1);
// //
// //				*(out_ptr) = '\0';
// //				printf("Processed info = %s\r\n", output);

// 				buffer_ptr = buffer;
//         	}
//         }
//         if (interrupt)
//         {
//         	cyhal_timer_stop(&process_data_timer);
//         	cyhal_timer_cfg_t timer_config = {true, (cyhal_timer_direction_t)CYHAL_TIMER_DIR_UP, true, TIMER_PERIOD_VALUE, TIMER_COMPARE_VALUE, 0 };
//         	cyhal_timer_configure(&process_data_timer, &timer_config);
//         	cyhal_timer_start(&process_data_timer);
//         	interrupt = false;
//         }
//     }
// }

// static void scl_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
// {
// 	if (0 != (GPIO_PRT_IN(SDA_port) & SDA_pin_mask))

// 	{
// 		*(buffer_ptr++) = (event == CYHAL_GPIO_IRQ_RISE) ? '1' : ((event == CYHAL_GPIO_IRQ_FALL) ? 'B' : 'F');
// 	}
// 	else
// 	{
// 		*(buffer_ptr++) = (event == CYHAL_GPIO_IRQ_RISE) ? '0' : ((event == CYHAL_GPIO_IRQ_FALL) ? 'A' : 'F');
// 	}

// 	if (*(buffer_ptr-1) >= 'A')
// 	{
// 		*(buffer_ptr++) = ' ';
// 	}

//     // Clear the timer
//     interrupt = true;
    
// }

// static void timer_callback(void *handler_arg, cyhal_gpio_event_t event)
// {
//     process = true;
    
//     // Clear the timer
//     cyhal_timer_stop(&process_data_timer);
//     cyhal_timer_cfg_t timer_config = {true, (cyhal_timer_direction_t)CYHAL_TIMER_DIR_UP, true, TIMER_PERIOD_VALUE, TIMER_COMPARE_VALUE, 0 };
//     cyhal_timer_configure(&process_data_timer, &timer_config);
//     cyhal_timer_start(&process_data_timer);
// }
