/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 6 MCU Hello World Example
*              for ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
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
#include "cy_retarget_io.h"
#include "cycfg.h"
#include "cy_dfu.h"


/*******************************************************************************
* Macros
*******************************************************************************/

/* LED blink timer clock value in Hz  */
#define LED_BLINK_TIMER_CLOCK_HZ          (10000)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD            (9999)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD_UPDATING   (3000)

/*******************************************************************************
* Function Name: Cy_OnResetUser
********************************************************************************
*
*  This function is called at the start of Reset_Handler().
*  DFU requires it to call Cy_DFU_OnResetApp0() in app#0.
*
*******************************************************************************/
void Cy_OnResetUser(void)
{
    Cy_DFU_OnResetApp0();
}


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);


/*******************************************************************************
* Global Variables
*******************************************************************************/
bool timer_interrupt_flag = false;
bool led_blink_active_flag = true;

/* Variable for storing character read from terminal */
uint8_t uart_read_value;

/* Timer object used for blinking the LED */
cyhal_timer_t led_blink_timer;

/* DFU related variables */
cy_en_dfu_status_t status;
/* DFU params, used to configure DFU */
cy_stc_dfu_params_t dfuParams;

const cyhal_timer_cfg_t led_blink_timer_cfg_Dfu_update =
 {
     .compare_value = 0,                 /* Timer compare value, not used */
     .period = LED_BLINK_TIMER_PERIOD_UPDATING,   /* Defines the timer period */
     .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
     .is_compare = false,                /* Don't use compare mode */
     .is_continuous = true,              /* Run timer indefinitely */
     .value = 0                          /* Initial value of counter */
 };

/* Interrupt handler callback function */
void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    /* Validate and switch to App1 */
    status = Cy_DFU_ValidateApp(1u, &dfuParams);
    if (status == CY_DFU_SUCCESS)
    {
    	Cy_DFU_TransportStop();
    	Cy_DFU_ExecuteApp(1u);
    }

}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It sets up a timer to trigger a 
* periodic interrupt. The main while loop checks for the status of a flag set 
* by the interrupt and toggles an LED at 1Hz to create an LED blinky. The 
* while loop also checks whether the 'Enter' key was pressed and 
* stops/restarts LED blinking.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

volatile uint32_t crc;
int main(void)
{
	uint32_t count = 0;

	uint32_t state;

    uint32_t appVerifyStartAddress;
    uint32_t appVerifySize;

	cy_rslt_t result;

	/* Timeout for Cy_DFU_Continue(), in milliseconds */
	const uint32_t paramsTimeout = 20u;

	/* Buffer to store DFU commands */
	CY_ALIGN(4) static uint8_t buffer[CY_DFU_SIZEOF_DATA_BUFFER];

	/* Buffer for DFU data packets for transport API */
	CY_ALIGN(4) static uint8_t packet[CY_DFU_SIZEOF_CMD_BUFFER ];

	/* Initialize dfuParams structure */
	dfuParams.timeout          = paramsTimeout;
	dfuParams.dataBuffer       = &buffer[0];
	dfuParams.packetBuffer     = &packet[0];
	status = Cy_DFU_Init(&state, &dfuParams);

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, 
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    cyhal_gpio_init(CYBSP_SW2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    cyhal_gpio_register_callback(CYBSP_SW2, gpio_interrupt_handler, NULL);
    cyhal_gpio_enable_event(CYBSP_SW2, CYHAL_GPIO_IRQ_FALL, 3, true);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "PSoC 6 MCU: Hello World! Example "
           "****************** \r\n\n");

    printf("Hello World!!!\r\n\n");
    
    printf("For more PSoC 6 MCU projects, "
           "visit our code examples repositories:\r\n\n");

    printf("1. ModusToolbox Examples:\r\n https://www.cypress.com/documentation"
            "/code-examples/psoc6-sdk-code-examples-modustoolbox-software\r\n\n");

    printf("2. Mbed OS Examples:\r\n https://www.cypress.com/documentation/"
           "code-examples/mbed-sdk-code-examples\r\n\n");

    /* Initialize timer to toggle the LED */
    timer_init();

    printf("Press 'Enter' key to pause or "
           "resume blinking the user LED \r\n\r\n");

    /* Initialize DFU communication */
     Cy_DFU_TransportStart();

    for (;;)
    {
    	status = Cy_DFU_Continue(&state, &dfuParams);
    	++count;
    	if (state == CY_DFU_STATE_FINISHED)
    	{
    		/* Finished loading the application image */
    		/* Validate DFU application, if it is valid then switch to it */

    		status = Cy_DFU_ValidateApp(2u, &dfuParams);
    		if (status == CY_DFU_SUCCESS)
    		{
    			Cy_DFU_TransportStop();
    			Cy_DFU_ExecuteApp(2u);
    		}
    		else if (status == CY_DFU_ERROR_VERIFY)
    		{
    			/*
    			 * Restarts loading, an alternatives are to Halt MCU here
    			 * or switch to the other app if it is valid.
    			 * Error code may be handled here, i.e. print to debug UART.
    			 */
    			status = Cy_DFU_Init(&state, &dfuParams);
    			Cy_DFU_TransportReset();
    		}
    	}
    	else if (state == CY_DFU_STATE_FAILED)
    	{
    		/* An error has happened during the loading process */
    		/* Handle it here */
    		/* In this Code Example just restart loading process */
    		status = Cy_DFU_Init(&state, &dfuParams);
    		Cy_DFU_TransportReset();
    	}
    	else if (state == CY_DFU_STATE_UPDATING)
    	{
    		printf("\n\n Updating\r\r");
    		uint32_t passed5seconds = (count >= (5000ul/paramsTimeout)) ? 1u : 0u;
    		/*
    		 * if no command has been received during 5 seconds when the loading
    		 * has started then restart loading.
    		 */
    		if (status == CY_DFU_SUCCESS)
    		{
    			count = 0u;
    		}
    		else if (status == CY_DFU_ERROR_TIMEOUT)
    		{
    			if (passed5seconds != 0u)
    			{
    				count = 0u;
    				Cy_DFU_Init(&state, &dfuParams);
    				Cy_DFU_TransportReset();
    			}
    		}
    		else
    		{
    			count = 0u;
    			/* Delay because Transport still may be sending error response to a host */
    			Cy_SysLib_Delay(paramsTimeout);
    			Cy_DFU_Init(&state, &dfuParams);
    			Cy_DFU_TransportReset();
    		}
    	}
    }
}


/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks 
* continuously and produces a periodic interrupt on every terminal count 
* event. The period is defined by the 'period' and 'compare_value' of the 
* timer configuration structure 'led_blink_timer_cfg'. Without any changes, 
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
 void timer_init(void)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg = 
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = LED_BLINK_TIMER_PERIOD,   /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction, 
       duration */
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                              7, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&led_blink_timer);
 }


/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;
    cyhal_gpio_toggle(CYBSP_USER_LED);
}

/* [] END OF FILE */
