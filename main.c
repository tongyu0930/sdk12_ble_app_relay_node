/*
 * main.c
 *
 *  Author: Tong Yu
 */

// 最后在删减makefile
#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "relay.h"
#include "nrf_error.h"


#define CENTRAL_LINK_COUNT       		0  			/**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT    		1  			/**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0 			/**< 不懂Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


static bool 							started_bro_sca				= false;
volatile bool 							first_time					= true;
extern volatile bool 					want_scan;
extern volatile uint8_t 				self_level;
extern volatile bool					scan_only_mode;


const ble_gap_adv_params_t m_adv_params =
  {
	.type        					= BLE_GAP_ADV_TYPE_ADV_IND,					// Undirected advertisement.
	//.p_peer_addr->addr_type 		= BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
	.p_peer_addr					= NULL,												// 我觉得null是不是默认就是static的address？
	.fp          					= BLE_GAP_ADV_FP_ANY,
	.interval    					= 0x0020,						// 虽然这个最小值时100ms，但是你可以通过timer以更快的频率启动关闭广播。
	.timeout     					= 0
  };

const ble_gap_adv_params_t m_adv_params2 =
  {
	.type        					= BLE_GAP_ADV_TYPE_ADV_NONCONN_IND,					// Undirected advertisement.
	//.p_peer_addr->addr_type 		= BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
	.p_peer_addr					= NULL,												// 我觉得null是不是默认就是static的address？
	.fp          					= BLE_GAP_ADV_FP_ANY,
	.interval    					= 0x0100,						// 虽然这个最小值时100ms，但是你可以通过timer以更快的频率启动关闭广播。
	.timeout     					= 0
  };

const ble_gap_scan_params_t m_scan_params =
  {
    .active      = 0,
    .use_whitelist   = 0,
    .adv_dir_report = 0,
    .interval    = 0x0040,
    .window      = 0x0010,
    .timeout     = 0
  };

const ble_gap_scan_params_t m_scan_params2 =
  {
    .active      = 0,
    .use_whitelist   = 0,
    .adv_dir_report = 0,
    .interval    = 0x0040,
    .window      = 0x0040,
    .timeout     = 0
  };



void scanning_start(void);
void init_storage(void);
void relay_init(void);




/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze how your product is supposed to react in case of Assert.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


void advertising_start(void)
{
    uint32_t err_code;
    if(self_level == 2)
    {
    	err_code = sd_ble_gap_adv_start(&m_adv_params);
    	APP_ERROR_CHECK(err_code);
    }else
    {
    	err_code = sd_ble_gap_adv_start(&m_adv_params2);
    	APP_ERROR_CHECK(err_code);
    }
}

void scanning_start(void)
{
	uint32_t err_code;
	if(scan_only_mode)
	{
		err_code = sd_ble_gap_scan_start(&m_scan_params);
		APP_ERROR_CHECK(err_code);
	}else
	{
		err_code = sd_ble_gap_scan_start(&m_scan_params2);
		APP_ERROR_CHECK(err_code);
	}
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    get_adv_data(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL); 													// Initialize the SoftDevice handler module.
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);									//Check the ram settings against the used number of links

    err_code = softdevice_enable(&ble_enable_params); 												// Enable BLE stack.
    APP_ERROR_CHECK(err_code);

    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch); 									// Register with the SoftDevice handler module for BLE events.
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void GPIOTE_IRQHandler(void)
{
    uint32_t err_code;

    if (NRF_GPIOTE->EVENTS_IN[2] != 0) // button2 开启 广播, 还不能完美关闭
    {
    	nrf_delay_us(200000);
        NRF_GPIOTE->EVENTS_IN[2] = 0;

        NRF_GPIO->OUT ^= (1 << 17);

        if(!started_bro_sca)
		{
			NRF_LOG_INFO("start scan\r\n");
			first_time 				= true;
			NRF_EGU3->INTENSET 		= EGU_INTENSET_TRIGGERED1_Msk;
			started_bro_sca 		= true;
		 }else
		 {
			NRF_LOG_INFO("stop broadcast and scan\r\n");
			NRF_EGU3->INTENCLR 		= EGU_INTENCLR_TRIGGERED1_Msk;

			if(want_scan)
			{
				err_code = sd_ble_gap_adv_stop();
				APP_ERROR_CHECK(err_code);
			}else
			{
				err_code = sd_ble_gap_scan_stop();
				APP_ERROR_CHECK(err_code);
			}
			started_bro_sca = false;
		 }
    }

    if (NRF_GPIOTE->EVENTS_IN[3] != 0)		// shift
    {
    	nrf_delay_us(200000);
        NRF_GPIOTE->EVENTS_IN[3] = 0;

        manual_init();
    }
}
// TODO: 去掉log
// TODO: 去掉button，去掉delay，去掉了就不能manual init 了
static void gpio_configure(void)
{
	NRF_GPIO->DIRSET = LEDS_MASK; // set register
	NRF_GPIO->OUTSET = LEDS_MASK; // clear register

	NRF_GPIO->PIN_CNF[BUTTON_1] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	NRF_GPIO->PIN_CNF[BUTTON_2] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	NRF_GPIO->PIN_CNF[BUTTON_3] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	NRF_GPIO->PIN_CNF[BUTTON_4] = (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)   |
								  (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
								  (GPIO_PIN_CNF_PULL_Pullup   << GPIO_PIN_CNF_PULL_Pos);

	nrf_delay_us(5000);																			// Do I have to delay?

	NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Task       << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_High    << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
							(19                            << GPIOTE_CONFIG_PSEL_Pos);			// 19 is the pin number for testing

	NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_1                      << GPIOTE_CONFIG_PSEL_Pos);

	NRF_GPIOTE->CONFIG[2] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_2                      << GPIOTE_CONFIG_PSEL_Pos);

	// 自己加的
	NRF_GPIOTE->CONFIG[3] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_3                      << GPIOTE_CONFIG_PSEL_Pos);

	NRF_GPIOTE->CONFIG[4] = (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos)     |
							(GPIOTE_CONFIG_OUTINIT_Low     << GPIOTE_CONFIG_OUTINIT_Pos)  |
							(GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
							(BUTTON_4                      << GPIOTE_CONFIG_PSEL_Pos);

	// Interrupt
	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Msk | GPIOTE_INTENSET_IN2_Msk | GPIOTE_INTENSET_IN3_Msk | GPIOTE_INTENSET_IN4_Msk;
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOWEST);
	NVIC_EnableIRQ(GPIOTE_IRQn);
}


void relay_init(void)
{
	// RTC1
	NRF_RTC2->TASKS_STOP  = 1;
	NRF_RTC2->TASKS_CLEAR = 1;
	NRF_RTC2->PRESCALER   = 0; // 24-bit COUNTER // 12 bit prescaler for COUNTER frequency (32768/(PRESCALER+1))
	NRF_RTC2->CC[0]       = (0x008000);
	NRF_RTC2->EVENTS_COMPARE[0] = 0;
	//NRF_RTC2->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk; // 让event_compare register达到cc的值就清零
	NRF_RTC2->TASKS_START = 1;
	NRF_LOG_INFO("count1 started\r\n");
	NRF_RTC2->EVTENSET	  = RTC_INTENSET_COMPARE0_Msk;

	NRF_PPI->CHENCLR      = (1 << 5);
	NRF_PPI->CH[5].EEP = (uint32_t) &NRF_RTC2->EVENTS_COMPARE[0];
	NRF_PPI->CH[5].TEP = (uint32_t) &NRF_EGU3->TASKS_TRIGGER[1];
	NRF_PPI->CHENSET   = PPI_CHENSET_CH5_Msk; 							// enable

	NVIC_ClearPendingIRQ(SWI3_EGU3_IRQn);
	NVIC_SetPriority(SWI3_EGU3_IRQn, 7);
	NVIC_EnableIRQ(SWI3_EGU3_IRQn);

	NRF_GPIO->OUT ^= (1 << 17);
	init_storage();
	//first_time 				= true;
	//NRF_EGU3->INTENSET 		= EGU_INTENSET_TRIGGERED1_Msk;
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("###################### System Started ####################\r\n");
    ble_stack_init();
    err_code = sd_ble_gap_tx_power_set(-20); // accepted values are -40, -30, -20, -16, -12, -8, -4, 0, 3, and 4 dBm
    APP_ERROR_CHECK(err_code);
    gpio_configure();
    relay_init();

    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
