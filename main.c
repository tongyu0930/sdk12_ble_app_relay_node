/*
 * main.c
 *
 *  Author: Tong Yu
 */

// 最后在删减makefile
#include "softdevice_handler.h"
#include "boards.h"
#include "relay.h"


#define CENTRAL_LINK_COUNT       		0  			/**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT    		1  			/**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0 			/**< 不懂Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define DEAD_BEEF                       0xDEADBEEF  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


volatile bool 							first_time					= true;
extern volatile uint8_t 				self_level;
extern volatile bool					scan_only_mode;


const ble_gap_adv_params_t m_adv_params =
  {
	.type        					= BLE_GAP_ADV_TYPE_ADV_IND,					// Undirected advertisement.
	.p_peer_addr					= NULL,												// 我觉得null是不是默认就是static的address？
	.fp          					= BLE_GAP_ADV_FP_ANY,
	.interval    					= 0x0020,						// 虽然这个最小值时100ms，但是你可以通过timer以更快的频率启动关闭广播。
	.timeout     					= 0
  };

const ble_gap_adv_params_t m_adv_params2 =
  {
	.type        					= BLE_GAP_ADV_TYPE_ADV_NONCONN_IND,					// Undirected advertisement.
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
    .interval    = 0x4000,
    .window      = 0x2000,
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
static void relay_init(void);


void advertising_start(void)
{
    if(self_level == 2)
    {
    	sd_ble_gap_adv_start(&m_adv_params);
    }else
    {
    	sd_ble_gap_adv_start(&m_adv_params2);
    }
}

void scanning_start(void)
{
	if(scan_only_mode)
	{
		sd_ble_gap_scan_start(&m_scan_params);
	}else
	{
		sd_ble_gap_scan_start(&m_scan_params2);
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
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL); 													// Initialize the SoftDevice handler module.
    ble_enable_params_t ble_enable_params;
    softdevice_enable_get_default_config(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT, &ble_enable_params);
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);									//Check the ram settings against the used number of links
    softdevice_enable(&ble_enable_params); 												// Enable BLE stack.
    softdevice_ble_evt_handler_set(ble_evt_dispatch); 									// Register with the SoftDevice handler module for BLE events.
}


/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    sd_app_evt_wait();
}


static void relay_init(void)
{
	NRF_RTC2->TASKS_STOP  = 1;
	NRF_RTC2->TASKS_CLEAR = 1;
	NRF_RTC2->PRESCALER   = 0; // 24-bit COUNTER // 12 bit prescaler for COUNTER frequency (32768/(PRESCALER+1))
	NRF_RTC2->CC[0]       = (0x008000);
	NRF_RTC2->EVENTS_COMPARE[0] = 0;
	NRF_RTC2->TASKS_START = 1;
	NRF_RTC2->EVTENSET	  = RTC_INTENSET_COMPARE0_Msk;

	NRF_PPI->CHENCLR      = (1 << 5);
	NRF_PPI->CH[5].EEP = (uint32_t) &NRF_RTC2->EVENTS_COMPARE[0];
	NRF_PPI->CH[5].TEP = (uint32_t) &NRF_EGU3->TASKS_TRIGGER[1];
	NRF_PPI->CHENSET   = PPI_CHENSET_CH5_Msk; 							// enable

	NVIC_ClearPendingIRQ(SWI3_EGU3_IRQn);
	NVIC_SetPriority(SWI3_EGU3_IRQn, 7);
	NVIC_EnableIRQ(SWI3_EGU3_IRQn);

	init_storage();
	first_time 				= true;
	sd_ble_gap_tx_power_set(-0); // accepted values are -40, -30, -20, -16, -12, -8, -4, 0, 3, and 4 dBm
	NRF_EGU3->INTENSET 		= EGU_INTENSET_TRIGGERED1_Msk;
}


int main(void)
{
    ble_stack_init();
    relay_init();

    for (;; )
    {
    	power_manage();
    }
}

