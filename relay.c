
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"

#define 	SELF_DEVICE_NUMBER 	1
extern volatile bool 									first_time;
volatile bool 											want_scan 		 	= false;
static uint8_t											self_device_level 	= 0;
static bool												need_broadcast 		= false;
static bool												alarm_mode 			= false;

static uint8_t	in_device_number			= 0;
static uint8_t	in_device_level				= 0;
static uint8_t	in_alarm_rssi				= 0;
static uint8_t	in_alarm_relay_success		= 0;
static uint8_t	in_alarm_number				= 0;
static uint8_t	in_alarm_running			= 0;
static uint8_t	in_alarm_tx_success			= 0;
static uint8_t	in_first_listener			= 0;

static uint8_t	out_device_number			= 0;
static uint8_t	out_device_level			= 0;
static uint8_t	out_alarm_rssi				= 0;
static uint8_t	out_alarm_relay_success		= 0;
static uint8_t	out_alarm_number			= 0;
static uint8_t	out_alarm_running			= 0;
static uint8_t	out_alarm_tx_success		= 0;
static uint8_t	out_first_listener			= 0;


void advertising_start(void);
void scanning_start(void);

void SWI3_EGU3_IRQHandler(void)											// it is used to shut down thoses PPIs that is used for clear timer2's offset
{
	uint32_t err_code;

    if (NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
        {
            NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
            (void) NRF_EGU3->EVENTS_TRIGGERED[1];

            //NRF_LOG_INFO("EGU[1]\r\n");

			if(first_time)
			{
				scanning_start();
				first_time 	= false;
				want_scan 	= false;
			}else
			{
				if(want_scan)
				{
					err_code = sd_ble_gap_adv_stop();
					APP_ERROR_CHECK(err_code);

					scanning_start();

					NRF_LOG_INFO("scanning\r\n");
					want_scan = false;

				}else
				{
					err_code = sd_ble_gap_scan_stop();
					APP_ERROR_CHECK(err_code);

					advertising_start();

					NRF_LOG_INFO("broadcasting\r\n");
					want_scan = true;
				}
			}

        }
}


void get_adv_data(ble_evt_t * p_ble_evt) // 就全写在这里，最后在拆开，二进制那方法也是
{
	uint32_t index = 0;

	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *)p_adv_report->data;

	while (index < p_adv_report->dlen)
	    {
	        uint8_t field_length = p_data[index];
	        uint8_t field_type   = p_data[index+1];

			if ( field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
			{
				uint8_t a = index+2;
				uint8_t b = 2;
				uint8_t c = field_length +1;

				uint8_t in_data[c];
				uint8_t out_data[c];

				in_data[0] 	= field_length; // 这两句话有没有都行把
				in_data[1] 	= field_type;

				while(a <= (index+field_length))
				{
					in_data[b] = p_data[a];
					NRF_LOG_INFO("p_data = %x\r\n", in_data[a]);
					a++;
					b++;
				}

				in_alarm_number					= in_data[2];
				in_alarm_running				= in_data[3];
				in_alarm_tx_success				= in_data[4];

				in_alarm_rssi					= in_data[5];
				in_device_number				= in_data[6];		// 也可以直接用 p_data[a+1]，这样是为了好识别
				in_device_level					= in_data[7];

				in_alarm_relay_success		 	= in_data[8];
				in_first_listener				= in_data[9];

/************************************************ 开始判断 *********************************************************************/



				// 一次扫描区间会扫描到来自不同node的信息，怎么样能把它们记录下来判断？

				if(in_alarm_running == 1)
				{
					out_alarm_rssi = p_adv_report->rssi;
					out_alarm_number = in_alarm_number;
					out_alarm_tx_success = 1;
					out_first_listener = SELF_DEVICE_NUMBER;
				}else
				{
					if(in_alarm_relay_success)
					{
						// TODO: stop broadcast and change to normal mode
						alarm_mode = false;
						return;
					}else
					{
						//if((in_device_level >= self_device_level) || (maybe_i_need_relay>=6)) // 如果收到相同sender的相同的信息超过6次，那就帮他转播
						if(in_device_level >= self_device_level)
						{
							need_broadcast = true;
							out_alarm_rssi = in_alarm_rssi; // 如果自己没能直接扫描到alarm，那就转发别人测得的rssi
							out_device_number = SELF_DEVICE_NUMBER;
							out_device_level  = self_device_level;
							out_alarm_relay_success = 1;
							out_first_listener = in_first_listener;
						}
					}
				}






/************************************************ 准备out_data *********************************************************************/
				out_data[0] = field_length;
				out_data[1] = field_type;
				out_data[2] = out_alarm_number;
				out_data[3] = out_alarm_running; // 这个位置是moving node的独有位置
				out_data[4] = out_alarm_tx_success;
				out_data[5] = out_alarm_rssi;
				out_data[6] = out_device_number;
				out_data[7] = out_device_level;
				out_data[8] = out_alarm_relay_success; // 但你收到别人的relay是再设置这个值。
				out_data[9] = out_first_listener;

				sd_ble_gap_adv_data_set(out_data, sizeof(out_data), NULL, 0);

				NRF_LOG_INFO("fdasfadsfafdas");

				alarm_mode = true;
				return;
			}
			index += field_length + 1;
	    }
}
