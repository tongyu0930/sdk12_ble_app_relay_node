
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"

extern volatile bool 									first_time;
extern volatile bool 									want_shift; 															// 不要赋值
volatile bool 											want_scan 		 = false;

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

            if(want_shift)
            {
            	want_shift = false;
            	return;

            }else
            {
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
}

void get_adv_data(ble_evt_t * p_ble_evt)
{
	uint32_t index = 0;
	//uint32_t err_code;

	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *)p_adv_report->data;

	while (index < p_adv_report->dlen)
	    {
	        uint8_t field_length = p_data[index];
	        uint8_t field_type   = p_data[index+1];

			if ( field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
			{
				uint8_t c = field_length +1;
				uint8_t pp_data[c];
				uint8_t a = index+2;
				uint8_t b = 2;

				pp_data[0] = field_length; // 你要relay的数据的长度和你扫描到的一样长
				pp_data[1] = field_type;

				while(a <= (index+field_length))
				{
					pp_data[b] = p_data[a];
					NRF_LOG_INFO("p_data = %x\r\n", p_data[a]);
					a++;
					b++;
				}

				sd_ble_gap_adv_data_set(pp_data, sizeof(pp_data), NULL, 0);

				NRF_LOG_INFO("rssi = %d\r\n", p_adv_report->rssi);
				//NRF_LOG_INFO("dlen = %d\r\n", p_adv_report->dlen); // 这个就是p_data（安卓手机上raw data）的length
			}
			index += field_length + 1;
	    }
}
