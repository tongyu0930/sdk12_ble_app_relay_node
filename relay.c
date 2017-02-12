


#include <stdlib.h>
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"
#include "relay.h"


const uint8_t 											SELF_DEVICE_NUMBER  = 0x05;
extern volatile bool 									first_time;
volatile bool 											want_scan 		 	= false;
volatile bool 											normal_mode 		= true;
volatile bool 											explore		 		= false;
volatile uint8_t 										explore_break_count		= false;
volatile bool 											explore_break		 		= false;
static uint8_t											self_device_level 	= 200;
static uint8_t											time_count 	        = 0;
//static bool											scan_only_mode 		= true;
static uint8_t 											count				= 2;
static uint8_t											self_event_number	= 1;
static const uint8_t 									init[12] 			= {0x0b,0xff,0,0,0,0,0,0,0,0,0,0};


static uint8_t	in_ALARM_NUMBER							= 0;
static uint8_t	in_this_device_tx_success				= 0;
static uint8_t	in_this_event_tx_success				= 0;
static uint8_t	in_alarm_device_number					= 0;
static uint8_t	in_alarm_rssi							= 0;
static uint8_t	in_first_listener_device_number			= 0;
static uint8_t	in_device_number						= 0;
static uint8_t	in_device_level							= 0;
static uint8_t	in_event_number							= 0;
static uint8_t	in_explore_mode							= 0;





struct storage * 		storage;
struct storage * 		event_pointer;
struct storage * 		event_pointer2;
struct storage
{
			uint8_t 			data[12];
	struct 	storage * 			next_storage;
};



void advertising_start(void);
void scanning_start(void);
void check_storage(uint8_t input_device_number, uint8_t input_event_number);
void check_storage_self_event(uint8_t input_self_event_number);
void free_memory(uint8_t input_event);




void SWI3_EGU3_IRQHandler(void)											// it is used to shut down thoses PPIs that is used for clear timer2's offset
{
	uint32_t err_code;

    if (NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
        {
            NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
            (void) NRF_EGU3->EVENTS_TRIGGERED[1];


            if(!explore && explore_break)
            {
            	explore_break_count++;

            	if(explore_break_count >= 40)
            	{
            		time_count = 0;
            		explore_break_count = 0;
            		explore_break = false;
            		NRF_LOG_INFO("explore mode ready\r\n");
            	}
            }


            if(count%2)			// 就是说scan的时间是不变的，idle 或 broadcast 的时间会有所减小
            {
            	if(normal_mode) { NRF_TIMER2->CC[0] = (0xFFFF) - (rand()%10000); }
            			   else { NRF_TIMER2->CC[0] = (0x7FFF) - (rand()%10000); }
            }

            count++;

            if(count == 200) { count = 2; }

			if(first_time)
			{
				scanning_start();
				first_time 	= false;
				want_scan 	= false;
			}else
			{
				if(want_scan)
				{
					if(!normal_mode)
					{
						err_code = sd_ble_gap_adv_stop();
						APP_ERROR_CHECK(err_code);
					}

					scanning_start();

					//NRF_LOG_INFO("scanning\r\n");
					want_scan = false;

				}else
				{
					err_code = sd_ble_gap_scan_stop();
					APP_ERROR_CHECK(err_code);

					if(storage->next_storage == NULL) { normal_mode = true;  }
												 else { normal_mode = false; }

					if(normal_mode)
					{
						want_scan = true;
						//NRF_LOG_INFO("normal mode\r\n");
						return;
					}else
					{
						if(explore) // for exlpore
						{
							time_count++;

							if(time_count >= 12)
							{
								explore = false;
								NRF_LOG_INFO("explore mode off\r\n");

								free_memory(0);

								if(self_event_number == 200)
								{
									self_event_number = 1;
								}

								check_storage(0,0); // 0号device肯定没有，所以会返回最后一个event的指针

								struct storage * new_storage 	= (struct storage *)calloc(1, sizeof(struct storage));
								new_storage->next_storage = NULL;
								memcpy(new_storage->data, init, sizeof(init));

								new_storage->data[5] = SELF_DEVICE_NUMBER;
								new_storage->data[6] = self_device_level;
								new_storage->data[8] = SELF_DEVICE_NUMBER;
								new_storage->data[9] = self_device_level;
								new_storage->data[10] = self_event_number++;


								event_pointer->next_storage = new_storage;
							}
						}

						sd_ble_gap_adv_data_set(storage->next_storage->data, sizeof(storage->next_storage->data), NULL, 0);
						advertising_start();
						want_scan = true;
						//NRF_LOG_INFO("broadcasting\r\n");
					}
				}
			}

        }
}


void init_dynamic_storage(void)
{
	storage 					= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(storage->data, init, sizeof(init));
	storage->next_storage 		= NULL;
}


void check_storage(uint8_t input_device_number, uint8_t input_event_number)	// function for checking if the device in the dynamic storage
{
	struct storage* temporary_pointer = storage;
	struct storage* temporary_pointer_before;

	temporary_pointer_before = temporary_pointer;

	while((temporary_pointer != NULL) && ((temporary_pointer->data[3] != input_device_number) || (temporary_pointer->data[4] != input_event_number)))
	{
		temporary_pointer_before = temporary_pointer;
		temporary_pointer = temporary_pointer->next_storage;						// 子列表里第一行里有device number，那就行了
	}

	event_pointer = temporary_pointer_before;

	return;
}


void check_storage_self_event(uint8_t input_self_event_number)	// function for checking if the device in the dynamic storage
{
	struct storage* temporary_pointer = storage;
	struct storage* temporary_pointer_before;

	temporary_pointer_before = temporary_pointer;

	while((temporary_pointer != NULL) && (temporary_pointer->data[10] != input_self_event_number))
	{
		temporary_pointer_before = temporary_pointer;
		temporary_pointer = temporary_pointer->next_storage;						// 子列表里第一行里有device number，那就行了
	}

	event_pointer = temporary_pointer_before;

	return;
}


void free_memory(uint8_t input_event)
{
	check_storage_self_event(input_event);

	if(event_pointer->next_storage != NULL) // 如果这个event确实存在
	{
		if(event_pointer->next_storage->next_storage == NULL) //  如果这个event为它所在的list的最后一个
		{
			free(event_pointer->next_storage);
			event_pointer->next_storage = NULL;	// 补上NULL
			NRF_LOG_INFO("last event %d freed\r\n", input_event);
		}else
		{
			event_pointer2 = event_pointer->next_storage->next_storage;
			free(event_pointer->next_storage);
			event_pointer->next_storage = event_pointer2; // 接上
			NRF_LOG_INFO("middle event %d freed\r\n", input_event);
		}
	}else
	{
		NRF_LOG_INFO("no this event\r\n");
	}

	return;
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

				uint8_t in_data[12];

				in_data[0] 	= field_length; // 这两句话有没有都行把
				in_data[1] 	= field_type;

				//while(a <= (index+field_length))
				while(a <= (index + 11)) // 这地方应该只能一个一个copy，不能用memcopy，因为p_data is not the same length as in_data
				{
					in_data[b] = p_data[a];
					//NRF_LOG_INFO("in_data = %x\r\n", in_data[a]);
					a++;
					b++;
				}
/************************************************ input data *********************************************************************/

				in_ALARM_NUMBER					= in_data[2]; 	// faller发出的alarm就是自己的编号，这个位置relay node 不要用。
				in_this_device_tx_success		= in_data[3];	// 作为input，如果自己的层级低，那就check这个值
				in_this_event_tx_success		= in_data[4];	// this device's this event can free();
				in_alarm_device_number			= in_data[5];	// who was falled
				in_alarm_rssi					= in_data[6];	// 如果自己亲身听见了alarm，这个in值为0，out自己听到的alarm的rssi，要不然就转发packet里的值
				in_first_listener_device_number	= in_data[7];
				in_device_number				= in_data[8];	// for both peer and self.
				in_device_level					= in_data[9];	// for both peer and self.
				in_event_number					= in_data[10];	// for both peer and self. as input, just copy to this_event_tx_success. as out put当变换广播内容时就＋1，需要有个广播列表，准别好了新的packet就copy过去。
				in_explore_mode					= in_data[11];  // 0: off. 1: on.

				//NRF_LOG_INFO("in_data = %x\r\n", in_data[3]);

/************************************************** 分层+汇报 机制 *******************************************************************************************/

				if(in_explore_mode && (time_count ==0))	// 只要收到一次 就开启explor mode， 为了防止你自己到时间停止了，然后别人还没停止，还继续传播explore指令，这样你听到指令又进入explore模式，没完没了了。
				{
					explore = true;
					explore_break = true;
				}

				if(explore)
				{
					if((in_device_level < self_device_level) && ((p_adv_report->rssi + 100) > 60)) 		// center is level 1
					{
						self_device_level = in_device_level + 1;
					}

					if(time_count == 0)
					{
						struct storage * new_storage 	= (struct storage *)calloc(1, sizeof(struct storage));
						event_pointer2 = storage->next_storage;
						new_storage->next_storage = event_pointer2;
						memcpy(new_storage->data, init, sizeof(init));

						new_storage->data[11] = 1;
						new_storage->data[10] = 0;		// self event number = 0

						storage->next_storage = new_storage;

						time_count++;

						NRF_LOG_INFO("explore packet created\r\n");
					}

					return;
				}

				// explore mode 不会进入下面！！！

/************************************************ free memory *******************************************************************************************/

				if(in_this_device_tx_success == SELF_DEVICE_NUMBER)	// 你存储的数据里的event number都是你自己生成的，所以不会重复。
				{
					free_memory(in_this_event_tx_success);
					return;
				}

/************************************************ 过滤机制 *******************************************************************************************/

				if(in_device_level <= self_device_level)
				{
					return;
				}

/************************************************ 储存机制 *******************************************************************************************/

// 如果你收到一个新event，那么device number和event number都被放到in_this_device_tx_success 和 in_this_event_tx_success 里了，然后储存起来，所以这个confirm就是packet来过的证据，就可以判断是不是扫描到了重复packet

				if(self_event_number == 200)
				{
					self_event_number = 1;
				}

				check_storage(in_device_number, in_event_number); // 搜寻目标其实是in_this_device_tx_success，因为你把in_device_number转到那里了

				if(event_pointer->next_storage == NULL)
				{

					struct storage * new_storage 	= (struct storage *)calloc(1, sizeof(struct storage));
					new_storage->next_storage = NULL;
					memcpy(new_storage->data, in_data, sizeof(in_data));

					new_storage->data[3] = in_device_number;
					new_storage->data[4] = in_event_number;

					new_storage->data[8] = SELF_DEVICE_NUMBER;
					new_storage->data[9] = self_device_level;
					new_storage->data[10] = self_event_number++;
					new_storage->data[11] = 0;

					if(in_ALARM_NUMBER != 0) // if in_ALARM_NUMBER== 0, then do nothing, because you already copied 0 to output data.
					{
						new_storage->data[2] = 0;
						new_storage->data[3] = in_ALARM_NUMBER; // alarm node 不用check in_data4，所以说data4的值是多少无所谓。
						new_storage->data[5] = in_ALARM_NUMBER;
						new_storage->data[6] = p_adv_report->rssi + 100;
						new_storage->data[7] = SELF_DEVICE_NUMBER;
					}

					event_pointer->next_storage = new_storage; 				// 接起来

					NRF_LOG_INFO("NEW!!! device: %d and event: %d self_event: %d\r\n", in_device_number, in_event_number, self_event_number-1);

				}else
				{
					NRF_LOG_INFO("duplicated device: %d and event: %d\r\n", in_device_number, in_event_number);
				}


				return;
			}

			index += field_length + 1;
	    }
}



