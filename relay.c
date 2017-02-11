


#include <stdlib.h>
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"
#include "relay.h"


const uint8_t 											SELF_DEVICE_NUMBER  = 0x05;
extern volatile bool 									first_time;
volatile bool 											want_scan 		 	= false;
static uint8_t											self_device_level 	= 0;
//static bool											scan_only_mode 		= true;
static uint8_t 											count				= 2;
static uint8_t											self_event_number	= 1;
static const uint8_t 									init[12] 			= {0,0,0,0,0,0,0,0,0,0,0,0};


static uint8_t	in_ALARM_NUMBER							= 0;
static uint8_t	in_this_device_tx_success				= 0;
static uint8_t	in_this_event_tx_success				= 0;
static uint8_t	in_alarm_device_number					= 0;
static uint8_t	in_alarm_rssi							= 0;
static uint8_t	in_first_listener_device_number			= 0;
static uint8_t	in_device_number						= 0;
static uint8_t	in_device_level							= 0;
static uint8_t	in_event_number							= 0;
static uint8_t	in_target_level							= 0;





struct device_storage * 	storage;
struct device_storage * 	device_pointer;
struct device_storage * 	device_pointer2;
struct event_storage * 		event_pointer;
struct event_storage * 		event_pointer2;

struct event_storage									// 子链表
{
			uint8_t 			data[12];
	struct 	event_storage * 	next_event_storage;
};

struct device_storage									// 链表
{
	struct event_storage * 		event;
	struct device_storage * 	next_device_storage;
};



static enum
{
	NORMAL_FORWARD,
    OUTSIDE_FORWARD,
    PARALLEL_FORWARD,
	ANY_FORWARD
} mode = NORMAL_FORWARD;





void advertising_start(void);
void scanning_start(void);
void check_device_number(uint8_t input);
void check_event_number(uint8_t input1);
void event_search(uint8_t input1);




void SWI3_EGU3_IRQHandler(void)											// it is used to shut down thoses PPIs that is used for clear timer2's offset
{
	uint32_t err_code;

    if (NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
        {
            NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
            (void) NRF_EGU3->EVENTS_TRIGGERED[1];

            if(count%2)
            {
            	NRF_TIMER2->CC[0] = (0xFFFF) - (rand()%10000) ;
            }

            count++;

            if(count == 200)
            {
            	count = 2;
            }



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

					//NRF_LOG_INFO("scanning\r\n");
					want_scan = false;

				}else
				{
					err_code = sd_ble_gap_scan_stop();
					APP_ERROR_CHECK(err_code);

					advertising_start();

					//NRF_LOG_INFO("broadcasting\r\n");
					want_scan = true;
				}
			}

        }
}


void init_dynamic_storage(void)
{
	storage 									= (struct device_storage *)calloc(1, sizeof(struct device_storage));
	struct event_storage * new_event_storage	= (struct event_storage *)calloc(1, sizeof(struct event_storage));
	memcpy(new_event_storage, init, sizeof(init));
	new_event_storage->next_event_storage 		= NULL;
	storage->next_device_storage 				= NULL;
	storage->event 								= new_event_storage;
}


void check_device_number(uint8_t input)	// function for checking if the device in the dynamic storage
{
	struct device_storage* temporary_pointer = storage;
	struct device_storage* temporary_pointer_before;
	//memset(&temporary_pointer_before, 0, sizeof(temporary_pointer_before));

	temporary_pointer_before = temporary_pointer;

	while((temporary_pointer != NULL) && (temporary_pointer->event->data[3] != input))  // in_data[3] is in_this_device_tx_success
	{
		temporary_pointer_before = temporary_pointer;
		temporary_pointer = temporary_pointer->next_device_storage;						// 子列表里第一行里有device number，那就行了
	}

	device_pointer = temporary_pointer_before;

	return;
}


void check_event_number(uint8_t input1)
{
	struct event_storage* temporary_event_pointer = device_pointer->next_device_storage->event;
	struct event_storage* temporary_event_pointer_before;
	//memset(&temporary_event_pointer_before, 0, sizeof(temporary_event_pointer_before));

	temporary_event_pointer_before = temporary_event_pointer;


	while((temporary_event_pointer != NULL) && (temporary_event_pointer->data[4] != input1))
	{
		temporary_event_pointer_before = temporary_event_pointer;
		temporary_event_pointer = temporary_event_pointer->next_event_storage;
	}

	event_pointer = temporary_event_pointer_before;

	return;
}


void event_search(uint8_t input1)
{
	struct device_storage* temporary_pointer = storage;
	struct device_storage* temporary_pointer_before;
	struct event_storage* temporary_event_pointer = storage->event;
	struct event_storage* temporary_event_pointer_before;
	//memset(&temporary_event_pointer_before, 0, sizeof(temporary_event_pointer_before));

	temporary_pointer_before = temporary_pointer;
	temporary_event_pointer_before = temporary_event_pointer;

	while((temporary_pointer != NULL) && (temporary_event_pointer->data[10] != input1))  // in_data[3] is in_this_device_tx_success
	{
		temporary_event_pointer = temporary_pointer->event;

		while((temporary_event_pointer != NULL) && (temporary_event_pointer->data[10] != input1))
		{
			temporary_event_pointer_before = temporary_event_pointer; //temporary_event_pointer_before 不会是结尾，因为temporary_event_pointer不会停在另一个device的开头的话
			temporary_event_pointer = temporary_event_pointer->next_event_storage;
		}

		if(temporary_event_pointer == NULL)
		{
			temporary_pointer_before = temporary_pointer;
			temporary_pointer = temporary_pointer->next_device_storage;
		}

	}

	device_pointer = temporary_pointer_before;
	event_pointer = temporary_event_pointer_before;

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

				while(a <= (index+field_length)) // 这地方应该只能一个一个copy，不能用memcopy，因为p_data is not the same length as in_data
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
				in_event_number					= in_data[10];	//  for both peer and self. as input, just copy to this_event_tx_success. as out put当变换广播内容时就＋1，需要有个广播列表，准别好了新的packet就copy过去。
				in_target_level					= in_data[11];  // 1: to center. 2: to outside. 3: to all. 0: to same level.

				//NRF_LOG_INFO("in_data = %x\r\n", in_data[3]);




/************************************************ free memory *******************************************************************************************/

				if(in_this_device_tx_success == SELF_DEVICE_NUMBER)	// 你存储的数据里的event number都是你自己生成的，所以不会重复。
				{
					event_search(in_this_event_tx_success);

					if(event_pointer->next_event_storage != NULL) // 如果这个event确实存在
					{
						if(event_pointer->next_event_storage->next_event_storage == NULL) //  如果这个event为它所在的list的最后一个
						{
							free(event_pointer->next_event_storage);
							event_pointer->next_event_storage = NULL;	// 补上NULL
							NRF_LOG_INFO("last event %d freed\r\n", in_this_event_tx_success);
						}else
						{
							event_pointer2 = event_pointer->next_event_storage->next_event_storage;
							free(event_pointer->next_event_storage);
							event_pointer->next_event_storage = event_pointer2; // 接上
							NRF_LOG_INFO("middle event %d freed\r\n", in_this_event_tx_success);
						}


						if(device_pointer->next_device_storage->event->next_event_storage == NULL)	// 如果这个device的list空了
						{
							if(device_pointer->next_device_storage->next_device_storage == NULL)	// 如果这个device list 后面没有device了
							{
								NRF_LOG_INFO("last device %d freed\r\n", device_pointer->next_device_storage->event->data[3]);
								free(device_pointer->next_device_storage);
								device_pointer->next_device_storage = NULL; // 补上NULL
							}else
							{
								NRF_LOG_INFO("middle device %d freed\r\n", device_pointer->next_device_storage->event->data[3]);
								device_pointer2 = device_pointer->next_device_storage->next_device_storage;
								free(device_pointer->next_device_storage);
								device_pointer->next_device_storage = device_pointer2; // 接上
							}
						}
					}else
					{
						NRF_LOG_INFO("no this event\r\n");
					}

					return;
				}

/*
				switch(in_target_level)
				{
				case 0: // to same level and to center
					if(in_device_level < self_device_level)
					{
						return;
					}else
					{
						mode = NORMAL_FORWARD; // 你同level的人让你帮着relay，但你就不用在让你旁边同level的relay 了，所以还是normal mode
					}
					break;

				case 1: // to center
					if(in_device_level <= self_device_level)
					{
						return;
					}else
					{
						mode = NORMAL_FORWARD;
					}
					break;

				case 2: // to outside
					if(in_device_level >= self_device_level)
					{
						return;
					}else
					{
						mode = OUTSIDE_FORWARD;
					}
					break;

				case 3: // to all
					mode = ANY_FORWARD;
					break;
				}*/
/************************************************ 储存机制 *******************************************************************************************/

// 如果你收到一个新event，那么device number和event number都被放到in_this_device_tx_success 和 in_this_event_tx_success 里了，然后储存起来，所以这个confirm就是packet来过的证据，就可以判断是不是扫描到了重复packet

if(self_event_number == 200)
{
	self_event_number = 1;
}


				check_device_number(in_device_number); // 搜寻目标其实是in_this_device_tx_success，因为你把in_device_number转到那里了

				if(device_pointer->next_device_storage == NULL)
				{
					// create new device.
					struct device_storage * new_device_storage 	= (struct device_storage *)calloc(1, sizeof(struct device_storage));
					struct event_storage * new_event_storage	= (struct event_storage *)calloc(1, sizeof(struct event_storage));
					struct event_storage * new_event_storage_0	= (struct event_storage *)calloc(1, sizeof(struct event_storage));	//每个新device最初都接着一个空event，为了防止temporary_event_pointer_before找不到地址。
					new_device_storage->next_device_storage = NULL;
					new_event_storage->next_event_storage = NULL;
					memcpy(new_event_storage->data, in_data, sizeof(in_data));
					memcpy(new_event_storage_0, init, sizeof(init));
					new_event_storage_0->data[3] = in_device_number;

					if(in_ALARM_NUMBER != 0) // if == 0, then do nothing, because you already copied 0 to output data.
					{
						new_event_storage->data[2] = 0;
						new_event_storage->data[3] = in_ALARM_NUMBER; // alarm node 不用check in_data4，所以说data4的值是多少无所谓。
						new_event_storage->data[5] = in_ALARM_NUMBER;
						new_event_storage->data[6] = p_adv_report->rssi;
						new_event_storage->data[7] = SELF_DEVICE_NUMBER;
					}
					new_event_storage->data[3] = in_device_number;
					new_event_storage->data[4] = in_event_number;

					new_event_storage->data[8] = SELF_DEVICE_NUMBER;
					new_event_storage->data[9] = self_device_level;
					new_event_storage->data[10] = self_event_number++;
					new_event_storage->data[11] = 1;

					new_event_storage_0->next_event_storage = new_event_storage;					// 接起来
					new_device_storage->event = new_event_storage_0;			 					 // 接起来
					device_pointer->next_device_storage 		= new_device_storage; 				// 接起来
					//device_pointer->next_device_storage->event 	= new_event_storage;			 	 // 接起来

					NRF_LOG_INFO("new device: %d and new event: %d event: %d\r\n", in_device_number, in_event_number, self_event_number-1);

				}else
				{
					check_event_number(in_event_number);

					if(event_pointer->next_event_storage == NULL)
					{
						struct event_storage * new_event_storage	= (struct event_storage *)calloc(1, sizeof(struct event_storage));
						new_event_storage->next_event_storage = NULL;
						memcpy(new_event_storage->data, in_data, sizeof(in_data));

						if(in_ALARM_NUMBER != 0) // if == 0, then do nothing, because you already copied 0 to output data.
						{
							new_event_storage->data[2] = 0;
							new_event_storage->data[3] = in_ALARM_NUMBER; // alarm node 不用check in_data4，所以说data4的值是多少无所谓。
							new_event_storage->data[5] = in_ALARM_NUMBER;
							new_event_storage->data[6] = p_adv_report->rssi;
							new_event_storage->data[7] = SELF_DEVICE_NUMBER;
						}
						new_event_storage->data[3] = in_device_number;
						new_event_storage->data[4] = in_event_number;

						new_event_storage->data[8] = SELF_DEVICE_NUMBER;
						new_event_storage->data[9] = self_device_level;
						new_event_storage->data[10] = self_event_number++;
						new_event_storage->data[11] = 1;

						event_pointer->next_event_storage = new_event_storage; // 接起来

						NRF_LOG_INFO("old device: %d and new event: %d event: %d\r\n", in_device_number, in_event_number, self_event_number-1);

					}else
					{
						NRF_LOG_INFO("duplicated device: %d and event: %d\r\n", in_device_number, in_event_number);

						//return;
					}
				}





/************************************************ 开始判断 *******************************************************************************************************/

				switch(mode)
				{
					case NORMAL_FORWARD:

						break;

					case OUTSIDE_FORWARD:

						break;

					case PARALLEL_FORWARD:

						break;

					case ANY_FORWARD:

						break;
				}




/************************************************ out_data *********************************************************************/



				//sd_ble_gap_adv_data_set(in_data, sizeof(in_data), NULL, 0);



				//return;
			}
			index += field_length + 1;
	    }
}
