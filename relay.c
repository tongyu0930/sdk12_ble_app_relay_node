


#include <stdlib.h>
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"
#include "relay.h"


const uint8_t 											SELF_DEVICE_NUMBER  = 1;

extern volatile bool 									first_time;
volatile bool 											want_scan 		 	= false;
static uint8_t											self_device_level 	= 0;
static bool												scan_only_mode 		= true;
static uint8_t 											count				= 2;


static uint8_t	in_device_number			= 0;
static uint8_t	in_device_level				= 0;
static uint8_t	in_alarm_rssi				= 0;
static uint8_t	in_this_event_tx_success		= 0;
static uint8_t	in_alarm_device_number		= 0;
static uint8_t	in_alarm_running			= 0;
static uint8_t	in_this_device_tx_success			= 0;
static uint8_t	in_who_found_the_alarm		= 0;
static uint8_t	in_packet_edition			= 0;

static uint8_t	out_device_number			= 0;
static uint8_t	out_device_level			= 0;
static uint8_t	out_alarm_rssi				= 0;
static uint8_t	out_this_event_tx_success		= 0;
static uint8_t	out_alarm_device_number		= 0;
//static uint8_t	out_alarm_running			= 0;
static uint8_t	out_this_device_tx_success		= 0;
static uint8_t	out_who_found_the_alarm		= 0;
static uint8_t	out_packet_edition			= 0;

//struct device_storage * device_foot;
static struct device_storage * head;

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
    BACK_TO_SCAN_ONLY,		 		/* Default state */
    NORMAL_FORWARD,   		/* Waiting for packets */
    ALARM_FORWARD   	 	/* Trying to transmit packet */
} mode = BACK_TO_SCAN_ONLY;





void advertising_start(void);
void scanning_start(void);
struct device_stroage* check_device_number(uint8_t input);




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

void create_dynamic_storage(void)
{
	head 								= (struct device_storage *)calloc(1, sizeof(struct device_storage));
	memset(&head, 0, sizeof(head));
	head->next_device_storage 			= NULL;
	head->event->next_event_storage 	= NULL;
	//device_foot 						= head;
}

struct device_stroage* check_device_number(uint8_t input)
{
	struct device_storage* temporary_event_pointer1;
	//memset(&temporary_event_pointer1, 0, sizeof(temporary_event_pointer1));
	temporary_event_pointer1 = head;

	while((temporary_event_pointer1 != NULL) && (temporary_event_pointer1->event->data[6] != input))
	{
		temporary_event_pointer1 = temporary_event_pointer1->next_device_storage;
	}
	if(temporary_event_pointer1 == NULL)
	{
		return (NULL);
	}else
	{
		return (struct device_stroage*) temporary_event_pointer1;
	}
}


void get_adv_data(ble_evt_t * p_ble_evt) // 就全写在这里，最后在拆开，二进制那方法也是
{
	uint32_t index = 0;

	struct device_storage* 	device_pointer;
	struct event_storage* 	temporary_event_pointer;
	memset(&device_pointer, 0, sizeof(device_pointer));
	memset(&temporary_event_pointer, 0, sizeof(temporary_event_pointer));

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

				while(a <= (index+field_length)) // 这地方应该只能一个一个copy，不能用memcopy，因为p_data is not the same length as in_data
				{
					in_data[b] = p_data[a];
					//NRF_LOG_INFO("in_data = %x\r\n", in_data[a]);
					a++;
					b++;
				}
/************************************************ input data *********************************************************************/

				in_alarm_device_number			= in_data[2];
				in_alarm_running				= in_data[3];
				in_this_device_tx_success		= in_data[4];
				in_this_event_tx_success		= in_data[8];
				in_alarm_rssi					= in_data[5];
				in_device_number				= in_data[6];		// 也可以直接用 p_data[a+1]，这样是为了好识别
				in_device_level					= in_data[7];
				in_who_found_the_alarm			= in_data[9];
				in_packet_edition				= in_data[10];

/************************************************ 检测是否为重复packet *********************************************************************/


				memcpy(device_pointer, check_device_number(in_device_number), sizeof(check_device_number(in_device_number)));
				//device_pointer = check_device_number(in_data[6]); // this funtion return 这个 device 的指针

				if(device_pointer == NULL) // 说明检测到最后也没发现 create new device
				{
					struct device_storage *new_device_storage = (struct device_storage *)calloc(1, sizeof(struct device_storage));
					new_device_storage->next_device_storage = NULL;
					new_device_storage->event->next_event_storage = NULL;
					memcpy(new_device_storage->event->data, in_data, sizeof(in_data));

					device_pointer = new_device_storage;
					//device_foot->next_device_storage = new_device_storage; // 以前的foot连接到新成员上，新成员就成为foot了，以前的foot就是倒数第二个
					//device_foot = new_device_storage;
				}else
				{
					temporary_event_pointer = device_pointer->event;

					while((temporary_event_pointer->next_event_storage != NULL) && (temporary_event_pointer->data[2] != in_alarm_device_number))
						{
							temporary_event_pointer = temporary_event_pointer->next_event_storage;
						}

						if(temporary_event_pointer == NULL) // create new event // 此时temporary_event_pointer 就等于“event_foot”->next_event_storage
						{
							struct event_storage* new_event_storage = (struct event_storage *)calloc(1, sizeof(struct event_storage));
							new_event_storage->next_event_storage = NULL;
							memcpy(new_event_storage->data, in_data, sizeof(in_data));

							temporary_event_pointer = new_event_storage; // 接起来
						}else
						{
							return; // 重复的packet
						}
				}

/************************************************ 开始判断 *********************************************************************/

				// 一次扫描区间会扫描到来自不同node的信息，怎么样能把它们记录下来判断？

				if(in_alarm_device_number != 0)	// 不等于零就说明有moving node在广播
				{
					mode = ALARM_FORWARD;
				}else
				{
					if(in_this_device_tx_success == SELF_DEVICE_NUMBER)	// 如果收到确认信息，那就回到扫描模式  // moving node里要加上相同的if语句
					{													// 然后在看那个packet
						mode = NORMAL_FORWARD;
					}else
					{
						//if((in_device_level >= self_device_level) || (maybe_i_need_relay>=6)) // 如果收到相同sender的相同的信息超过6次，那就帮他转播
						if(in_device_level >= self_device_level)
						{
							mode = NORMAL_FORWARD;
						}
					}
				}



				switch(mode)
				    {
				        case BACK_TO_SCAN_ONLY:
				        	// TODO: stop broadcast and change to normal mode
				        	scan_only_mode = true;
				        	return;

				        case NORMAL_FORWARD:
				        	out_alarm_rssi 				= in_alarm_rssi; // 如果自己没能直接扫描到alarm，那就转发别人测得的rssi
				        	out_who_found_the_alarm 	= in_who_found_the_alarm;
				        	out_this_device_tx_success 	= in_device_number;
				        	out_alarm_device_number 	= in_alarm_device_number;
				            break;

				        case ALARM_FORWARD:
				        	out_alarm_rssi 				= p_adv_report->rssi;
				        	out_who_found_the_alarm 	= SELF_DEVICE_NUMBER;
				        	out_this_device_tx_success 	= in_alarm_device_number;
				        	out_alarm_device_number 	= in_alarm_device_number;
							break;
				    }



				out_device_level  			= self_device_level;
				out_device_number			= SELF_DEVICE_NUMBER;

				out_packet_edition++;
				if(out_packet_edition == 200)
				{
					out_packet_edition = 0;
				}


/************************************************ out_data *********************************************************************/
				out_data[0] = field_length;
				out_data[1] = field_type;

				out_data[2] = out_alarm_device_number;	 	// moving node 只需要read/write这两行
				//out_data[3] = out_alarm_running;
				out_data[4] = out_this_device_tx_success; 	// 这个里面含有需要停止广播的device的名字

				out_data[5] = out_alarm_rssi;
				out_data[6] = out_device_number;
				out_data[7] = out_device_level;				// center 的 level是1
				out_data[8] = out_this_event_tx_success; 	// 但你收到别人的relay是再设置这个值。
				out_data[9] = out_who_found_the_alarm;

				out_data[10]= out_packet_edition;


				sd_ble_gap_adv_data_set(out_data, sizeof(out_data), NULL, 0);



				return;
			}
			index += field_length + 1;
	    }
}
