


#include <stdlib.h>
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"
#include "relay.h"


const 				uint8_t 				SELF_NUMBER  					= 0x05;
static 				uint8_t					self_level 						= 200;

extern volatile 	bool 					first_time;
volatile 			bool 					want_scan 		 				= false;
volatile 			bool 					scan_only_mode 					= true;		// broadcast list 里空时
volatile 			uint8_t 				init_break_count				= 0;
static 				uint8_t					init_time_count 	        	= 0;
static 				uint8_t 				loop							= 2;
static 				uint8_t 				delete_block_list_count			= 0;
static const 		uint8_t 				init[12] 						= {0x0b,0xff,'T','O','N','G',0,0,0,0,0,0};



static volatile enum
{
    INIT_MODE,
    NORMAL_MODE
} mode = NORMAL_MODE;

static volatile enum
{
    LOWER_LEVEL_NODE,
	HIGHER_LEVEL_NODE,
	SAME_LEVEL_NODE
} node_type;

static volatile enum
{
	BROADCAST_PACKET,
	BLOCK_PACKET
} creat_packet;


struct storage									// linked list
{
			uint8_t 			data[12];
	struct 	storage * 			next_storage;
};
struct storage * 		broadcast_list;
struct storage * 		block_list;
struct storage * 		packet_pointer;
struct storage * 		packet_pointer2;




void advertising_start(void);
void scanning_start(void);
void init_storage(void);
void check_list(struct storage * list_name, uint8_t value1, uint8_t input1, uint8_t value2, uint8_t input2, uint8_t value3, uint8_t input3);
void delete_packet(struct storage* input_packet);
void delete_block_list(void);

// TODO: 定时发送report：15mins
// TODO: center也要能检测alarm



void SWI3_EGU3_IRQHandler(void)
{
	uint32_t err_code;

    if (NRF_EGU3->EVENTS_TRIGGERED[1] != 0)
	{
		NRF_EGU3->EVENTS_TRIGGERED[1] = 0;
		(void) NRF_EGU3->EVENTS_TRIGGERED[1];

		/************************************************** short break after init mode ***********************************************************************/

		if(mode == NORMAL_MODE && init_break_count)
		{
			init_break_count++;

			if(init_break_count >= 20)
			{
				init_time_count = 0;
				init_break_count = 0;
				NRF_LOG_INFO("init mode ready\r\n");
			}
		}

		/************************************************** delete_block_list_count ******************************************************************************/

		if(block_list->next_storage != NULL)
		{
			delete_block_list_count++;
		}
		if(delete_block_list_count >= 20)
		{
			delete_block_list();
			delete_block_list_count = 0;
		}


		/************************************************** loop interval change ******************************************************************************/

		if(loop%2)			// 就是说scan的时间是不变的，idle 或 broadcast 的时间会有所减小
		{
			if(scan_only_mode) { NRF_TIMER2->CC[0] = (0xFFFF) - (rand()%10000); }
					   else { NRF_TIMER2->CC[0] = (0x7FFF) - (rand()%10000); }
			// TODO: 去掉变频扫描
		}

		loop++;

		if(loop == 200) { loop = 2; }

		/************************************************** loop ****************************************************************************************/

		if(first_time)
		{
			scanning_start();
			first_time 	= false;
			want_scan 	= false;
		}else
		{
			if(want_scan)
			{
				if(!scan_only_mode)
				{
					err_code = sd_ble_gap_adv_stop();
					APP_ERROR_CHECK(err_code);
				}

				scanning_start(); // TODO: 加上第二个scan参数

				NRF_LOG_INFO("scanning\r\n");
				want_scan = false;

			}else
			{

				err_code = sd_ble_gap_scan_stop();
				APP_ERROR_CHECK(err_code);

				if(broadcast_list->next_storage == NULL) { scan_only_mode = true;  }
											 else { scan_only_mode = false; }

				if(scan_only_mode)
				{
					want_scan = true;
					//NRF_LOG_INFO("normal mode\r\n");
					return;
				}else
				{
					if(mode == INIT_MODE) // for init mode
					{
						init_time_count++;

						if(init_time_count >= 15)
						{
							mode = NORMAL_MODE;
							NRF_LOG_INFO("init mode off, back to normal mode\r\n");

							/************************************************** delete init packet ********************************************************/

							delete_packet(broadcast_list);
							NRF_LOG_INFO("init packet deleted\r\n");

							/************************************************** creat report *************************************************************/

							check_list(broadcast_list, 0, 0, 0, 0, 0, 0); // init mode肯定不会是9，所以会返回最后一个event的指针

							struct storage * new_packet = (struct storage *)calloc(1, sizeof(struct storage));
							new_packet->next_storage 	 = NULL;
							memcpy(new_packet->data, init, sizeof(init));

							new_packet->data[7] = self_level;
							new_packet->data[8] = SELF_NUMBER;

							packet_pointer->next_storage = new_packet;

							NRF_LOG_INFO("self report packet created\r\n");
						}
					}

					sd_ble_gap_adv_data_set(broadcast_list->next_storage->data, sizeof(broadcast_list->next_storage->data), NULL, 0);
					advertising_start();
					want_scan = true;
					NRF_LOG_INFO("broadcasting\r\n");
				}
			}
		}
	}
}


void init_storage(void)
{
	broadcast_list 					= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(broadcast_list->data, init, sizeof(init));
	broadcast_list->next_storage 		= NULL;

	block_list						= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(block_list->data, init, sizeof(init));
	block_list->next_storage 		= NULL;

	NRF_LOG_INFO("2 lists created\r\n");
	NRF_LOG_INFO("init mode ready\r\n");
	NRF_LOG_INFO("normal mode on\r\n");
}


void check_list(struct storage * list_name, uint8_t value1, uint8_t input1, uint8_t value2, uint8_t input2, uint8_t value3, uint8_t input3)
{
	struct storage* temporary_pointer = list_name;
	struct storage* temporary_pointer_before;

	temporary_pointer_before = temporary_pointer;

	while((temporary_pointer != NULL) && ((temporary_pointer->data[value1] != input1) || (temporary_pointer->data[value2] != input2) || (temporary_pointer->data[value3] != input3)))
	{
		temporary_pointer_before = temporary_pointer;
		temporary_pointer = temporary_pointer->next_storage;
	}

	packet_pointer = temporary_pointer_before;

	return;
}


void delete_packet(struct storage* input_packet_pointer)
{
	if(input_packet_pointer->next_storage->next_storage == NULL) //  如果最后一个
	{
		free(input_packet_pointer->next_storage);
		input_packet_pointer->next_storage = NULL;	// 补上NULL
		NRF_LOG_INFO("last event freed\r\n");
	}else
	{
		packet_pointer2 = input_packet_pointer->next_storage->next_storage;
		free(input_packet_pointer->next_storage);
		input_packet_pointer->next_storage = packet_pointer2; // 接上
		NRF_LOG_INFO("middle event freed\r\n");
	}
	return;
}


void delete_block_list(void)
{
	struct storage* temporary_pointer = block_list;
	struct storage* temporary_pointer_before;

	temporary_pointer_before = temporary_pointer;

	while(block_list->next_storage != NULL)		// 每次循环free 掉最后一个
	{
		temporary_pointer = block_list;

		while(temporary_pointer->next_storage != NULL)
		{
			temporary_pointer_before = temporary_pointer;
			temporary_pointer = temporary_pointer->next_storage;
		}

		free(temporary_pointer);
		temporary_pointer_before->next_storage = NULL;

	NRF_LOG_INFO("block_list all deleted\r\n");
	//free_alarm_count = 0;
	}
	return;
}


void get_adv_data(ble_evt_t * p_ble_evt) // 没必要二进制encode了，都不是只有1和0
{
	uint32_t index = 0;

	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *)p_adv_report->data;

	while (index < p_adv_report->dlen)
	    {
	        uint8_t field_length = p_data[index];
	        uint8_t field_type   = p_data[index+1];

			if ((field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA) || (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME))
			{
				uint8_t a = index+2;
				/************************************************ check origin *********************************************************************/

				if(!((p_data[a]== 'T') && (p_data[a+1]== 'O') && (p_data[a+2]== 'N') && (p_data[a+3]== 'G')))
				{
					return;
				}

/************************************************ if motion node *********************************************************************/

				if((field_length == 7) && (p_data[a+4] >=2))	// motion nodes' number start from 2
				{
					check_list(block_list, 7, p_data[a+4], 8, SELF_NUMBER, 1, 0xff); // 不要检查level，因为level可能变
					if(packet_pointer->next_storage != NULL)
					{
						return;
					}

					check_list(broadcast_list, 7, p_data[a+4], 8, SELF_NUMBER, 1, 0xff); // 两次check的顺序不要颠倒，要不packet_pointer就不对了
					if(packet_pointer->next_storage != NULL)
					{
						return;
					}

					struct storage * new_packet = (struct storage *)calloc(1, sizeof(struct storage));
					new_packet->next_storage 	 = NULL;
					memcpy(new_packet->data, init, sizeof(init));

					new_packet->data[6] = 1;
					new_packet->data[7] = p_data[a+4];
					new_packet->data[8] = SELF_NUMBER;
					if(p_adv_report->rssi >= 0)
					{
						new_packet->data[9] = 1;
					}else
					{
						new_packet->data[9] = -(p_adv_report->rssi);
					}
					new_packet->data[10] = self_level;

					packet_pointer->next_storage = new_packet; 				// 接起来
					NRF_LOG_INFO("new alarm packet created\r\n");

					return;
				}

/************************************************ if center node *********************************************************************/

				if((field_length == 8))
				{
					NRF_LOG_INFO("center node!!!!\r\n");
					/************************************************ update self_level and filt bad signal**********************************************************/

					if(p_adv_report->rssi >= (-90))
					{
						self_level = 2;
					}else
					{
						return;
					}

					/************************************************ mode **********************************************************/

					if((p_data[a+4]== 0) && (p_data[a+5]== 0) && (p_data[a+6]== 0))
					{
						if(init_time_count == 0)
						{
							mode = INIT_MODE;
							init_break_count++;

							/************************************************ create init packet *********************************************************************/

							struct storage * new_packet = (struct storage *)calloc(1, sizeof(struct storage));
							packet_pointer2 			= broadcast_list->next_storage;
							new_packet->next_storage 	= packet_pointer2;
							memcpy(new_packet->data, init, sizeof(init));

							new_packet->data[11] = 1;

							broadcast_list->next_storage = new_packet;

							init_time_count++;

							NRF_LOG_INFO("init mode enter, init packet created\r\n");
						}else
						{
							return;
						}
					}else
					{
						check_list(broadcast_list, 7, p_data[a+4], 8, p_data[a+5], 9, p_data[a+6]);

						if(packet_pointer->next_storage == NULL)
						{
							return;
						}else
						{
							delete_packet(packet_pointer);
							// TODO: add to block list
						}
					}
					return;
				}

/************************************************ if relay node *********************************************************************/

				if(field_length != 11)
				{
					return;
				}
				//NRF_LOG_INFO("in_data = %x\r\n", in_data[3]);

				/************************************************ update self_level and filt bad signal**********************************************************/

				if(p_adv_report->rssi >= (-90))
				{
					if(p_data[a+8] < self_level) // rssi 的数值波动也没关系   -100就基本断了 最大－20
					{
						self_level = p_data[a+8] + 1;
					}
				}else
				{
					return;
				}

				/************************************************ check mode *********************************************************************/

				if((p_data[a+9] == 1) && (init_time_count ==0))	// 只要收到一次 就开启init mode， 为了防止你自己到时间停止了，然后别人还没停止，还继续传播init指令，这样你听到指令又进入init模式，没完没了了。
				{
					mode = INIT_MODE;
					init_break_count++;
				}

				/************************************************ switch mode *********************************************************************/

				switch(mode)
				{
				case INIT_MODE:
					if(init_time_count == 0)
					{
						/************************************************ create init packet *********************************************************************/

						struct storage * new_packet = (struct storage *)calloc(1, sizeof(struct storage));
						packet_pointer2 			= broadcast_list->next_storage;
						new_packet->next_storage 	= packet_pointer2;
						memcpy(new_packet->data, init, sizeof(init));

						new_packet->data[11] = 1;

						broadcast_list->next_storage = new_packet;

						init_time_count++;

						NRF_LOG_INFO("init mode enter, init packet created\r\n");
					}
					break; // init mode 不会进入下面！！！不会听到alarm！！但是init break时就正常了。


				case NORMAL_MODE:
					/************************************************ check block list *********************************************************************/

					check_list(block_list, 7, p_data[a+5], 8, p_data[a+6], 9, p_data[a+7]);
					if(packet_pointer->next_storage != NULL)
					{
						return;
					}

					/************************************************ check which level's packet ******************************************************************/


					if(p_data[a+8] < self_level)
					{
						node_type = LOWER_LEVEL_NODE;
					}else
					{
						if(p_data[a+8] == self_level)
						{
							node_type = SAME_LEVEL_NODE;
						}else
						{
							node_type = HIGHER_LEVEL_NODE;
						}
					}


					/************************************************ check packet *********************************************************************/

					check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6], 9, p_data[a+7]);

					switch(node_type)
					{
					case LOWER_LEVEL_NODE:
						if(packet_pointer->next_storage == NULL)
						{
							return;
						}else
						{
							delete_packet(packet_pointer);
							creat_packet = BLOCK_PACKET;
						}
						break;

					case SAME_LEVEL_NODE:
						if(packet_pointer->next_storage != NULL)
						{
							delete_packet(packet_pointer);
						}
						creat_packet = BLOCK_PACKET;
						break;

					case HIGHER_LEVEL_NODE:
						if(packet_pointer->next_storage == NULL)
						{
							creat_packet = BROADCAST_PACKET;
						}else
						{
							return;
						}
						break;
					}

					/************************************************ creat new packet ***************************************************************************/

					struct storage * new_packet = (struct storage *)calloc(1, sizeof(struct storage));
					new_packet->next_storage 	 = NULL;
					memcpy(new_packet->data, init, sizeof(init));

					switch(creat_packet)
					{
					case BROADCAST_PACKET:
						new_packet->data[7] = p_data[a+5];
						new_packet->data[8] = p_data[a+6];
						new_packet->data[9] = p_data[a+7];
						new_packet->data[10] = self_level;
						packet_pointer->next_storage = new_packet; 				// 接起来
						break;

					case BLOCK_PACKET:
						check_list(block_list, 0, 0, 0, 0, 0, 0);	// 让指针指向最后
						new_packet->data[7] = p_data[a+5];
						new_packet->data[8] = p_data[a+6];
						new_packet->data[9] = p_data[a+7];
						//new_packet->data[10] = self_level;	// block packet 不用添加其他没用的数值 因为这些数值也不会被广播或者被check
						packet_pointer->next_storage = new_packet; 				// 接起来
						delete_block_list_count = 0;
						break;
					}
					//NRF_LOG_INFO("NEW!!! device: %d and event: %d self_event: %d\r\n", in_device_number, in_event_number, self_event_number-1);
					break;
				}
				return;
			}
			index += field_length + 1;
	    }
}



