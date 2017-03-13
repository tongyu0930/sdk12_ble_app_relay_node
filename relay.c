


#include <stdlib.h>
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"
#include "relay.h"


const 				uint8_t 				SELF_NUMBER  					= 0x05;
static 				uint8_t					self_level 						= 5;

extern volatile 	bool 					first_time;
volatile 			bool 					want_scan 		 				= false;
volatile 			bool 					scan_only_mode 					= true;		// broadcast list 里空时
volatile 			bool 					copy_data_check					= true;
volatile 			uint8_t 				init_break_count				= 0;
static 				uint8_t					init_time_count 	        	= 0;
static 				uint8_t 				loop							= 2;
static 				uint8_t 				delete_block_list_count			= 0;
static const 		uint8_t 				init[13] 						= {0x0c,0xff,'T','O','N','G',0,0,0,0,0,0,0};
static 		 		uint8_t 				datacheck[13];
static 		 		uint8_t					broadcast_count					= 0;
static 		 		uint8_t					message_number					= 1;
volatile 			bool 					level_changed 					= false;




static volatile enum
{
    INIT_MODE,
    NORMAL_MODE
} mode = NORMAL_MODE;

static volatile enum
{
    LOWER_LEVEL,
	HIGHER_LEVEL,
	SAME_LEVEL
} node_level;

static volatile enum
{
    RELAY_NODE,
	ALARM_NODE,
	CENTER_NODE
} node_type;

static volatile enum
{
	BROADCAST_PACKET,
	BLOCK_PACKET,
	RELAY_ACK_PACKET,
	INIT_PACKET,
	ALARM_ACK_PACKET,
	ALARM_PACKET
} create_packet;


struct storage									// linked list
{
			uint8_t 	data[13];
	struct 	storage * 	next_storage;
};
struct storage * 		broadcast_list;
struct storage * 		block_list;
struct storage * 		ack_list;

struct storage * 		packet_pointer;
struct storage * 		packet_pointer2;





void advertising_start(void);
void scanning_start(bool);
void init_storage(void);
void check_list(struct storage * list_name, uint8_t value1, uint8_t input1, uint8_t value2, uint8_t input2);
void delete_packet(struct storage* input_packet);
void delete_block_list(void);
uint8_t generate_messsage_number(void);

// TODO: 定时发送report：15mins
// TODO: center也要能检测alarm


void init_storage(void)
{
	broadcast_list 					= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(broadcast_list->data, init, sizeof(init));
	broadcast_list->next_storage 		= NULL;

	block_list						= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(block_list->data, init, sizeof(init));
	block_list->next_storage 		= NULL;

	ack_list						= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(ack_list->data, init, sizeof(init));
	ack_list->next_storage 		= NULL;

	NRF_LOG_INFO("3 lists created\r\n");
	NRF_LOG_INFO("init mode ready\r\n");
	NRF_LOG_INFO("normal mode on\r\n");
}


void check_list(struct storage * list_name, uint8_t value1, uint8_t input1, uint8_t value2, uint8_t input2)
{
	struct storage* temporary_pointer = list_name;
	struct storage* temporary_pointer_before;

	temporary_pointer_before = temporary_pointer;

	while((temporary_pointer != NULL) && ((temporary_pointer->data[value1] != input1) || (temporary_pointer->data[value2] != input2)))
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

		NRF_LOG_INFO("block_list one event deleted\r\n");
	}
	NRF_LOG_INFO("block_list all deleted\r\n");
	return;
}

uint8_t generate_messsage_number(void)
{
	message_number++;
	if(message_number > 200)
	{
		message_number = 1;
	}
	return message_number;
}

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
		if(delete_block_list_count >= 100)
		{
			delete_block_list();
			delete_block_list_count = 0;
		}
		/************************************************** loop interval change ******************************************************************************/
		if(loop%2)			// 就是说scan的时间是不变的，idle 或 broadcast 的时间会有所减小
		{
			if(scan_only_mode) { NRF_TIMER2->CC[0] = (0xFFFF) - (rand()%10000); }
					   else { NRF_TIMER2->CC[0] = (0x7FFF) - (rand()%10000); }
			// TODO: 去掉变频扫描,最后测电流时再搞，万一不省电再搞回来。
		}
		loop++;
		if(loop == 250) { loop = 2; }
		/************************************************** loop ****************************************************************************************/
		if(first_time)
		{
			scanning_start(true);
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
					scanning_start(false);
				}else
				{
					//scanning_start(true); //两种scan参数
					scanning_start(false);
				}
				NRF_LOG_INFO("scanning\r\n");
				want_scan = false;
			}else
			{
				err_code = sd_ble_gap_scan_stop(); // stop scanning
				APP_ERROR_CHECK(err_code);

				if(ack_list->next_storage != NULL)	// 如果播报ack时遇上initmode，那也没关系，ack播一次就没了，然后就进入init，进入init时不会在听到其他relay and alarm node的声音了
				{
					sd_ble_gap_adv_data_set(ack_list->next_storage->data, sizeof(ack_list->next_storage->data), NULL, 0);
					advertising_start();
					NRF_GPIO->OUT ^= (1 << 20);
					delete_packet(ack_list);
					scan_only_mode = false;	// 设为false，在扫描前才会先关闭广播再扫描，要不然就直接扫描了，然后你如果再开启广播就出现错误
					want_scan = true;
					NRF_LOG_INFO("broadcasting confirm list\r\n");
					//NRF_EGU3->INTENCLR 		= EGU_INTENCLR_TRIGGERED1_Msk;
				}else
				{
					if(broadcast_list->next_storage == NULL)
					{
						scan_only_mode = true;
					}else
					{
						/************************************************** 广播次数计数 ******************************************************************************/
						if(memcmp(datacheck, broadcast_list->next_storage->data, sizeof(datacheck)) == 0) // 如果经过扫描后比较起来还一样，说明没扫描到ACK啊
						{
							copy_data_check = false;
							broadcast_count++;
							if((broadcast_count > 100) && (level_changed == false))
							{
								broadcast_list->next_storage->data[6] = self_level + 1;
								datacheck[6] = broadcast_list->next_storage->data[6];
								level_changed = true;
								NRF_LOG_INFO("level cahnged\r\n");
							}
							if(broadcast_count > 200)
							{
								delete_packet(broadcast_list); // 如果降级后也没人要的话，这个packet被抛弃了，要不然这辈子就卡在这个packet上了
								broadcast_count = 0;
								level_changed = false;
								NRF_LOG_INFO("paceket removed because nobody want this\r\n");
							}
						}else
						{
							broadcast_count = 0;
							level_changed = false;
							NRF_LOG_INFO("top of the broadcast list changed\r\n");
							copy_data_check = true;
						}
						scan_only_mode = false;
					}
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
							broadcast_count = 0; // 为了防治initmode时，initpacket被误以为是发不出去的packet。
							//level_changed = false; // 你既让不让broadcast count＋＋那这个就不会是true
							if(init_time_count >= 15)
							{
								mode = NORMAL_MODE;
								NRF_LOG_INFO("init mode off, back to normal mode\r\n");
								/************************************************** delete init packet ********************************************************/
								delete_packet(broadcast_list);
								NRF_LOG_INFO("init packet deleted\r\n");
								/************************************************** creat report *************************************************************/
								check_list(broadcast_list, 0, 0, 0, 0); // init mode肯定不会是9，所以会返回最后一个event的指针
								struct storage * new_packet = (struct storage *)malloc(sizeof(struct storage));
								new_packet->next_storage 	 = NULL;
								memcpy(new_packet->data, init, sizeof(init));
								new_packet->data[6] = self_level;
								new_packet->data[7] = SELF_NUMBER;
								new_packet->data[8] = generate_messsage_number();
								new_packet->data[10] = SELF_NUMBER;
								new_packet->data[11] = self_level;
								packet_pointer->next_storage = new_packet;
								NRF_LOG_INFO("self report packet created\r\n");
							}
						}
						sd_ble_gap_adv_data_set(broadcast_list->next_storage->data, sizeof(broadcast_list->next_storage->data), NULL, 0);
						if(copy_data_check == true)
						{
							memcpy(datacheck, broadcast_list->next_storage->data, sizeof(broadcast_list->next_storage->data)); // scan时广播列表第一个可能就被删掉了，然后scan后再和广播列表第一个比较就回不同
						}
						advertising_start();
						NRF_LOG_INFO("broadcasting broadcast list\r\n");
						want_scan = true;
					}
				}
			}
		}
	}
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
				NRF_LOG_INFO("RSSI = %d\r\n", -(p_adv_report->rssi));
				return;
				uint8_t a = index+2;
				/************************************************ check origin *********************************************************************/

				if(!((p_data[a]== 'T') && (p_data[a+1]== 'O') && (p_data[a+2]== 'N') && (p_data[a+3]== 'G')))
				{
					return;

				}
				/************************************************ check node type *********************************************************************/
				if(p_data[a+4]== 0)
				{
					node_type = ALARM_NODE;
				}else
				{
					if(p_data[a+7] > 1)
					{
						return; // 如果是给alarm的2nd ACK 就不管了	，所以说initmode不要 把这位设置为大于1的数
					}else
					{
						if(p_data[a+4]== 1)
						{
							node_type = CENTER_NODE;
						}else
						{
							node_type = RELAY_NODE;
							if(p_data[a+4] < self_level)
							{
								node_level = LOWER_LEVEL;
							}else
							{
								if(p_data[a+4] == self_level)
								{
									node_level = SAME_LEVEL;
								}else
								{
									node_level = HIGHER_LEVEL;
								}
							}
						}
					}
				}
				/************************************************ switch node type *********************************************************************/
				switch(node_type)
				{
				case ALARM_NODE:	// alarm nodes' number start from 2
					if(mode == INIT_MODE)
					{
						return;
					}
					check_list(block_list, 12, p_data[a+7], 1, 0xff);	// data[1] 肯定是 0xff
					if(packet_pointer->next_storage != NULL)
					{
						if(p_data[a+6] > 10) // 如果alarm的count大于10了，再给他ACK
						{
							check_list(ack_list, 9, p_data[a+7], 1, 0xff);
							if(packet_pointer->next_storage == NULL)
							{
								create_packet = ALARM_ACK_PACKET;
							}else
							{
								return;
							}
						}else
						{
							return;
						}
					}else
					{
						check_list(broadcast_list, 9, p_data[a+7], 1, 0xff); // 两次check的顺序不要颠倒，要不packet_pointer就不对了
						if(packet_pointer->next_storage != NULL)
						{
							return;
						}else
						{
							create_packet = ALARM_PACKET;
						}
					}
					break;

				case CENTER_NODE:
					NRF_LOG_INFO("center node!!!!\r\n");
					/************************************************ update self_level and filt bad signal**********************************************************/
					if(p_adv_report->rssi >= (-90)) // 可接受的最低信号强度
					{
						self_level = 2;
					}else
					{
						return;
					}
					/************************************************ mode **********************************************************/
					if((p_data[a+5]== 0) && (p_data[a+6]== 0))
					{
						if(init_time_count == 0)
						{
							mode = INIT_MODE;
							init_break_count++;
							init_time_count++;
							create_packet = INIT_PACKET;
						}else
						{
							return;
						}
					}else
					{
						check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6]);
						if(packet_pointer->next_storage == NULL)
						{
							return;
						}else
						{
							delete_packet(packet_pointer);
							create_packet = BLOCK_PACKET;
						}
					}
					break;

				case RELAY_NODE:
					/************************************************ update self_level and filter bad signal**********************************************************/
					if(p_adv_report->rssi >= (-90))
					{
						if(p_data[a+4] < self_level) // rssi 的数值波动也没关系   -100就基本断了 最大－20
						{
							self_level = p_data[a+4] + 1; // 这主要是为了防治有新的node加入，可能你的level就提升了。
							NRF_LOG_INFO("self level updated")
						}
						if(mode == INIT_MODE)
						{
							broadcast_list->next_storage->data[6] = self_level;
							NRF_LOG_INFO("init level updated")
						}
					}else
					{
						return;
					}
					/************************************************ check mode *********************************************************************/
					if((p_data[a+5] == 0) && (p_data[a+6] == 0))	// 只要收到一次 就开启init mode， 为了防止你自己到时间停止了，然后别人还没停止，还继续传播init指令，这样你听到指令又进入init模式，没完没了了。
					{
						if(init_time_count ==0)
						{
							mode = INIT_MODE;
							init_break_count++;
						}else
						{
							return;
						}
					}
					/************************************************ switch mode *********************************************************************/
					switch(mode)
					{
					case INIT_MODE:
						if(init_time_count == 0)
						{
							init_time_count++;
							create_packet = INIT_PACKET;//checked
						}
						break; // init mode 不会听到alarm！！但是init break时就正常了。

					case NORMAL_MODE:
						/************************************************ check packet *********************************************************************/
						switch(node_level)
						{
						case LOWER_LEVEL:
							check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6]); // 比较rssi的原因是可能你会同时听见个人摔倒。那比较number和rssi就够了啊！万一rssi一样呢？
							if(packet_pointer->next_storage == NULL)
							{
								return;
							}else
							{
								delete_packet(packet_pointer);
								create_packet = BLOCK_PACKET;
							}
							break;

						case SAME_LEVEL:
							check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6]);
							if(packet_pointer->next_storage != NULL)
							{
								delete_packet(packet_pointer);
								create_packet = BLOCK_PACKET;
							}else
							{
								check_list(block_list, 7, p_data[a+5], 8, p_data[a+6]);
								if(packet_pointer->next_storage != NULL)
								{
									return;
								}else
								{
									create_packet = BLOCK_PACKET;
								}
							}
							break;

						case HIGHER_LEVEL:
							if(p_data[a+7] == 1) // 不要管higer发来的ACK，即便对你来说是个新packet，既然higher能发出来ACK说明你同级的人已经再or完成relay了，你就不用好心在加到广播列表了，也不用加入block
							{
								return;
							}else
							{
								check_list(block_list, 7, p_data[a+5], 8, p_data[a+6]);
								if(packet_pointer->next_storage != NULL)
								{
									check_list(ack_list, 7, p_data[a+5], 8, p_data[a+6]);
									if(packet_pointer->next_storage != NULL)
									{
										return;
									}else
									{
										create_packet = RELAY_ACK_PACKET;
									}
								}else
								{
									check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6]);
									if(packet_pointer->next_storage != NULL)
									{
										return;
									}else
									{
										create_packet = BROADCAST_PACKET;
									}
								}
							}
							break;
						}
					}
					break;
				}
/************************************************ creat new packet 其实你真可以发送前再创建packet，但就是到时候还要再区分一次message内容，还是这样直接制作好方便*********************************************************************************************/
				struct storage * new_packet = (struct storage *)malloc(sizeof(struct storage));
				new_packet->next_storage 	 = NULL;
				memcpy(new_packet->data, init, sizeof(init));

				switch(create_packet)
				{
				case BROADCAST_PACKET:
					new_packet->data[6] = self_level;
					new_packet->data[7] = p_data[a+5];
					new_packet->data[8] = p_data[a+6];
					new_packet->data[10] = p_data[a+8];
					new_packet->data[11] = p_data[a+9];
					new_packet->data[12] = p_data[a+10];
					packet_pointer->next_storage = new_packet; 				// 接起来
					NRF_LOG_INFO("new broadcast packet created\r\n");
					break;

				case BLOCK_PACKET:
					check_list(block_list, 0, 0, 0, 0);				// 让指针指向最后
					new_packet->data[7] = p_data[a+5];
					new_packet->data[8] = p_data[a+6];
					new_packet->data[12] = p_data[a+10]; 					// block packet 不用添加其他没用的数值 因为这些数值也不会被广播或者被check
					packet_pointer->next_storage = new_packet; 				// 接起来
					delete_block_list_count = 0;							//reset count
					NRF_LOG_INFO("new block packet created\r\n");
					break;

				case RELAY_ACK_PACKET:
					new_packet->data[6] = self_level;
					new_packet->data[7] = p_data[a+5];
					new_packet->data[8] = p_data[a+6];
					new_packet->data[9] = 1;
					packet_pointer->next_storage = new_packet; 				// 接起来
					NRF_LOG_INFO("new relay ack packet created\r\n");
					break;

				case INIT_PACKET:
					packet_pointer2 			= broadcast_list->next_storage;
					new_packet->next_storage 	= packet_pointer2;
					new_packet->data[6] = self_level;
					broadcast_list->next_storage = new_packet;
					NRF_LOG_INFO("init mode enter, init packet created\r\n");
					break;

				case ALARM_PACKET:
					new_packet->data[6] = self_level;
					new_packet->data[7] = SELF_NUMBER;
					new_packet->data[8] = generate_messsage_number();
					new_packet->data[9] = p_data[a+7];
					new_packet->data[10] = SELF_NUMBER;
					if(p_adv_report->rssi >= 0)
					{
						new_packet->data[11] = 1;
					}else
					{
						new_packet->data[11] = -(p_adv_report->rssi);
					}
					new_packet->data[12] = p_data[a+7];
					packet_pointer->next_storage = new_packet; 				// 接起来
					NRF_LOG_INFO("new alarm packet created\r\n");
					break;

				case ALARM_ACK_PACKET:
					new_packet->data[6] = self_level;
					new_packet->data[7] = SELF_NUMBER;						// 有必要加这句吗，不加会不会被以为是initmode
					new_packet->data[9] = p_data[a+7];
					packet_pointer->next_storage = new_packet; 				// 接起来
					NRF_LOG_INFO("new alarm ack packet created\r\n");
					break;
				}
			return;
		}
		index += field_length + 1;
	}
}

//NRF_LOG_INFO("NEW!!! device: %d and event: %d self_event: %d\r\n", in_device_number, in_event_number, self_event_number-1);
//NRF_LOG_INFO("in_data = %x\r\n", in_data[3]);
