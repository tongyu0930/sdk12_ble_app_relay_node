/*
 * relay.c
 *
 *  Author: Tong Yu
 *  debug
 */


#include <stdlib.h>
#include "ble_gap.h"
#include "ble.h"
#include "nrf_log.h"
#include "app_error.h"


#define 									SELF_NUMBER  					 4
#define 									REPORT_SENDING_INTERVAL			 20
#define 									INIT_TIME_LENGTH				 5
#define 									BREAK_AFTER_INIT				 10
#define 									DELETE_BLOCK_LIST_COUNT			 40
#define 									ADVERTISING_CHANGE				 15
#define 									ADVERTISING_LIMIT				 20
#define 									MINIMUM_SIGNAL_ACCEPT			-70
#define 									MINIMUM_SIGNAL_ACCEPT_CENTER	-70
#define 									LOOP_PERIOD						 0x008000 //0x008000 is 1 second
#define 									ALARM_SENDING_TIME				 10


		volatile			  uint8_t		self_level 						= 20;	// 这地方会出bug，如果一个人听到的是255，那他自己会把自己设置为256，也就是0了。
		volatile 			  bool 			want_scan 		 				= false;
		volatile 			  bool 			scan_only_mode 					= true;	// broadcast list 里空时
		volatile static		  uint8_t 		init_break_count				= 0;
		volatile static 	  uint8_t		init_time_count 	        	= 0;
				 static 	  uint8_t 		delete_block_list_count			= 0;
				 static const uint8_t 		init[13] 						= {0x0c,0xff,'T','O','N','G',0,0,0,0,0,0,0};
				 static		  uint8_t 		datacheck[13];
				 static		  uint8_t		broadcast_count					= 0;
				 static		  uint8_t		message_number					= 1;
				 static		  bool 			adv_packet_content_changed 					= false;
				 static		  bool 			self_report_count_start			= false;
				 static		  uint8_t		self_report_count				= 0;


volatile static enum
{
    INIT_MODE,
    NORMAL_MODE
} mode = NORMAL_MODE;

static enum
{
    HIGH_LEVEL,
	LOW_LEVEL,
	SAME_LEVEL
} node_level;

static enum
{
    RELAY_NODE,
	ALARM_NODE,
	CENTER_NODE
} node_type;

static enum
{
	BROADCAST_PACKET,
	BLOCK_PACKET,
	RELAY_ACK_PACKET,
	INIT_PACKET,
	ALARM_ACK_PACKET,
	ALARM_PACKET,
	NONE
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
void scanning_start(void);
void init_storage(void);
static void check_list(struct storage * list_name, uint8_t value1, uint8_t input1, uint8_t value2, uint8_t input2);
static void delete_packet(struct storage* input_packet);
static void delete_block_list(void);
static uint8_t generate_messsage_number(void);
static void create_self_report(void);
void try_advertising(void);
void packet_process(uint8_t *p_data, uint8_t a, int8_t RSSI);
void packet_creation(uint8_t *p_data, uint8_t a, int8_t RSSI);



void init_storage(void)
{
	broadcast_list 					= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(broadcast_list->data, init, sizeof(init));
	broadcast_list->next_storage 	= NULL;

	block_list						= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(block_list->data, init, sizeof(init));
	block_list->next_storage 		= NULL;

	ack_list						= (struct storage *)calloc(1, sizeof(struct storage));
	memcpy(ack_list->data, init, sizeof(init));
	ack_list->next_storage 			= NULL;

	NRF_LOG_INFO("3 lists created\r\n");
	NRF_LOG_INFO("init mode ready\r\n");
	NRF_LOG_INFO("normal mode on\r\n");
}


static void check_list(struct storage * list_name, uint8_t value1, uint8_t input1, uint8_t value2, uint8_t input2)
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


static void delete_packet(struct storage* input_packet_pointer)
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


static void delete_block_list(void)
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

static uint8_t generate_messsage_number(void)
{
	message_number++;
	if(message_number > 200)
	{
		message_number = 1;
	}
	return message_number;
}

static void create_self_report(void)
{
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
	try_advertising();
}


void SWI3_EGU3_IRQHandler(void)  // TODO: 改到100ms call一次 或者说你搞两个timer. 改成只有一个advertising list
{
	NRF_RTC2->EVENTS_COMPARE[0] 	= 0;
	NRF_RTC2->TASKS_CLEAR 			= 1;
	NRF_EGU3->EVENTS_TRIGGERED[1] 	= 0;

	/************************************************** reporting 计数 ******************************************************************************/
	if(self_report_count_start) // TODO: 降频到1秒，利用count
	{
		self_report_count++;
		if(self_report_count >= REPORT_SENDING_INTERVAL)
		{
			create_self_report();
			self_report_count = 0;
		}
	}
	/************************************************** delete_block_list_count ******************************************************************************/
	if(block_list->next_storage != NULL)
	{
		delete_block_list_count++;
	}
	if(delete_block_list_count >= DELETE_BLOCK_LIST_COUNT)
	{
		delete_block_list();
		delete_block_list_count = 0;
	}

	/************************************************** 广播次数计数 ******************************************************************************/
	if(scan_only_mode == false)
	{
		broadcast_count++;
		if((broadcast_count > ADVERTISING_CHANGE) && (adv_packet_content_changed == false))				// TODO: 去掉同级帮助
		{
			broadcast_list->next_storage->data[9] = 1;
			broadcast_list->next_storage->data[8] = generate_messsage_number();
			datacheck[9] = broadcast_list->next_storage->data[9];
			datacheck[8] = broadcast_list->next_storage->data[8];
			adv_packet_content_changed = true;
			NRF_LOG_INFO("packet level cahnged\r\n");
			try_advertising();
		}
		if(broadcast_count > ADVERTISING_LIMIT)
		{
			delete_packet(broadcast_list); // 如果降级后也没人要的话，这个packet被抛弃了，要不然这辈子就卡在这个packet上了
			broadcast_count = 0;
			adv_packet_content_changed = false;
			NRF_LOG_INFO("paceket removed because nobody want this!!!!!!!\r\n");
			try_advertising();
		}
	}
	/************************************************** init mode 计数 ******************************************************************************/
	if(mode == INIT_MODE) // for init mode
	{
		init_time_count++;
		broadcast_count = 0; // 为了防治initmode时，initpacket被误以为是发不出去的packet。
		//adv_packet_content_changed = false; // 你既让不让broadcast count＋＋那这个就不会是true
		if(init_time_count >= INIT_TIME_LENGTH)
		{
			mode = NORMAL_MODE;
			NRF_LOG_INFO("init mode off, back to normal mode\r\n");
			delete_packet(broadcast_list);
			NRF_LOG_INFO("init packet deleted\r\n");
			create_self_report();
			try_advertising();
		}
	}
	/************************************************** short break after init mode ***********************************************************************/
	if(mode == NORMAL_MODE && init_break_count)
	{
		init_break_count++;
		if(init_break_count >= BREAK_AFTER_INIT)
		{
			init_time_count = 0;
			init_break_count = 0;
			NRF_LOG_INFO("init mode ready\r\n");
		}
	}
}


void get_adv_data(ble_evt_t * p_ble_evt) // 没必要二进制encode了，都不是只有1和0
{
	uint32_t index = 0;
	ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report; // 这个report里还有peer地址，信号强度等可以利用的信息。
	uint8_t *p_data = (uint8_t *)p_adv_report->data;
	int8_t RSSI = p_adv_report->rssi;

	while (index < p_adv_report->dlen)
	    {
	        uint8_t field_length = p_data[index];
	        uint8_t field_type   = p_data[index+1];

			if ((field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA) || (field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME))
			{
//				NRF_LOG_INFO("RSSI = %d\r\n", RSSI);
//				if(RSSI >= (MINIMUM_SIGNAL_ACCEPT)) // 可接受的最低信号强度
//				{
//					NRF_GPIO->OUT &= ~(1 << 20);
//				}else
//				{
//					NRF_GPIO->OUT |= (1 << 20);
//				}
//				return;

//				NRF_LOG_INFO("in_data = %x\r\n", p_data[index+2]);
//				p_data[index+2] = 0x55;
//				NRF_LOG_INFO("in_dfaddata = %x\r\n", p_data[index+2]);

				uint8_t a = index+2;
				/************************************************ check origin *********************************************************************/

				if(!((p_data[a]== 'T') && (p_data[a+1]== 'O') && (p_data[a+2]== 'N') && (p_data[a+3]== 'G')))
				{
					return;
				}
				/************************************************ check node type *********************************************************************/
				if(field_type == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME)
				{
					node_type = CENTER_NODE;
				}else if (field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
				{
					if(p_data[a+4]== 0)
					{
						node_type = ALARM_NODE;
					}else
					{
						if((p_data[a+7] > 1) && (p_data[a+8] == 0))
						{
							return; // 如果是给alarm的2nd ACK 就不管了	，所以说initmode不要 把这位设置为大于1的数
						}else
						{
							node_type = RELAY_NODE;
							if(p_data[a+4] < self_level)
							{
								node_level = HIGH_LEVEL;
							}else
							{
								if(p_data[a+4] == self_level)
								{
									node_level = SAME_LEVEL;
								}else
								{
									if(p_data[a+4] == self_level + 1)
									{
										node_level = LOW_LEVEL;
									}else
									{
										return; // 只接受level＋1的relay node，可能level－1和level－2都收到你的packet，多余了
									}
								}
							}
						}
					}
				}

			packet_process(p_data, a, RSSI);
			packet_creation(p_data, a, RSSI);
			try_advertising();		// 如果是要发送ACK的话，那么接受这肯定是满血搜索状态，所以发一个ACK它也应该可以收到。CenterNode也应该减小ACK的发送时间
			return;
		}
		index += field_length + 1;
	}
}

void packet_process(uint8_t *p_data, uint8_t a, int8_t RSSI)
{
	create_packet = NONE;

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
			if(p_data[a+6] > ALARM_SENDING_TIME) // 如果alarm的count大于10了，再给他ACK
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
//					NRF_LOG_INFO("Center node!!! \r\n");
		/************************************************ update self_level and filt bad signal**********************************************************/
		if(RSSI >= (MINIMUM_SIGNAL_ACCEPT_CENTER)) // 这样只有当信号足够强时才更新level，就可以避免信号若有时能收到有时收不到的情况了。
		{
			self_level = 2;
		}

		/************************************************ mode **********************************************************/
		if((p_data[a+4] == 0x30) && (p_data[a+5] == 0x31) && (p_data[a+6] == 0x30)
		&& (p_data[a+7] == 0x30) && (p_data[a+8] == 0x30) && (p_data[a+9] == 0x30)) // “TONG01000000"
		{
			if((p_data[a+10] != 0x30) || (p_data[a+11] != 0x30))
			{
				return; // 不管center给alarm的ACK
			}

			self_report_count_start = true;

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
		{	// TONG01xxxx
			uint8_t num1 = (p_data[a+6] - 48) * 16 + (p_data[a+7] - 48);
			uint8_t num2 = (p_data[a+8] - 48) * 16 + (p_data[a+9] - 48);
			check_list(broadcast_list, 7, num1, 8, num2);
			if(packet_pointer->next_storage == NULL)
			{
				return;
			}else
			{
				p_data[a+5] = packet_pointer->next_storage->data[7];
				p_data[a+6] = packet_pointer->next_storage->data[8];
				p_data[a+10] = packet_pointer->next_storage->data[12];
				delete_packet(packet_pointer);
				create_packet = BLOCK_PACKET;
			}
		}
		break;

	case RELAY_NODE:
//					NRF_LOG_INFO("Relay node!!! \r\n");
		/************************************************ update self_level and filter bad signal**********************************************************/
		if(RSSI >= (MINIMUM_SIGNAL_ACCEPT))
		{
			if(p_data[a+4] < self_level) // rssi 的数值波动也没关系   -100就基本断了 最大－20
			{
				if(self_level < 255)
				{
					self_level = p_data[a+4] + 1; // 这主要是为了防治有新的node加入，可能你的level就提升了。
					self_report_count_start = true;
//					NRF_LOG_INFO("self level changed to %d \r\n",self_level)
				}
			}
			if(mode == INIT_MODE)
			{
				broadcast_list->next_storage->data[6] = self_level;
				NRF_LOG_INFO("init packet level updated \r\n")
				return;
			}
		} // 这样只有当信号足够强时才更新level，就可以避免信号若有时能收到有时收不到的情况了。

		/************************************************ check mode *********************************************************************/
		if((p_data[a+5] == 0) && (p_data[a+6] == 0))	// 只要收到一次 就开启init mode， 为了防止你自己到时间停止了，然后别人还没停止，还继续传播init指令，这样你听到指令又进入init模式，没完没了了。
		{
			NRF_LOG_INFO("double 0 \r\n")
			if(init_time_count == 0)
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
			case HIGH_LEVEL:
				check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6]); // 比较rssi的原因是可能你会同时听见个人摔倒。那比较number和rssi就够了啊！万一rssi一样呢？
				if(packet_pointer->next_storage == NULL)
				{
					//return;
					check_list(block_list, 7, p_data[a+5], 8, p_data[a+6]);
					if(packet_pointer->next_storage == NULL)
					{
						create_packet = BLOCK_PACKET;
					}else
					{
						return;
					}
				}else
				{
					p_data[a+10] = packet_pointer->next_storage->data[12];//如果是ack的话，data［12］为空，所以要自己加上  然后如果alarmnode在发你就知道是重复的了
					delete_packet(packet_pointer);						  // 如果你要是move packet到屏蔽列表就不用这一步了 那就改成转移呗，该创建的创建，该转移的转移，用memcopy。
					create_packet = BLOCK_PACKET;
				}
				break;

			case SAME_LEVEL:
				check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6]);
				if(packet_pointer->next_storage != NULL)
				{
					if((p_data[a+7] == 1) && (p_data[a+8] != 0)) // this is a help-asking packet
					{
						return;
					}
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
						if((p_data[a+7] == 1) && (p_data[a+8] != 0))	// this is a help-asking packet  去掉同级帮助吧，有点复杂，并且也没用
						{
							check_list(broadcast_list, 7, p_data[a+5], 8, p_data[a+6]);
							if(packet_pointer->next_storage == NULL)
							{
								create_packet = BROADCAST_PACKET;	// 帮同级转发 如果两个同时请求互相帮助，那将是没完没了
								NRF_LOG_INFO("I am helping same level! \r\n")
							}
						}else
						{
							create_packet = BLOCK_PACKET;
						}
					}
				}
				break;

			case LOW_LEVEL:
				if((p_data[a+7] == 1) && (p_data[a+8] == 0))
				{
					return; // 不要管higer发来的ACK，即便对你来说是个新packet，既然higher能发出来ACK说明你同级的人已经再or完成relay了，你就不用好心在加到广播列表了，也不用加入block
				}else
				{
					check_list(block_list, 7, p_data[a+5], 8, p_data[a+6]); // 我先检查block list主要是为了创建新packet时的指针指向问题
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
}

void packet_creation(uint8_t *p_data, uint8_t a, int8_t RSSI)
{
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
		if(RSSI >= 0)
		{
			new_packet->data[11] = 1;
		}else
		{
			new_packet->data[11] = -RSSI;
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

	case NONE:
		break;
	}
}


void try_advertising(void)		// not TODO: 让这个方程被call一次100ms，然后去掉创建完packet就call这个函数. 这样的话如果两个node 广播一样的packet，那这个packet就会消失。就应该每次收到packet就try广播
{
	if(ack_list->next_storage != NULL)	// 如果播报ack时遇上initmode，那也没关系，ack播一次就没了，然后就进入init，进入init时不会在听到其他relay and alarm node的声音了
	{
		sd_ble_gap_adv_data_set(ack_list->next_storage->data, sizeof(ack_list->next_storage->data), NULL, 0);
		delete_packet(ack_list);
		NRF_LOG_INFO("broadcasting confirm list\r\n");

	}else if(broadcast_list->next_storage != NULL)
	{
		if(memcmp(datacheck, broadcast_list->next_storage->data, sizeof(datacheck)) != 0) // 如果经过扫描后比较起来bu一样
		{
			sd_ble_gap_adv_data_set(broadcast_list->next_storage->data, sizeof(broadcast_list->next_storage->data), NULL, 0);
			memcpy(datacheck, broadcast_list->next_storage->data, sizeof(broadcast_list->next_storage->data)); // scan时广播列表第一个可能就被删掉了，然后scan后再和广播列表第一个比较就回不同
			broadcast_count = 0;
			adv_packet_content_changed = false;
			NRF_LOG_INFO("broadcasting broadcast list\r\n");
		}
	}else
	{
		if(scan_only_mode == false)
		{
			sd_ble_gap_adv_stop();
			sd_ble_gap_scan_stop();
			scan_only_mode = true;
			scanning_start();
			NRF_GPIO->OUT ^= (1 << 17);
			NRF_LOG_INFO("broadcasting stopped \r\n");
		}
		return;
	}

	if(scan_only_mode == true)
	{
		sd_ble_gap_scan_stop();
		scan_only_mode = false;
		scanning_start();
		advertising_start();
		NRF_GPIO->OUT ^= (1 << 17);
	}
}


void manual_init(void)
{
	if(init_time_count == 0)
	{
		mode = INIT_MODE;
		init_break_count++;
		init_time_count++;

		struct storage * new_packet = (struct storage *)malloc(sizeof(struct storage));
		new_packet->next_storage 	 = NULL;
		memcpy(new_packet->data, init, sizeof(init));
		packet_pointer2 			= broadcast_list->next_storage;
		new_packet->next_storage 	= packet_pointer2;
		new_packet->data[6] = self_level;
		broadcast_list->next_storage = new_packet;
		NRF_LOG_INFO("init mode enter, init packet created\r\n");
	}else
	{
		return;
	}
}

//NRF_LOG_INFO("NEW!!! device: %d and event: %d self_event: %d\r\n", in_device_number, in_event_number, self_event_number-1);
//NRF_LOG_INFO("in_data = %x\r\n", in_data[3]);
