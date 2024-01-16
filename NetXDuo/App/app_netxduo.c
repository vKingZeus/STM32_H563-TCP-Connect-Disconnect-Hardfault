/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_netxduo.c
 * @author  MCD Application Team
 * @brief   NetXDuo applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
#include "nxd_dhcp_client.h"
/* USER CODE BEGIN Includes */
#include "stm32h5xx_it.h"
#include "../../Core/hal_src/hal.h"
//#include "../../Core/serialcommand_src/serialcommand.h"
#include "../../Core/global.h"
#include "../../Core/ssd1306_src/ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TX_THREAD      NxAppThread;
NX_PACKET_POOL NxAppPool;
NX_IP          NetXDuoEthIpInstance;
TX_SEMAPHORE   DHCPSemaphore;
TX_SEMAPHORE   DISCOSemaphore;
NX_DHCP        DHCPClient;
/* USER CODE BEGIN PV */
uint8_t     ETH_rxBuffer[255];
uint8_t 	g_ethCount;
char      	g_ERXBuffer[255];


TX_THREAD AppTCPThread; // TCP
TX_THREAD AppLinkThread; //DHCP
TX_THREAD serialParserThread;

ULONG IpAddress;
ULONG NetMask;
ULONG socket_state;
ULONG ip_address;

extern ULONG IpHost;
extern ULONG IpDevice;

int g_nConnected;

NX_TCP_SOCKET TCPSocket;
NX_PACKET *packet_ptr;
NX_PACKET *data_packet = NX_NULL;
char STM32_IP[16];
char IP_PC[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static VOID nx_app_thread_entry (ULONG thread_input);
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr);
/* USER CODE BEGIN PFP */
//static VOID serialParser_Thread(ULONG thread_input);
static VOID App_TCP_Thread_Entry(ULONG thread_input);
extern VOID tcp_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port);
ULONG check_ethConnection(void);
extern void getTCP_status (void);
extern void TCPserver_listen(void);
void eth_Display(char *_ethBuffer);
/* USER CODE END PFP */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
	UINT ret = NX_SUCCESS;
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;
	(void)byte_pool;
	CHAR *pointer;
	nx_system_initialize();
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		return TX_POOL_ERROR;
	}
	ret = nx_packet_pool_create(&NxAppPool, "NetXDuo App Pool", DEFAULT_PAYLOAD_SIZE, pointer, NX_APP_PACKET_POOL_SIZE);
	if (ret != NX_SUCCESS)
	{
		return NX_POOL_ERROR;
	}
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		return TX_POOL_ERROR;
	}
	ret = nx_ip_create(&NetXDuoEthIpInstance, "NetX Ip instance", NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK, &NxAppPool, nx_stm32_eth_driver,
			pointer, Nx_IP_INSTANCE_THREAD_SIZE, NX_APP_INSTANCE_PRIORITY);

	if (ret != NX_SUCCESS)
	{
		return NX_NOT_SUCCESSFUL;
	}
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		return TX_POOL_ERROR;
	}
	ret = nx_arp_enable(&NetXDuoEthIpInstance, (VOID *)pointer, DEFAULT_ARP_CACHE_SIZE);

	if (ret != NX_SUCCESS)
	{
		return NX_NOT_SUCCESSFUL;
	}
	ret = nx_icmp_enable(&NetXDuoEthIpInstance);
	if (ret != NX_SUCCESS)
	{
		return NX_NOT_SUCCESSFUL;
	}
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer,2 *  DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		return TX_POOL_ERROR;
	}
	ret = tx_thread_create(&AppTCPThread, "App TCP Thread", App_TCP_Thread_Entry, 0, pointer, NX_APP_THREAD_STACK_SIZE,
			NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_DONT_START);

	if (ret != TX_SUCCESS)
	{
		return NX_NOT_SUCCESSFUL;
	}
	ret = nx_tcp_enable(&NetXDuoEthIpInstance);
	if (ret != NX_SUCCESS)
	{
		return NX_NOT_SUCCESSFUL;
	}
	ret = nx_udp_enable(&NetXDuoEthIpInstance);
	if (ret != NX_SUCCESS)
	{
		return NX_NOT_SUCCESSFUL;
	}
	if (tx_byte_allocate(byte_pool, (VOID **) &pointer, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
	{
		return TX_POOL_ERROR;
	}
	ret = tx_thread_create(&NxAppThread, "NetXDuo App thread", nx_app_thread_entry , 0, pointer, NX_APP_THREAD_STACK_SIZE,
			NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

	if (ret != TX_SUCCESS)
	{
		return TX_THREAD_ERROR;
	}
	ret = nx_dhcp_create(&DHCPClient, &NetXDuoEthIpInstance, "DHCP Client");
	if (ret != NX_SUCCESS)
	{
		return NX_DHCP_ERROR;
	}
	tx_semaphore_create(&DHCPSemaphore, "DHCP Semaphore", 0);
	tx_semaphore_create(&DISCOSemaphore, "DISCONNECT Semaphore", 0); // peut être modifié si pas correcte
	return ret;
}
static VOID ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr)
{
	tx_semaphore_put(&DHCPSemaphore);
}
static VOID nx_app_thread_entry (ULONG thread_input)
{
	UINT ret = NX_SUCCESS;
	ret = nx_ip_address_change_notify(&NetXDuoEthIpInstance, ip_address_change_notify_callback, NULL);
	if (ret != NX_SUCCESS)
	{
		Error_Handler();
	}
	ret = nx_dhcp_start(&DHCPClient);
	if (ret != NX_SUCCESS)
	{
		Error_Handler();
	}
	if(tx_semaphore_get(&DHCPSemaphore, NX_APP_DEFAULT_TIMEOUT) != TX_SUCCESS)
	{
		Error_Handler();
	}
	ret = nx_ip_address_get(&NetXDuoEthIpInstance, &IpAddress, &NetMask);

	if (ret != TX_SUCCESS)
	{
		Error_Handler();
	}
	char szMessage[256];
	PRINT_IP_ADDRESS(IpAddress);
	ssd1306_writeLineLCD("Ethernet Detector", 0, 0);
	ssd1306_writeLineLCD("V0.98.00", 0, 11);
	sprintf(szMessage, "IP:%s", STM32_IP);
	ssd1306_writeLineLCD(szMessage, 0, 22);
	sprintf(szMessage, "PORT: %d", DEFAULT_PORT);
	ssd1306_writeLineLCD(szMessage, 0, 33);
	HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, SET);
	tx_thread_resume(&AppTCPThread);
	tx_thread_relinquish();
	return;
}
VOID extern tcp_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port)
																																				{
	tx_semaphore_put(&DHCPSemaphore);
																																				}
static VOID App_TCP_Thread_Entry(ULONG thread_input)
{
	UINT ret;
	g_ethCount = 0;
	static int cnt = 0;
	ret = nx_tcp_socket_create(&NetXDuoEthIpInstance, &TCPSocket, "TCP Server Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, WINDOW_SIZE, NX_NULL, NX_NULL);
	if (ret != NX_SUCCESS)
	{
		Error_Handler();
	}
	ret = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, DEFAULT_PORT, &TCPSocket, MAX_TCP_CLIENTS, tcp_listen_callback);
	if (ret != NX_SUCCESS)
	{
		Error_Handler();
	}
	if(tx_semaphore_get(&DHCPSemaphore, TX_WAIT_FOREVER) != TX_SUCCESS)
	{
		Error_Handler();
	}
	ret = nx_tcp_server_socket_accept(&TCPSocket, TX_WAIT_FOREVER);
	if (ret != NX_SUCCESS)
	{
		Error_Handler();
	}
	nx_tcp_socket_info_get(&TCPSocket, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &socket_state, NULL, NULL, NULL);
	if (socket_state == NX_TCP_ESTABLISHED)
	{
		if (cnt < 1)
		{
			cnt ++;
			Output(TCP_CONNECTED);
			clearDisplay();
			Display("Connected", 0, 0);
			g_nConnected = 1;
			while(1)
			{
//				if (tx_semaphore_get(&DISCOSemaphore, TX_NO_WAIT) == TX_SUCCESS)
//				{
				HAL_GPIO_TogglePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin);
				tx_thread_sleep(5);
				nx_tcp_socket_info_get(&TCPSocket, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &socket_state, NULL, NULL, NULL);
				if (socket_state != NX_TCP_ESTABLISHED)
				{
					tx_semaphore_put(&DISCOSemaphore);
					HAL_GPIO_WritePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin, RESET);
					HAL_GPIO_WritePin(LED2_YELLOW_GPIO_Port, LED2_YELLOW_Pin, RESET);
					HAL_GPIO_WritePin(LED3_RED_GPIO_Port, LED3_RED_Pin, SET);
					clearDisplay();
					Display("Disconnected", 0, 0);
					g_nConnected = 0;
					while(1);
				}
			}
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void handleDisconnectEvent()
{
	tx_semaphore_put(&DISCOSemaphore);
}
