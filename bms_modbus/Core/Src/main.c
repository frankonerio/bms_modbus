/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "delay.h"
#include "spi.h"
#include "modbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "socket.h"
#include "wizchip_conf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{ Bit_RESET = 0,
  Bit_SET
}BitAction;

#define LDR_VALUE  ADC_CHANNEL_1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE BEGIN PD */
#define HTTP_SOCKET 0
//#define PORT_TCPS 5000
#define DATA_BUF_SIZE 2048
uint8_t gDATABUF[DATA_BUF_SIZE];
#define SEPARATOR            "=============================================\r\n"
#define WELCOME_MSG  		 "Welcome to STM32Nucleo Ethernet configuration\r\n"
#define NETWORK_MSG  		 "Network configuration:\r\n"
#define IP_MSG 		 		 "  IP ADDRESS:  %d.%d.%d.%d\r\n"
#define NETMASK_MSG	         "  NETMASK:     %d.%d.%d.%d\r\n"
#define GW_MSG 		 		 "  GATEWAY:     %d.%d.%d.%d\r\n"
#define MAC_MSG		 		 "  MAC ADDRESS: %x:%x:%x:%x:%x:%x\r\n"
#define GREETING_MSG 		 "Well done guys! Welcome to the IoT world. Bye!\r\n"
#define CONN_ESTABLISHED_MSG "Connection established with remote IP: %d.%d.%d.%d:%d\r\n"
#define SENT_MESSAGE_MSG	 "Sent a message. Let's close the socket!\r\n"
#define WRONG_RETVAL_MSG	 "Something went wrong; return value: %d\r\n"
#define WRONG_STATUS_MSG	 "Something went wrong; STATUS: %d\r\n"
#define LISTEN_ERR_MSG		 "LISTEN Error!\r\n"

#define PRINT_STR(msg) do  {										\
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);		\
} while(0)

#define PRINT_HEADER() do  {													\
  HAL_UART_Transmit(&huart2, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
  HAL_UART_Transmit(&huart2, (uint8_t*)WELCOME_MSG, strlen(WELCOME_MSG), 100);	\
  HAL_UART_Transmit(&huart2, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
} while(0)

#define PRINT_NETINFO(netInfo) do { 																					\
  HAL_UART_Transmit(&huart2, (uint8_t*)NETWORK_MSG, strlen(NETWORK_MSG), 100);											\
  sprintf(msg, MAC_MSG, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2], netInfo.mac[3], netInfo.mac[4], netInfo.mac[5]);\
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, IP_MSG, netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3]);										\
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, NETMASK_MSG, netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3]);								\
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
  sprintf(msg, GW_MSG, netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3]);										\
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
} while(0)

char msg[60];

#define LISTEN_PORT 502
#define RECEIVE_BUFF_SIZE 128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int16_t statusClient;
uint8_t statusSlave;
uint8_t tcp_error = 0;
uint8_t rtu_error = 0;
uint8_t TX_buffer[128];
uint8_t send_size;
uint8_t RX_buffer[128];
uint8_t recv_size;
uint16_t cntSend = 0;

uint8_t flag_busy_RTU = 0;
uint8_t cntBufferRTU = 0;
uint8_t FrameNrBytes = 0;

char writeValue[60];
uint8_t cnt;
BitAction operatorOk = 0;
uint32_t address_mem = 0;

uint8_t S1 = 0;
uint8_t S2 = 0;
uint8_t S3 = 0;


#define DATA_BUF_SIZE   2048
uint8_t gDATABUF[DATA_BUF_SIZE];

wiz_NetInfo gWIZNETINFO = {
			.mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
			.ip = {192, 168, 137, 14},
			.sn = {255, 255, 255, 0},
			.gw = {192, 168, 3, 1},
			.dns= {8,8,8,8},
			.dhcp= NETINFO_STATIC
		  };
uint8_t RX_buffer[RX_buffer_length];

uint16_t holding_register[64]= {
		1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,
		17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,
		33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,
		49,50,51,52,53,54,55,56,57,58,59,60,61,62,63
};


//variables for multythread http
uint32_t sentsize[_WIZCHIP_SOCK_NUM_];
uint32_t filesize[_WIZCHIP_SOCK_NUM_];
uint8_t http_state[_WIZCHIP_SOCK_NUM_];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void  wizchip_select(void);
void  wizchip_deselect(void);
void  wizchip_write(uint8_t wb);
uint8_t wizchip_read();


void network_init(void);								// Initialize Network information and display it
//int16_t tcp_client(uint8_t sn, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t buffer_send_size, uint8_t* buffer_recv_size, uint8_t* destip, uint16_t destport);
int32_t tcp_http_mt(uint8_t, uint8_t*, uint16_t);		// Multythread TCP server
void HTTP_reset(uint8_t sockn);


// initialize the dependent host peripheral
void platform_init(void);
void reverse(char s[]);
void new_itoa(int n, char s[]);
void communication();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i;
	uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

   /* Chip selection call back */

  HAL_GPIO_WritePin(GPIO_W5500_RESET_GPIO_Port,GPIO_W5500_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIO_W5500_RESET_GPIO_Port,GPIO_W5500_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);

	 /* SPI Read & Write callback function */
	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);

	////////////////////////////////////////////////////////////////////////
	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
	{
	   //init fail

	   while(1);
	}

	/* Network initialization */
	network_init();

	//memset(holding_registers,0,sizeof(holding_registers));
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	 wizchip_setnetinfo(&gWIZNETINFO);
	  PRINT_NETINFO(gWIZNETINFO);

	  ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);

	  //Configure PHY by software for maximum compatibility
	  //so that user can use any model w5500 board
	  //else the PINs on the board configure it, which may
	  //lead to different configurations in different boards.
	  wiz_PhyConf phyconf;

	  phyconf.by=PHY_CONFBY_SW;
	  phyconf.duplex=PHY_DUPLEX_FULL;
	  phyconf.speed=PHY_SPEED_10;//10MBps Ethernet link speed
	  phyconf.mode=PHY_MODE_AUTONEGO;//best to go with auto-negotiation

	  ctlwizchip(CW_SET_PHYCONF, (void*) &phyconf);//PHY Configuration Command
	  //*** End Phy Configuration

	  PHYStatusCheck();
	  PrintPHYConf();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  printf("\r\nInitializing server socket\r\n");

	  	  //Parameters in order socket_id, protocol TCP or UDP, Port number, Flags=0
	  	  //Return value is socket ID on success
	  	  if(socket(1,Sn_MR_TCP,LISTEN_PORT,0)!=1)
	  	  {
	  		  //error
	  		  printf("Cannot create Socket!\r\n");
	  		  while(1);//halt here
	  	  }

	  	  //success
	  	  printf("Socket Created Successfully ! \r\n");

	  	  uint8_t socket_io_mode=SOCK_IO_BLOCK;

	  	  ctlsocket(1, CS_SET_IOMODE , &socket_io_mode);//set blocking IO mode

	  	  printf("Start listening on port %d ! \r\n",LISTEN_PORT);
	  	  printf("Waiting for a client connection. \r\n");

	  	  //Make it a passive socket (i.e. listen for connection)
	  	  if(listen(1)!=SOCK_OK)//our socket id is 1 (w5500 have 8 sockets from 0-7)
	  	  {
	  		  //error
	  		  printf("Cannot listen on port %d",LISTEN_PORT);

	  		  while(1);

	  	  }

	  	  uint8_t sr=0x00;//socket status register

	  	  do
	  	  {
	  		  sr=getSn_SR(1);//read status reg (SR of socket 1)
	  	  }while (sr!=0x17 && sr!=0x00);

	  	  if(sr==0x00)
	  	  {
	  		  printf("Some error occurred on server socket. Please restart.\r\n");
	  		  while(1);
	  	  }

	  	  if(sr==0x17)
	  	  {
	  		  //we come here only when a client has connected.
	  		  //Now we can read data from the socket
	  		  printf("A client connected!\r\n");
	  		  printf("Waiting for Client Data ...!\r\n");

	  		  while(1)
	  		  {
	  			  int len=recv(1,RX_buffer, RX_buffer_length);

	  			  if(len==SOCKERR_SOCKSTATUS)
	  			  {
	  				  //client has disconnected
	  				  printf("Client has disconnected\r\n");
	  				  printf("*** SESSION OVER ***\r\n\r\n");
	  				  break;
	  			  }

	  			 RX_buffer[len]='\0';

	  			  printf("Received %d bytes from client\r\n",len);
	  			  printf("Data Received: %s",  RX_buffer);

	  			  //Echo the data back encloused in a [] pair
//	  			  send(1,(uint8_t*)"[",1);//starting sq bracket
//	  			  send(1,receive_buff,len);// the data
//	  			  send(1,(uint8_t*)"]",1);//closing sq bracket
//
	  			  memset(TX_buffer,0,sizeof(TX_buffer));
	  			  modbus_receive(RX_buffer, TX_buffer, len, holding_register);
	  			  holding_register[3] = holding_register[3] + 1;

//	  			send(1,test_buffer,4);//closing sq bracket

//	  			  uint8_t test_buffer[29]= {0,10,0,0,0,23,1,3,20,0,1,0,2,0,3,0,4,0,5,0,6,0,7,0,8,0,9,0,10};
//	  			  send(1,test_buffer,29);//closing sq bracket

	  			  printf("\r\nECHO sent back to client\r\n");

	  			  //Look for quit message and quit if received
	  			  if(strcmp((char*) RX_buffer,"QUIT")==0)
	  			  {
	  				  printf("Received QUIT command from client\r\n");
	  				  printf("Disconnecting ... \r\n");
	  				  printf("*** SESSION OVER ***\r\n\r\n");
	  				  disconnect(1);//disconnect from the clinet
	  				  break;//come out of while loop
	  			  }

	  		  }//While loop (as long as client is connected)

	  	  }//if block, client connect success
    /* USER CODE BEGIN 3 */

  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_W5500_CS_Pin|LD2_Pin|GPIO_W5500_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_W5500_CS_Pin LD2_Pin GPIO_W5500_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_W5500_CS_Pin|LD2_Pin|GPIO_W5500_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_SYSTICK_Callback()
{
	DelayIncCnt();
}

uint8_t W5500_rxtx(uint8_t data)
{
	uint8_t rxdata;

	HAL_SPI_TransmitReceive(&hspi2, &data, &rxdata, 1, 50);

	return (rxdata);
}

void  wizchip_select(void)
{
	W5500_select();
}

void  wizchip_deselect(void)
{
	W5500_release();
}

void  wizchip_write(uint8_t wb)
{
	W5500_tx(wb);
}

uint8_t wizchip_read()
{
   return W5500_rx();
}

//////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////
void network_init(void)
{
   uint8_t tmpstr[6];

	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);

	ctlwizchip(CW_GET_ID,(void*)tmpstr);
}

void UWriteData(const char data)
{
	while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==RESET);

	huart2.Instance->DR=data;

}

int __io_putchar(int ch)
{
	UWriteData(ch);
	return ch;
}

void PHYStatusCheck(void)
{
	uint8_t tmp;

	do
	{
		printf("\r\nChecking Ethernet Cable Presence ...");
		ctlwizchip(CW_GET_PHYLINK, (void*) &tmp);

		if(tmp == PHY_LINK_OFF)
		{
			printf("NO Cable Connected!");
			HAL_Delay(1500);
		}
	}while(tmp == PHY_LINK_OFF);

	printf("Good! Cable got connected!");

}

void PrintPHYConf(void)
{
	wiz_PhyConf phyconf;

	ctlwizchip(CW_GET_PHYCONF, (void*) &phyconf);

	if(phyconf.by==PHY_CONFBY_HW)
	{
		printf("\r\nPHY Configured by Hardware Pins");
	}
	else
	{
		printf("\r\nPHY Configured by Registers");
	}

	if(phyconf.mode==PHY_MODE_AUTONEGO)
	{
		printf("\r\nAutonegotiation Enabled");
	}
	else
	{
		printf("\r\nAutonegotiation NOT Enabled");
	}

	if(phyconf.duplex==PHY_DUPLEX_FULL)
	{
		printf("\r\nDuplex Mode: Full");
	}
	else
	{
		printf("\r\nDuplex Mode: Half");
	}

	if(phyconf.speed==PHY_SPEED_10)
	{
		printf("\r\nSpeed: 10Mbps");
	}
	else
	{
		printf("\r\nSpeed: 100Mbps");
	}
}
void communication(){

}
// Error codes and messages:
// sn   -  Success: The socket number 'sn' passed as a parameter
// 1    -  SOCK_OK
// -1   -  SOCKERR_SOCKNUM  - Invalid socket number
// -5   -  SOCKERR_SOCKMODE - Invalid operation in the socket
// -13  -  SOCKERR_TIMEOUT  - Timeout occurred
// -7   -  SOCKERR_SOCKSTATUS - Invalid socket status for socket operation
// -14  -  SOCKERR_DATALEN    - Zero data length
// -12  -  SOCKERR_IPINVALID   - Wrong server IP address
// -11  -  SOCKERR_PORTZERO    - Server port zero
// -4   -  SOCKERR_SOCKCLOSED  - Socket unexpectedly closed
// -6   -  SOCKERR_SOCKFLAG    - Invalid socket flag
// -1000 -  SOCK_FATAL  - Result is a fatal error about socket process.
// 0    -  SOCK_BUSY         - Socket is busy
// 200  -  SUCCESS_RECEIVE

int16_t tcp_client(uint8_t sn, uint8_t* tx_buf, uint8_t* rx_buf, uint8_t buffer_send_size, uint8_t* buffer_recv_size, uint8_t* destip, uint16_t destport)
{
   int32_t ret; // return value for SOCK_ERRORs
   uint16_t size = 0;
   // Destination (TCP Server) IP info (will be connected)
   // >> loopback_tcpc() function parameter
   // >> Ex)
   //	uint8_t destip[4] = 	{192, 168, 0, 214};
   //	uint16_t destport = 	5000;

   // Port number for TCP client (will be increased)
   uint16_t any_port = 	502;
   static uint8_t flagSent = 0;

   // Socket Status Transitions
   // Check the W5500 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED:
         if(getSn_IR(sn) & Sn_IR_CON)	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
         {
#ifdef _LOOPBACK_DEBUG_
			printf("%d:Connected to - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
			setSn_IR(sn, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
         }

         //////////////////////////////////////////////////////////////////////////////////////////////
         // Data Transaction Parts; Handle the [data receive and send] process
         //////////////////////////////////////////////////////////////////////////////////////////////
         if(flagSent == 0)
         {
        	 flagSent = 1;
    		 ret = send(sn, tx_buf, buffer_send_size); // Data send process (User's buffer -> Destination through H/W Tx socket buffer)
    		 if(ret < 0) // Send Error occurred (sent data length < 0)
    		 {
    			 close(sn); // socket close
    			 flagSent = 0;
    			 return ret;
    		 }
         }
         else if((size = getSn_RX_RSR(sn)) > 0) // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
         {

			ret = recv(sn, rx_buf, size); // Data Receive process (H/W Rx socket buffer -> User's buffer)

			if(ret <= 0) return ret; // If the received data length <= 0, receive failed and process end
			*buffer_recv_size = size;
			flagSent = 0;
			return 200; //Success receive
         }

		 //////////////////////////////////////////////////////////////////////////////////////////////
         break;

      case SOCK_CLOSE_WAIT :
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:CloseWait\r\n",sn);
#endif
         if((ret=disconnect(sn)) != SOCK_OK) return ret;
#ifdef _LOOPBACK_DEBUG_
         printf("%d:Socket Closed\r\n", sn);
#endif
         break;

      case SOCK_INIT :
#ifdef _LOOPBACK_DEBUG_
    	 printf("%d:Try to connect to the %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
    	 if( (ret = connect(sn, destip, destport)) != SOCK_OK) return ret;	//	Try to TCP connect to the TCP server (destination)
         break;

      case SOCK_CLOSED:
    	  close(sn);
    	  if((ret=socket(sn, Sn_MR_TCP, any_port++, 0x00)) != sn) return ret; // TCP socket open with 'any_port' port number
#ifdef _LOOPBACK_DEBUG_
    	 //printf("%d:TCP client loopback start\r\n",sn);
         //printf("%d:Socket opened\r\n",sn);
#endif
         break;
      default:
         break;
   }
   return 1;
}

#if (ENABLE_MB_RTU == 1)&&(ENABLE_MB_TCP == 0)
 //Armazena a informação recebida via ModbusRTU em um buffer
void RTU_RX_Int(UART_HandleTypeDef* huart)
{
	uint8_t c;

	if(huart->Instance == USART2)
	{
		HAL_UART_Receive(huart, &c, 1,100);
		if(cntBufferRTU < 128)
		{
			RX_buffer[cntBufferRTU] = c;
			cntBufferRTU++;
		}
		flag_busy_RTU = 1;
		DELAY_SetTime_2(0);
	}

}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
