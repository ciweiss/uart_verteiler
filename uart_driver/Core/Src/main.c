/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
UART_HandleTypeDef hlpuart2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define cobs_size_increase 2

#define  cobs_buffer_decoded_size  512
#define  cobs_buffer_encoded_size  cobs_buffer_decoded_size + cobs_size_increase
uint8_t cobs_buffer_encoded[cobs_buffer_encoded_size];
uint8_t cobs_buffer_decoded[cobs_buffer_decoded_size];

#define RING_BUFFER_SIZE 512

typedef struct
{
	UART_HandleTypeDef* uart_handle;
	ring_buffer_t ringbuffer_rx;
	uint8_t ringbuffer_rx_arr[RING_BUFFER_SIZE];
	ring_buffer_t ringbuffer_tx;
	uint8_t ringbuffer_tx_arr[RING_BUFFER_SIZE];
	uint8_t message_counter;
    uint8_t rx_byte;
    uint8_t tx_byte;
    size_t rx_message_size_encoded;
    size_t tx_message_size_encoded;
    // cobs decoding is done sequentially -> only one buffer needed
    uint8_t* cobs_buffer_encoded;
    uint8_t* cobs_buffer_decoded;

} UART_Buffer;

void rx_interrupt_callback(UART_Buffer* uart_buffer)
{
	if(!(ring_buffer_is_full(&uart_buffer->ringbuffer_rx) && uart_buffer->message_counter != 0))
	{
		ring_buffer_queue(&uart_buffer->ringbuffer_rx, uart_buffer->rx_byte);
		if(uart_buffer->rx_byte == 0)
		{
			uart_buffer->message_counter++;
		}
	}
	HAL_UART_Receive_IT(uart_buffer->uart_handle, &uart_buffer->rx_byte, 1);
}

void tx_interrupt_callback(UART_Buffer* uart_buffer)
{
	if (!ring_buffer_is_empty(&uart_buffer->ringbuffer_tx) )
	{
		ring_buffer_dequeue(&uart_buffer->ringbuffer_tx, &uart_buffer->tx_byte);
		HAL_UART_Transmit_IT(uart_buffer->uart_handle, &uart_buffer->tx_byte, 1);
	}
}

typedef enum
{
    NEW_MESSAGE_IN_DECODED_COBS_BUFFER  	= 0x00,
	TOO_FEW_BYTES_IN_RINGBUFFER     		= 0x01,
    COBS_DECODING_FAILED 					= 0x02,
    NO_ZERO_BYTE_IN_RINGBUFFER  			= 0x03,
    NO_NEW_MESSAGE				  			= 0x04,
    ERROR_DURING_RETRIEVING_FROM_RINGBUFFER	= 0x05,
} message_status;

message_status retrieve_message(UART_Buffer* uart_buffer)
{
	if(uart_buffer->message_counter > 0 || ring_buffer_is_full(&uart_buffer->ringbuffer_rx))
	{
		size_t num_of_items_in_buffer = ring_buffer_num_items(&uart_buffer->ringbuffer_rx);
		size_t index_of_zero = 0;

		while(index_of_zero < num_of_items_in_buffer)
		{
			uint8_t peeked_item;
			ring_buffer_peek(&uart_buffer->ringbuffer_rx, &peeked_item, index_of_zero);
			if(peeked_item == 0)
			{
				break;
			}
			index_of_zero++;
		}
//		return TOO_FEW_BYTES_IN_RINGBUFFER;

		if(index_of_zero == num_of_items_in_buffer) // ringbuffer is full and no zeros
		{
//			uart_buffer->ringbuffer.tail_index = 0;
//			uart_buffer->ringbuffer.head_index = 0;
//			uart_buffer->ringbuffer.tail_index = ((uart_buffer->ringbuffer.tail_index + index_of_zero) & uart_buffer->ringbuffer.buffer_mask);

			uart_buffer->message_counter = 0;
			return NO_ZERO_BYTE_IN_RINGBUFFER;
		}
		else
		{
			uart_buffer->message_counter--;
		}
		if(index_of_zero + 1 < uart_buffer->rx_message_size_encoded) // too few bytes for complete message
		{
			uart_buffer->ringbuffer_rx.tail_index = ((uart_buffer->ringbuffer_rx.tail_index + index_of_zero + 1) & uart_buffer->ringbuffer_rx.buffer_mask);
			return TOO_FEW_BYTES_IN_RINGBUFFER;
		}
		// remove extra bytes before beginning of message
		size_t extra_bytes_before_message = max(index_of_zero + 1 - uart_buffer->rx_message_size_encoded, 0);
		uart_buffer->ringbuffer_rx.tail_index = ((uart_buffer->ringbuffer_rx.tail_index + extra_bytes_before_message) & uart_buffer->ringbuffer_rx.buffer_mask);

		size_t number_of_retrieved_bytes = ring_buffer_dequeue_arr(&uart_buffer->ringbuffer_rx, uart_buffer->cobs_buffer_encoded, uart_buffer->rx_message_size_encoded);

		if(number_of_retrieved_bytes == uart_buffer->rx_message_size_encoded)
		{

//			HAL_UART_Transmit(uart_buffer->uart_handle, &number_of_retrieved_bytes, 4, 1);

			cobs_decode_result res = cobs_decode(uart_buffer->cobs_buffer_decoded, cobs_buffer_decoded_size, uart_buffer->cobs_buffer_encoded, uart_buffer->rx_message_size_encoded-1);
			if (res.status == COBS_DECODE_OK)
			{
				return NEW_MESSAGE_IN_DECODED_COBS_BUFFER;
			}
			else
			{
				return COBS_DECODING_FAILED;
			}
		}
		else
		{
			return ERROR_DURING_RETRIEVING_FROM_RINGBUFFER;
		}
	}
	else
	{
		return NO_NEW_MESSAGE;
	}
}

UART_Buffer uart_buffer_pc;
UART_Buffer uart_buffer_motors[6];


//#define down_rx_message_length 5

//uint8_t buffer_up_byte;
//uint8_t buffer_down_byte[6];

//uint8_t up_rx_message_counter = 0;
//uint8_t down_rx_message_counter[6] = {0};


//#define buffer_down_size  24
//uint8_t buffer_down[buffer_down_size];

#define buffer_pc_tx_size  24
//#define  cobs_buffer_size  100

//#define up_tx_message_length  24
//#define up_tx_message_length_encoded up_tx_message_length + cobs_size_increase

uint8_t buffer_pc_tx[buffer_pc_tx_size];


//uint8_t buffer_tmp[4];
UART_HandleTypeDef* motor_controller[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_LPUART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void setup()
{
	// setup pc connected uart buffer
	uart_buffer_pc.uart_handle = &hlpuart2;
	ring_buffer_init(&uart_buffer_pc.ringbuffer_rx, uart_buffer_pc.ringbuffer_rx_arr, RING_BUFFER_SIZE);
	ring_buffer_init(&uart_buffer_pc.ringbuffer_tx, uart_buffer_pc.ringbuffer_tx_arr, RING_BUFFER_SIZE);
	uart_buffer_pc.message_counter = 0;
	uart_buffer_pc.rx_message_size_encoded = 25 + cobs_size_increase;
	uart_buffer_pc.tx_message_size_encoded = 24 + cobs_size_increase;
	uart_buffer_pc.cobs_buffer_encoded = cobs_buffer_encoded;
	uart_buffer_pc.cobs_buffer_decoded = cobs_buffer_decoded;
	HAL_UART_Receive_IT(uart_buffer_pc.uart_handle, &uart_buffer_pc.rx_byte, 1);

	// setup motor driver connected uart buffers
	motor_controller[0]=&huart2;
	motor_controller[1]=&huart3; //
	motor_controller[2]=&huart4; //
	motor_controller[3]=&huart1;
	motor_controller[4]=&huart5; //
	motor_controller[5]=&huart6; //
	for(int i=0; i<6; i++)
	{
		uart_buffer_motors[i].uart_handle = motor_controller[i];
		ring_buffer_init(&uart_buffer_motors[i].ringbuffer_rx, uart_buffer_motors[i].ringbuffer_rx_arr, RING_BUFFER_SIZE);
		ring_buffer_init(&uart_buffer_motors[i].ringbuffer_tx, uart_buffer_motors[i].ringbuffer_tx_arr, RING_BUFFER_SIZE);
		uart_buffer_motors[i].message_counter = 0;
		uart_buffer_motors[i].rx_message_size_encoded = 4 + cobs_size_increase;
		uart_buffer_motors[i].tx_message_size_encoded = 5 + cobs_size_increase;
		uart_buffer_motors[i].cobs_buffer_encoded = cobs_buffer_encoded;
		uart_buffer_motors[i].cobs_buffer_decoded = cobs_buffer_decoded;
		HAL_UART_Receive_IT(uart_buffer_motors[i].uart_handle, &uart_buffer_motors[i].rx_byte, 1);
	}
}

void clear_uart_buffer_overflow_flags()
{
	if (__HAL_UART_GET_FLAG(&hlpuart2, UART_FLAG_ORE)) {
		// Overrun error occurred, need to clear the flag
		__HAL_UART_CLEAR_OREFLAG(&hlpuart2); // Use the clear macro
		HAL_UART_Receive_IT(uart_buffer_pc.uart_handle, &uart_buffer_pc.rx_byte, 1);
//		uint8_t ok_message[] = "ov";
//		HAL_UART_Transmit(&hlpuart2, ok_message, 3, 1);
	}
	for(int i=0;i<6;i++)
	{
		if (__HAL_UART_GET_FLAG(uart_buffer_motors[i].uart_handle, UART_FLAG_ORE)) {
			// Overrun error occurred, need to clear the flag
			__HAL_UART_CLEAR_OREFLAG(uart_buffer_motors[i].uart_handle); // Use the clear macro
			HAL_UART_Receive_IT(uart_buffer_motors[i].uart_handle, &uart_buffer_motors[i].rx_byte, 1);
//			uint8_t ok_message[] = "om";
//			HAL_UART_Transmit(&hlpuart2, ok_message, 3, 1);
		}
	}
}

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
//	USART1_IRQHandler;
  /* USER CODE BEGIN 1 */

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
//  	__disable_irq();

	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	MX_LPUART2_UART_Init();
  /* USER CODE BEGIN 2 */
  for(int i=0;i<24;i++){
	  buffer_pc_tx[i]=0;
//	  buffer_down[i]=0;
  }
  float temp=256.0;
  for(int i=0;i<6;i++){
	  memcpy(&buffer_pc_tx[i*4],&temp,4);
  }
  uint32_t timestamp = 0;
  uint32_t upward_send_interval = 10;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	setup();
	timestamp = HAL_GetTick();
//	uint8_t nnnn;

//	__enable_irq();
	while (1)
	{
//		if(HAL_GetTick() - timestamp > 2000)
//		{
//			timestamp = HAL_GetTick();
//			uint8_t ok_message[] = "b";
//			HAL_UART_Transmit(&hlpuart2, ok_message, 1, 1);
//		}
		clear_uart_buffer_overflow_flags();

//		// handle new messages from pc
		message_status stat = retrieve_message(&uart_buffer_pc);
		if (stat == NEW_MESSAGE_IN_DECODED_COBS_BUFFER)
		{
			// start debugging
//			cobs_encode_result res = cobs_encode(cobs_buffer_encoded, cobs_buffer_encoded_size, &cobs_buffer_decoded[1], 24);
//			cobs_buffer_encoded[25] = 0; // zero delimiter byte
//			HAL_UART_Transmit(&hlpuart2, cobs_buffer_encoded, 26, 3);
			// end debugging

			// forward messages to motor controllers
			for(int i=0;i<6;i++)
			{
				uint8_t message[uart_buffer_motors[i].tx_message_size_encoded - cobs_size_increase];
				message[0] = cobs_buffer_decoded[0]; // controllbyte of message
				memcpy(&message[1], &cobs_buffer_decoded[1+i*4], uart_buffer_motors[i].tx_message_size_encoded - cobs_size_increase - 1);

				cobs_encode_result res = cobs_encode(cobs_buffer_encoded, cobs_buffer_encoded_size, message, uart_buffer_motors[i].tx_message_size_encoded - cobs_size_increase);
				cobs_buffer_encoded[uart_buffer_motors[i].tx_message_size_encoded - 1] = 0; // add zero delimiter byte
				ring_buffer_queue_arr(&uart_buffer_motors[i].ringbuffer_tx, cobs_buffer_encoded, uart_buffer_motors[i].tx_message_size_encoded);
				tx_interrupt_callback(&uart_buffer_motors[i]);
//				HAL_UART_Transmit(motor_controller[i], cobs_buffer_encoded, 5, 1);
//				HAL_UART_Transmit(&hlpuart2, message, 5, 1);
			}
////			// start debugging
//			cobs_encode_result res = cobs_encode(cobs_buffer_encoded, cobs_buffer_encoded_size, &cobs_buffer_decoded[1], 24);
//			cobs_buffer_encoded[25] = 0; // zero delimiter byte
//			HAL_UART_Transmit(&hlpuart2, cobs_buffer_encoded, 26, 3);
////			// end debugging
		}

//		// handle new messages from motor controllers
		for(int i=0;i<6;i++)
		{
			message_status stat = retrieve_message(&uart_buffer_motors[i]);
//			HAL_UART_Transmit(&hlpuart2, &stat, 1, 1);

//			float stat_as_float = stat;
//			memcpy(&buffer_pc_tx[i*4], &stat_as_float, 4);

//			buffer_pc_tx[i*4] = 0;
//			buffer_pc_tx[i*4+1] = 0;
//			buffer_pc_tx[i*4+2] = 0;
//			buffer_pc_tx[i*4+3] = 0;

//			if (stat != NO_NEW_MESSAGE)
//			{
//				HAL_UART_Transmit(&hlpuart2, &stat, 1, 1);
//			}
			if (stat == NEW_MESSAGE_IN_DECODED_COBS_BUFFER)
			{
				memcpy(&buffer_pc_tx[i*4], cobs_buffer_decoded, uart_buffer_motors[i].rx_message_size_encoded  - cobs_size_increase);
			}
		}

		if(HAL_GetTick() - timestamp > upward_send_interval)
		{
			timestamp = HAL_GetTick();
			cobs_encode_result res = cobs_encode(cobs_buffer_encoded, cobs_buffer_encoded_size, buffer_pc_tx, buffer_pc_tx_size);
			cobs_buffer_encoded[uart_buffer_pc.tx_message_size_encoded - 1] = 0; // add zero delimiter byte
			ring_buffer_queue_arr(&uart_buffer_pc.ringbuffer_tx, cobs_buffer_encoded, uart_buffer_pc.tx_message_size_encoded);
			tx_interrupt_callback(&uart_buffer_pc);
		}
	}
	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART2_UART_Init(void)
{

  /* USER CODE BEGIN LPUART2_Init 0 */

  /* USER CODE END LPUART2_Init 0 */

  /* USER CODE BEGIN LPUART2_Init 1 */

  /* USER CODE END LPUART2_Init 1 */
  hlpuart2.Instance = LPUART2;
  hlpuart2.Init.BaudRate = 115200;
  hlpuart2.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart2.Init.StopBits = UART_STOPBITS_1;
  hlpuart2.Init.Parity = UART_PARITY_NONE;
  hlpuart2.Init.Mode = UART_MODE_TX_RX;
  hlpuart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart2.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART2_Init 2 */

  /* USER CODE END LPUART2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==uart_buffer_pc.uart_handle)
	{
		rx_interrupt_callback(&uart_buffer_pc);
	}
	else if(huart==uart_buffer_motors[0].uart_handle)
	{
		rx_interrupt_callback(&uart_buffer_motors[0]);
	}
	else if(huart==uart_buffer_motors[1].uart_handle)
	{
		rx_interrupt_callback(&uart_buffer_motors[1]);
	}
	else if(huart==uart_buffer_motors[2].uart_handle)
	{
		rx_interrupt_callback(&uart_buffer_motors[2]);
	}
	else if(huart==uart_buffer_motors[3].uart_handle)
	{
		rx_interrupt_callback(&uart_buffer_motors[3]);
	}
	else if(huart==uart_buffer_motors[4].uart_handle)
	{
		rx_interrupt_callback(&uart_buffer_motors[4]);
	}
	else if(huart==uart_buffer_motors[5].uart_handle)
	{
		rx_interrupt_callback(&uart_buffer_motors[5]);
	}
}

//ring_buffer_size_t nn;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==uart_buffer_pc.uart_handle)
	{
		tx_interrupt_callback(&uart_buffer_pc);
	}
	else if(huart==uart_buffer_motors[0].uart_handle)
	{
		tx_interrupt_callback(&uart_buffer_motors[0]);
	}
	else if(huart==uart_buffer_motors[1].uart_handle)
	{
		tx_interrupt_callback(&uart_buffer_motors[1]);
	}
	else if(huart==uart_buffer_motors[2].uart_handle)
	{
		tx_interrupt_callback(&uart_buffer_motors[2]);
	}
	else if(huart==uart_buffer_motors[3].uart_handle)
	{
		tx_interrupt_callback(&uart_buffer_motors[3]);
	}
	else if(huart==uart_buffer_motors[4].uart_handle)
	{
		tx_interrupt_callback(&uart_buffer_motors[4]);
	}
	else if(huart==uart_buffer_motors[5].uart_handle)
	{
		tx_interrupt_callback(&uart_buffer_motors[5]);
	}
}
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
#ifdef USE_FULL_ASSERT
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
