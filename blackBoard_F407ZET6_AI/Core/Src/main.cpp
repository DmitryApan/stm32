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
#include "usb_device.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "XPT2046_touch.h"
#include "sine_model.h"
#include "tensorflow/lite/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
namespace {
tflite::ErrorReporter *error_reporter = nullptr;
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *model_input = nullptr;
TfLiteTensor *model_output = nullptr;

// Create an area of memory to use for input, output, and intermediate arrays.
// Finding the minimum value for your model may require some trial and error.
constexpr uint32_t kTensorArenaSize = 4 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
}

extern const float INPUT_RANGE = 2.f * 3.14159265359f;
const uint16_t INFERENCE_PER_CYCLE = 70;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEN_RACKET 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint8_t gameField[32][24] = {0};

uint16_t xBall = 0, yBall = 0;
uint16_t xBallLast = 0, yBallLast = 0;
int16_t xSpeedBall = 1, ySpeedBall = 1;

uint16_t xRacket = 13, yRacket = 23;
uint16_t xRacketLast = 0, yRacketLast = 0;
int16_t xSpeedRacket = 0;

uint16_t touchX = 0, touchY = 0;
bool touch = false;
uint16_t touchTime = 0;

typedef struct {
	float speedX;
	float speedY;
	float curX;
	float curY;
} BallMove;

BallMove ballMovHistory[128] = {0};
uint8_t indexBallMove = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void initGame() {
	lcdTearingOn(false);

	lcdFillRGB(0xFFFF);

	for (int i = 0; i < 32; i++) {
		lcdDrawVLine(10 * i, 0, 239 - 10, 0xEF9D);
	}

	for (int i = 0; i < (24 - 1); i++) {
		lcdDrawHLine(0, 319, 10 * i, 0xEF9D);
	}
}

void gameRender() {

	lcdFillRect(10 * xBallLast, 10 * yBallLast, 10, 10, COLOR_WHITE);
	lcdFillRect(10 * xBall, 10 * yBall, 10, 10, COLOR_RED);

	xBallLast = xBall;
	yBallLast = yBall;

	if ((xRacket != xRacketLast) || (yRacket != yRacketLast)) {
		lcdFillRect(10 * xRacketLast, 10 * yRacketLast + 1, 10 * (LEN_RACKET), 9, COLOR_WHITE);
		lcdFillRect(10 * xRacket, 10 * yRacket + 1, 10 * (LEN_RACKET), 9, COLOR_BLUE);

		xRacketLast = xRacket;
		yRacketLast = yRacket;
	}
}

void gameLogic() {
	xSpeedRacket = 0;

	// Работа нейросети
	model_input->data.f[0] = (xSpeedBall < 0) ? 0.0f : 1.0f;
	model_input->data.f[1] = (ySpeedBall < 0) ? 0.0f : 1.0f;
	model_input->data.f[2] = xBall / 31.0f;
	model_input->data.f[3] = yBall / 22.0f;

	// Run inference, and report any error
	TfLiteStatus invoke_status = interpreter->Invoke();
	if (invoke_status != kTfLiteOk) {
		return;
	}

	float x_racket = model_output->data.f[0];
	uint16_t xRacketScale = round(x_racket * (float)(32 - LEN_RACKET));
	DebugLogFormatted("%d \n", xRacketScale);

	int deltaXracket = xRacketScale - xRacket;

	if (abs(deltaXracket) > 20)
		xSpeedRacket = 3;
	else if (abs(deltaXracket) > 10)
		xSpeedRacket = 2;
	else if (abs(deltaXracket))
		xSpeedRacket = 1;

	if (deltaXracket < 0)
		xSpeedRacket *= -1;

	// обработка нажатий пользователем
	if(XPT2046_TouchPressed())
	{
		if(XPT2046_TouchGetCoordinates(&touchX, &touchY))
		{
			touch = true;
			touchTime++;

			if (touchTime > 4) {
				if (touchX > (320 / 2)) {
					xSpeedRacket = -3;
				} else {
					xSpeedRacket = 3;
				}
			} else if (touchTime > 1) {
				if (touchX > (320 / 2)) {
					xSpeedRacket = -2;
				} else {
					xSpeedRacket = 2;
				}
			}
		}
	} else {
		if (touch) {
			touch = false;
			touchTime = 0;

			if (touchX > (320 / 2)) {
				xSpeedRacket = -1;
			} else {
				xSpeedRacket = 1;
			}
		}
	}

	// обработка координат ракетки
	if (xSpeedRacket > 0)
		for (int i = 0; i < xSpeedRacket; i++)
			if (xRacket + LEN_RACKET - 1 < 31)
			xRacket++;

	if (xSpeedRacket < 0)
		for (int i = 0; i > xSpeedRacket; i--)
			if (xRacket)
				xRacket--;

	// обработка скорости и координат шарика
	if (xSpeedBall > 0) {
		if (xBall >= 31) {
			xBall--;
			xSpeedBall = -1;
		} else {
			xBall++;
		}
	} else {
		if (!xBall) {
			xBall++;
			xSpeedBall = 1;
		} else {
			xBall--;
		}
	}

	if (ySpeedBall > 0) {
		if (yBall != 22) {
			yBall++;
		} else {
			if ((xBallLast >= xRacketLast) && xBallLast <= (xRacketLast + LEN_RACKET - 1)) {
				yBall--;
				ySpeedBall = -1;
			} else {
				xBall = 0;
				yBall = 0;
				xSpeedBall = 1;
				ySpeedBall = 1;
			}
		}
	} else {
		if (!yBall) {
			yBall++;
			ySpeedBall = 1;
		} else {
			yBall--;
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_FSMC_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	lcdInit();
	lcdSetOrientation(LCD_ORIENTATION_LANDSCAPE);
	lcdFillRGB(0xFFFF);
	LCD_BL_ON();

	static tflite::MicroErrorReporter micro_error_reporter;
	error_reporter = &micro_error_reporter;

	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	model = tflite::GetModel(sine_model);

	if (model->version() != TFLITE_SCHEMA_VERSION) {
		TF_LITE_REPORT_ERROR(error_reporter,
				"Model provided is schema version %d not equal "
						"to supported version %d.", model->version(),
				TFLITE_SCHEMA_VERSION);
		return 0;
	}

	// This pulls in all the operation implementations we need.
	static tflite::ops::micro::AllOpsResolver resolver;

	// Build an interpreter to run the model with.
	static tflite::MicroInterpreter static_interpreter(model, resolver,
			tensor_arena, kTensorArenaSize, error_reporter);
	interpreter = &static_interpreter;

	// Allocate memory from the tensor_arena for the model's tensors.
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
		return 0;
	}

	// Obtain pointers to the model's input and output tensors.
	model_input = interpreter->input(0);
	model_output = interpreter->output(0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	initGame();

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//		ballMovHistory[indexBallMove].speedX = (xSpeedBall < 0) ? 0.0f : 1.0f;
//		ballMovHistory[indexBallMove].speedY = (ySpeedBall < 0) ? 0.0f : 1.0f;
//		ballMovHistory[indexBallMove].curX = xBall / 31.0f;
//		ballMovHistory[indexBallMove++].curY = yBall / 22.0f;
//
//		if (yBall == 22) {
//
//			static char cycleStr[2048] = {0};
//			static char stepStr[32] = {0};
//
//			cycleStr[0] = 0;
//
//			for (int i = 0; i < indexBallMove; i++)
//			{
//				sprintf(stepStr, "%1.1f %1.1f %1.2f %1.2f %1.2f\n", ballMovHistory[i].speedX, ballMovHistory[i].speedY, ballMovHistory[i].curX, ballMovHistory[i].curY, xRacket / (float)(32 - (LEN_RACKET)));
//				strcat(cycleStr, stepStr);
//			}
//
//			DebugLogFormatted("%s", cycleStr);
//
//			indexBallMove = 0;
//		}

		gameLogic();
		gameRender();

		HAL_Delay(100);

		/* USER CODE END 3 */
	}
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, T_SCK_Pin|LCD_BL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(T_MOSI_GPIO_Port, T_MOSI_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : T_CS_Pin */
	GPIO_InitStruct.Pin = T_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(T_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : T_SCK_Pin LCD_BL_Pin */
	GPIO_InitStruct.Pin = T_SCK_Pin|LCD_BL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : T_IRQ_Pin */
	GPIO_InitStruct.Pin = T_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(T_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : T_MISO_Pin */
	GPIO_InitStruct.Pin = T_MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(T_MISO_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : T_MOSI_Pin */
	GPIO_InitStruct.Pin = T_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(T_MOSI_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void) {

	/* USER CODE BEGIN FSMC_Init 0 */

	/* USER CODE END FSMC_Init 0 */

	FSMC_NORSRAM_TimingTypeDef Timing = {0};

	/* USER CODE BEGIN FSMC_Init 1 */

	/* USER CODE END FSMC_Init 1 */

	/** Perform the SRAM1 memory initialization sequence
	 */
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK4;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
	/* Timing */
	Timing.AddressSetupTime = 2;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 2;
	Timing.BusTurnAroundDuration = 0;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN FSMC_Init 2 */

	/* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

