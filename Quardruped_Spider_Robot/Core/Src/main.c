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
#include <stdio.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float alpha;
    float beta;
    float gamma;
} angle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PCA9685_ADDRESS 0x80
// Datasheet link --> https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
#define PCA9685_MODE1         0x0         // as in the datasheet page no 10/52
#define PCA9685_PRE_SCALE     0xFE        // as in the datasheet page no 13/52
#define PCA9685_LED0_ON_L     0x6         // as in the datasheet page no 10/52
#define PCA9685_MODE1_SLEEP_BIT      4    // as in the datasheet page no 14/52
#define PCA9685_MODE1_AI_BIT         5    // as in the datasheet page no 14/52
#define PCA9685_MODE1_RESTART_BIT    7    // as in the datasheet page no 14/52
#define NUM_LEGS 4
#define NUM_SERVOS_PER_LEG 3
#define NUM_SETS 20

// Mảng lưu trữ góc cho từng chân, từng bộ giá trị và từng servo
const float servoAngles[NUM_LEGS][NUM_SETS][NUM_SERVOS_PER_LEG] = {
    // Dữ liệu cho chân 1
    {
        {1.5,  0.3,	 -2.5},
        {49.5, 92.7, 98.3},
        {50.1, 75.9, 60.7},
        {52.4, 81.3, 94.5},
        {53.9, 85.2, 69.8},
        {56.7, 65.0, 101.9},
        {58.3, 76.8, 89.0},
        {61.1, 98.4, 62.3},
        {62.5, 81.9, 102.7},
        {64.8, 92.1, 97.4},
        {66.2, 85.5, 63.1},
        {68.9, 91.3, 95.8},
        {71.5, 88.0, 54.6},
        {73.7, 65.9, 101.3},
        {75.6, 72.7, 90.9},
        {78.4, 94.5, 61.7},
        {80.0, 79.2, 98.6},
        {82.9, 83.1, 66.5},
        {84.3, 99.0, 54.3},
        {85.7, 88.8, 97.2}
    },
    // Dữ liệu cho chân 2
    {
        {48.1, 65.7, 101.6},
        {49.0, 91.4, 97.5},
        {50.3, 74.2, 61.3},
        {51.9, 82.5, 92.9},
        {53.8, 86.1, 70.2},
        {56.1, 66.9, 99.4},
        {57.5, 78.7, 88.1},
        {60.2, 96.3, 63.4},
        {61.8, 82.1, 101.2},
        {63.5, 93.0, 96.8},
        {65.0, 84.8, 62.9},
        {67.3, 90.7, 94.7},
        {69.6, 87.6, 56.2},
        {71.1, 64.7, 100.9},
        {73.4, 73.2, 89.3},
        {76.5, 93.7, 64.8},
        {78.7, 80.1, 97.9},
        {80.9, 84.0, 65.3},
        {83.1, 98.7, 53.9},
        {84.6, 87.5, 99.8}
    },
    // Dữ liệu cho chân 3
    {
        {47.5, 66.2, 100.3},
        {48.9, 92.9, 96.1},
        {50.4, 73.8, 60.1},
        {51.2, 80.8, 93.5},
        {54.3, 87.9, 71.5},
        {55.7, 64.3, 98.2},
        {57.8, 77.2, 87.4},
        {59.4, 97.1, 65.0},
        {61.6, 83.5, 100.7},
        {62.9, 91.7, 95.3},
        {65.4, 86.4, 63.5},
        {66.9, 89.9, 94.0},
        {68.1, 66.7, 55.4},
        {70.3, 72.5, 99.7},
        {73.0, 94.1, 88.8},
        {74.8, 78.3, 66.9},
        {76.2, 91.0, 99.1},
        {79.4, 82.7, 64.0},
        {81.0, 97.9, 53.5},
        {82.7, 86.5, 98.4}
    },
    // Dữ liệu cho chân 4
    {
        {46.8, 64.9, 99.8},
        {49.3, 90.3, 96.7},
        {51.0, 73.0, 59.4},
        {53.1, 79.7, 94.1},
        {55.4, 88.7, 72.8},
        {57.3, 65.8, 98.9},
        {58.6, 76.3, 86.2},
        {60.5, 95.2, 64.1},
        {62.4, 84.9, 101.5},
        {63.7, 92.4, 97.1},
        {66.1, 85.1, 62.1},
        {67.8, 89.1, 95.4},
        {68.9, 67.1, 54.8},
        {71.6, 73.9, 101.0},
        {72.5, 93.3, 89.5},
        {75.9, 79.4, 65.6},
        {78.1, 90.5, 98.2},
        {79.7, 83.8, 63.2},
        {82.0, 98.5, 54.0},
        {83.8, 87.0, 100.0}
    }
};

//// Hàm đọc góc từ file
//void readAnglesFromFile(const char* filename, int legIndex) {
//    FILE *file = fopen(filename, "r");
//    if (file == NULL) {
//        printf("Không thể mở file %s\n", filename);
//        return;
//    }
//    for (int i = 0; i < NUM_SETS; i++) {
//        fscanf(file, "%f %f %f", &servoAngles[legIndex][i][0], &servoAngles[legIndex][i][1], &servoAngles[legIndex][i][2]);
//    }
//
//    fclose(file);
//}
//void loadAllAngles() {
//    // Đọc dữ liệu góc từ các file vào các mảng
//    readAnglesFromFile("leg1_angles.txt", 0);
//    readAnglesFromFile("leg2_angles.txt", 1);
//    readAnglesFromFile("leg3_angles.txt", 2);
//    readAnglesFromFile("leg4_angles.txt", 3);
//}

void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
  uint8_t readValue;
  // Read all 8 bits and set only one bit to 0/1 and write all 8 bits back
  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  if (Value == 0) readValue &= ~(1 << Bit);
  else readValue |= (1 << Bit);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
  HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency)
{
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;
  //  internal 25 MHz oscillator as in the datasheet page no 1/52
  else prescale = 25000000 / (4096 * frequency);
  // prescale changes 3 to 255 for 1526Hz to 24Hz as in the datasheet page no 1/52
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(uint16_t frequency)
{
  PCA9685_SetPWMFrequency(frequency); // 50 Hz for servo
  PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
  uint8_t registerAddress;
  uint8_t pwm[4];
  registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
  // See example 1 in the datasheet page no 18/52
  pwm[0] = OnTime & 0xFF;  //BYTE 	thấp
  pwm[1] = OnTime>>8;	   //byte cao
  pwm[2] = OffTime & 0xFF;
  pwm[3] = OffTime>>8;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
  float Value;
  // độ phân giải của pwm của pca9685 là 12bit -> tương ứng với giá trị 2^12-1=4095
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
  Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

void init_gait(void)
{
	  // nhin tu phia truoc
	  //1: truoc phai
		PCA9685_SetServoAngle(0, 45); //tang : nguoc kim
		PCA9685_SetServoAngle(1, 135 ); // tang : huong len
		PCA9685_SetServoAngle(2, 135); //tang : thu vao
	  //2: truoc trai
		PCA9685_SetServoAngle(4, 140); //tang : nguoc kim
		PCA9685_SetServoAngle(5, 45); //tang : huong xuong
		PCA9685_SetServoAngle(6, 45); //tang : mo ra
	  //3: sau trai
		PCA9685_SetServoAngle(8, 45); // tang : nguoc kim
		PCA9685_SetServoAngle(9, 45); // tang : huong xuong
		PCA9685_SetServoAngle(10, 45); // tang : mo ra
	  //4: sau phai
		PCA9685_SetServoAngle(12, 135); // tang : nguoc kim
		PCA9685_SetServoAngle(13, 125); // tang : huong len
		PCA9685_SetServoAngle(14, 125); //tang : thu vao

}
void test(void)
{
	  // nhin tu phia truoc
	  //1: truoc phai
		PCA9685_SetServoAngle(0, 90); //tang : nguoc kim //goc
		PCA9685_SetServoAngle(1, 135); // tang : huong len
		PCA9685_SetServoAngle(2, 135); //tang : thu vao
	  //2: truoc trai
		PCA9685_SetServoAngle(4, -45 + 180 + 5); //tang : nguoc kim // goc +180
		PCA9685_SetServoAngle(5, 45); //tang : huong xuong
		PCA9685_SetServoAngle(6, 45); //tang : mo ra
	  //3: sau trai
		PCA9685_SetServoAngle(8, -135 + 180); // tang : nguoc kim // goc +180
		PCA9685_SetServoAngle(9, 45); // tang : huong xuong
		PCA9685_SetServoAngle(10, 45); // tang : mo ra
	  //4: sau phai
		PCA9685_SetServoAngle(12, 90); // tang : nguoc kim// goc
		PCA9685_SetServoAngle(13, 135-10); // tang : huong len
		PCA9685_SetServoAngle(14, 135-10); //tang : thu vao

}
void moveRobotStraight() {
	for (int set = 0; set < NUM_SETS; set++) {  // bo goc
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (int servo = 0; servo < NUM_SERVOS_PER_LEG; servo++) {
            	int channel = 0;
            	float adjustedAngle = servoAngles[leg][set][servo];
				switch (leg) {
					case 0: channel = servo; break;           // Chân 1: Kênh 0 đến 2
					case 1: 							      // Chân 2: Kênh 4 đến 6
						channel = servo + 4;
						if (servo == 0) {
							adjustedAngle += 180.0;
						}
						break;
					case 2: 								  // Chân 3: Kênh 8 đến 10
						channel = servo + 8;
						if (servo == 0) {
						adjustedAngle += 180.0;
						}
						break;
					case 3: channel = servo + 12; break;      // Chân 4: Kênh 12 đến 14
				}
				PCA9685_SetServoAngle(channel, adjustedAngle);
            }
        }
        // Chờ một khoảng thời gian ngắn trước khi lặp lại, để tạo chuyển động mượt mà
        HAL_Delay(1000);  // Điều chỉnh thời gian chờ để phù hợp với tốc độ di chuyển mong muốn
    }
}
/*void walk(uint8_t leg, float alpha, float beta, float gamma)
{
		if (leg == 0)
		{
		  	alpha = 90 - alpha - 10;
		  	beta = beta;
		  	gamma += 90;
		}
		else if (leg == 1)
		{
		  	alpha += 90 + 7;
		  	beta = 180 - beta;
		  	gamma = 90 - gamma + 3;
		}
		else if (leg == 2)
		{
		  	alpha += 90 + 5;
		  	beta = 180 - beta + 8;
		  	gamma = 90 - gamma - 10;
		}
		else if (leg == 3)
		{
		  	alpha = 90 - alpha - 10;
		  	beta = beta + 6;
		  	gamma += 90 - 10;
		}
	  uint8_t Channel_0 = leg*4;
	  uint8_t Channel_1 = leg*4 + 1;
	  uint8_t Channel_2 = leg*4 + 2;
	  PCA9685_SetServoAngle(Channel_0, alpha);
	  PCA9685_SetServoAngle(Channel_1, beta);
	  PCA9685_SetServoAngle(Channel_2, gamma);
}*/
void bouncing(void)
{
	  // nhin tu phia truoc
	  //1: truoc phai
		PCA9685_SetServoAngle(0, 45); //tang : nguoc kim
		PCA9685_SetServoAngle(1, 135 - 60); // tang : huong len
		PCA9685_SetServoAngle(2, 135 - 60); //tang : thu vao
	  //2: truoc trai
		PCA9685_SetServoAngle(4, 140); //tang : nguoc kim
		PCA9685_SetServoAngle(5, 45 + 60); //tang : huong xuong
		PCA9685_SetServoAngle(6, 45 + 60); //tang : mo ra
	  //3: sau trai
		PCA9685_SetServoAngle(8, 45); // tang : nguoc kim
		PCA9685_SetServoAngle(9, 45 + 60); // tang : huong xuong
		PCA9685_SetServoAngle(10, 45 + 60); // tang : mo ra
	  //4: sau phai
		PCA9685_SetServoAngle(12, 135); // tang : nguoc kim
		PCA9685_SetServoAngle(13, 125 - 60); // tang : huong len
		PCA9685_SetServoAngle(14, 125 - 60); //tang : thu vao

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(50); // 50Hz for servo
  init_gait();

  //loadAllAngles();  // Đọc dữ liệu góc từ file
  HAL_Delay(1000);
  test();
  //moveRobotStraight();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//bouncing();
	//HAL_Delay(500);
	//init_gait();
	//HAL_Delay(500);

    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
