/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define max 50
#define e_max 1000
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
int  des_dev, cur_dev;                                //variables for error, set-point and process variable
int ARR =10;
float kp = 10 , ki = 0, kd = 4;                     //PID constants
/*float Kp = 5;                                       //set up the constants value
float Ki = 0.001;
float Kd = 0.5;
int P, I, D; */
int i, j, m, n, k;
int speed = 40;                                    //base speed for motor PWM
//int lastError = 0;
unsigned int sensor[5],h[5];                        // an array to hold sensor values
//const uint8_t basespeed = 40;
//const uint8_t maxspeed = 100;
//unsigned char found_left=0;
//unsigned char found_straight=0;
//unsigned char found_right=0;
int sensor_weight[] = {-4 , -2 , 0 , 2 , 4};
int error,Error;                                    //variables for error, set-point and process variable
int result;                       //variables for error, set-point and process variable
int c=0, maze_solved=0,temp=0, mode=0;
int e_sum=0, e_diff=0, e_old=0, pid_out;
char path[100];  //array for storing path data of dry run
int pathLength=0;
//char line_maze = "", solve_maze = ""; // Memory string



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
//int read_sensor();
//float PID_control(float err);
int readSensor();           //for taking sensor input
float err_calc();           //for calculating error value
float pid(float);           //returns PID correction output
void motorPIDcontrol(int base);
void Read_Path();
void Save_Path(char state);  //saves dry run data
void Make_Decession(char Incoming_Command);
float PID_control(float err);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*-----------------------------------------------------------Forward-------------------------------------------------------*/
void Forward(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ARR*PWM_L);     //Left
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ARR*PWM_R);     //Right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

}
/*-----------------------------------------------------------Backward-------------------------------------------------------*/
void Backward(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ARR*PWM_L);      //Left  A_EN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ARR*PWM_R);      //Right B_EN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}
/*---------------------------------------------------------------stop--------------------------------------------------------------*/
void stop()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // A_EN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_0, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // B_EN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
}
/*-------------------------------------------------------------------Turn_Right------------------------------------------------------------*/
void Turn_Left(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ARR*PWM_L);      //Left  A_EN
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ARR*PWM_R);      //Right B_EN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

}
/*-------------------------------------------------------------------Turn_Left----------------------------------------------------------*/
void Turn_Right(double PWM_L,double PWM_R)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ARR*PWM_L);     //Left
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ARR*PWM_R);     //Right
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

}
/*-----------------------------------------------*******----**----runExtraInch--------------------------------------------------------*/
void runExtraInch() //running few steps for alignment and node identification
{
	Forward(40,40);          //OCR1A = 500;  	OCR1B = 500;
	HAL_Delay(150);         //_delay_ms(330);
	stop();
	HAL_Delay(800);
}
/*-------------------------------------------------------------------------------------------------------------*/
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);                       //starts the PMW
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*-----------------------------------*/
	         // Turn_Right(0,60);    Turn_Left(60,0);
	  //Read_Path();
       mode = readSensor();
		 switch(mode)
		 {
		  case 27:   //11011  [ cases for straight and curved path following ]
		  case 29:   //11101
		  case 23:   //10111
		  case 25:   //11001
		  case 19:   //10011
		  case 17:   //10001
			  motorPIDcontrol(speed);
			 // Forward(50,50);
			  break;
		  case 31:                                //11111
			  stop();
			  break;
	/*		 case 30:                             //11110  [ cases for a right turn or straight ]
				  stop();
				  HAL_Delay(500);
				  Turn_Right(0,50);
			  	  HAL_Delay(1400);
			  	  break;
			  case 15:                             //01111  [cases for left turn]
				  stop();
				  HAL_Delay(500);
				  Turn_Left(50,0);
				  HAL_Delay(1400);
				 break;*/
		 default:                                //for any other case straight and curved path following
			 motorPIDcontrol(speed);

		 }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*--------------------------------------------------------------readSensor--------------------------------------------------------*/
int readSensor()
{
	sensor[0]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	sensor[1]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);
	sensor[2]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
	sensor[3]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);
	sensor[4]= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4);

	k =0;
	for(  i=0;i<5;i++){
k= sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4]*1;

	}
	return(k);
}
/*--------------------------------------------------------------err_calc---------------------------------------------------------*/
 float err_calc()
{
	 cur_dev=0;
	//Error = 0;
	des_dev =0;
	h[5] = 0;
	m=0;
	for(i=1;i<4;i++)
	{
		if(sensor[i]==0)
		{
			h[i]=1;
		}
		else
		{
			h[i]=0;
			m++;
		}
		cur_dev += h[i] * sensor_weight[i];     //cur_dev stores deviation value //Error
	}
		if(m==0)
		{
			m=1;
		}
		cur_dev = (cur_dev / m);    //*1000;
		error =des_dev - cur_dev;
		return error;
}
 /*-------------------------------------------------------------PID_control------------------------------------------------------*/
 float PID_control(float err)
 {
	   e_diff = err - e_old;
 	    int PID = (kp * err) + (kd * e_diff) ;
 	   e_old = err;
 		/* if(PID > max)       //condition for keeping the pid output within the limits of the base value of motor rpm
 		 {
 			PID = max;
 		 }
 		 else if (PID < -max)
 		 {
 			PID = -max;
 		 }*/
 	   return PID;
 }
 /*---------------------------------------------------------pid---------------------------------------------------------------*/
 float pid(float err)
{
	 e_sum += err;
	 e_diff = err - e_old;

	 if (e_sum > e_max)        //condition for keeping the integral of error over a period of time within a limit
	 {
		 e_sum = e_max;
	 }
	 else if (e_sum < -e_max)
	 {
		 e_sum = -e_max;
	 }

	 pid_out = (kp * err) + (ki * e_sum) + (kd * e_diff);
	 e_old = err;

	 if(pid_out > max)       //condition for keeping the pid output within the limits of the base value of motor rpm
	 {
		 pid_out = max;
	 }
	 else if (pid_out < -max)
	 {
		 pid_out = -max;
	 }
	 return pid_out;
}
/*-----------------------------------------------------motorPIDcontrol----------------------------------------------------------*/
 void motorPIDcontrol(int base)
 {
       //Forward( base - (pid(err_calc())), base + (pid(err_calc())));
	 Forward( base + (PID_control(err_calc())), base - (PID_control(err_calc())));
 }
/*------------------------------------------------------Read_Path--------------------------------------------------------------*/
 void Read_Path()
 {
	 while(!maze_solved)  //condition for Read Path
	 {
		 mode = readSensor();
		 switch(mode)
		 {
		 case 00:    //00000 all black
			 stop();
			 runExtraInch();
			 temp = 0;
			 temp = readSensor();
			 if(temp == 0) //all black in next step then end of maze
			 {
				// mazeEnd();
			 }
			 else //any case in next step after all black - left turn
			 {
				 Turn_Left(0,40);
				 HAL_Delay(110);
				 Save_Path('L');          //value ‘L’ saved for left turn
			 }
			 break;

		 case 31:                       //11111 [ all white - end of line so U-turn ]
			 stop();
			 HAL_Delay(100);
			 Backward(40,40);          //right motor set for reverse rotation for U-turning
			 HAL_Delay(180);
			 Save_Path('U');           //value ‘U’ saved for U-turn
			 break;

		  case 27:   //11011  [ cases for straight and curved path following ]
		  case 25:   //11001
		  case 19:   //10011
		  case 17:   //10001
		  case 29:   //11101
		  case 23:   //10111
			  Forward(40,40);
			  //motorPIDcontrol(speed);
			 break;

		  case 30:   //11110  [ cases for a right turn or straight ]
		  case 28:   //11100
			  stop();
			  runExtraInch();
			  temp = 0;
			  temp = readSensor();
			  if(temp == 31)            //case for a right turn
			  {
				  Turn_Right(40,0);
				  HAL_Delay(110);
				  Save_Path('R');       //value ‘R’ saved for right turn
			  }
			  else        //case for straight
			  {
				  Forward(40,40);
				  //motorPIDcontrol(speed);
				  Save_Path('S');       //value ‘S’ saved for straight
			  }
			 break;

		  case 15:   //01111  [cases for left turn]
		  case 7:   //00111
			  stop();
			  runExtraInch();
			  Turn_Left(0,40);
			  HAL_Delay(110);
			  Save_Path('L');     //value ‘L’ saved for left turn
			 break;

		 default: //for any other case straight and curved path following
			 Forward(40,40);
			 //motorPIDcontrol(speed);
		 }
	 }
 }
/*-----------------------------------------------------------Save_Path----------------------------------------------------------*/
 void Save_Path(char state)
 {
	 path[pathLength] = state;
	 pathLength++;
	 HAL_Delay(1);

 }

void Make_Decession(char Incoming_Command)
{
  switch (Incoming_Command) {
    case 'S':
    	motorPIDcontrol(speed); //Go_one_step();
      break;

    case 'R':
      //GO_RIGHT();
      break;

    case 'L':
      //GO_LEFT();
      break;

    case 'O':
      //Finish_Point();
      break;
  }
}
/*------------------------------------------------------------------------------------------------------------------------*/
//FOLLOW_THE_RULES()

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
