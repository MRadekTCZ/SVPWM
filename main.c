/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FPGA 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#include <math.h>
#include <string.h>
#include "lcd_i2c.h"
#include "sin_lookup.h"
#include "string.h"
#include "MRB_VSI.h"
#define PI 3.141592653//;
#define one_by_sqrt6 0.40824829
#define one_by_sqrt2 0.70710678
#define sqrt2_by_sqrt3 0.816496581
#define pi_by_3 1.04719755
#define Ts 0.00002 // 200 kHz timer
#define Deadtime 0.00002 // 200 kHz timer
#define Timp 0.0005 //1kHz  updating sin,cons and atan2, switching happens 6xTimp -> 6kHz
float u_alfa;
float u_beta;
float u_zero; //zero U can be added if assymetry
float angle; //actual angle of PLL alfa beta vectors
int sektor; // actual sector of PLL angle (1 of 6);
float u_alfa_1; //part of alfa vector
float u_alfa_2; //part of beta vector
float u_beta_1;
float u_beta_2;
double imp_counter;
double time1_vector,time2_vector,time0_vector;
float U_dc;
short int OUT[6][3] = {{1,0,0},{1,1,0},{0,1,0},{0,1,1},{0,0,1},{1,0,1}}; //space vectors
short int OUT0[3] = {0,0,0}; //zero vectors, 0 [0,0,0] or 7 [1,1,1]
float Magnitude; //Magnitude of sinusoid for U/f control
double time; //time counter
float frequency;
unsigned short int T1,T2,T3,T4,T5,T6;

union TState
{
struct ThyriState
{
	short unsigned int GPIO0:1;
	short unsigned int GPIO1:1;
	short unsigned int GPIO2:1;
	short unsigned int GPIO3:1;
	short unsigned int GPIO4:1;
	short unsigned int GPIO5:1;
	short unsigned int rest:10;
}GpioState;
short unsigned int GPIO16bit;
}TState16bit,T_LastState16bit, Bits_changed_xor; //16 bit gpio bits for port C 6 ports represented by tranzistors


volatile static uint16_t adc[2]; //adc read DMA
uint32_t cntrl;  //32bits for 4 bytes from UART -> remote control
uint16_t f_set;  //intiger frequency from UART, has to be divided by 10;

struct lcd_disp disp;
char str_freq[8]; //frequency for sending through uart

enum VSI_STATE VSI_state;
enum VSI_CONTROL VSI_control;
struct RAMP_PARAMS Ramp;


//TO FPGA
uint16_t u_timer_vector1,u_timer_vector2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	U_dc = 540; //Napiecie DC, normalnie by³oby mierzone na uk³adzie posredniczaczym pomiedzy zaciskami kondensatora

	TState16bit.GpioState.rest = 0;
	T_LastState16bit.GpioState.rest = 0;

	Ramp.ramp_enable = 1;
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
  MX_DMA_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim10);
#ifndef FPGA
  HAL_TIM_Base_Start_IT(&htim11);
#endif
 HAL_TIM_Base_Start_IT(&htim3);


  HAL_ADC_Start_DMA(&hadc1, &adc, 2);
  HAL_UART_Transmit_IT(&huart2, "\nF0     ", strlen("\nF0     "));
  HAL_UART_Receive_IT(&huart2, &cntrl, 4);

  disp.addr=(0x27<<1);
  disp.bl=1;
  lcd_init(&disp);
  lcd_clear(&disp);
  sprintf((char *)disp.f_line,"STARTING");
  HAL_Delay(2000);
  lcd_display(&disp);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  f_set = frequency;
	  if(VSI_state == on)
	  {
		  sprintf((char *)disp.f_line,"INV ON freq:%d", f_set);
		  HAL_Delay(400);
		  lcd_display(&disp);

	  }
	  else if (VSI_state == off)
	  {
		  sprintf((char *)disp.f_line,"INV OFF freq:%d", f_set);
		  HAL_Delay(400);
		  lcd_display(&disp);
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{


		// ON AND OFF
		if (cntrl == 0x31314141) //Start command AA11
		{
		VSI_state = on;
		HAL_UART_Transmit_IT(&huart2, "\nVSI on ", strlen("\nVSI on ")); // confirm sending message
		ErrorCode = NoError;
		}
		else if (cntrl == 0x32324141) //Off command AA22
		{
		VSI_state = off;
		HAL_UART_Transmit_IT(&huart2, "\nVSI off", strlen("\nVSI off")); // confirm sending message
		ErrorCode = NoError;
		}

		// SEE ACTUAL FREQUENCY
		else if (cntrl == 0x31314343) //Off command CC11 command to print frequency
		{
		MRB_FLOAT_TO_ASCII(str_freq,frequency);
		//HAL_UART_Transmit_IT(&huart2, "\nSet frequency: " , strlen("\nSet frequency: "));
		HAL_UART_Transmit_IT(&huart2, str_freq , 8); // confirm sending message
		ErrorCode = NoError;
		}
		//START RAMP
		else if (cntrl == 0x45454444) //Start ramp command DDEE
				{
				VSI_control = ramp;
				HAL_UART_Transmit_IT(&huart2, "\nRamp ok", strlen("\nRamp ok")); // confirm sending message
				ErrorCode = NoError;
				}

		// SOFT START
		else if ((cntrl|0xFFFF0000) == 0xFFFF4443) //Off command CDxx command to make a soft start, xx is time of soft start lasts till reaching 50 Hz
		{
			if(MRB_ASCII_ISNUMBER((cntrl&0xFF000000)>>24)&&MRB_ASCII_ISNUMBER((cntrl&0x00FF0000)>>16))
			{
				if(MRB_ASCII_TO_NUMBER((cntrl&0x00FF0000)>>16) > 2) //Maximal time of soft start is 29
				{
					HAL_UART_Transmit_IT(&huart2, "\nF4     ", strlen("\nF4     ")); //Set time is too long
					ErrorCode = SetTimeTooLong;
				}
				else
				{
					Ramp.ramp_time = MRB_ASCII_TO_NUMBER((cntrl&0x00FF0000)>>16)*10+MRB_ASCII_TO_NUMBER((cntrl&0xFF000000)>>24);
					HAL_UART_Transmit_IT(&huart2, "\nSSrt ok", strlen("\nSSrt ok")); //command  time set ok
					VSI_control = soft_start;
					ErrorCode = NoError;
				}
			}else
				{
				HAL_UART_Transmit_IT(&huart2, "\nF3     ", strlen("\nF3     ")); //Wrong input - this is not a number
				ErrorCode = NotANumber;
				}
		}

		// SOFT STOP
		else if ((cntrl|0xFFFF0000) == 0xFFFF4543) //Off command CExx command to make a soft stop, xx is time of soft stop lasts till reaching 0 Hz
		{
			if(MRB_ASCII_ISNUMBER((cntrl&0xFF000000)>>24)&&MRB_ASCII_ISNUMBER((cntrl&0x00FF0000)>>16))
			{
				if(MRB_ASCII_TO_NUMBER((cntrl&0x00FF0000)>>16) > 2) //Maximal time of soft start is 29
				{
					HAL_UART_Transmit_IT(&huart2, "\nF4     ", strlen("\nF4     ")); //Set time is too long
					ErrorCode = SetTimeTooLong;
				}
				else
				{
					Ramp.ramp_time = MRB_ASCII_TO_NUMBER((cntrl&0x00FF0000)>>16)*10+MRB_ASCII_TO_NUMBER((cntrl&0xFF000000)>>24);
					HAL_UART_Transmit_IT(&huart2, "\nStop ok", strlen("\nStop ok")); //command  time set ok
					VSI_control = soft_stop;
					ErrorCode = NoError;
				}
			}else
				{
				HAL_UART_Transmit_IT(&huart2, "\nF3     ", strlen("\nF3     ")); //Wrong input - this is not a number
				ErrorCode = NotANumber;
				}
		}

		// SET frequency of ramp
		else if ((cntrl|0xFFFFFF00) == 0xFFFFFF44) //Dxxx command to set ramp end frequency
		{
			if(MRB_ASCII_ISNUMBER((cntrl&0xFF000000)>>24)&&MRB_ASCII_ISNUMBER((cntrl&0x00FF0000)>>16)&&MRB_ASCII_ISNUMBER((cntrl&0x0000FF00)>>8))
			{
					if(MRB_ASCII_TO_NUMBER((cntrl&0x0000FF00)>>8) > 5) //Maximal possible frequency is 59.9
					{
						HAL_UART_Transmit_IT(&huart2, "\nF2     ", strlen("\nF2     ")); //Set Frequency is too high
						ErrorCode = SetFreqTooHigh;
					}
					else
					{
						Ramp.set_ramp_freqency = MRB_ASCII_TO_NUMBER((cntrl&0x0000FF00)>>8)*10+MRB_ASCII_TO_NUMBER((cntrl&0x00FF0000)>>16)+MRB_ASCII_TO_NUMBER((cntrl&0xFF000000)>>24)*0.1;

						//Te czesc projektu mozna by rozszerzyc o konwerter string -> int
						HAL_UART_Transmit_IT(&huart2, "\nfreq ok", strlen("\nfreq ok")); //command set freq ok
						ErrorCode = NoError;
					}

			}else
				{

				HAL_UART_Transmit_IT(&huart2, "\nF3     ", strlen("\nF3     ")); //Wrong input - frequency is not a number
				ErrorCode = NotANumber;
				}
		}
		// SET time of ramp
		else if ((cntrl|0xFFFFFF00) == 0xFFFFFF45) //Exxx command to set ramp end frequency
		{
			if(MRB_ASCII_ISNUMBER((cntrl&0xFF000000)>>24)&&MRB_ASCII_ISNUMBER((cntrl&0x00FF0000)>>16)&&MRB_ASCII_ISNUMBER((cntrl&0x0000FF00)>>8))
			{
					if(MRB_ASCII_TO_NUMBER((cntrl&0x0000FF00)>>8) > 5) //Maximal possible frequency is 59.9
					{
						HAL_UART_Transmit_IT(&huart2, "\nF2     ", strlen("\nF2     ")); //Set Frequency is too high
						ErrorCode = SetFreqTooHigh;
					}
					else
					{
						Ramp.ramp_time = MRB_ASCII_TO_NUMBER((cntrl&0x0000FF00)>>8)*10+MRB_ASCII_TO_NUMBER((cntrl&0x00FF0000)>>16)+MRB_ASCII_TO_NUMBER((cntrl&0xFF000000)>>24)*0.1;

						//Te czesc projektu mozna by rozszerzyc o konwerter string -> int
						HAL_UART_Transmit_IT(&huart2, "\ntime ok", strlen("\ntime ok")); //command set freq ok
						ErrorCode = NoError;
					}

			}else
				{

				HAL_UART_Transmit_IT(&huart2, "\nF3     ", strlen("\nF3     ")); //Wrong input - frequency is not a number
				ErrorCode = NotANumber;
				}
		}



		// SET FREQUENCY IMMIDIATELY
		//If command B -> Set speed 0 - 500 (which is equeal to 0 - 50.0 Hz) //B(speed - 3 marks)
		else if ((cntrl|0xFFFFFF00) == 0xFFFFFF42)
		{
			if(MRB_ASCII_ISNUMBER((cntrl&0xFF000000)>>24)&&MRB_ASCII_ISNUMBER((cntrl&0x00FF0000)>>16)&&MRB_ASCII_ISNUMBER((cntrl&0x0000FF00)>>8))
			{
				if(MRB_ASCII_TO_NUMBER((cntrl&0x0000FF00)>>8) > 5) //Maximal possible frequency is 59.9
				{
					HAL_UART_Transmit_IT(&huart2, "\nF2     ", strlen("\nF2     ")); //Set Frequency is too high
					ErrorCode = SetFreqTooHigh;
				}
				else
				{
					frequency = MRB_ASCII_TO_NUMBER((cntrl&0x0000FF00)>>8)*10+MRB_ASCII_TO_NUMBER((cntrl&0x00FF0000)>>16)+MRB_ASCII_TO_NUMBER((cntrl&0xFF000000)>>24)*0.1;

					//Te czesc projektu mozna by rozszerzyc o konwerter string -> int
					HAL_UART_Transmit_IT(&huart2, "\nfreq ok", strlen("\nfreq ok")); //command set freq ok
					ErrorCode = NoError;
				}

			}else
				{

				HAL_UART_Transmit_IT(&huart2, "\nF3     ", strlen("\nF3     ")); //Wrong input - frequency is not a number
				ErrorCode = NotANumber;
				}


		}

		// ERROR INPUT
		else
			{
			HAL_UART_Transmit_IT(&huart2, "\nF1     ", strlen("\nF1     ")); //Wrong input
			ErrorCode = WrongInputCode;
			}
		}


		// WAIT FOR ANOTHER RECEIVE
		VSI_state ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_UART_Receive_IT(&huart2, &cntrl, 4);

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	//Timer for switching transistors 6 kHz

	if(htim->Instance == TIM10)
	{


		if ( (VSI_state == on) &&  frequency > 0.5)
		{


		Magnitude = frequency*0.02*330;
		//transformacja Clarka
		time = time + Timp;

		u_alfa = Magnitude*sin(2*PI*frequency*time);
		u_beta=Magnitude*cos(2*PI*frequency*time);
		angle=atan2f(u_beta,u_alfa);

		if(angle<0) angle=angle+2*PI; // shift 2pi for negative values

		sektor=floor(angle);
		switch (sektor)
				{
				case 0:
					u_alfa_1=sqrt2_by_sqrt3*U_dc;
					u_beta_1=0;
					u_alfa_2=one_by_sqrt6*U_dc;
					u_beta_2=one_by_sqrt2*U_dc;

					break;
				case 1:
					u_alfa_1=one_by_sqrt6*U_dc;
					u_beta_1=one_by_sqrt2*U_dc;
					u_alfa_2=-one_by_sqrt6*U_dc;
					u_beta_2=one_by_sqrt2*U_dc;
					break;
				case 2:
					u_alfa_1=-one_by_sqrt6*U_dc;
					u_beta_1=one_by_sqrt2*U_dc;
					u_alfa_2=-sqrt2_by_sqrt3*U_dc;
					u_beta_2=0;
					break;
				case 3:
					u_alfa_1=-sqrt2_by_sqrt3*U_dc;
					u_beta_1=0;
					u_alfa_2=-one_by_sqrt6*U_dc;
					u_beta_2=-one_by_sqrt2*U_dc;
					break;
				case 4:
					u_alfa_1=-one_by_sqrt6*U_dc;
					u_beta_1=-one_by_sqrt2*U_dc;
					u_alfa_2=one_by_sqrt6*U_dc;
					u_beta_2=-one_by_sqrt2*U_dc;
					break;
				case 5:
					u_alfa_1=one_by_sqrt6*U_dc;
					u_beta_1=-one_by_sqrt2*U_dc;
					u_alfa_2=sqrt2_by_sqrt3*U_dc;
					u_beta_2=0;
					break;
				}
		time1_vector =	Timp*( u_alfa*u_beta_2-u_beta*u_alfa_2)/(u_alfa_1*u_beta_2-u_beta_1*u_alfa_2);
		time2_vector = Timp*( -u_alfa*u_beta_1+u_beta*u_alfa_1)/(u_alfa_1*u_beta_2-u_beta_1*u_alfa_2);
		time0_vector = Timp - time1_vector - time2_vector;
#ifdef FPGA
		u_timer_vector1 = (time1_vector*500000);
		u_timer_vector2 = (time2_vector*500000);
		HAL_GPIO_WritePin(GPIOC, ~((u_timer_vector1<<8)+u_timer_vector2) , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, ((u_timer_vector1<<8)+u_timer_vector2) , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, ~((u_timer_vector1&0xC0)<<2) , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, ((u_timer_vector1&0xC0)<<2) , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, (~(sektor<<8))&0x0F00 , GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, (sektor<<8) , GPIO_PIN_SET);
#endif
		}
		else
		{
			u_timer_vector1 = 0;
			u_timer_vector2 = 0;
			HAL_GPIO_WritePin(GPIOC, 0xFFFF , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, 0x0FFF , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, 0x0F00 , GPIO_PIN_RESET);

		}


	}

#ifndef FPGA
	////Timer for couting and interrupting 200 kHz
	if(htim->Instance == TIM11)
	{
		if ( (VSI_state == on) && frequency > 0.5)
		{
		//Obliczanie aktualnego optymalnego wektora zerowego - aby prze³¹czaæ jak najmniej tranzystorów
		if ((T1+T2+T3)== 2)
		{
		OUT0[0]=1;
		OUT0[1]=1;
		OUT0[2]=1;
		}
		else if ((T1+T2+T3)== 1)
		{
		OUT0[0]=0;
		OUT0[1]=0;
		OUT0[2]=0;
		}
		if (imp_counter<=time1_vector)
		{
		T1 = OUT[sektor][0];
		T4 = !OUT[sektor][0];
		T2 = OUT[sektor][1];
		T5 = !OUT[sektor][1];
		T3 = OUT[sektor][2];
		T6 = !OUT[sektor][2];
		}

		else if (imp_counter<=(time1_vector+time2_vector))
		{
		if (sektor==5)
		{
		T1 = OUT[0][0];
		T4 = !OUT[0][0];
		T2 = OUT[0][1];
		T5 = !OUT[0][1];
		T3 = OUT[0][2];
		T6 = !OUT[0][2];
		}
		else
		{
		T1 = OUT[sektor+1][0];
		T4 = !OUT[sektor+1][0];
		T2 = OUT[sektor+1][1];
		T5 = !OUT[sektor+1][1];
		T3 = OUT[sektor+1][2];
		T6 = !OUT[sektor+1][2];
		}
		}
		else if (imp_counter<=Timp)
		{
		T1 = OUT0[0];
		T4 = !OUT0[0];
		T2 = OUT0[1];
		T5 = !OUT0[1];
		T3 = OUT0[2];
		T6 = !OUT0[2];
		}
		else if (imp_counter<=(Timp+time2_vector))
		{
		if (sektor==5)
		{
		T1 = OUT[0][0];
		T4 = !OUT[0][0];
		T2 = OUT[0][1];
		T5 = !OUT[0][1];
		T3 = OUT[0][2];
		T6 = !OUT[0][2];

		}
		else
		{
		T1 = OUT[sektor+1][0];
		T4 = !OUT[sektor+1][0];
		T2 = OUT[sektor+1][1];
		T5 = !OUT[sektor+1][1];
		T3 = OUT[sektor+1][2];
		T6 = !OUT[sektor+1][2];
		}
		}
		else if (imp_counter<=(Timp+time2_vector+time1_vector))
		{
		T1 = OUT[sektor][0];
		T4 = !OUT[sektor][0];
		T2 = OUT[sektor][1];
		T5 = !OUT[sektor][1];
		T3 = OUT[sektor][2];
		T6 = !OUT[sektor][2];
		}
		else if (imp_counter<=2*Timp)
		{
		T1 = OUT0[0];
		T4 = !OUT0[0];
		T2 = OUT0[1];
		T5 = !OUT0[1];
		T3 = OUT0[2];
		T6 = !OUT0[2];
		}
		imp_counter=imp_counter+Ts;
		if (imp_counter>=2*Timp)
		{
			imp_counter=0;
		}
		T_LastState16bit.GPIO16bit = TState16bit.GPIO16bit;
		//T1 -> GPIOC 0
		//T2 -> GPIOC 2
		//T3 -> GPIOC 4
		//T4 -> GPIOC 1
		//T5-> GPIOC 3
		//T6 -> GPIOC 5
		TState16bit.GpioState.GPIO0 = T1;
		TState16bit.GpioState.GPIO1 = T4;
		TState16bit.GpioState.GPIO2 = T2;
		TState16bit.GpioState.GPIO3 = T5;
		TState16bit.GpioState.GPIO4 = T3;
		TState16bit.GpioState.GPIO5 = T6;

		Bits_changed_xor.GPIO16bit = TState16bit.GPIO16bit^T_LastState16bit.GPIO16bit; //if there was a change of tranzistor, Bitmask isn't equal zero
		HAL_GPIO_WritePin(GPIOC, ((Bits_changed_xor.GPIO16bit)) , GPIO_PIN_RESET);
		//Bits for GPIO are masked with Bits_changed_xor. If there was o change, tranzistor will delay turning on till next cycle -> cycle lasts for a Deadtime
		//1 cycle last for 0.2us (200ns) -> thats a time of Deadtime;
		//HAL_GPIO_WritePin(GPIOC, (TState16bit.GPIO16bit & (~Bits_changed_xor.GPIO16bit)) , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, (TState16bit.GPIO16bit) , GPIO_PIN_SET);
		}
		else{
			TState16bit.GPIO16bit = 0x0;
			HAL_GPIO_WritePin(GPIOC, (0xFFFF) , GPIO_PIN_RESET);
			}

	}
#endif
	if(htim->Instance == TIM3)
	{
		//Master algorithm control //// 100 Hz updating
		//Place for PIDs etc, observers algorithms etc.
		if (VSI_state == on)
		{

			switch(VSI_control)
			{
			case wait:
				break;
			case soft_start:
				MRB_VSI_SoftStart(&frequency, Ramp.ramp_time, 0.01);
				if (fabs(frequency - 50) < 0.05)
					{
					VSI_control = wait;
					}
				break;


			case soft_stop:
				MRB_VSI_SoftStop(&frequency, Ramp.ramp_time, 0.01);
				if (fabs(frequency - 0) < 0.05)
					{
					VSI_control = wait;
					}
				break;

			case ramp:

				MRB_VSI_Ramp(&frequency,Ramp.set_ramp_freqency, Ramp.ramp_time, 0.01, &Ramp.ramp_enable);

				if (fabs(frequency - Ramp.set_ramp_freqency) < 0.05)
					{
					VSI_control = wait;
					Ramp.ramp_enable = 1;
					}
				else
				break;
			}

		}
	}


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_pin)
{
	if(GPIO_pin == GPIO_PIN_13)
	{
		VSI_state = on;
	}
	if(GPIO_pin == GPIO_PIN_14)
	{
		VSI_state = off;
	}
	if(GPIO_pin == GPIO_PIN_15)
	{

		frequency=adc[0]*0.014648; //set frequency from 0 to 60 Hz
	}
	VSI_state ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);
}

#ifdef SINLUT
unsigned int SLT_sinus(unsigned int time)
{
	return slt[time];
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
