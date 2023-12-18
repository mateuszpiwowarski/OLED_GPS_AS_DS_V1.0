/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "gps_neo6.h" //GPS

#include "onewire.h"	//DS18B20
#include "ds18b20.h"

#include "fonts.h"	//OLED
#include "ssd1306.h"

#include "stdio.h"
#include "string.h"
#include "math.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef enum {
    PAGE_ONE,
    PAGE_TWO,
    PAGE_THREE,
    PAGE_FOUR,
    NUM_PAGES
} PAGE;

#define BUTTON_DEBOUNCE_DELAY 200

volatile PAGE currentPage = PAGE_ONE;
volatile uint8_t buttonPressed = 0;
volatile uint16_t lastButtonPressTime = 0;
volatile uint8_t buttonState = 0;

#define MIN_SPEED_KM 1.5    // Minimum speed (in km/h) to consider that the GPS is moving
#define GPS_EARTH_RADIUS_KM          6371.0 // Earth's radius in kilometers
#define DEG_TO_RAD                   0.0174533 // Degree to Radian conversion factor

double last_latitude = 0.0;  // Used to store the last received latitude value
double last_longitude = 0.0; // Used to store the last received longitude value
double total_distance = 0.0; // Used to store the total travelled distance



NEO6_State GpsState;

char LatitudeString[32];
char LongitudeString[32];
char SpeedString[32];
char SatelitesNumberString[32];



uint16_t readValue;
float sensitivity = 0.04; // 0.04 for 50A Model
float rawVoltage;
float current;
char current_string[32];



char total_distance_string[32];
double current_distance;

float temperature;
char temp_string[16];


/**
  * @brief  Returns the Haversine distance between two GPS coordinates.
  * @param  lat1 Latitude of first coordinate
  * @param  lon1 Longitude of first coordinate
  * @param  lat2 Latitude of second coordinate
  * @param  lon2 Longitude of second coordinate
  * @retval Haversine distance between the two coordinates
  */
double haversine_km(double lat1, double lon1, double lat2, double lon2);

double haversine_km(double lat1, double lon1, double lat2, double lon2)
{
    // Convert degrees to radians
    lat1 *= DEG_TO_RAD;
    lon1 *= DEG_TO_RAD;
    lat2 *= DEG_TO_RAD;
    lon2 *= DEG_TO_RAD;

    // Apply haversine formula
    double d_lat = lat2 - lat1;
    double d_lon = lon2 - lon1;
    double a = sin(d_lat / 2) * sin(d_lat / 2) +
               cos(lat1) * cos(lat2) *
               sin(d_lon / 2) * sin(d_lon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Multiply by Earth's radius to obtain distance in km
    return GPS_EARTH_RADIUS_KM * c;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void displayPage(PAGE page);
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start(&hadc1);
  SSD1306_Init();
  DS18B20_Init(DS18B20_Resolution_12bits);
  HAL_GPIO_WritePin(TEST_Pin_GPIO_Port, TEST_Pin_Pin, 0);
  NEO6_Init(&GpsState, &huart1);

  uint32_t DS18B20_delay = 0;
  uint32_t NEO6_delay = 0;
  uint32_t ADC_delay = 0;
  uint32_t updateScreenTime = 0;
  uint32_t buttonPressTime = 0;
  uint32_t HAL_GPIO_TogglePin_delay = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//////////////////////////////////////////////////////////////////////////////////////
//							      DS18B20_Temperature_Detection

	 if((HAL_GetTick() - DS18B20_delay) > 250)
	 {
		 DS18B20_ReadAll();
		 DS18B20_StartAll();

		 uint8_t ROM_tmp[8];
		 uint8_t i;		//Iterator

	    for(i = 0; i < DS18B20_Quantity(); i++)
	    {
	       if(DS18B20_GetTemperature(i, &temperature))
		   {
		    DS18B20_GetROM(i, ROM_tmp);
		   	memset(temp_string, 0, sizeof(temp_string));

		   	sprintf(temp_string, "%.2f *C", temperature);
		   }
		}
		DS18B20_delay = HAL_GetTick();
	 }


	 if((HAL_GetTick() - HAL_GPIO_TogglePin_delay) > 2000)
	 {
		 HAL_GPIO_TogglePin(LED_Pin_GPIO_Port, LED_Pin_Pin);
		 HAL_GPIO_TogglePin_delay = HAL_GetTick();
	 }


//////////////////////////////////////////////////////////////////////////////
//					Measuring_Current

	  if((HAL_GetTick() - ADC_delay) > 250)
	   {
		  HAL_ADC_PollForConversion(&hadc1,1000);
		  readValue = HAL_ADC_GetValue(&hadc1);
		  rawVoltage = (float) readValue * 3.300 / 4105;
		  current =(rawVoltage - 2.500)/sensitivity;
		  sprintf(current_string,"%.2f A ", current);

		  ADC_delay = HAL_GetTick();
	   }

///////////////////////////////////////////////////////////////////////////////
// 					GPS_Update_Function

	  if((HAL_GetTick() - NEO6_delay) > 500)
	  {
	  NEO6_Task(&GpsState);

	  	  if (GpsState.SpeedKilometers >= MIN_SPEED_KM)
	  	  {
	  		  	  if ( (last_latitude != 0.0) && (last_longitude != 0.0))
	  		  	  {
	  		  		  // Calculate the trip distance using the haversine function
	  		  		  current_distance = haversine_km(last_latitude, last_longitude, GpsState.Latitude, GpsState.Longitude);
	  		  		  GpsState.total_distance += current_distance;
	  		  	  }

	  		  	  last_latitude = GpsState.Latitude;
	  		  	  last_longitude = GpsState.Longitude;
	  	  }


	  	  sprintf(total_distance_string, "%.3f km", GpsState.total_distance); // write into the string the total distance with 2 decimal precision
	  	  sprintf(LatitudeString, "%f", GpsState.Latitude);
	  	  sprintf(LongitudeString, "%f", GpsState.Longitude);
	  	  sprintf(SpeedString, "%.2f km/h", GpsState.SpeedKilometers);
	  	  sprintf(SatelitesNumberString, "%d", GpsState.SatelitesNumber);
	  	  NEO6_delay = HAL_GetTick();
	  }

/////////////////////////////////////////////////////////////////////////////////
//    					Handle_Button_Update_Displayed_Pages

	  if(HAL_GetTick() - buttonPressTime >= 200)
	   {
		  if(buttonPressed)
		   {
			  buttonPressed = 0;
			  if(++currentPage >= NUM_PAGES)
			  {
				  currentPage = PAGE_ONE;
			  }
	         displayPage(currentPage);
		   }
		 SSD1306_UpdateScreen();
	     buttonPressTime = HAL_GetTick();
	   }

///////////////////////////////////////////////////////////////////////////////////
//						Update_Information_OnThe_Current_Page

	  if(HAL_GetTick() - updateScreenTime >= 250)
	      {
	          displayPage(currentPage);  // This is where you update information on the current page
	          updateScreenTime = HAL_GetTick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == GpsState.neo6_huart)
  {
    NEO6_ReceiveUartChar(&GpsState);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == P1_Pin)
    {
        uint16_t currentTime = HAL_GetTick();

        if (currentTime - lastButtonPressTime >= BUTTON_DEBOUNCE_DELAY)
        {
            lastButtonPressTime = currentTime;
            buttonState = !buttonState;

            if(buttonState)
            {
                buttonPressed = 1;
            }
        }
    }
}


void displayPage(PAGE page)
{
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    switch(page)
    {
        case PAGE_ONE:

    		 SSD1306_GotoXY (0, 0);
    		 SSD1306_Puts ("TEM:", &Font_7x10, 1);
    	     SSD1306_GotoXY (28,0);
    	     SSD1306_Puts (temp_string, &Font_7x10, 1);

    	     SSD1306_GotoXY (00, 10);
    	   	 SSD1306_Puts ("CURR:", &Font_7x10, 1);
    	     SSD1306_GotoXY (40,10);
    	     SSD1306_Puts (current_string, &Font_7x10, 1);


    	     SSD1306_GotoXY (0, 20);
    	     SSD1306_Puts ("S_GPS:", &Font_7x10, 1);
    	     SSD1306_GotoXY (40,20);
    	     SSD1306_Puts (SpeedString, &Font_7x10, 1);

    	     SSD1306_GotoXY (0, 30);
    	     SSD1306_Puts ("DIS:", &Font_7x10, 1);
    	     SSD1306_GotoXY (35,30);
    	     SSD1306_Puts (total_distance_string, &Font_7x10, 1);

    	     SSD1306_GotoXY (0, 40);
    	   	 SSD1306_Puts ("LAT:", &Font_7x10, 1);
    	     SSD1306_GotoXY (35,40);
    	     SSD1306_Puts (LatitudeString, &Font_7x10, 1);

    	     SSD1306_GotoXY (0, 50);
    	   	 SSD1306_Puts ("LON:", &Font_7x10, 1);
    	     SSD1306_GotoXY (35,50);
    	     SSD1306_Puts (LongitudeString, &Font_7x10, 1);


    	     SSD1306_GotoXY (85, 0);
    	     SSD1306_Puts ("S:", &Font_7x10, 1);
    	     SSD1306_GotoXY (100,0);
    	     SSD1306_Puts (SatelitesNumberString, &Font_7x10, 1);
    	     break;


        case PAGE_TWO:


            SSD1306_GotoXY (23 , 0);
            SSD1306_Puts ("TEMPERATURE:", &Font_7x10, 1);
            SSD1306_GotoXY (20 ,10);
            SSD1306_Puts (temp_string, &Font_11x18, 1);

            SSD1306_GotoXY (33 , 30);
            SSD1306_Puts ("CURRENT:", &Font_7x10, 1);
            SSD1306_GotoXY (28 ,40);
            SSD1306_Puts (current_string, &Font_11x18, 1);
            break;




        case PAGE_THREE:

            SSD1306_GotoXY (33, 0);
            SSD1306_Puts ("SPEED GPS:", &Font_7x10, 1);
            SSD1306_GotoXY (18,10);
            SSD1306_Puts (SpeedString, &Font_11x18, 1);

            SSD1306_GotoXY (15, 29);
            SSD1306_Puts ("TOTAL DISTANCE:", &Font_7x10, 1);
            SSD1306_GotoXY (15,39);
            SSD1306_Puts (total_distance_string, &Font_7x10, 1);

    	    SSD1306_GotoXY (85, 50);
    	    SSD1306_Puts ("S:", &Font_7x10, 1);
    	    SSD1306_GotoXY (100,45);
    	    SSD1306_Puts (SatelitesNumberString, &Font_11x18, 1);
            break;



        case PAGE_FOUR:

            SSD1306_GotoXY (25,0 );
            SSD1306_Puts ("LATITUDE:", &Font_7x10, 1);
            SSD1306_GotoXY (8,10);
            SSD1306_Puts (LatitudeString, &Font_11x18, 1);

            SSD1306_GotoXY (25, 30);
            SSD1306_Puts ("LONGITUDE:", &Font_7x10, 1);
            SSD1306_GotoXY (8,40);
            SSD1306_Puts (LongitudeString, &Font_11x18, 1);
            break;

        default:
        	// handle the case when "page" has a value other than those above
            break;
    }

    SSD1306_UpdateScreen();
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
