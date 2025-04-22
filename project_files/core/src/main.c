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
#include <stdio.h>
#include <math.h>
#include "string.h"
#include "stdbool.h"

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
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
// PCA9685 Address
#define PCA9685_ADDR 0x80

// Configure PCA9685 Board
#define MODE1_REG 0x0
// Sleep-Mode
#define MODE1_SLEEP_BIT 4
// Auto-Increment Mode
#define MODE1_AI_BIT 5
// Restart
#define MODE1_RESTART_BIT 7
// Configure Prescaler to get frequency values
#define PRESCALE_REG 0xFE
// First register corresponding to turning on Channel 0
#define LED0_ON_L 0x6

// old filtering stuff based on the arduino guide for emg, but that code is in C++ so I tried my best to convert it over
// But I wrote new code that includes moving average and pattern detection. I still wanna have this just in case the new one has issues
/* Here's the link for the arduino code I used for wirting this code: https://github.com/YeezB/EMG_Filter/tree/master/src
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 * ---------------------     ------------------------
#define SAMPLE_RATE 1000
#define HUM_FREQ 50  // 50Hz for EU, change to 60Hz for US
static int Threshold = 0;
typedef struct {
    float states[2];
    float num[3];
    float den[3];
} Filter2nd;

typedef struct {
    float states[4];
    float num[6];
    float den[6];
    float gain;
} Filter4th;

// Filter instances
Filter2nd LPF, HPF;
Filter4th Notch;

void filter2nd_init(Filter2nd* filter, float num[3], float den[3]) {
    for (int i = 0; i < 3; i++) {
        filter->num[i] = num[i];
        filter->den[i] = den[i];
    }
    filter->states[0] = filter->states[1] = 0;
}

void filter4th_init(Filter4th* filter, float num[6], float den[6], float gain) {
    for (int i = 0; i < 6; i++) {
        filter->num[i] = num[i];
        filter->den[i] = den[i];
    }
    filter->gain = gain;
    for (int i = 0; i < 4; i++) {
        filter->states[i] = 0;
    }
}

void init_filters() {
    // Coefficients (from .cpp file)
    float lpf_num[3] = {0.1311, 0.2622, 0.1311};
    float lpf_den[3] = {1.0000, -0.7478, 0.2722};

    float hpf_num[3] = {0.9150, -1.8299, 0.9150};
    float hpf_den[3] = {1.0000, -1.8227, 0.8372};

    float notch_num[6] = {0.5869, -1.1146, 0.5869, 1.0499, -2.0000, 1.0499};
    float notch_den[6] = {1.0000, -1.8844, 0.9893, 1.0000, -1.8991, 0.9892};
    float notch_gain = 1.4399;

    // Initialize filters
    filter2nd_init(&LPF, lpf_num, lpf_den);
    filter2nd_init(&HPF, hpf_num, hpf_den);
    filter4th_init(&Notch, notch_num, notch_den, notch_gain);
}

float filter2nd_update(Filter2nd* filter, float input) {
    float tmp = (input - filter->den[1] * filter->states[0] - filter->den[2] * filter->states[1]) / filter->den[0];
    float output = filter->num[0] * tmp + filter->num[1] * filter->states[0] + filter->num[2] * filter->states[1];

    filter->states[1] = filter->states[0];
    filter->states[0] = tmp;

    return output;
}

float filter4th_update(Filter4th* filter, float input) {
    float stageOut = filter->num[0] * input + filter->states[0];
    filter->states[0] = filter->num[1] * input + filter->states[1] - filter->den[1] * stageOut;
    filter->states[1] = filter->num[2] * input - filter->den[2] * stageOut;

    float stageIn = stageOut;
    stageOut = filter->num[3] * stageOut + filter->states[2];
    filter->states[2] = filter->num[4] * stageIn + filter->states[3] - filter->den[4] * stageOut;
    filter->states[3] = filter->num[5] * stageIn - filter->den[5] * stageOut;

    return filter->gain * stageOut;
}

void reset_filter_states() {
    for (int i = 0; i < 2; i++) {
        LPF.states[i] = 0;
        HPF.states[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        Notch.states[i] = 0;
    }
}
---------------------     ------------------------
---------------------     ------------------------
---------------------     ------------------------
---------------------     ------------------------
*/


















// new filtering stuff
// Define parameters for the rAW EMG filter - adjusted for the observed adc patterns
#define BUFFER_SIZE 16      // Reduced size = faster response, and vice versa
#define PEAK_THRESHOLD 3    // Threshold for detecting initial peak (higher = less prone to detect)
#define DIP_THRESHOLD 20    // Threshold for detecting the following dip
#define AMPLIFICATION 20.0  // Amplification factor (raw signal only in range of 1930 - 1950 from rest to activation)
#define LOW_PASS_ALPHA 0.2  // Value in LPF formula (for responsiveness in smoothing)
#define HIGH_PASS_ALPHA 0.1 // Value in HPF formula (for transient detection)

typedef struct {
    int raw_buffer[BUFFER_SIZE];
    int buf_i;
    float baseline;
    float filtered_value;
    float lp_value;
    float hp_value;
    float prev_high_pass_input;
    int last_few_values[4];  // for pattern detecion
    int last_few_index;
    int flex_detected;
    int initialized;
} EMGFilter;

// Initialize filter
void init_filter(EMGFilter* filter) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        filter->raw_buffer[i] = 0;
    }
    filter->buf_i = 0;
    filter->baseline = 1930;  // rest value is around 1930 for raw signal so we'll start here
    filter->filtered_value = 80; // env signal was around 80 at rest so I'm trying to mimick that
    filter->lp_value = 1930;
    filter->hp_value = 0;
    filter->prev_high_pass_input = 1930;

    for (int i = 0; i < 4; i++) {
        filter->last_few_values[i] = 1930;
    }
    filter->last_few_index = 0;
    filter->flex_detected = 0;
    filter->initialized = 0;
}

// Calculate baseline/ average
float calculate_baseline(EMGFilter* filter) {
    float sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += filter->raw_buffer[i];
    }
    return sum / BUFFER_SIZE;
}

// Apply simple low-pass filter (this is for smoothing)
float apply_low_pass(float prev_value, float new_sample, float alpha) {
    return prev_value + alpha * (new_sample - prev_value);
}

// Apply high-pass filter (this for drift)
float apply_high_pass(float prev_high_pass, float prev_input, float new_input, float alpha) {
    float new_high_pass = alpha * (prev_high_pass + new_input - prev_input);
    return new_high_pass;
}

// Check for flex pattern (I noticed a small rise followed by larger drop ie. 1930 to 1950 then 1950 to 1890)
int detect_flex_pattern(EMGFilter* filter, int new_value) {
    // Update the buffer of recent values
    filter->last_few_values[filter->last_few_index] = new_value;
    filter->last_few_index = (filter->last_few_index + 1) % 4;

    int idx = filter->last_few_index;
    int v0 = filter->last_few_values[idx];
    idx = (idx + 1) % 4;
    int v1 = filter->last_few_values[idx];
    idx = (idx + 1) % 4;
    int v2 = filter->last_few_values[idx];
    idx = (idx + 1) % 4;
    int v3 = filter->last_few_values[idx];

    // Check for pattern: baseline to small then rise to significant drop
    int baseline_range = filter->baseline - 5 <= v0 && v0 <= filter->baseline + 5;
    int rise = v1 > filter->baseline + 3;
    int drop = v2 < filter->baseline - 10;
    int returning = v3 > v2;

    return (baseline_range && rise && drop && returning);
}

int process_emg(EMGFilter* filter, int raw_value) {
    // Add new value to buffer
    filter->raw_buffer[filter->buf_i] = raw_value;
    filter->buf_i = (filter->buf_i + 1) % BUFFER_SIZE;

    // If we haven't filled the buffer yet, just return a default value
    if (!filter->initialized && filter->buf_i != 0) {
        filter->last_few_values[filter->last_few_index] = raw_value;
        filter->last_few_index = (filter->last_few_index + 1) % 4;
        return 130; // normal rest value when idle
    }

    if (filter->buf_i == 0) {
        filter->initialized = 1;
    }

    // calculate moving average
    filter->baseline = calculate_baseline(filter);

    // Apply LPF to smooth signal
    filter->lp_value = apply_low_pass(filter->lp_value, raw_value, LOW_PASS_ALPHA);

    // Apply HPF to detect changes
    filter->hp_value = apply_high_pass(
        filter->hp_value,
        filter->prev_high_pass_input,
        filter->lp_value,
        HIGH_PASS_ALPHA
    );
    filter->prev_high_pass_input = filter->lp_value;

    // Calculate absolute value of the high-pass filtered signal
    float rectified = fabs(filter->hp_value);

    // Amplify the signal (want to have range of 80 - 4095 in the end)
    float amplified = rectified * AMPLIFICATION;

    // Check for the flex pattern
    int flex_pattern = detect_flex_pattern(filter, raw_value);

    // If detected, set a high value
    if (flex_pattern) {
        filter->flex_detected = 15; // Hold the flex state for 15 cycles (might change this later, just needed it 15 for testing to see, but it makes it slow to open)
    } else if (filter->flex_detected > 0) {
        filter->flex_detected--;
    }

    // Mapping to range (80-4095)
    if (filter->flex_detected > 0 || amplified > PEAK_THRESHOLD) {
        // increase gradually to maximum when flexing is detected
        float target = 4095;
        filter->filtered_value = apply_low_pass(filter->filtered_value, target, 0.3);
    } else {
        // decrease gradually to rest value
        float target = 80;
        filter->filtered_value = apply_low_pass(filter->filtered_value, target, 0.1);
    }

    return (int)filter->filtered_value;
}


















// PCA setup and servo control
// Got reference code for calibration from this website: https://www.micropeta.com/video113
// datasheet for PCA stuff: https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf
void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value) {
  uint8_t readValue;

  // Read the contents of the register given
  HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDR, Register, 1, &readValue, 1, 10);

  // Modify the Bit given to either clear (0) or set (1)
  if (Value == 0) {
    readValue &= ~(1 << Bit);
  } else {
    readValue |= (1 << Bit);
  };

  // Place updated 8-bits value to register
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDR, Register, 1, &readValue, 1, 10);
  HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(uint16_t frequency) {
  uint8_t prescale_val;
  int max_frequency = 1526;
  int min_frequency = 24;

  // scales frequency of 24Hz to 1526Hz to between 256 to 3, as per page 1 and
  // 25 of datasheet
  if (frequency >= max_frequency) {
    prescale_val = 0x03;
  } else if (frequency <= min_frequency) {
    prescale_val = 0xFF;
  } else {
    prescale_val = 25000000 / (4096 * frequency);
    prescale_val -= 1;
  }

  // Put PCA9685 to sleep (required as per page 25)
  PCA9685_SetBit(MODE1_REG, MODE1_SLEEP_BIT, 1);

  // Update new prescale value to register
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDR, PRESCALE_REG, 1, &prescale_val, 1,
                    10);

  // Wake up PCA9685 board from sleep
  PCA9685_SetBit(MODE1_REG, MODE1_SLEEP_BIT, 0);

  // Restart board for use
  PCA9685_SetBit(MODE1_REG, MODE1_RESTART_BIT, 1);
}

void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime) {
  uint8_t pwm[4];

  // Get register address for selected channel (0-15)
  uint8_t registerAddress = LED0_ON_L + (4 * Channel);
  // See example 1 in the datasheet page no 18/52
  pwm[0] = OnTime & 0xFF;
  pwm[1] = OnTime >> 8;
  pwm[2] = OffTime & 0xFF;
  pwm[3] = OffTime >> 8;
  HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDR, registerAddress, 1, pwm, 4, 10);
}

void PCA9685_SetServoAngle(uint8_t Channel, float Angle) {
  float Value;
  // 50 Hz servo then 4095 Value --> 20 milliseconds
  // 0 degree --> 0.5 ms(102.4 Value) and 180 degree --> 2.5 ms(511.9 Value)
  // From https://www.micropeta.com/video113 direct
  Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
  PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}

void PCA9685_Init(uint16_t frequency) {
  // Set PWM Frequency for Board
  PCA9685_SetPWMFrequency(frequency);
  // Set Auto increment to write to all registers
  PCA9685_SetBit(MODE1_REG, MODE1_AI_BIT, 1);
}

void print_msg(char * msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

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

  // old filter, don't touch this
  /* ---------------------     ------------------------
   * ---------------------     ------------------------
   * ---------------------     ------------------------
   * ---------------------     ------------------------
  reset_filter_states();
  init_filters();
  ---------------------     ------------------------
  ---------------------     ------------------------
  ---------------------     ------------------------
  ---------------------     ------------------------
  */





  // new filter
  // Initialize the filter
      EMGFilter filter;
      init_filter(&filter);

      int filtered_value;









  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(50);  // 50Hz for servo

  /* USER CODE END 2 */
  HAL_ADC_Start(&hadc3);
  uint16_t adc_res; // mask = 0xff00;
  char message[100];
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	bool close = false; // used for making sure we don't go to thumbs up after close as the adc values drop after action
	bool thumb = false;
	bool spider_man = false; // same logic but for spider man gesture
	PCA9685_SetServoAngle(0, 155); // index
	PCA9685_SetServoAngle(1, 160); // middle
	PCA9685_SetServoAngle(2, 160); // thumb
	PCA9685_SetServoAngle(3, 160);
	PCA9685_SetServoAngle(4, 160);
	HAL_Delay(800);
  while (1)
  {
    /* USER CODE END WHILE */

	  HAL_ADC_Start(&hadc3);
	  if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK)  // Wait for conversion
	      {
	          adc_res = HAL_ADC_GetValue(&hadc3);
	      }
	  		adc_res = HAL_ADC_GetValue(&hadc3);
			sprintf(message, "adc_res=%d\r\n", adc_res);
			print_msg(message);






	  		//new filter stuff
	  		// Process the raw ADC value through the filter
//	  		filtered_value = process_emg(&filter, adc_res);
//
//	  		//Print values for debugging
//	  		sprintf(message, "Raw: %d\r\n", adc_res);
//	  		print_msg(message);
//	  		sprintf(message, "Filtered: %d\r\n", filtered_value);
//	  		print_msg(message);

	  		// uncomment this to use raw emg values which are filtered for gestures
//			if (filtered_value > 3100) {
//				// for close
//				PCA9685_SetServoAngle(0, 60);
//				PCA9685_SetServoAngle(1, 60);
//				PCA9685_SetServoAngle(2, 70);
//				PCA9685_SetServoAngle(3, 60);
//				PCA9685_SetServoAngle(4, 80);
//				HAL_Delay(50);
//				close = true;
//			} else if (filtered_value >= 600 && !close) {
//				PCA9685_SetServoAngle(0, 60);
//				PCA9685_SetServoAngle(1, 60);
//				PCA9685_SetServoAngle(2, 160); // thumb
//				PCA9685_SetServoAngle(3, 60);
//				PCA9685_SetServoAngle(4, 80);
//				HAL_Delay(50);
//			} else if (filtered_value < 150){
//				// for open
//				PCA9685_SetServoAngle(0, 155); // index
//				PCA9685_SetServoAngle(1, 160); // middle
//				PCA9685_SetServoAngle(2, 160); // thumb
//				PCA9685_SetServoAngle(3, 160);
//				PCA9685_SetServoAngle(4, 160);
//				HAL_Delay(50);
//				close = false;
//			}
//	  		HAL_Delay(250);

	  		// uncomment this for using the env values for gestures
			if (adc_res > 1000) {
				// for close
				PCA9685_SetServoAngle(0, 60);
				PCA9685_SetServoAngle(1, 60);
				PCA9685_SetServoAngle(2, 70);
				PCA9685_SetServoAngle(3, 60);
				PCA9685_SetServoAngle(4, 80);
				HAL_Delay(600);
				close = true;
			} else if (adc_res >= 300 && adc_res <= 700 && !close && !spider_man) {
				PCA9685_SetServoAngle(0, 60);
				PCA9685_SetServoAngle(1, 60);
				PCA9685_SetServoAngle(2, 160); // thumb
				PCA9685_SetServoAngle(3, 60);
				PCA9685_SetServoAngle(4, 80);
				thumb = true;
				HAL_Delay(600);
//			} else if (adc_res >= 250 && adc_res <= 500 && !close && !thumb){
//				// for open
//				PCA9685_SetServoAngle(0, 155); // index
//				PCA9685_SetServoAngle(1, 60); // middle
//				PCA9685_SetServoAngle(2, 70); // thumb
//				PCA9685_SetServoAngle(3, 160); // pinky
//				PCA9685_SetServoAngle(4, 80);
//				HAL_Delay(600);
//				spider_man = true;
			} else if (adc_res < 200){
				// for open
				PCA9685_SetServoAngle(0, 155); // index
				PCA9685_SetServoAngle(1, 160); // middle
				PCA9685_SetServoAngle(2, 160); // thumb
				PCA9685_SetServoAngle(3, 160);
				PCA9685_SetServoAngle(4, 160);
				HAL_Delay(600);
				close = false;
				spider_man = false;
				thumb = false;
			}
	  		HAL_Delay(100);










			// ------------------------ old filter sttff ----------------------------------------------
	  		/*
	  		 * ---------------------     ------------------------
	  		 * ---------------------     ------------------------
	  		 * ---------------------     ------------------------
	  		 * ---------------------     ------------------------
	  		 * ---------------------     ------------------------
	  		 * ---------------------     ------------------------
			float notchFiltered = filter4th_update(&Notch, (float)adc_res);
			float lowPassFiltered = filter2nd_update(&LPF, notchFiltered);
			float highPassFiltered = filter2nd_update(&HPF, lowPassFiltered);

			// Compute Envelope (Squared Value)
			int envelope = (int)((highPassFiltered/10) * (highPassFiltered/10));

			// Apply Threshold
			envelope = (envelope > Threshold) ? envelope : 0;

			// Print filtered value
			//sprintf(message, "Filtered EMG: %d\n", envelope);
			//print_msg(message);
//			sprintf(message, "%lu\n", envelope);
//			HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), 100);


//			float filteredValue = process_EMG((float)adc_res);
//			int intPart = (int)filteredValue;  // Get integer part
//			int decimalPart = (int)((filteredValue - intPart) * 1000); // Extract 3 decimal places
//
//			// Ensure positive decimal part even for negative values
//			if (decimalPart < 0) {
//				decimalPart = -decimalPart;
//			}
//
//			sprintf(message, "Filtered EMG: %d.%03d\n", intPart, decimalPart);
//			print_msg(message);
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 * ---------------------     ------------------------
 *
 	 	 	 */
			// ------------------------ old filter sttff ----------------------------------------------


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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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
