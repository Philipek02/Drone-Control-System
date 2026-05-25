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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motors.h"
#include "ibus.h"
#include "bno055.h"
#include "pid.h"
#include "bno055_stm32.h"
#include <stdio.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart2;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t arm;
} rc_t;

typedef struct {
    float y;
    uint8_t initialized;
} LPF_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t control_loop_flag = 0; // flaga dla TIM6
volatile rc_t rc;

#define CTRL_DT 0.005f // 200 Hz

// Kanały iBUS są indeksowane od zera:
// CH1 -> 0, CH2 -> 1, CH3 -> 2, CH4 -> 3, CH5 -> 4 itd.
// Ustaw tutaj kanał, na którym realnie masz SWD.
#define RC_CH_ROLL      0   // CH1
#define RC_CH_PITCH     1   // CH2
#define RC_CH_THROTTLE  2   // CH3
#define RC_CH_YAW       3   // CH4
#define RC_CH_ARM       4   // CH5

#define RC_DEADBAND        0.05f

#define RC_ROLL_SIGN       -1.0f
#define RC_PITCH_SIGN      -1.0f
#define RC_YAW_SIGN         1.0f

#define MAX_ANGLE_DEG      25.0f
#define MAX_ROLL_RATE_DPS  160.0f
#define MAX_PITCH_RATE_DPS 160.0f
#define MAX_YAW_RATE_DPS   80.0f

#define GYRO_LPF_ALPHA     0.2f

#define ARM_OFF_US         1300u
#define ARM_ON_US          1700u
#define THROTTLE_LOW_US    1050u

#define RC_MIN_VALID_US     900u
#define RC_MAX_VALID_US    2100u

#define DEBUG_UART          1

static PID_t pid_roll_angle;
static PID_t pid_pitch_angle;
static PID_t pid_roll_rate;
static PID_t pid_pitch_rate;
static PID_t pid_yaw_rate;

static LPF_t lpf_roll_rate;
static LPF_t lpf_pitch_rate;
static LPF_t lpf_yaw_rate;

static bool flight_armed = false;
static bool arm_switch_was_down = false;

// Nastawy startowe. Na pierwsze próby zostaw Ki=0 i Kd=0.
volatile float dbg_angle_Kp = 1.6f;
volatile float dbg_angle_Ki = 0.0f;
volatile float dbg_angle_Kd = 0.001f;

volatile float dbg_rate_Kp = 0.25f;
volatile float dbg_rate_Ki = 0.0f;
volatile float dbg_rate_Kd = 0.001f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint16_t clampu16(uint16_t x, uint16_t lo, uint16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float apply_deadband(float x, float db)
{
    if (x > -db && x < db) return 0.0f;

    // Przeskalowanie po deadbandzie, żeby zakres dalej kończył się na -1..1.
    if (x > 0.0f) return (x - db) / (1.0f - db);
    return (x + db) / (1.0f - db);
}

static inline float rc_us_to_norm(uint16_t us)
{
    float x = ((float)us - 1500.0f) / 500.0f;
    return clampf(x, -1.0f, 1.0f);
}

static bool rc_value_valid(uint16_t us)
{
    return (us >= RC_MIN_VALID_US && us <= RC_MAX_VALID_US);
}

static bool rc_channels_valid(const rc_t *r)
{
    return rc_value_valid(r->roll) &&
           rc_value_valid(r->pitch) &&
           rc_value_valid(r->throttle) &&
           rc_value_valid(r->yaw) &&
           rc_value_valid(r->arm);
}

static float lpf_update(LPF_t *f, float x, float alpha)
{
    if (!f->initialized)
    {
        f->y = x;
        f->initialized = 1;
        return f->y;
    }

    f->y = f->y + alpha * (x - f->y);
    return f->y;
}

static void reset_all_pids(void)
{
    PID_Reset(&pid_roll_angle);
    PID_Reset(&pid_pitch_angle);
    PID_Reset(&pid_roll_rate);
    PID_Reset(&pid_pitch_rate);
    PID_Reset(&pid_yaw_rate);
}

static void disarm_now(void)
{
    flight_armed = false;
    reset_all_pids();
    motors_stop_all();
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim6);
  ibus_init();
  bno055_assignI2C(&hi2c3);

  bno055_setup();
  bno055_setOperationModeConfig();
  bno055_enableExternalCrystal();

  // Do pierwszych lotów: fusion acc + gyro, bez magnetometru.
  bno055_setOperationMode(BNO055_OPERATION_MODE_IMU);
  HAL_Delay(100);

  PID_Init(&pid_roll_angle,  dbg_angle_Kp, dbg_angle_Ki, dbg_angle_Kd,
           -MAX_ROLL_RATE_DPS,  MAX_ROLL_RATE_DPS,  -50.0f, 50.0f);

  PID_Init(&pid_pitch_angle, dbg_angle_Kp, dbg_angle_Ki, dbg_angle_Kd,
           -MAX_PITCH_RATE_DPS, MAX_PITCH_RATE_DPS, -50.0f, 50.0f);

  PID_Init(&pid_roll_rate,   dbg_rate_Kp, dbg_rate_Ki, dbg_rate_Kd,
           -250.0f, 250.0f, -80.0f, 80.0f);

  PID_Init(&pid_pitch_rate,  dbg_rate_Kp, dbg_rate_Ki, dbg_rate_Kd,
           -250.0f, 250.0f, -80.0f, 80.0f);

  PID_Init(&pid_yaw_rate,    0.25f, 0.0f, 0.0f,
           -150.0f, 150.0f, -50.0f, 50.0f);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  motors_stop_all();

  // Jeśli trzymasz przycisk przy starcie -> kalibracja ESC.
  // UWAGA: śmigła zdjęte.
  if (HAL_GPIO_ReadPin(B1_cal_GPIO_Port, B1_cal_Pin) == GPIO_PIN_RESET)
  {
      HAL_Delay(4000);
      esc_calibrate_all();

      while (1)
      {
          HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
          HAL_Delay(50);
      }
  }

  // Czekaj na pierwsze poprawne ramki iBUS, ale NIE armuj automatycznie.
  uint32_t t0 = HAL_GetTick();
  while (!ibus_is_signal_present())
  {
      ibus_process();

      if (HAL_GetTick() - t0 > 200)
      {
          t0 = HAL_GetTick();
          HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      }

      motors_stop_all();
  }

  // Wymuszamy, żeby przed pierwszym ARM przełącznik był widziany w pozycji DISARM.
  arm_switch_was_down = false;
  flight_armed = false;
  motors_stop_all();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
static uint16_t led_counter = 0;

  while (1)
  {
      // Czekaj na flagę z TIM6, czyli 200 Hz.
      if (!control_loop_flag)
          continue;

      control_loop_flag = 0;

      led_counter++;
      if (led_counter >= 200)
      {
          led_counter = 0;
          HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      }

      ibus_process();

      rc_t rc_now;
      rc_now.roll     = ibus_read_channel(RC_CH_ROLL);
      rc_now.pitch    = ibus_read_channel(RC_CH_PITCH);
      rc_now.throttle = ibus_read_channel(RC_CH_THROTTLE);
      rc_now.yaw      = ibus_read_channel(RC_CH_YAW);
      rc_now.arm      = ibus_read_channel(RC_CH_ARM);

      // Kopia globalna tylko do podglądu w debuggerze.
      rc = rc_now;

      bool signal_ok       = ibus_is_signal_present();
      bool channels_ok     = rc_channels_valid(&rc_now);
      bool throttle_low    = rc_now.throttle < THROTTLE_LOW_US;
      bool arm_switch_off  = rc_now.arm < ARM_OFF_US;
      bool arm_switch_on   = rc_now.arm > ARM_ON_US;

      // Warstwa bezpieczeństwa STM32: brak ramek, złe kanały albo SWD w dół = DISARM.
      if (!signal_ok || !channels_ok)
      {
          arm_switch_was_down = false;
          disarm_now();
          continue;
      }

      if (arm_switch_off)
      {
          arm_switch_was_down = true;
          disarm_now();
          continue;
      }

      // SWD góra rozpoczyna armowanie, ale tylko po wcześniejszym stanie DISARM
      // i tylko gdy gaz jest na minimum.
      if (!flight_armed && arm_switch_was_down && arm_switch_on && throttle_low)
      {
          flight_armed = true;
          reset_all_pids();
      }

      if (!flight_armed)
      {
          motors_stop_all();
          reset_all_pids();
          continue;
      }

      // Dron jest ARMED, ale gaz nadal minimalny: silniki stop, PID wyzerowany.
      // Później możesz tu zrobić idle, np. 1050..1080 us, ale na start bezpieczniej stop.
      if (throttle_low)
      {
          motors_stop_all();
          reset_all_pids();
          continue;
      }

      uint16_t throttle_us = clampu16(rc_now.throttle, MOTOR_MIN_US, MOTOR_MAX_US);

      // Wejścia pilota: 1000..2000 us -> -1..1 + deadband.
      float roll_in  = apply_deadband(rc_us_to_norm(rc_now.roll),  RC_DEADBAND);
      float pitch_in = apply_deadband(rc_us_to_norm(rc_now.pitch), RC_DEADBAND);
      float yaw_in   = apply_deadband(rc_us_to_norm(rc_now.yaw),   RC_DEADBAND);

      float roll_angle_sp  = RC_ROLL_SIGN  * roll_in  * MAX_ANGLE_DEG;
      float pitch_angle_sp = RC_PITCH_SIGN * pitch_in * MAX_ANGLE_DEG;
      float yaw_rate_sp    = RC_YAW_SIGN   * yaw_in   * MAX_YAW_RATE_DPS;

      // BNO055: fused Euler do pętli angle + gyro do pętli rate.
      bno055_vector_t euler = bno055_getVectorEuler();
      bno055_vector_t gyro  = bno055_getVectorGyroscope();

      // Mapowanie osi według Twojego obecnego ustawienia.
      // Te znaki KONIECZNIE sprawdź bez śmigieł.
      float roll_angle_meas  =  (float)euler.y;
      float pitch_angle_meas = -(float)euler.z;

      float roll_rate_meas  =  (float)gyro.y;
      float pitch_rate_meas = -(float)gyro.z;
      float yaw_rate_meas   =  (float)gyro.x;

      // Filtruj gyro PRZED PID rate, nie po PID.
      roll_rate_meas  = lpf_update(&lpf_roll_rate,  roll_rate_meas,  GYRO_LPF_ALPHA);
      pitch_rate_meas = lpf_update(&lpf_pitch_rate, pitch_rate_meas, GYRO_LPF_ALPHA);
      yaw_rate_meas   = lpf_update(&lpf_yaw_rate,   yaw_rate_meas,   GYRO_LPF_ALPHA);

      // Pętla zewnętrzna ANGLE: wynik to zadana prędkość kątowa.
      float roll_rate_sp = PID_Update(&pid_roll_angle,
                                      roll_angle_sp,
                                      roll_angle_meas,
                                      CTRL_DT);

      float pitch_rate_sp = PID_Update(&pid_pitch_angle,
                                       pitch_angle_sp,
                                       pitch_angle_meas,
                                       CTRL_DT);

      // Pętla wewnętrzna RATE: wynik to korekta PWM do miksera.
      float u_roll = PID_Update(&pid_roll_rate,
                                roll_rate_sp,
                                roll_rate_meas,
                                CTRL_DT);

      float u_pitch = PID_Update(&pid_pitch_rate,
                                 pitch_rate_sp,
                                 pitch_rate_meas,
                                 CTRL_DT);

      float u_yaw = PID_Update(&pid_yaw_rate,
                               yaw_rate_sp,
                               yaw_rate_meas,
                               CTRL_DT);

//#if DEBUG_UART
//      static uint8_t uart_div = 0;
//      uart_div++;
//      if (uart_div >= 40) // 200 Hz / 40 = 5 Hz
//      {
//          uart_div = 0;
//          printf("ARM=%u THR=%u R=%.1f P=%.1f GR=%.1f GP=%.1f U=%.1f %.1f %.1f\r\n",
//                 (unsigned)flight_armed,
//                 (unsigned)throttle_us,
//                 (double)roll_angle_meas,
//                 (double)pitch_angle_meas,
//                 (double)roll_rate_meas,
//                 (double)pitch_rate_meas,
//                 (double)u_roll,
//                 (double)u_pitch,
//                 (double)u_yaw);
//      }
//#endif

      mixer_update(u_roll, u_pitch, u_yaw, throttle_us);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        control_loop_flag = 1;   // flaga dla pętli głównej
    }
}

// obsluga uart2 - odczyt na serial porcie
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
