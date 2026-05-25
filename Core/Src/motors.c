#include "motors.h"


extern TIM_HandleTypeDef htim1;
// htim1 musi być widoczny

// Tablica aktualnych wartości PWM do debug
static uint16_t motor_output_us[4] = {1000,1000,1000,1000};


void motors_arm(void)
{
    motors_stop_all();
    HAL_Delay(2000);
}


void motors_stop_all(void)
{
    for (int i = 1; i <= 4; i++)
        set_motor_us(i, MOTOR_MIN_US);
}

void motors_set_all_us(uint16_t us)
{
    for (int i = 1; i <= 4; i++)
        set_motor_us(i, us);
}

// Ustawia PWM w mikrosekundach na danym silniku
void set_motor_us(uint8_t motor_id, uint16_t us)
{
    // Zabezpieczenia podstawowy clamp
    if (motor_id < 1 || motor_id > 4) return;

    if (us < MOTOR_MIN_US) us = MOTOR_MIN_US;
    if (us > MOTOR_MAX_US) us = MOTOR_MAX_US;

    motor_output_us[motor_id - 1] = us;

    switch(motor_id)
    {
        case 1:  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us); break;
        case 2:  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, us); break;
        case 3:  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, us); break;
        case 4:  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us); break;
        default: break;
    }
}


// Miksowanie sygnałów roll/pitch/yaw oraz throttle do 4 silników
//
//        (Front)
//      M2       M3
//
//      M1       M4
//        (Back)
//
void mixer_update(float u_roll, float u_pitch, float u_yaw, uint16_t throttle_us)
{
    float m1 = throttle_us - u_roll - u_pitch - u_yaw; // M1: rear left
    float m2 = throttle_us - u_roll + u_pitch + u_yaw; // M2: front left
    float m3 = throttle_us + u_roll + u_pitch - u_yaw; // M3: front right
    float m4 = throttle_us + u_roll - u_pitch + u_yaw; // M4: rear right

    // Zachowaj proporcje miksowania, jeśli któryś silnik wychodzi poza zakres.
    float max_m = m1;
    if (m2 > max_m) max_m = m2;
    if (m3 > max_m) max_m = m3;
    if (m4 > max_m) max_m = m4;

    float min_m = m1;
    if (m2 < min_m) min_m = m2;
    if (m3 < min_m) min_m = m3;
    if (m4 < min_m) min_m = m4;

    if (max_m > MOTOR_MAX_US)
    {
        float shift = max_m - MOTOR_MAX_US;
        m1 -= shift;
        m2 -= shift;
        m3 -= shift;
        m4 -= shift;
    }

    if (min_m < MOTOR_MIN_US)
    {
        float shift = MOTOR_MIN_US - min_m;
        m1 += shift;
        m2 += shift;
        m3 += shift;
        m4 += shift;
    }

    set_motor_us(1, (uint16_t)m1); // rear left
    set_motor_us(2, (uint16_t)m2); // front left
    set_motor_us(3, (uint16_t)m3); // front right
    set_motor_us(4, (uint16_t)m4); // rear right
}

void esc_calibrate_all(void)
{
    // !!! ZDEJMIJ ŚMIGŁA !!!
    HAL_Delay(3000);

    motors_set_all_us(2000);
    HAL_Delay(3000);          // czas na wykrycie MAX i sygnały dźwiękowe

    motors_set_all_us(1000);
    HAL_Delay(3000);          // czas na zapis MIN

    motors_set_all_us(1000);  // zostaw na minimum
}

