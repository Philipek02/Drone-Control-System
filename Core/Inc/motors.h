#ifndef MOTORS_H
#define MOTORS_H

#include "stdint.h"
#include "stm32l4xx_hal.h"


// Min i max dla esc
#define MOTOR_MIN_US   1000
#define MOTOR_MAX_US   2000


// uzbrojenie silnikow
void motors_arm(void);

// Zatrzymanie wszystkich silników
void motors_stop_all(void);

// set predkosc
void set_motor_us(uint8_t motor_id, uint16_t us);

// Miksowanie pid i throttle
void mixer_update(float u_roll, float u_pitch, float u_yaw, uint16_t throttle_us);

// kalibracja esc - w implementacji opis
void esc_calibrate_all(void);


//

#endif // MOTORS_H
