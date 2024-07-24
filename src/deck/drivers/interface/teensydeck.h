/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * teensydeck.h: teensy deck driver
 */

#ifndef _TEENSY_H_
#define _TEENSY_H_

#include "deck_core.h"

struct __attribute__((__packed__)) serial_control_in {
    float thrust;
    //state
    float roll; //roll target
    float pitch; //pitch target
    float yaw; //yaw rate target
    // gyro values
    float roll_gyro;
    float pitch_gyro;
    float yaw_gyro;
    // accelerometer values
    float x_acc;
    float y_acc;
    float z_acc;
    //CHECKSUM
    uint8_t checksum_in;
};

struct __attribute__((__packed__)) serial_control_out {
    //torque commands
    int16_t torque_x; //torque x
    int16_t torque_y; //torque y
    int16_t torque_z; //torque z
    int16_t x_integ;
    int16_t y_integ;
    //CHECKSUM
    uint8_t checksum_out;
};

void teensyInit(DeckInfo* info);

bool teensyTest(void);
void teensyTask(void* arg);

extern bool teensyGetStatus(void);
extern int16_t teensyGetRollTorque(void);
extern int16_t teensyGetPitchTorque(void);
extern int16_t teensyGetYawTorque(void);
extern int16_t teensyGetRollInteg(void);
extern int16_t teensyGetPitchInteg(void);

extern struct serial_control_out myserial_control_out;
extern bool status;

#endif /* _TEENSY_H_ */
