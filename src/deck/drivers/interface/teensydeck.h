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

struct __attribute__((__packed__)) serial_pid_in {
    // thrust command (used for resetting the PID)
    float thrust;
    //state
    float roll; //estimated roll
    float pitch; //estimated pitch
    float yaw; //estimated yaw
    //targets 
    float roll_t; //roll target
    float pitch_t; //pitch target
    float yaw_t; //yaw target
     //Rolling message out
    // float rolling_msg_in;
    // uint8_t rolling_msg_in_id;
    //CHECKSUM
    uint8_t checksum_in;
};

struct __attribute__((__packed__)) serial_pid_out {
    //roll commands
    float roll_p; //roll p
    float roll_i; //roll i
    float roll_d; //roll d
    //pitch commands
    float pitch_p; //pitch p
    float pitch_i; //pitch i
    float pitch_d; //pitch d
    //yaw commands
    float yaw_p; //yaw p
    float yaw_i; //yaw i
    float yaw_d; //yaw d
     //Rolling message out
    // float rolling_msg_out;
    // uint8_t rolling_msg_out_id;
    //CHECKSUM
    uint8_t checksum_out;
};

void teensyInit(DeckInfo* info);

bool teensyTest(void);
void teensyTask(void* arg);

extern bool teensyGetStatus(void);
extern float teensyGetRollRateP(void);
extern float teensyGetRollRateI(void);
extern float teensyGetRollRateD(void);
extern float teensyGetPitchRateP(void);
extern float teensyGetPitchRateI(void);
extern float teensyGetPitchRateD(void);

extern struct serial_pid_out myserial_pid_out;
extern bool status;

#endif /* _TEENSY_H_ */
