/* Sam
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * relative_loc.c - implementation for relative localization
 */
#include<stdio.h>

#include "relative_loc.h"
#include "debug.h"

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"

#include "param.h"

#include "log.h"
#include "system.h"

#include <radiolink.h>
#include "estimator_kalman.h"

/*To include the TDOA protocal*/

static bool isInit;

// Define covariance constants
static float Qv = 1.0f; // velocity deviation
static float Qr = 0.7f; // yaw rate deviation
static float Ruwb = 2.0f; // ranging deviation
static float InitCovPos = 1000.0f;
static float InitCovYaw = 1.0f;

#define NumAgent 2

// Relative state variable
static relaVariable_t relaVar[NumAgent];

// The system input variable
static float_t inputVar[NumAgent][STATE_DIM_rl];



