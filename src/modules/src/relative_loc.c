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
#include "crc32.h"
#include "system.h"

#include <radiolink.h>
#include "estimator_kalman.h"
// Include our custom cummunication protocal
#include "lpsTdoa4Tag.h"
#include "eventtrigger.h"        // add eventtrigger logging

// declare events
EVENTTRIGGER(samqiao, float, xij, float, yij, float, rij);
EVENTTRIGGER(rAgent, float, rAgent_vx, float, rAgent_vy, float, rAgent_yr, float, rAgent_h)      // remote agent
EVENTTRIGGER(lAgent, float, lAgent_vx, float, lAgent_vy, float, lAgent_yr, float, lAgent_h)      // local  agent
EVENTTRIGGER(interRange, float, inter_ranging, float, pxij, float, pyij, float, prij) // interrange and covariance
static bool isInit;


// Simulation parameters
// Pxy = 0.1 ; Pr = 0.1; Qxy = 0.1; Qr = 0.1; numRob = 2; R_k = 0.1*0.1

// Define our covariance constants (to be tuned)
static float Qv = 0.01f; // velocity deviation
static float Qr = 0.01f; // yaw rate deviation
static float Ruwb = 0.1f; // ranging deviation changed from 2.0f
static float InitCovPos = 0.01f;
static float InitCovYaw = 0.01f;

// // Shuai's parameters
// static float Qv = 1.0f; // velocity deviation
// static float Qr = 0.7f; // yaw rate deviation
// static float Ruwb = 2.0f; // ranging deviation
// static float InitCovPos = 1000.0f;
// static float InitCovYaw = 1.0f;


#define NumAgent 2

// Relative state variable
static relaVariable_t relaVar[NumAgent];

// The system input variable 
// [Sam] STATE_DIM_rl = 3 due to enum
static float_t inputVar[NumAgent][STATE_DIM_rl];

// Linerization Matrix
static float A[STATE_DIM_rl][STATE_DIM_rl];
static float h[STATE_DIM_rl]={0};

// Define matrices: Height and A linearization matrix
static arm_matrix_instance_f32 H = {1,STATE_DIM_rl,h};
static arm_matrix_instance_f32 Am = {STATE_DIM_rl,STATE_DIM_rl,(float *) A};

// Temporary matrices for the covariance updates
static float tmpNN1d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN1m = { STATE_DIM_rl, STATE_DIM_rl, tmpNN1d};

static float tmpNN2d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN2m = { STATE_DIM_rl, STATE_DIM_rl, tmpNN2d};

// Kalman gain shape 3 by 1
static float K[STATE_DIM_rl];
static arm_matrix_instance_f32 Km = {STATE_DIM_rl, 1, (float *)K};

// Covariance 
static float tmpNN3d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN3d};

// Some random matrix 3 by 1 
static float HTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 HTm = {STATE_DIM_rl, 1, HTd};
static float PHTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 PHTm = {STATE_DIM_rl, 1, PHTd};

// A flag for control (fly or not)
static bool fullConnect = false;

// A flag to start/end relative localization
static bool enableRL = false;

// A watchdog for detecting the connection
static int8_t connectCount = 0;

// Remote agent: receive body frame velocity vx,vy, gz (yaw rate) and distance
static float vxj, vyj, rj;

// Local agent: local body frame velocity vx,vy, gz (yaw rate) and distance
static float vxi, vyi, ri;

// Interrange measurement between local agent i and remote agent j
static float dij;

// Height of robot i and j
static float hi, hj;

// Define some basic matrix math operations
// Transpose
static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
// Inverse
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
// Matrix multiplication
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
// Square root
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

// Initialize the task if not initialized
void relativeLocoInit(void)
{
  if (isInit)
    return;
  xTaskCreate(relativeLocoTask,"relative_Localization",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
  isInit = true;
}

// Define the relative localization task
void relativeLocoTask(void* arg){
    systemWaitStart();
    // [Sam] zero out all the covariance matrix: 
    for(int n=0;n<NumAgent;n++){
        for(int i=0;i<STATE_DIM_rl;i++){
            for(int j=0;j<STATE_DIM_rl;j++){
                relaVar[n].P[i][j] = 0;
            }
        }
        
        // For every agent: initialize the covariance in the diagonal
        relaVar[n].P[STATE_rlX][STATE_rlX] = InitCovPos;
        relaVar[n].P[STATE_rlY][STATE_rlY] = InitCovPos;
        relaVar[n].P[STATE_rlYaw][STATE_rlYaw] = InitCovYaw;
        // Also initialize all the states to be zero
        
        // [Sam] Temporary Logic
        if(n == 0){
            relaVar[n].S[STATE_rlX] = 0.0;
        }else{ //n = 1
          if(AGENT_ID == 10){ // A
              relaVar[n].S[STATE_rlX] = -2.0;
          }else if(AGENT_ID == 11){// B
              relaVar[n].S[STATE_rlX] = 2.0;
          }else{
            relaVar[n].S[STATE_rlX] = 0.0;
          }
        }
        
        relaVar[n].S[STATE_rlY] = 0.0;
        relaVar[n].S[STATE_rlYaw] = 0.0;

        // [Sam] Connection flag set to false
        relaVar[n].receiveFlag = false;
    } // end of the outter for loop

    // The main while loop that executes the EKF
    while(1){
        //first delay a little bit
        vTaskDelay(20);
        if(!enableRL){
          continue;
        }
        // DEBUG_PRINT("Start relative localization!!!!!!!!!!!!!!!.\n");
        int agent_id = 1;
        // for(int agent_id=1;agent_id<NumAgent;agent_id++){
        // We need to get remote agent body frame velocity and yaw rates 
        // Interrange can be obtained as well
        if(getRemoteInfo(&dij,&vxj,&vyj,&rj,&hj)){
          connectCount = 0;
          // We also need to get local agent body frame velocity
          estimatorKalmanGetSwarmInfo(&vxi, &vyi, &ri, &hi);
          // Check the receive flag of this agent
          if(relaVar[agent_id].receiveFlag){
        
            uint32_t osTick = xTaskGetTickCount();
            // Calculate dt in the EKF (timestep) 
            // Time elpased / how many ticks per sec
            float dtEKF = (float) (osTick - relaVar[agent_id].oldTimetick)/configTICK_RATE_HZ;
            // Update current time
            relaVar[agent_id].oldTimetick = osTick;
            
            // [Sam] SD log input remote agent
            eventTrigger_interRange_payload.inter_ranging = dij;

            // [Sam] SD log input remote agent
            eventTrigger_rAgent_payload.rAgent_vx = vxj;
            eventTrigger_rAgent_payload.rAgent_vy = vyj;
            eventTrigger_rAgent_payload.rAgent_yr = rj;
            eventTrigger_rAgent_payload.rAgent_h  = hj;
            
            // [Sam] SD log input local agent
            eventTrigger_lAgent_payload.lAgent_vx = vxi;
            eventTrigger_lAgent_payload.lAgent_vy = vyi;
            eventTrigger_lAgent_payload.lAgent_yr = ri;
            eventTrigger_lAgent_payload.lAgent_h = hi;

            // Execute the relative localization EKF
            relativeEKF(agent_id, vxi, vyi, ri, hi, vxj, vyj, rj, hj, dij, dtEKF);
            
            // Maybe it should be i?
            inputVar[agent_id][STATE_rlX] = vxj;
            inputVar[agent_id][STATE_rlY] = vyj;
            inputVar[agent_id][STATE_rlYaw] = rj;

            // Logging event for the relative localization variable
            // eventTrigger_samqiao_payload.AGENT_ID = AGENT_ID;
            if(agent_id == 1){
              eventTrigger_samqiao_payload.xij = relaVar[agent_id].S[STATE_rlX];
              eventTrigger_samqiao_payload.yij = relaVar[agent_id].S[STATE_rlY];
              eventTrigger_samqiao_payload.rij = relaVar[agent_id].S[STATE_rlYaw];

              // covariance
              eventTrigger_interRange_payload.pxij = relaVar[agent_id].P[STATE_rlX][STATE_rlX];
              eventTrigger_interRange_payload.pyij = relaVar[agent_id].P[STATE_rlY][STATE_rlY];
              eventTrigger_interRange_payload.prij = relaVar[agent_id].P[STATE_rlYaw][STATE_rlYaw];

              eventTrigger(&eventTrigger_samqiao);
              eventTrigger(&eventTrigger_rAgent);
              eventTrigger(&eventTrigger_lAgent);
              eventTrigger(&eventTrigger_interRange);
            }
          }else{
            // [Sam] In the case where I cannot get the info from the remote agent
            relaVar[agent_id].oldTimetick = xTaskGetTickCount();
            relaVar[agent_id].receiveFlag = true;
            fullConnect = true;
          }
        }
          // }
        // Increment the counter
        connectCount++;

        if(connectCount>100){
            // disable control if there is no ranging after 1 sec
            fullConnect = false;
        }
      
    }
}

void relativeEKF(int agent_id, float vxi, float vyi, float ri, float hi, float vxj, float vyj, float rj, float hj, float dij, float dt){
  // local agent input and remote agent input
  // create the covariance matrix
  arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *) relaVar[agent_id].P};

    // agent j's pos in i's frame
  float xij = relaVar[agent_id].S[STATE_rlX];
  float yij = relaVar[agent_id].S[STATE_rlY];
  float phi_ij = relaVar[agent_id].S[STATE_rlYaw];

  // the cosine and the sine of the current yaw
  float cyaw = arm_cos_f32(phi_ij);
  float syaw = arm_sin_f32(phi_ij);

  // predication for the next timestamp
  relaVar[agent_id].S[STATE_rlX] = xij + (cyaw*vxj - syaw*vyj - vxi + ri*yij)*dt;
  relaVar[agent_id].S[STATE_rlY] = yij + (syaw*vxj + cyaw*vyj - vyi - ri*xij)*dt;
  relaVar[agent_id].S[STATE_rlYaw] = phi_ij + (rj - ri)*dt;

  // [Sam] The linearized matrix A
  A[0][0] = 1;
  A[0][1] = ri*dt;
  A[0][2] = (-syaw*vxj - cyaw*vyj)*dt;
  A[1][0] = -ri*dt;
  A[1][1] = 1;
  A[1][2] = (cyaw*vxj - syaw*vyj)*dt;
  A[2][0] = 0;
  A[2][1] = 0;
  A[2][2] = 1;

  // Compute A@P and save it in tmpNN1m
  mat_mult(&Am, &Pm, &tmpNN1m);
  // Compute A' and save it in tmpNN2m
  mat_trans(&Am, &tmpNN2m);
  // Compute A@P@A' and save it Pm
  mat_mult(&tmpNN1m, &tmpNN2m, &Pm);

  // Q_k prime that contains the nonlinearity (propagation)
  // BQB' = [ Qv*c^2 + Qv*s^2 + Qr*y^2 + Qv,                       -Qr*x*y, -Qr*y]
  //        [                       -Qr*x*y, Qv*c^2 + Qv*s^2 + Qr*x^2 + Qv,  Qr*x]
  //        [                         -Qr*y,                          Qr*x,  2*Qr]*dt^2

  float dt_squared =  dt*dt;

  relaVar[agent_id].P[0][0] += (2*Qv + yij*yij*Qr)*dt_squared;
  relaVar[agent_id].P[0][1] += (-xij*yij*Qr)*dt_squared;
  relaVar[agent_id].P[0][2] += (-yij*Qr)*dt_squared;
  relaVar[agent_id].P[1][0] += (-xij*yij*Qr)*dt_squared;
  relaVar[agent_id].P[1][1] += (2*Qv + xij*xij*Qr)*dt_squared;
  relaVar[agent_id].P[1][2] += (xij*Qr)*dt_squared;
  relaVar[agent_id].P[2][0] += (-yij*Qr)*dt_squared;
  relaVar[agent_id].P[2][1] += (xij*Qr)*dt_squared;
  relaVar[agent_id].P[2][2] += (2*Qr)*dt_squared;

  // Update the state variables prior estimate
  xij = relaVar[agent_id].S[STATE_rlX];
  yij = relaVar[agent_id].S[STATE_rlY];
  // Calculate distance predication 
  float distPred = arm_sqrt(xij*xij + yij*yij + (hi-hj)*(hi-hj)) + 0.0001f;

  // For the measurement update
  float distMeas = dij;

  // Measurement Jacobian
  h[0] = xij/distPred;
  h[1] = yij/distPred;
  h[2] = 0;

  // Some intermediate matrices  for measurement covariance update
  // H transpose
  mat_trans(&H, &HTm); 
  // PH'
  mat_mult(&Pm, &HTm, & PHTm);
  // HPH' + R Note: This is a scalar
  float HPHR = powf(Ruwb, 2);
  // This computes HPH' + R which is S in our framework
  for(int dimension=0; dimension<STATE_DIM_rl; dimension++){
    // this obviously only works if the update is scalar (as in this function)
    HPHR += H.pData[dimension]*PHTd[dimension];
  }
  // Now we can calculate the Kalman Gain
  for(int dimension=0; dimension<STATE_DIM_rl; dimension++){
    // kalman gain = (PH' (HPH' + R )^-1)
    K[dimension] = PHTd[dimension]/HPHR;
    // State update
    relaVar[agent_id].S[dimension] = relaVar[agent_id].S[dimension] + K[dimension] * (distMeas - distPred);
  }
  // Calculate KH, which is KD in our framework
  // KH
  mat_mult(&Km, &H, &tmpNN1m);
  // Now we can compute KH - I in a for loop fashion (diagonal)
  for(int i=0; i<STATE_DIM_rl; i++){
    tmpNN1d[STATE_DIM_rl*i+i] -= 1;
  }

  // [Sam] Covariance posteriori update
  // (KH-I)'
  mat_trans(&tmpNN1m, &tmpNN2m);
  // (KH-I)*P
  mat_mult(&tmpNN1m, &Pm, &tmpNN3m);
  // (KH - I)*P*(KH - I)' + ?
  mat_mult(&tmpNN3m, &tmpNN2m, &Pm);
  
  // Adding the R term
  for (int i=0; i<STATE_DIM_rl; i++) {
    for (int j=i; j<STATE_DIM_rl; j++) {
      float v = K[i] * Ruwb * Ruwb * K[j];
      float p = 0.5f* relaVar[agent_id].P[i][j] + 0.5f*relaVar[agent_id].P[j][i] + v; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        relaVar[agent_id].P[i][j] = relaVar[agent_id].P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        relaVar[agent_id].P[i][j] = relaVar[agent_id].P[j][i] = MIN_COVARIANCE;
      } else {
        relaVar[agent_id].P[i][j] = relaVar[agent_id].P[j][i] = p;
      }
    }
  }
}


// This function helps the control to read the state information
bool relativeInfoRead(float_t* relaVarParam, float_t* inputVarParam){
  // If control is connected
  if(fullConnect){
    for(int i=0; i<NumAgent; i++){
      // [Sam] Populate the input variables using addresses
      *(relaVarParam + i*STATE_DIM_rl + 0) = relaVar[i].S[STATE_rlX];
      *(relaVarParam + i*STATE_DIM_rl + 1) = relaVar[i].S[STATE_rlY];
      *(relaVarParam + i*STATE_DIM_rl + 2) = relaVar[i].S[STATE_rlYaw];
      *(inputVarParam + i*STATE_DIM_rl + 0) = inputVar[i][STATE_rlX];
      *(inputVarParam + i*STATE_DIM_rl + 1) = inputVar[i][STATE_rlY];
      *(inputVarParam + i*STATE_DIM_rl + 2) = inputVar[i][STATE_rlYaw];
    }
    return true;
  }
  else
    return false;
}

// [Sam] Logging information
LOG_GROUP_START(relative_pos)
// first agent
LOG_ADD(LOG_FLOAT, rlX0, &relaVar[0].S[STATE_rlX])  // xij
LOG_ADD(LOG_FLOAT, rlY0, &relaVar[0].S[STATE_rlY]) // yij
LOG_ADD(LOG_FLOAT, rlYaw0, &relaVar[0].S[STATE_rlYaw]) // rij
// second agent
LOG_ADD(LOG_FLOAT, rlX1, &relaVar[1].S[STATE_rlX])
LOG_ADD(LOG_FLOAT, rlY1, &relaVar[1].S[STATE_rlY])
LOG_ADD(LOG_FLOAT, rlYaw1, &relaVar[1].S[STATE_rlYaw])
// LOG_ADD(LOG_FLOAT, rlX2, &relaVar[2].S[STATE_rlX])
// LOG_ADD(LOG_FLOAT, rlY2, &relaVar[2].S[STATE_rlY])
// LOG_ADD(LOG_FLOAT, rlYaw2, &relaVar[2].S[STATE_rlYaw])
LOG_GROUP_STOP(relative_pos)

// Log the initial covariance parameters
// PARAM_GROUP_START(arelative_pos)
// PARAM_ADD(PARAM_FLOAT, noiFlow, &Qv) // make sure the name is not too long
// PARAM_ADD(PARAM_FLOAT, noiGyroZ, &Qr)
// PARAM_ADD(PARAM_FLOAT, noiUWB, &Ruwb)
// PARAM_ADD(PARAM_FLOAT, Ppos, &InitCovPos)
// PARAM_ADD(PARAM_FLOAT, Pyaw, &InitCovYaw)
// PARAM_GROUP_STOP(arelative_pos)

PARAM_GROUP_START(rl_param)
PARAM_ADD_CORE(PARAM_UINT8, start_rl, &enableRL) /* use to start/stop logging*/
PARAM_GROUP_STOP(rl_param)

