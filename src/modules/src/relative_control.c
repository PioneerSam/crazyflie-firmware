#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_loc.h"
#include "relative_control.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include <stdlib.h> // random
#include "lpsTdoa4Tag.h"
#include "configblock.h"
#include "uart2.h"
#include "log.h"

#define NumAgent 2

// Some useful boolean flags
static bool isInit;
static bool onGround = true;
static bool keepFlying = false; // Control of takeoff and landing

static setpoint_t setpoint;
static float relaVarInCtrl[NumAgent][STATE_DIM_rl];
static float inputVarInCtrl[NumAgent][STATE_DIM_rl];
static uint8_t selfID;
static float height;

// PID gains
static float relaCtrl_p = 2.0f;
static float relaCtrl_i = 0.0001f;
static float relaCtrl_d = 0.01f;

// static float NDI_k = 2.0f;
// static char c = 0; // monoCam

// the setpoint class is defined in syablizer_types.h
// this sets the setpoint to hover at z
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate){
    setpoint->mode.z = modeAbs;
    setpoint->position.z = z;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitudeRate.yaw = yawrate;
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->velocity.x = vx;
    setpoint->velocity.y = vy;
    // true means the velocity if given in body frames
    setpoint->velocity_body = true;
    commanderSetSetpoint(setpoint,3);
}

static void flyRandomIn1meter(void){
    // sample a yaw angle from 0 to 2pi
    float randomYaw = (rand()/(float) RAND_MAX)*6.28f;
    // sample a velocity from 0 to 1
    float randomVel = (rand()/(float) RAND_MAX);
    // project onto x and y direction
    float vxBody = randomVel * cosf(randomYaw);
    float vyBody = randomVel * sinf(randomYaw);
    // send 100 setpoints with random speed
    for(int i=1; i<100; i++){
        setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
        vTaskDelay(M2T(10));
    }
    // send 100 setpoints back
    for(int i=1; i<100; i++){
        setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
        vTaskDelay(M2T(10));
    }
}


// Define some parameters 
// If a >= 0, then a is 1 else a = -1
#define SIGN(a) ((a>=0)?1:-1)
static float targetX;
static float targetY;
static float PreErr_x = 0;
static float PreErr_y = 0;
// For integrator error
static float IntErr_x = 0;
static float IntErr_y = 0;
// Previous time tracker
static uint32_t PreTime;

// tarX and tarY are the target X and Y
static void formation0asCenter(float tarX, float tarY){
    // Time elasped
    float dt = (float)(xTaskGetTickCount()-PreTime)/configTICK_RATE_HZ;
    PreTime = xTaskGetTickCount();
    if(dt>1) // skip the first run of the EKF
    return;

    // PID control for the formation flight
    float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]);
    float err_y = -(tarY - relaVarInCtrl[0][STATE_rlY]);

    // P
    float pid_vx = relaCtrl_p * err_x;
    float pid_vy = relaCtrl_p * err_y;

    float dx = (err_x - PreErr_x)/dt;
    float dy = (err_y - PreErr_y)/dt;

    // Update the error in the last timestep
    PreErr_x = err_x;
    PreErr_y = err_y;

    // D
    pid_vx += relaCtrl_d * dx;
    pid_vy += relaCtrl_d * dy;

    IntErr_x += err_x * dt;
    IntErr_y += err_y * dt;

    // Clip the integral component
    pid_vx += relaCtrl_i * constrain(IntErr_x, -0.5, 0.5);
    pid_vy += relaCtrl_i * constrain(IntErr_y, -0.5, 0.5);

    // Clip the final PID input
    pid_vx = constrain(pid_vx, -1.5f, 1.5f);
    pid_vy = constrain(pid_vy, -1.5f, 1.5f);

    setHoverSetpoint(&setpoint, pid_vx, pid_vy, height, 0);

    // float rep_x = 0.0f;
    // float rep_y = 0.0f;
    // for(uint8_t i=0; i<NumAgent; i++){
    //   if(i!=selfID){
    //     float dist = relaVarInCtrl[i][STATE_rlX]*relaVarInCtrl[i][STATE_rlX] + relaVarInCtrl[i][STATE_rlY]*relaVarInCtrl[i][STATE_rlY];
    //     dist = sqrtf(dist);
    //     rep_x += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlX]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlX]);
    //     rep_y += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlY]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlY]);
    //   }
    // }
    // rep_x = constrain(rep_x, -1.5f, 1.5f);
    // rep_y = constrain(rep_y, -1.5f, 1.5f);

    // pid_vx = constrain(pid_vx + rep_x, -1.5f, 1.5f);
    // pid_vy = constrain(pid_vy + rep_y, -1.5f, 1.5f);
}


// Initialize the relative control task
void relativeControlInit(void)
{
  if (isInit)
    return;

  // [Sam] Identification methods
  selfID = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f) - 5);

  xTaskCreate(relativeControlTask,"relative_Control",configMINIMAL_STACK_SIZE, NULL,3,NULL );
  isInit = true;
}


// The main loop and relative control task
void relativeControlTask(void* arg){
    // Target setpoints
    static const float targetList[7][STATE_DIM_rl]={{0.0f, 0.0f, 0.0f}, {-1.0f, 0.5f, 0.0f}, {-1.0f, -0.5f, 0.0f}, {-1.0f, -1.5f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {-2.0f, 0.0f, 0.0f}};
    static uint32_t ctrlTick;
    systemWaitStart();
    // height = (float)selfID*0.1f+0.2f;
    // static height
    height = 0.5f;
    // main loop starts
    while(1){
        // delay
        vTaskDelay(10);

        // [Sam] some code commented
        // keepFlying = command_share(selfID, keepFlying);    // need to change to our own function
        // Call function from relative_loc.c to get the current states
        if(relativeInfoRead((float *) relaVarInCtrl, (float *) inputVarInCtrl) && keepFlying){
            // take off first and check if we are on the ground
            if(onGround){
                // take off to a z value of 0.3f by setting the setpoint five times
                for(int i=0; i<5; i++){
                    setHoverSetpoint(&setpoint, 0, 0, 0.3f, 0);
                    vTaskDelay(M2T(100));
                }
                // [Sam] unsynchronize
                for(int i=0; i<10*selfID;i++){
                    setHoverSetpoint(&setpoint, 0, 0, 0.3f, 0);
                    vTaskDelay(M2T(100));
                }
                // now we have taken off
                onGround = false;
                ctrlTick = xTaskGetTickCount();
            }

            // control loop
            // setHoverSetpoint(&setpoint, 0, 0, height, 0); // hover
            // Time elapsed
            uint32_t tickInterval = xTaskGetTickCount() - ctrlTick;
            // random flight within first 10 seconds (need to change)
            if(tickInterval < 20000){
                flyRandomIn1meter();
                targetX = relaVarInCtrl[0][STATE_rlX];
                targetY = relaVarInCtrl[0][STATE_rlY];
            }
            else{
                // from 20 secs to 50 secs
                if((tickInterval > 20000) && (tickInterval < 50000)){ // 0-random, other formation
                    if(selfID==0)
                        // first agent fly random trajectory
                        flyRandomIn1meter();
                    else
                        formation0asCenter(targetX,targetY);
                }

                // from 50 secs to 70 secs
                if((tickInterval > 50000) && (tickInterval < 70000)){
                    if(selfID==0)
                        flyRandomIn1meter();
                    else{
                        // [Sam] seems like we need to do a rotation
                        targetX = -cosf(relaVarInCtrl[0][STATE_rlYaw])*targetList[selfID][STATE_rlX] + sinf(relaVarInCtrl[0][STATE_rlYaw])*targetList[selfID][STATE_rlY];
                        targetY = -sinf(relaVarInCtrl[0][STATE_rlYaw])*targetList[selfID][STATE_rlX] - cosf(relaVarInCtrl[0][STATE_rlYaw])*targetList[selfID][STATE_rlY];
                        formation0asCenter(targetX, targetY); 
                    }
                }

                // above 70 secs
                if(tickInterval > 70000){
                    if(selfID==0)
                        setHoverSetpoint(&setpoint, 0, 0, height, 0);
                    else
                        formation0asCenter(targetX,targetY);
                }
            }
        }else{
            // landing procedure
            if(!onGround){
                for(int i=0; i<5; i++){
                    setHoverSetpoint(&setpoint, 0, 0, 0.3f-(float)i*0.05f, 0);
                    vTaskDelay(M2T(10));
                }
                onGround = true;
            }
        }
    }
}



// Logging PID info
PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_ADD(PARAM_FLOAT, relaCtrl_p, &relaCtrl_p)
PARAM_ADD(PARAM_FLOAT, relaCtrl_i, &relaCtrl_i)
PARAM_ADD(PARAM_FLOAT, relaCtrl_d, &relaCtrl_d)
PARAM_GROUP_STOP(relative_ctrl)

// LOG_GROUP_START(mono_cam)
// LOG_ADD(LOG_UINT8, charCam, &c)
// LOG_GROUP_STOP(mono_cam)
