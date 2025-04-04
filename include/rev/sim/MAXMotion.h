/*
 * Copyright (c) 2024 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

//
// Created by Rylan on 8/2/2024.
//

#ifndef FRC_REVLIB_SRC_MAIN_DRIVER_INCLUDE_REV_SIM_MAXMOTION_H_
#define FRC_REVLIB_SRC_MAIN_DRIVER_INCLUDE_REV_SIM_MAXMOTION_H_

#include <rev/CANSparkDriver.h>

extern "C" {

typedef struct {
    float kMAXMotionMaxVelocity;
    float kMAXMotionMaxAccel;
    float kMAXMotionMaxJerk;
    float kAllowedClosedLoopError;
    c_Spark_MAXMotionPositionMode kMAXMotionPositionMode;
} maxmotion_constants_t;

#define VELOCITY_MARGIN(maxAccel) maxAccel / 20.0f
#define SECONDS_PER_MINUTE 60.0f

float smartermotion_calculate_trapezoidal_position_target(
    const maxmotion_constants_t* motion_params, const float curr_pos,
    const float curr_vel, const float setpoint, const float last_setpoint,
    const float dt);
float smartermotion_calculate_acceleration_direction(
    const float curr_vel, const float diffSign, const float diffErr,
    const float convertedMaxVel, const float convertedMaxAccel);
float smartermotion_calculate_distance_to_stop(const float velocity,
                                               const float convertedMaxAccel);
float smartermotion_calculate_position_delta(const float curr_vel,
                                             const float dt,
                                             const float accelDirection,
                                             const float convertedMaxAccel);

extern float smartervelocity_calculate_output_velocity(
    maxmotion_constants_t* motion_params, float setpoint, float prev_vel,
    float dt);

}  // extern "C"
#endif  // FRC_REVLIB_SRC_MAIN_DRIVER_INCLUDE_REV_SIM_MAXMOTION_H_
