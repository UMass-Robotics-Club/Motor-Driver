/*
 * Copyright (c) 2024-2025 REV Robotics
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

#pragma once

#include <stdint.h>

#include "rev/CANCommonParameters.h"

extern "C" {

typedef enum {
    c_Spark_kParamOK = 0,
    c_Spark_kInvalidID = 1,
    c_Spark_kMismatchType = 2,
    c_Spark_kAccessMode = 3,
    c_Spark_kInvalid = 4,
    c_Spark_kNotImplementedDeprecated = 5
} c_Spark_ParameterStatus;

typedef enum {
    c_Spark_kCanID = 0,
    c_Spark_kInputMode = 1,
    c_Spark_kMotorType = 2,
    c_Spark_kCommAdvance = 3,
    c_Spark_kSensorType = 4,
    c_Spark_kCtrlType = 5,
    c_Spark_kIdleMode = 6,
    c_Spark_kInputDeadband = 7,
    c_Spark_kLegacyFeedbackSensorPID0 = 8,
    c_Spark_kClosedLoopControlSensor = 9,
    c_Spark_kPolePairs = 10,
    c_Spark_kCurrentChop = 11,
    c_Spark_kCurrentChopCycles = 12,
    c_Spark_kP_0 = 13,
    c_Spark_kI_0 = 14,
    c_Spark_kD_0 = 15,
    c_Spark_kF_0 = 16,
    c_Spark_kIZone_0 = 17,
    c_Spark_kDFilter_0 = 18,
    c_Spark_kOutputMin_0 = 19,
    c_Spark_kOutputMax_0 = 20,
    c_Spark_kP_1 = 21,
    c_Spark_kI_1 = 22,
    c_Spark_kD_1 = 23,
    c_Spark_kF_1 = 24,
    c_Spark_kIZone_1 = 25,
    c_Spark_kDFilter_1 = 26,
    c_Spark_kOutputMin_1 = 27,
    c_Spark_kOutputMax_1 = 28,
    c_Spark_kP_2 = 29,
    c_Spark_kI_2 = 30,
    c_Spark_kD_2 = 31,
    c_Spark_kF_2 = 32,
    c_Spark_kIZone_2 = 33,
    c_Spark_kDFilter_2 = 34,
    c_Spark_kOutputMin_2 = 35,
    c_Spark_kOutputMax_2 = 36,
    c_Spark_kP_3 = 37,
    c_Spark_kI_3 = 38,
    c_Spark_kD_3 = 39,
    c_Spark_kF_3 = 40,
    c_Spark_kIZone_3 = 41,
    c_Spark_kDFilter_3 = 42,
    c_Spark_kOutputMin_3 = 43,
    c_Spark_kOutputMax_3 = 44,
    c_Spark_kInverted = 45,
    c_Spark_kOutputRatio = 46,
    c_Spark_kSerialNumberLow = 47,
    c_Spark_kSerialNumberMid = 48,
    c_Spark_kSerialNumberHigh = 49,
    c_Spark_kLimitSwitchFwdPolarity = 50,
    c_Spark_kLimitSwitchRevPolarity = 51,
    c_Spark_kHardLimitFwdEn = 52,
    c_Spark_kHardLimitRevEn = 53,
    c_Spark_kSoftLimitFwdEn = 54,
    c_Spark_kSoftLimitRevEn = 55,
    c_Spark_kRampRate = 56,
    c_Spark_kFollowerID = 57,
    c_Spark_kFollowerConfig = 58,
    c_Spark_kSmartCurrentStallLimit = 59,
    c_Spark_kSmartCurrentFreeLimit = 60,
    c_Spark_kSmartCurrentConfig = 61,
    c_Spark_kSmartCurrentReserved = 62,
    c_Spark_kMotorKv = 63,
    c_Spark_kMotorR = 64,
    c_Spark_kMotorL = 65,
    c_Spark_kMotorRsvd1 = 66,
    c_Spark_kMotorRsvd2 = 67,
    c_Spark_kMotorRsvd3 = 68,
    c_Spark_kEncoderCountsPerRev = 69,
    c_Spark_kEncoderAverageDepth = 70,
    c_Spark_kEncoderSampleDelta = 71,
    c_Spark_kEncoderInverted = 72,
    c_Spark_kEncoderRsvd1 = 73,
    c_Spark_kVoltageCompMode = 74,
    c_Spark_kCompensatedNominalVoltage = 75,
    c_Spark_kSmartMotionMaxVelocity_0 = 76,
    c_Spark_kSmartMotionMaxAccel_0 = 77,
    c_Spark_kSmartMotionMinVelOutput_0 = 78,
    c_Spark_kSmartMotionAllowedClosedLoopError_0 = 79,
    c_Spark_kSmartMotionAccelStrategy_0 = 80,
    c_Spark_kSmartMotionMaxVelocity_1 = 81,
    c_Spark_kSmartMotionMaxAccel_1 = 82,
    c_Spark_kSmartMotionMinVelOutput_1 = 83,
    c_Spark_kSmartMotionAllowedClosedLoopError_1 = 84,
    c_Spark_kSmartMotionAccelStrategy_1 = 85,
    c_Spark_kSmartMotionMaxVelocity_2 = 86,
    c_Spark_kSmartMotionMaxAccel_2 = 87,
    c_Spark_kSmartMotionMinVelOutput_2 = 88,
    c_Spark_kSmartMotionAllowedClosedLoopError_2 = 89,
    c_Spark_kSmartMotionAccelStrategy_2 = 90,
    c_Spark_kSmartMotionMaxVelocity_3 = 91,
    c_Spark_kSmartMotionMaxAccel_3 = 92,
    c_Spark_kSmartMotionMinVelOutput_3 = 93,
    c_Spark_kSmartMotionAllowedClosedLoopError_3 = 94,
    c_Spark_kSmartMotionAccelStrategy_3 = 95,
    c_Spark_kIMaxAccum_0 = 96,
    c_Spark_kSlot3Placeholder1_0 = 97,
    c_Spark_kSlot3Placeholder2_0 = 98,
    c_Spark_kSlot3Placeholder3_0 = 99,
    c_Spark_kIMaxAccum_1 = 100,
    c_Spark_kSlot3Placeholder1_1 = 101,
    c_Spark_kSlot3Placeholder2_1 = 102,
    c_Spark_kSlot3Placeholder3_1 = 103,
    c_Spark_kIMaxAccum_2 = 104,
    c_Spark_kSlot3Placeholder1_2 = 105,
    c_Spark_kSlot3Placeholder2_2 = 106,
    c_Spark_kSlot3Placeholder3_2 = 107,
    c_Spark_kIMaxAccum_3 = 108,
    c_Spark_kSlot3Placeholder1_3 = 109,
    c_Spark_kSlot3Placeholder2_3 = 110,
    c_Spark_kSlot3Placeholder3_3 = 111,
    c_Spark_kPositionConversionFactor = 112,
    c_Spark_kVelocityConversionFactor = 113,
    c_Spark_kClosedLoopRampRate = 114,
    c_Spark_kSoftLimitFwd = 115,
    c_Spark_kSoftLimitRev = 116,
    c_Spark_kSoftLimitRsvd0 = 117,
    c_Spark_kSoftLimitRsvd1 = 118,
    c_Spark_kAnalogRevPerVolt = 119,
    c_Spark_kAnalogRotationsPerVoltSec = 120,
    c_Spark_kAnalogAverageDepth = 121,
    c_Spark_kAnalogSensorMode = 122,
    c_Spark_kAnalogInverted = 123,
    c_Spark_kAnalogSampleDelta = 124,
    c_Spark_kAnalogRsvd0 = 125,
    c_Spark_kAnalogRsvd1 = 126,
    c_Spark_kDataPortConfig = 127,
    c_Spark_kAltEncoderCountsPerRev = 128,
    c_Spark_kAltEncoderAverageDepth = 129,
    c_Spark_kAltEncoderSampleDelta = 130,
    c_Spark_kAltEncoderInverted = 131,
    c_Spark_kAltEncodePositionFactor = 132,
    c_Spark_kAltEncoderVelocityFactor = 133,
    c_Spark_kAltEncoderRsvd0 = 134,
    c_Spark_kAltEncoderRsvd1 = 135,
    c_Spark_kHallSensorSampleRate = 136,
    c_Spark_kHallSensorAverageDepth = 137,
    c_Spark_kNumParameters = 138,
    c_Spark_kDutyCyclePositionFactor = 139,
    c_Spark_kDutyCycleVelocityFactor = 140,
    c_Spark_kDutyCycleInverted = 141,
    c_Spark_kDutyCycleSensorMode = 142,
    c_Spark_kDutyCycleAverageDepth = 143,
    c_Spark_kDutyCycleSampleDelta = 144,
    c_Spark_kDutyCycleOffsetv1p6p2 = 145,
    c_Spark_kDutyCycleRsvd0 = 146,
    c_Spark_kDutyCycleRsvd1 = 147,
    c_Spark_kDutyCycleRsvd2 = 148,
    c_Spark_kPositionPIDWrapEnable = 149,
    c_Spark_kPositionPIDMinInput = 150,
    c_Spark_kPositionPIDMaxInput = 151,
    c_Spark_kDutyCycleZeroCentered = 152,
    c_Spark_kDutyCyclePrescaler = 153,
    c_Spark_kDutyCycleOffset = 154,
    c_Spark_kProductId = 155,
    c_Spark_kDeviceMajorVersion = 156,
    c_Spark_kDeviceMinorVersion = 157,
    c_Spark_kStatus0Period = 158,
    c_Spark_kStatus1Period = 159,
    c_Spark_kStatus2Period = 160,
    c_Spark_kStatus3Period = 161,
    c_Spark_kStatus4Period = 162,
    c_Spark_kStatus5Period = 163,
    c_Spark_kStatus6Period = 164,
    c_Spark_kStatus7Period = 165,
    c_Spark_kMAXMotionMaxVelocity_0 = 166,
    c_Spark_kMAXMotionMaxAccel_0 = 167,
    c_Spark_kMAXMotionMaxJerk_0 = 168,
    c_Spark_kMAXMotionAllowedClosedLoopError_0 = 169,
    c_Spark_kMAXMotionPositionMode_0 = 170,
    c_Spark_kMAXMotionMaxVelocity_1 = 171,
    c_Spark_kMAXMotionMaxAccel_1 = 172,
    c_Spark_kMAXMotionMaxJerk_1 = 173,
    c_Spark_kMAXMotionAllowedClosedLoopError_1 = 174,
    c_Spark_kMAXMotionPositionMode_1 = 175,
    c_Spark_kMAXMotionMaxVelocity_2 = 176,
    c_Spark_kMAXMotionMaxAccel_2 = 177,
    c_Spark_kMAXMotionMaxJerk_2 = 178,
    c_Spark_kMAXMotionAllowedClosedLoopError_2 = 179,
    c_Spark_kMAXMotionPositionMode_2 = 180,
    c_Spark_kMAXMotionMaxVelocity_3 = 181,
    c_Spark_kMAXMotionMaxAccel_3 = 182,
    c_Spark_kMAXMotionMaxJerk_3 = 183,
    c_Spark_kMAXMotionAllowedClosedLoopError_3 = 184,
    c_Spark_kMAXMotionPositionMode_3 = 185,
    c_Spark_kForceEnableStatus0 = 186,
    c_Spark_kForceEnableStatus1 = 187,
    c_Spark_kForceEnableStatus2 = 188,
    c_Spark_kForceEnableStatus3 = 189,
    c_Spark_kForceEnableStatus4 = 190,
    c_Spark_kForceEnableStatus5 = 191,
    c_Spark_kForceEnableStatus6 = 192,
    c_Spark_kForceEnableStatus7 = 193,
    c_Spark_kFollowerModeLeaderId = 194,
    c_Spark_kFollowerModeIsInverted = 195,
    c_Spark_kDutyCycleEncoderStartPulseUs = 196,
    c_Spark_kDutyCycleEncoderEndPulseUs = 197,
    c_Spark_NumParameters
} c_Spark_ConfigParameter;

c_REVLib_ParameterType c_Spark_GetParameterType(
    c_Spark_ConfigParameter parameterId);

uint32_t c_Spark_GetParameterDefaultValue(c_Spark_ConfigParameter parameterId);

const char* c_Spark_GetParameterName(c_Spark_ConfigParameter parameterId);

// For debugging purposes, to verify internal state.
c_Spark_ConfigParameter c_Spark_GetConfigParameter(
    c_Spark_ConfigParameter parameterId);

}  // extern "C"
