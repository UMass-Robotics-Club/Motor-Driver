/*
 * Copyright (c) 2020-2024 REV Robotics
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

#include <hal/Types.h>

#include "rev/REVLibErrors.h"

#define API_ID_FROM_ARB_ID(ARBID) (((ARBID) >> 6) & 0x3FF)

extern "C" {

typedef enum {
    c_SparkMax_SparkMax,
    c_SparkMax_SparkFlex,
    c_SparkMax_Unknown = 255
} c_SparkMax_SparkModel;

extern int32_t CreateCANID(int32_t deviceID, int32_t apiID);
extern uint64_t ArrToUint64(uint8_t* data8);
extern void Uint64ToArr(uint64_t data, uint8_t* arr);
extern c_REVLib_ErrorCode CANSendAndReceive(
    HAL_CANHandle handle, int32_t sendApiId, uint8_t* sendData,
    int32_t sendDataSize, int32_t receiveApiId, uint8_t* receiveData,
    uint32_t maxRetries, uint32_t timeoutMs);
extern c_REVLib_ErrorCode CANSendAndReceiveRTR(
    HAL_CANHandle handle, int32_t sendApiId, int32_t dataSize,
    uint8_t* receiveData, uint32_t maxRetries, uint32_t timeoutMs);
}  // extern "C"
