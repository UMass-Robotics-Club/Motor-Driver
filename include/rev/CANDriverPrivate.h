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

#pragma once

#include <stdint.h>

#include <cstring>
#include <mutex>
#include <set>

class CAN_ExistingDeviceIds {
public:
    CAN_ExistingDeviceIds() : m_existingDeviceIds() {}

    CAN_ExistingDeviceIds(const CAN_ExistingDeviceIds&) = delete;
    CAN_ExistingDeviceIds& operator=(const CAN_ExistingDeviceIds&) = delete;
    CAN_ExistingDeviceIds(CAN_ExistingDeviceIds&&) = delete;
    CAN_ExistingDeviceIds& operator=(CAN_ExistingDeviceIds&&) = delete;

    bool InsertDevice(uint8_t deviceId) {
        std::scoped_lock<std::mutex> lock{m_mutex};
        return m_existingDeviceIds.insert(deviceId).second;
    }

    bool ContainsDevice(uint8_t deviceId) {
        std::scoped_lock<std::mutex> lock{m_mutex};
        return m_existingDeviceIds.count(deviceId) > 0;
    }

    void RemoveDevice(uint8_t deviceId) {
        std::scoped_lock<std::mutex> lock{m_mutex};
        m_existingDeviceIds.erase(deviceId);
    }

private:
    std::mutex m_mutex;
    std::set<uint8_t> m_existingDeviceIds;
};

enum { kMinCANId = 0, kMaxCANId = 62 };
static inline bool CAN_CheckId(int deviceId) {
    return (deviceId >= kMinCANId) && (deviceId <= kMaxCANId);
}
