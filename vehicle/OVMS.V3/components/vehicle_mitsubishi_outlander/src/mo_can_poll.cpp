/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          02th May 2020
 ;
 ;    (C) 2020       Tamás Kovács (KommyKT)
 ;
 ; Permission is hereby granted, free of charge, to any person obtaining a copy
 ; of this software and associated documentation files (the "Software"), to deal
 ; in the Software without restriction, including without limitation the rights
 ; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 ; copies of the Software, and to permit persons to whom the Software is
 ; furnished to do so, subject to the following conditions:
 ;
 ; The above copyright notice and this permission notice shall be included in
 ; all copies or substantial portions of the Software.
 ;
 ; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 ; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 ; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 ; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 ; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 ; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 ; THE SOFTWARE.
 */
#include "vehicle_mitsubishi_outlander.h"
#include "mo_obd_pids.h"

static const char *TAG = "v-mo-canpoll";

/**
 * Incoming poll reply messages
 */
void OvmsVehicleMitsubishiOutlander::IncomingPollReply(canbus* bus, uint16_t type, uint16_t pid, uint8_t* data, uint8_t length, uint16_t mlremain)
{
    switch (m_poll_moduleid_low) {
        case bmuRxId:
            IncomingBmuPoll(pid, data, length, mlremain);
            break;
            
        case obcRxId:
            IncomingObcPoll(pid, data, length, mlremain);
            break;
            
        case fmcuRxId: //Front Motor Control Unit FMCU
        {
            // 27 10 48 0
            // 49 0 18 4E
            //ESP_LOGW(TAG, "FMCU Data: %X %X %X %X Len: %u Rem: %u",data[0],data[1],data[2],data[3],length,mlremain);
            //unsigned int frontRPM = data[2]*256+data[3]-20000;
            //ESP_LOGI(TAG, "Front Motor RPM = %u",frontRPM);
            break;
        }
            
        case rmcuRxId:  //Rear Motor Control Unit RMCU
        {
            // 27 10 48 0
            // 49 0 18 4E
            //ESP_LOGW(TAG, "RMCU Data: %X %X %X %X Len: %u Rem: %u",data[0],data[1],data[2],data[3],length,mlremain);
            //unsigned int rearRPM = data[2]*256+data[3]-20000;
            //ESP_LOGI(TAG, "Rear Motor RPM = %u",rearRPM);
            break;
        }
            
        default:
            ESP_LOGW(TAG, "Unknown module: %03x", m_poll_moduleid_low);
    }
}
