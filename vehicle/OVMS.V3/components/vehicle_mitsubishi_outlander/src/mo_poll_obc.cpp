/*
;    Project:       Open Vehicle Monitor System
;    Date:          3rd September 2020
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011       Sonny Chen @ EPRO/DX
;    (C) 2020       Chris Staite
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
#include "metrics_standard.h"
static const char *TAG = "v-mo-bmu";

void OvmsVehicleMitsubishiOutlander::IncomingObcPoll(
        uint16_t pid, uint8_t* data, uint8_t length, uint16_t remain)
{
    switch (pid)
    {
        case 1:
            OBCresponse1(m_poll_ml_frame, data);
            break;
        case 2:
            OBCresponse2(m_poll_ml_frame, data);
            break;
        case 3:
            OBCresponse3(m_poll_ml_frame, data);
            break;
        default:
            break;
    }
}

void OvmsVehicleMitsubishiOutlander::OBCresponse1(uint16_t m_poll_ml_frame, uint8_t* data)
{
    switch (m_poll_ml_frame) {
        case 0:
        {
            unsigned int chargeCount100V = data[0] * 256 + data[1];
            unsigned int chargeCount200V = data[2] * 256 + data[3];
            ESP_LOGV(TAG, "PID = 1 chargeCount100V = %u chargeCount200V = %u", chargeCount100V, chargeCount200V);
            break;
        }
        case 1:
        {
            unsigned int chargeTime100V = (data[0] * 256 + data[1])*256+data[2];
            unsigned int chargeTime200V = (data[3] * 256 + data[4])*256+data[5];
            unsigned int currentChargeTime = data[6];
            ESP_LOGV(TAG, "PID = 1 chargeTime100V = %u chargeTime200V = %u Current Charge Time = %u", chargeTime100V, chargeTime200V, currentChargeTime);
            break;
        }
        case 2:
        {
            unsigned int abnormalStops = data[0] * 256 + data[1];
            unsigned int activeCount = data[2] * 256 + data[3];
            if (activeCount > dcdcCount) {
                dcdcCount = activeCount;
            }
            unsigned int activeTime = (unsigned int)data[6]
            + ((unsigned int)data[5]<<8)
            + ((unsigned int)data[4]<<16);
            ESP_LOGV(TAG, "PID = 1 abnormalStops = %u DC-DC active count = %u DC-DC Active Time = %u Secs", abnormalStops, activeCount, activeTime);
            break;
        }
        case 3:
        {
            unsigned int currentActiveTime = data[0] * 256 + data[1];
            unsigned int dcdcFails = data[2];
            ESP_LOGV(TAG, "PID = 1 DC-DC Current Time = %u Secs DC-DC Fails = %u", currentActiveTime, dcdcFails);
            break;
        }
        default:
            break;
    }
}

void OvmsVehicleMitsubishiOutlander::OBCresponse2(uint16_t m_poll_ml_frame, uint8_t* data)
{
    switch (m_poll_ml_frame) {
        case 0:
        {
            //bool obcPermit = data[0]&0x80;
            //bool reservedCharging = data[0]&0x40;
            StandardMetrics.ms_v_charge_inprogress->SetValue(data[0]&0x20);
            bool chargFlag = data[0]&0x20;
            if (chargFlag) ESP_LOGE(TAG, "Charge in Progress");
            bool abnormalStop = data[0]&0x10;
            if (abnormalStop) ESP_LOGE(TAG, "Abnormal Stop");
            bool s2Switch = data[0]&0x08;
            if(s2Switch) ESP_LOGE(TAG, "Something Happened");
            //bool coolRequest = data[0]&0x04;
            //if (coolRequest) ESP_LOGW(TAG, "Cooling Request");
            //bool controlPSrequest = data[0]&0x02;
            //bool pfcStartup = data[0]&0x01;
            // bool inputCurrent0PointCheck = data[1]&0x80;
            //bool outputLimitHisV = data[1]&0x40;
            // bool outputLimitHisA = data[1]&0x20;
            //bool currentInputVoltType = data[1]&0x10;
            //bool controlLimit = data[1]&0x08;
            //bool outputLimit = data[1]&0x04;
            //bool outputLimitJudgeTemp = data[1]&0x02;
            //bool outputLimitJudgeInputV = data[1]&0x01;
            unsigned int inputVoltage = data[2]*256+data[3];
            //if (mi_SC){
            //    StandardMetrics.ms_v_charge_voltage->SetValue(inputVoltage/10, Volts);
            //}
            ESP_LOGV(TAG, "PID = 2 inputVoltage = %.1f FLAGS %X %X",(float) inputVoltage/10, data[0], data[1]);
            break;
        }
        case 1:
        {
            //ESP_LOGW("766 2 1" , "Data:%02x %02x %02x %02x %02x %02x %02x %02x LENG:%02x REM:%02x", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], length, mlremain);
            /*
             unsigned int inputCurrent = data[0];
             unsigned int outputVoltage = data[1]*256+data[2];
             unsigned int outputCurrent = data[3];
             unsigned int voltageTarget = data[4]*256+data[5];
             unsigned int currentTarget = data[6];
             
             if (mi_SC){
             StandardMetrics.ms_v_charge_current->SetValue(inputCurrent/40, Amps);
             StandardMetrics.ms_v_charge_power->SetValue((StandardMetrics.ms_v_charge_voltage->AsFloat() * StandardMetrics.ms_v_charge_current->AsFloat())/1000, Watts);
             }
             
             v_c_power_dc->SetValue((StandardMetrics.ms_v_charge_voltage->AsFloat() * StandardMetrics.ms_v_charge_current->AsFloat()) / 1000, kW);
             if ( (StdMetrics.ms_v_charge_voltage->AsInt() > 0) && (StdMetrics.ms_v_charge_current->AsFloat() > 0.0) )
             {
             StandardMetrics.ms_v_charge_inprogress->SetValue(true);
             StandardMetrics.ms_v_charge_kwh->SetValue(StandardMetrics.ms_v_charge_kwh->AsFloat() + (v_c_power_dc->AsFloat() / 36000.0));
             ms_v_charge_dc_kwh->SetValue(StandardMetrics.ms_v_charge_kwh->AsFloat());
             }
             */
            //ESP_LOGW(TAG, "PID = 2 inputCurrent = %.1f outputVoltage = %.1f outputCurrent = %.1f voltageTarget = %.1f currentTarget = %d",(float)inputCurrent/10, (float)outputVoltage/10, (float)outputCurrent/10, (float)voltageTarget/10, currentTarget);
            //v-mo-canpoll: PID = 2 inputCurrent = 0.0 outputVoltage = 16.3 outputCurrent = 0.2 voltageTarget = 0.0 currentTarget = 0
            break;
        }
        case 2:
        {
            unsigned int powerSupplyV = data[0];
            unsigned int pfcOutputV = data[1];
            unsigned int pilotDutyRatio = data[2];
            unsigned int rfcTemp = data[3];
            unsigned int inverterTemp1 = data[4];
            unsigned int inverterTemp2 = data[5];
            unsigned int insideTemp = data[6];
            StandardMetrics.ms_v_env_cabintemp->SetValue(insideTemp, Fahrenheit);
            ESP_LOGV(TAG, "PID = 2 powerSupplyV = %.2f pfcOutputV = %d pilotDutyRatio = %d %% rfcTemp = %.1f inverterTemp1 = %.1f inverterTemp2 = %.1f insideTemp = %.1f ", (float) powerSupplyV/10, pfcOutputV, pilotDutyRatio, (float) rfcTemp, (float) inverterTemp1, (float) inverterTemp2, (float) insideTemp);
            break;
        }
        case 3:
        {
            unsigned int canSupplyV = data[0];
            unsigned int motorCoolantTemp = data[1];
            ESP_LOGV(TAG, "PID = 2 canSupplyV = %.1f motorCoolantTemp = %.1f",(float) canSupplyV/10, (float) motorCoolantTemp);
            break;
        }
        default:
            break;
    }
}

void OvmsVehicleMitsubishiOutlander::OBCresponse3(uint16_t m_poll_ml_frame, uint8_t* data)
{
    switch (m_poll_ml_frame) {
        case 0:
        {
            //ESP_LOGW("766 3 0" , "Data:  %02x %02x %02x %02x LENG:%02x REM:%02x", data[0], data[1], data[2], data[3], length, mlremain);
            unsigned int DCDCoutputTargetV = data[1]*256+data[2];
            unsigned int DCDCInputV = data[3]*2;
            ESP_LOGV(TAG, "PID = 3 DCDCoutputTargetV = %.2f DCDCInputV = %d",(float) DCDCoutputTargetV / 100, DCDCInputV);
            break;
        }
        case 1:
        {
            //ESP_LOGW("766 3 1" , "Data  :%02x %02x %02x %02x %02x %02x %02x LENG:%02x REM:%02x", data[0], data[1], data[2], data[3], data[4], data[5], data[6],  length, mlremain);
            unsigned int DCDCInputA = data[0];
            unsigned int DCDCoutputS = data[1];
            StandardMetrics.ms_v_charge_12v_voltage->SetValue(DCDCoutputS/10, Volts);
            unsigned int DCDCoutputB = data[2];
            unsigned int DCDCoutputCurrent = data[3];
            StandardMetrics.ms_v_charge_12v_current->SetValue(DCDCoutputCurrent/10, Amps);
            StandardMetrics.ms_v_bat_12v_current->SetValue(DCDCoutputCurrent/10, Amps);
            StandardMetrics.ms_v_charge_12v_power->SetValue(StandardMetrics.ms_v_charge_12v_voltage->AsFloat()*StandardMetrics.ms_v_charge_12v_current->AsFloat(), Watts);
            unsigned int DCDCpowerSupplyV = data[4];
            unsigned int DCDCInverterTemp1 = data[5];
            unsigned int DCDCrectifierTemp = data[6];
            StandardMetrics.ms_v_inv_temp->SetValue(DCDCInverterTemp1, Fahrenheit);
            ESP_LOGV(TAG, "PID = 3 DCDCInputA = %d DCDCoutputS = %.1f DCDCoutputB = %.1f DCDCoutputCurrent = %d DCDCpowerSupplyV = %.1f DCDCInverterTemp1 = %.1f DCDCrectifierTemp = %.1f",DCDCInputA, (float) DCDCoutputS/10, (float) DCDCoutputB/10, DCDCoutputCurrent/10, (float) DCDCpowerSupplyV/10, (float) DCDCInverterTemp1, (float) DCDCrectifierTemp);
            break;
        }
        case 2:
        {
            //ESP_LOGW("766 3 2" , "Data:  %02x %02x %02x LENG:%02x REM:%02x", data[0], data[1], data[2], length, mlremain);
            unsigned int DCDCInverterTemp2 = data[0];
            unsigned int DCDCpowerSupplyVcan = data[1];
            unsigned int DCDCmotorTemp = data[2];
            ESP_LOGV(TAG, "PID = 3 DCDCInverterTemp2 = %.1f DCDCpowerSupplyVcan = %.1f DCDCmotorTemp = %.1f", (float) DCDCInverterTemp2, (float) DCDCpowerSupplyVcan / 10, (float) DCDCmotorTemp);
            break;
        }
        default:
            break;
    }
}
