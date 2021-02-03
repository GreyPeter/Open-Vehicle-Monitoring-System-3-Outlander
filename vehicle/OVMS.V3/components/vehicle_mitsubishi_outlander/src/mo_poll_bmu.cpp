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

void OvmsVehicleMitsubishiOutlander::IncomingBmuPoll(
        uint16_t pid, uint8_t* data, uint8_t length, uint16_t remain)
{
    switch (pid)
    {
        case 1:
            BMUresponse1(m_poll_ml_frame, data);
            break;
        case 2:
            BMUresponse2(m_poll_ml_frame, data, length, remain);
            break;
        case 3:
            BMUresponse3(m_poll_ml_frame, data, length, remain);
            break;
        default:
            break;
    }
}

void OvmsVehicleMitsubishiOutlander::BMUresponse1(uint16_t m_poll_ml_frame, uint8_t* data)
{
    switch (m_poll_ml_frame) {
        case 0:
        {
            //ESP_LOGW("762 PID 1 Line 1", "Data: %02X, %02X, %02X, %02X", data[0], data[1], data[2], data[3]);
            //real SOC Range 61 (0%) to 209 (100%) so (61-60)*2/3 = 0.66% (210-60)*2/3 = 100%
            OvmsMetricFloat* xmi_bat_soc_real = MyMetrics.InitFloat("xmi.b.soc.real", 10, 0, Percentage);
            float realSOC = data[0]-60;
            xmi_bat_soc_real->SetValue(realSOC*2/3);
            ESP_LOGI(TAG, "Real State of Charge = %.1f %%", (float)realSOC/2);
            // displayed SOC = 3D = 61
            float displayedSOC = data[1]-60;
            OvmsMetricFloat* xmi_bat_soc_display = MyMetrics.InitFloat("xmi.b.soc.display", 10, 0, Percentage);
            xmi_bat_soc_display->SetValue(displayedSOC*2/3);
            StandardMetrics.ms_v_bat_soc->SetValue(displayedSOC*2/3, Percentage);
            // Battery Cell Max Voltage 0x0EEE = 3822  0x0ED7 = 3799 Calculated in BMUresponse2()
            //float batteryCellMaxVoltage = data[2]*256+data[3];
            //StandardMetrics.ms_v_bat_pack_vmax->SetValue(batteryCellMaxVoltage/1000, Volts);
            break;
        }
        case 1:
        {
            //ESP_LOGW("762 PID 1 Line 2", "Data: %02X, %02X, %02X, %02X, %02X, %02X, %02X", data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
            // Calculated in BMUresponse2()
            //unsigned int batteryPackMinV = data[1] * 256 + data[2];
            //StandardMetrics.ms_v_bat_pack_vmin->SetValue(batteryPackMinV/1000, Volts);
            unsigned int  batteryVoltage = data[4] * 256 + data[5];
            StandardMetrics.ms_v_bat_voltage->SetValue(batteryVoltage/10, Volts);
            // Calculated in BMUresponse2()
            //StandardMetrics.ms_v_bat_pack_tmax->SetValue(data[6], Fahrenheit);
            break;
        }
        case 2:
        {
            //Calculated in BMUresponse2()
            //StandardMetrics.ms_v_bat_pack_tmin->SetValue(data[1], Fahrenheit);
            //avgModuleTemp = (StandardMetrics.ms_v_bat_pack_tmax->AsInt() + StandardMetrics.ms_v_bat_pack_tmin->AsInt()) / 2;
            //StandardMetrics.ms_v_bat_pack_tavg->SetValue(avgModuleTemp, Celcius);
            break;
        }
        case 3:
        {
            break;
        }
        case 4:
        {
            // battery "max" capacity
            StandardMetrics.ms_v_bat_cac->SetValue(((data[2] * 256.0 + data[3]) / 10.0));
            // battery remain capacity
            ms_v_bat_cac_rem->SetValue(((data[4] * 256.0 + data[5]) / 10.0));
            //max charging kW
            StandardMetrics.ms_v_charge_climit->SetValue(data[6] / 4.0, Amps);
            break;
        }
        case 5:
        {
            //max output kW
            unsigned int batteryMaxOutput = data[0];
            ms_v_bat_max_output->SetValue(batteryMaxOutput / 4.0);
            break;
        }
        case 6:
        {
            //ESP_LOGW(TAG, "Fan PWM Output = %u", data[0]);
            //ESP_LOGW(TAG, "Fan RPM = %u", data[1]*256+data[2]);
            //ESP_LOGW(TAG, "Outside Discharge time = %u",data[3]*256+data[4]);
            //ESP_LOGW(TAG, "Outsisde Discharge Integrated current = %u",data[5]*256+data[6]);
            break;
        }
        case 7:
        {
            //Calculated in BMUresponse2()
            //unsigned int averageCellVoltage = data[2]*256+data[3];
            //StandardMetrics.ms_v_bat_pack_vavg->SetValue((float)averageCellVoltage/1000);
            unsigned int maxCellVoltageDiff = data[4]*256+data[5];
            //ESP_LOGW(TAG, "Voltage Difference max = %.4f", (float)maxCellVoltageDiff/1000);
            StandardMetrics.ms_v_bat_pack_vstddev_max->SetValue((float)maxCellVoltageDiff/1000);
            //ESP_LOGV(TAG, "Internal Resistance Difference max = %d2", data[6]/10);
            break;
        }
            
        default:
            break;
    }
}

void OvmsVehicleMitsubishiOutlander::BMUresponse2(uint16_t m_poll_ml_frame, uint8_t* data, uint8_t length, uint16_t mlremain)
{
    //Process a line of data
    if(m_poll_ml_frame == 0) //First line of data
    {
        voltCell = 0;
    }
    //ESP_LOGW(TAG, "Frame:%X Data: %x %x %x %x %x %x %x Length: %u Remaining: %u", m_poll_ml_frame, data[0], data[1], data[2], data[3], data[4], data[5], data[6], length, mlremain);
    for (int i=0; i < length; i++)
    {
        {
            if (data[i] < 0xFE && voltCell < 160)
                cellVolts[voltCell++] = data[i];
        }
    }
    
    // All data processed
    if (mlremain == 0) {
        
        int cell = 0;
        double minV = 5.000;
        double maxV = 0.000;
        double voltage = 0.000;
        double totalVoltage = 0.00;
        for (int i = 0; i < 80; i++) {
            cell = i * 2;
            voltage = (double)(cellVolts[cell]*256 + cellVolts[cell+1])/1000;
            totalVoltage = totalVoltage + voltage;
            if(voltage < minV) minV = voltage;
            if(voltage > maxV) maxV = voltage;
            StandardMetrics.ms_v_bat_cell_voltage->SetElemValue(i, voltage);
            if(voltage < StandardMetrics.ms_v_bat_cell_vmin->AsFloat(i)) StandardMetrics.ms_v_bat_cell_vmin->SetElemValue(i,voltage);
            if(voltage > StandardMetrics.ms_v_bat_cell_vmax->AsFloat(i)) StandardMetrics.ms_v_bat_cell_vmax->SetElemValue(i,voltage);
            /*
            if(StandardMetrics.ms_v_bat_cell_vmin->AsFloat(i) == 0)
            {
                StandardMetrics.ms_v_bat_cell_vmin->SetElemValue(i,voltage);
            } else {
                if(StandardMetrics.ms_v_bat_cell_vmin->AsFloat(i) < voltage) StandardMetrics.ms_v_bat_cell_vmin->SetElemValue(i,voltage);
            }
            
            if(StandardMetrics.ms_v_bat_cell_vmax->AsFloat(i) == 0)
            {
                StandardMetrics.ms_v_bat_cell_vmax->SetElemValue(i,voltage);
            } else {
                if(StandardMetrics.ms_v_bat_cell_vmax->AsFloat(i) > voltage) StandardMetrics.ms_v_bat_cell_vmax->SetElemValue(i,voltage);
            }
             */
        }
        StandardMetrics.ms_v_bat_pack_vmin->SetValue(minV);
        StandardMetrics.ms_v_bat_pack_vmax->SetValue(maxV);
        StandardMetrics.ms_v_bat_pack_vavg->SetValue((StandardMetrics.ms_v_bat_pack_vmin->AsFloat()+StandardMetrics.ms_v_bat_pack_vmax->AsFloat())/2);
        //ESP_LOGW(TAG, "Total Voltage = %.2f", totalVoltage);
        voltCell = 0;
    }
}
void OvmsVehicleMitsubishiOutlander::BMUresponse3(uint16_t m_poll_ml_frame, uint8_t* data, uint8_t length, uint16_t mlremain)
{
    //Process a line of data
    if(m_poll_ml_frame == 0) //First line of data
    {
        tempCell = 0;
    }
    for (int i=0; i < length; i++)
    {
        {
            if (data[i] < 0xFE && tempCell < 40)
                cellTemps[tempCell++] = data[i];
        }
    }
    
    // All data processed
    if (mlremain == 0) {
        float temperature;
        int minT = 50;
        int maxT = 0;
        for (int i = 0; i < 40; i++) {
            temperature = UnitConvert(Fahrenheit, Celcius, (float) cellTemps[i]);
            if(temperature < minT) minT = temperature;
            if(temperature > maxT) maxT = temperature;
            StandardMetrics.ms_v_bat_cell_temp->SetElemValue(i,temperature);
            if(temperature < StandardMetrics.ms_v_bat_cell_tmin->AsFloat(i)) StandardMetrics.ms_v_bat_cell_tmin->SetElemValue(i,temperature);
            if(temperature > StandardMetrics.ms_v_bat_cell_tmax->AsFloat(i)) StandardMetrics.ms_v_bat_cell_tmax->SetElemValue(i,temperature);
            /*
            if(StandardMetrics.ms_v_bat_cell_tmin->AsFloat(i) == 0)
            {
                StandardMetrics.ms_v_bat_cell_tmin->SetElemValue(i,temperature);
            } else {
                if(StandardMetrics.ms_v_bat_cell_tmin->AsFloat(i) < temperature) StandardMetrics.ms_v_bat_cell_tmin->SetElemValue(i,temperature);
            }
            
            if(StandardMetrics.ms_v_bat_cell_tmax->AsFloat(i) == 0)
            {
                StandardMetrics.ms_v_bat_cell_tmax->SetElemValue(i,temperature);
            } else {
                if(StandardMetrics.ms_v_bat_cell_tmax->AsFloat(i) > temperature) StandardMetrics.ms_v_bat_cell_tmax->SetElemValue(i,temperature);
            }
             */
        }
        StandardMetrics.ms_v_bat_pack_tmin->SetValue(minT, Celcius);
        StandardMetrics.ms_v_bat_pack_tmax->SetValue(maxT, Celcius);
        StandardMetrics.ms_v_bat_pack_tavg->SetValue((StandardMetrics.ms_v_bat_pack_tmin->AsFloat()+StandardMetrics.ms_v_bat_pack_tmax->AsFloat())/2);
        StandardMetrics.ms_v_bat_temp->SetValue(maxT);
        tempCell = 0;
    }
}
