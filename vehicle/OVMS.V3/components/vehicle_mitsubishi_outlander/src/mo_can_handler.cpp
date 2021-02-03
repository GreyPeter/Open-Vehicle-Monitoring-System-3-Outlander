/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          11th September 2020
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

#include "ovms_log.h"
#include "vehicle_mitsubishi_outlander.h"
#include "mo_obd_pids.h"
static const char *TAG = "v-mo-can";

void OvmsVehicleMitsubishiOutlander::IncomingFrameCan1(CAN_frame_t* p_frame)
{
    uint8_t *d = p_frame->data.u8;
    switch (p_frame->MsgID) {
        case 0x101: //freq10 //Key status 00=OFF 04=ON 44=?? 84=??
            if (d[0] && 4) {  // Car is on
                if(!StdMetrics.ms_v_env_on->AsBool()){ // Just been turned on
                    ESP_LOGI(TAG,"Car is on");
                    //POLLSTATE_RUNNING;
                    StdMetrics.ms_v_env_charging12v->SetValue( true );
                    StdMetrics.ms_v_env_awake->SetValue(true);
                    // Reset trip values
                    StandardMetrics.ms_v_bat_energy_recd->SetValue(0);
                    StandardMetrics.ms_v_bat_energy_used->SetValue(0);
                    mo_cum_energy_recd_wh = 0.0f;
                    mo_cum_energy_used_wh = 0.0f;
                }
                StdMetrics.ms_v_env_on->SetValue(true);
            } else {
                ESP_LOGV(TAG,"Car is off");
                StdMetrics.ms_v_env_awake->SetValue(false);
                StdMetrics.ms_v_env_on->SetValue(false);
                StdMetrics.ms_v_pos_speed->SetValue( 0 );
                StdMetrics.ms_v_env_charging12v->SetValue( false );
                if (StandardMetrics.ms_v_charge_state->AsString()  != "charging")
                  {
                  //POLLSTATE_OFF;
                  }
            }
            break;
        case 0x154: // Fuel and odometer
        {
            float realFuel = (unsigned int)d[0]*100/255;
            float dashFuel = (unsigned int)d[1]*100/255;
            float lowFuel = (unsigned int)d[2]*100/255;
            float fuelStart = (unsigned int)d[3]*100/255;
            ESP_LOGV(TAG, "Real Fuel=%.2f Dash Fuel=%.2f Low Fuel=%.2f Start Fuel=%.2f", realFuel, dashFuel, lowFuel, fuelStart);
            
            unsigned int odometer = (unsigned int)d[7]
            + ((unsigned int)d[6]<<8)
            + ((unsigned int)d[5]<<16);
            StandardMetrics.ms_v_pos_odometer->SetValue((float)odometer/10, Kilometers);
            break;
        }
        case 0x210: // Throttle Pedal ????
        {
            ESP_LOGV(TAG, "Pedal = %u", d[2]*100/255);
            unsigned int throttle = d[2]*100;
            StandardMetrics.ms_v_env_throttle->SetValue((float)throttle/255);
            break;
        }
        case 0x328: // Speed
            ESP_LOGV(TAG, "Speed %u kph", d[0]*256+d[1]);
            StandardMetrics.ms_v_pos_speed->SetValue(d[0]*256+d[1]);
            //ESP_LOGW(TAG, "Power %u Wh/km", d[0]*256+d[1]);
            break;
        case 0x345: // Range
        {
            ESP_LOGV(TAG, "Full Range = %u KM", (d[6] * 256 + d[7]));
            //StandardMetrics.ms_v_bat_range_ideal->SetValue(d[6] * 256 + d[7], Kilometers);
            StandardMetrics.ms_v_bat_range_est->SetValue((signed int)d[5], Kilometers);
            ESP_LOGV(TAG, "EV Range = %u KM", d[5]);
            break;
        }
            case 0x387://freq100 // Main Battery volt and current
        {
            StandardMetrics.ms_v_bat_current->SetValue(((d[2] * 256.0 + d[3]) - 32700) / 100.0, Amps);
            StandardMetrics.ms_v_bat_voltage->SetValue((d[4] * 256.0 + d[5]) / 10.0, Volts);
            StandardMetrics.ms_v_bat_power->SetValue((StandardMetrics.ms_v_bat_voltage->AsFloat(0, Volts) * StandardMetrics.ms_v_bat_current->AsFloat(0, Amps)) / 1000.0 * -1.0, kW);
        }
            case 0x418://freq50 // Transmissin state determination
        {
            switch (d[0]) {
                case 80: //P
                    StandardMetrics.ms_v_env_gear->SetValue(-1);
                    break;
                case 82: //R
                    StandardMetrics.ms_v_env_gear->SetValue(-2);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                case 78: //N
                    StandardMetrics.ms_v_env_gear->SetValue(0);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                case 68: //D
                    StandardMetrics.ms_v_env_gear->SetValue(1);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                case 131: //B
                    StandardMetrics.ms_v_env_gear->SetValue(2);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                case 50: //C
                    StandardMetrics.ms_v_env_gear->SetValue(3);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                    
                default:
                    break;
            }
        }
        case 0x425: // Wipers & Doors
        {
            switch (d[0]) {
                case 0x00:
                    //ESP_LOGI(TAG, "Wipers off");
                    break;
                case 0x04:
                    //ESP_LOGI(TAG, "Wipers Auto");
                    break;
                case 0x02:
                    ESP_LOGI(TAG, "Wipers Slow");
                    break;
                case 0x0a:
                    ESP_LOGI(TAG, "Wipers Fast");
                    break;
                case 0x03:
                    ESP_LOGI(TAG, "Wipers 03");
                    break;
                case 0x0b:
                    ESP_LOGI(TAG, "Wipers 0b");
                    break;
                default:
                    break;
            }
            switch (d[1]) {
                case 0x00:
                    StandardMetrics.ms_v_door_fr->SetValue(false);
                    StandardMetrics.ms_v_door_rr->SetValue(false);
                    StandardMetrics.ms_v_door_rl->SetValue(false);
                    StandardMetrics.ms_v_door_fl->SetValue(false);
                    StandardMetrics.ms_v_door_trunk->SetValue(false);
                    break;
                case 0x03:
                    //ESP_LOGI(TAG, "Drivers door open");
                    StandardMetrics.ms_v_door_fr->SetValue(true);
                    break;
                case 0x09:
                    //ESP_LOGI(TAG, "Drivers back door open");
                    StandardMetrics.ms_v_door_rr->SetValue(true);
                    break;
                case 0x21:
                    //ESP_LOGI(TAG, "Back door open");
                    StandardMetrics.ms_v_door_trunk->SetValue(true);
                    break;
                case 0x11:
                    //ESP_LOGI(TAG, "Passanger back door open");
                    StandardMetrics.ms_v_door_rl->SetValue(true);
                    break;
                case 0x05:
                    //ESP_LOGI(TAG, "Passanger front door open");
                    StandardMetrics.ms_v_door_fl->SetValue(true);
                    break;
                default:
                    break;
            }
            switch (d[2]) {
                case 0x0a: //UNLOCKED
                    //ESP_LOGW(TAG, "UNLOCKED");
                    StandardMetrics.ms_v_env_locked->SetValue(false);
                    break;
                case 0x05: //Locked
                    //ESP_LOGI(TAG, "LOCKED");
                    StandardMetrics.ms_v_env_locked->SetValue(true);
                    break;
                default:
                    break;
            }
            break;
        }
        case 0x608: // Temperatures
            StandardMetrics.ms_v_mot_temp->SetValue(d[0], Fahrenheit);
            StandardMetrics.ms_v_env_temp->SetValue(d[1], Fahrenheit);
            break;
            
        case 0x6FA://freq10 // VIN determination //6FA VIN2
        {
            ESP_LOGV(TAG, "VIN being processed");
            
            if (d[0]==0)
                for (int k = 0; k < 7; k++) m_vin[k] = d[k+1];
            else if (d[0] == 1)
                for (int k = 0; k < 7; k++) m_vin[k+7] = d[k+1];
            else if ( d[0] == 2 )
            {
                m_vin[14] = d[1];
                m_vin[15] = d[2];
                m_vin[16] = d[3];
                m_vin[17] = 0;
                if ( ( m_vin[0] !=0) && ( m_vin[7] !=0) )
                {
                    StandardMetrics.ms_v_vin->SetValue(m_vin);
                }
            }
            break;
        }
            
        default:
            break;
    }
}
