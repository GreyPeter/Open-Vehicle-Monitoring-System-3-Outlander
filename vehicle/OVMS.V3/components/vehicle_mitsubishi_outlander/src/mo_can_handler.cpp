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
    
    switch ( p_frame->MsgID )
    {
        case 0x101: //freq10 //Key status 00=OFF 04=ON 44=?? 84=??
        {
            if (d[0] && 4) {  // Car is on
                vehicle_mitsubishi_car_on(true);
                StdMetrics.ms_v_env_awake->SetValue(true);
                StdMetrics.ms_v_env_on->SetValue(true);
            } else {
                vehicle_mitsubishi_car_on(false);
                StdMetrics.ms_v_env_awake->SetValue(false);
                StdMetrics.ms_v_env_on->SetValue(false);
            }
            break;
        }
            
            //   7d 00 00 00 00 7D 1F FF
            //   7d 00 00 04 08 7D 11 17
            //   7d 00 00 1C 08 7D 11 17
            
            //   78 06 00 04 08 7d 11 17
            //   78 06 00 1c 08 7d 11 17
        case 0x143: //
        {
            //ESP_LOGW(TAG, "0x143");
            break;
        }
            
            // 00 00 00 00 00 00 00 00
            // 04 00 00 00 00 00 00 00
        case 0x144: // ????
        {
            //ESP_LOGW(TAG, "0x144");
            break;
        }
            
            //      4E 20 9F FF DF FF 1F FF
            //      4E 20 AF FF 8F FF 0F FF
            //      4E 20 6F FF 8F FF 0F FF
            
            //  4e 20 af ff 8f ff 0f ff
            //  4e 20 6f ff 8f ff 0f ff
        case 0x145: // ????
        {
            //ESP_LOGW(TAG, "0x145");
            break;
        }
            
            //  82 ff ff ff fd 02 00 00
        case 0x151: // ????
        {
            //ESP_LOGW(TAG, "CASE %4x Data %02x %02x %02x %02x %02x %02x %02x %02x", p_frame->MsgID, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
            break;
        }
            
            // 5a 5a 60 5a 0a 09 30 ac
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
            
            // 00 00 22 00 00 00 00 00
        case 0x185: // ????
        {
            //ESP_LOGW(TAG, "0x185");
            break;
        }
            
        case 0x210: // Throttle Pedal ????
        {
            //ESP_LOGW(TAG, "Pedal = %u", d[2]*100/255);
            unsigned int throttle = d[2]*100;
            StandardMetrics.ms_v_env_throttle->SetValue((float)throttle/255);
            break;
        }
            /*
             case 0x214: // Speed ????
             {
             //ESP_LOGW(TAG, "Speed1 = %u", d[0]*256+d[1]/2);
             //ESP_LOGW(TAG, "Wheel1 = %u", d[2]*256+d[3]/2);
             //ESP_LOGW(TAG, "Wheel2 = %u", d[4]*256+d[5]/2);
             break;
             }
             
             case 0x215: // Speed ????
             {
             //ESP_LOGW(TAG, "Speed2 = %u", d[0]*256+d[1]/2);
             //ESP_LOGW(TAG, "Wheel3 = %u", d[2]*256+d[3]/2);
             //ESP_LOGW(TAG, "Wheel4 = %u", d[4]*256+d[5]/2);
             break;
             }
             
             case 0x32D: // ?????
             {
             
             break;
             }
             */
        case 0x345: // Range
        {
            //ESP_LOGW(TAG, "Range = %u KM", (d[6] * 256 + d[7]));
            StandardMetrics.ms_v_bat_range_est->SetValue((signed int)d[5], Kilometers);
            
            unsigned int speed = (unsigned int)d[7] + ((unsigned int)d[6]<<8);
            StandardMetrics.ms_v_charge_limit_range->SetValue(speed , Kilometers);
            break;
        }
            /*
             //  27 10 52 20 20 00 08 64
             case 0x346: // EV Power
             {
             //ESP_LOGW(TAG, "EV Power = %u %%", ((d[0] * 256 + d[1]) - 10000) / 100);
             unsigned int evPower = (d[0] * 256 + d[1]) - 10000;
             StandardMetrics.ms_v_bat_power->SetValue((float) evPower / 100, kW);
             break;
             }
             
             case 0x347: // ?????
             {
             
             break;
             }
             case 0x348: // ?????
             {
             
             break;
             }
             */
            //case 0x353: //Charger Temp
            //{
            //ESP_LOGW(TAG, "0x353 Charger Temp");
            //    break;
            //}
            
            /*
             // 00    0    7F    80    01    FF    00    0
             // 00    0    7F    80    00    00    2E    0
             // 30    0    7F    80    C0    00    2E    0
             // 00    0    7F    80    40    00    2E    0
             // 00    3    7F    80    00    00    2E    0
             // 20    3    7F    80    00    00    2E    0
             case 0x359: // ?????
             {
             
             break;
             }
             case 0x35A: // ?????
             {
             
             break;
             }
             */
            
        case 0x373:
        case 0x374:
        case 0x375:
        {
            ESP_LOGW(TAG, "CASE %4x Data %02x %02x %02x %02x %02x %02x %02x %02x", p_frame->MsgID, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
            break;
        }
            
            //01 fe 7f 88 0c b0 50 4c
        case 0x387://freq100 // Main Battery volt and current
        {  // 1kwh -» 3600000Ws -- freq100 -» 360000000   360000W per 100mS
            
            //StandardMetrics.ms_v_bat_current->SetValue((((((d[2] * 256.0) + d[3])) - 32768)) / 100.0, Amps);
            StandardMetrics.ms_v_bat_current->SetValue(((d[2] * 256.0 + d[3]) - 32700) / 100.0, Amps);
            StandardMetrics.ms_v_bat_voltage->SetValue((d[4] * 256.0 + d[5]) / 10.0, Volts);
            StandardMetrics.ms_v_bat_power->SetValue((StandardMetrics.ms_v_bat_voltage->AsFloat(0, Volts) * StandardMetrics.ms_v_bat_current->AsFloat(0, Amps)) / 1000.0 * -1.0, kW);
            v_c_power_dc->SetValue( StandardMetrics.ms_v_bat_power->AsFloat() * -1.0, kW );
            
            // Check Charge State
            if (StandardMetrics.ms_v_bat_current->AsFloat() > 1) {
                if (StdMetrics.ms_v_env_on->AsBool()) {
                    //ESP_LOGW(TAG, "Car is regenerating power");
                    StandardMetrics.ms_v_charge_voltage->SetValue(StandardMetrics.ms_v_bat_voltage->AsFloat(),Volts);
                    StandardMetrics.ms_v_charge_current->SetValue(StandardMetrics.ms_v_bat_current->AsFloat(), Amps);
                    StandardMetrics.ms_v_charge_power->SetValue((StandardMetrics.ms_v_charge_voltage->AsFloat() * StandardMetrics.ms_v_charge_current->AsFloat()), Watts);
                    StandardMetrics.ms_v_charge_kwh->SetValue(StandardMetrics.ms_v_charge_power->AsFloat(), kWh);
                    ESP_LOGV(TAG, "Car is regenerating power. Regen = %.2f", StandardMetrics.ms_v_charge_kwh->AsFloat());
                    mi_SC = false;
                    
                } else {
                    
                    StandardMetrics.ms_v_charge_voltage->SetValue(240, Volts);
                    StandardMetrics.ms_v_charge_current->SetValue(StandardMetrics.ms_v_bat_power->AsFloat()*1000 / 240 * -1.0, Amps);
                    StandardMetrics.ms_v_charge_power->SetValue((StandardMetrics.ms_v_charge_voltage->AsFloat() * StandardMetrics.ms_v_charge_current->AsFloat()) / 1000, kW);
                    StandardMetrics.ms_v_charge_kwh->SetValue(StandardMetrics.ms_v_charge_power->AsFloat(), kWh);
                    v_c_power_ac->SetValue((StandardMetrics.ms_v_charge_voltage->AsFloat() * StandardMetrics.ms_v_charge_current->AsFloat()) / 1000, kW);
                    ESP_LOGV(TAG, "Car is Charging. Charge = %.2f", StandardMetrics.ms_v_charge_kwh->AsFloat());
                    mi_SC = true;
                    
                }
            } else {
                //ESP_LOGW(TAG, "Car is ON");
                StandardMetrics.ms_v_charge_voltage->SetValue(0);
                StandardMetrics.ms_v_charge_current->SetValue(0);
                StandardMetrics.ms_v_charge_power->SetValue(0);
                StandardMetrics.ms_v_charge_kwh->SetValue(0);
                mi_SC = false;
            }
            
            if (!StandardMetrics.ms_v_charge_pilot->AsBool())
            {
                if ( StandardMetrics.ms_v_bat_power->AsInt() < 0)
                {
                    StandardMetrics.ms_v_bat_energy_recd->SetValue((StandardMetrics.ms_v_bat_energy_recd->AsFloat()
                                                                    + (StandardMetrics.ms_v_bat_power->AsFloat() / -360000.0)));
                    //ESP_LOGW(TAG, "Energy Received = %f", StandardMetrics.ms_v_bat_energy_recd->AsFloat());
                    ms_v_trip_park_energy_recd->SetValue((ms_v_trip_park_energy_recd->AsFloat() + (StandardMetrics.ms_v_bat_power->AsFloat() / -360000.0)));
                    ms_v_trip_charge_energy_recd->SetValue((ms_v_trip_charge_energy_recd->AsFloat() + (StandardMetrics.ms_v_bat_power->AsFloat() / -360000.0)));
                }
                else
                {
                    StandardMetrics.ms_v_bat_energy_used->SetValue( ( StandardMetrics.ms_v_bat_energy_used->AsFloat()
                                                                     +(StandardMetrics.ms_v_bat_power->AsFloat() / 360000.0)));
                    //ESP_LOGW(TAG, "Energy Used = %f", StandardMetrics.ms_v_bat_energy_used->AsFloat());
                    ms_v_trip_park_energy_used->SetValue((ms_v_trip_park_energy_used->AsFloat() + (StandardMetrics.ms_v_bat_power->AsFloat() / 360000.0)));
                    ms_v_trip_charge_energy_used->SetValue((ms_v_trip_charge_energy_used->AsFloat() + (StandardMetrics.ms_v_bat_power->AsFloat() / 360000.0)));
                }
            }
            else
            {
                // energy usage
                if ( StandardMetrics.ms_v_bat_power->AsInt() < 0)
                {
                    StandardMetrics.ms_v_bat_energy_recd->SetValue((StandardMetrics.ms_v_bat_energy_recd->AsFloat() + (StandardMetrics.ms_v_bat_power->AsFloat() / -360000.0)));
                    //ESP_LOGW(TAG, "Charge Energy Received = %f", StandardMetrics.ms_v_bat_energy_recd->AsFloat());
                }
                else
                {
                    StandardMetrics.ms_v_bat_energy_used->SetValue((StandardMetrics.ms_v_bat_energy_used->AsFloat() + (StandardMetrics.ms_v_bat_power->AsFloat() / 360000.0)));
                    //ESP_LOGW(TAG, "Charge Energy Used = %f", StandardMetrics.ms_v_bat_energy_used->AsFloat());
                    
                }
            }
            //StandardMetrics.ms_v_charge_kwh->SetValue(-StandardMetrics.ms_v_bat_power->AsFloat());
            
            if (StandardMetrics.ms_v_charge_inprogress->AsBool())
            {
                ms_v_charge_ac_kwh->SetValue((ms_v_charge_ac_kwh->AsFloat() + (StandardMetrics.ms_v_bat_power->AsFloat() / -360000.0)));
                //ESP_LOGW(TAG,"Charge in progress. Charge DC = %.2f", ms_v_charge_ac_kwh->AsFloat());
            }
            /*
             if (mi_SC == true)
             {
             //set battery voltage/current to charge voltage/current, when car in Park, and charging
             StandardMetrics.ms_v_charge_voltage->SetValue(StandardMetrics.ms_v_bat_voltage->AsFloat());
             StandardMetrics.ms_v_charge_current->SetValue(StandardMetrics.ms_v_bat_current->AsFloat() * 1.0);
             StandardMetrics.ms_v_charge_kwh->SetValue(ms_v_charge_dc_kwh->AsFloat());
             }
             */
            //min power
            if (v_b_power_max->AsFloat() > StandardMetrics.ms_v_bat_power->AsFloat())
                v_b_power_max->SetValue(StandardMetrics.ms_v_bat_power->AsFloat());
            //max power
            if (v_b_power_min->AsFloat() < StandardMetrics.ms_v_bat_power->AsFloat())
                v_b_power_min->SetValue(StandardMetrics.ms_v_bat_power->AsFloat());
            
            break;
        }
            /*
             //7C 00 00 00 00 00 00 00
             //01 00 00 00 00 00 00 02
             case 0x388: // ?????
             {
             
             break;
             }
             
             // 24    21    08    0    0    0    0    0
             // 11    21    08    0    0    0    0    0
             // 11    21    00    0    0    0    0    0
             // 11    61    20    0    0    0    0    0
             // 66    64    20    0    0    0    0    0
             // 66    64    22    0    0    0    0    0
             // 66    64    26    0    0    0    0    0
             
             //66 04 20 00 00 00 00 00
             case 0x38C: // ?????
             {
             
             break;
             }
             */
        case 0x418://freq50 // Transmissin state determination
        {
            
            switch (d[0]){
                case 80: //P
                {
                    StandardMetrics.ms_v_env_gear->SetValue(-1);
                    break;
                }
                case 82: //R
                {
                    StandardMetrics.ms_v_env_gear->SetValue(-2);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                }
                case 78: //N
                {
                    StandardMetrics.ms_v_env_gear->SetValue(0);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                }
                case 68: //D
                {
                    StandardMetrics.ms_v_env_gear->SetValue(1);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                }
                case 131: //B
                {
                    StandardMetrics.ms_v_env_gear->SetValue(2);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                }
                case 50: //C
                {
                    StandardMetrics.ms_v_env_gear->SetValue(3);
                    StandardMetrics.ms_v_env_parktime->SetValue(0);
                    break;
                }
            }
            break;
        }
            
        case 0x425: // Doors
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
        {
            int coolant = d[0];
            StandardMetrics.ms_v_mot_temp->SetValue(coolant, Fahrenheit);
            int ambiant = d[1];
            StandardMetrics.ms_v_env_temp->SetValue(ambiant, Fahrenheit);
            //ESP_LOGW(TAG, "0x608 TEMPERATURES Coolant %.1f C Ambiant %.1f", (float)coolant/2, (float)ambiant/2);
            break;
        }
            
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
        case 0x6fd:
        {
            //ESP_LOGW(TAG, "0x6fd");
            break;
        }
            
        default:
            break;
    }
}
