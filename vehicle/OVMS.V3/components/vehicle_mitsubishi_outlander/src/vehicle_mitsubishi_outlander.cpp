/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          14th March 2017
 ;
 ;    Changes:
 ;    1.0  Initial release
 ;
 ;    (C) 2011       Michael Stegen / Stegen Electronics
 ;    (C) 2011-2017  Mark Webb-Johnson
 ;    (C) 2011        Sonny Chen @ EPRO/DX
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

#include <stdio.h>
#include "mo_obd_pids.h"
#include "vehicle_mitsubishi_outlander.h"
#define VERSION "1.0.0"

static const char *TAG = "v-mo";

static const OvmsVehicle::poll_pid_t obdii_polls[] =
{
    { bmuTxId, bmuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 10, 10 }, 0, ISOTP_STD }, // cac
    { bmuTxId, bmuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x02,{ 0, 10, 10 }, 0, ISOTP_STD }, // cac
    { bmuTxId, bmuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x03,{ 0, 10, 10 }, 0, ISOTP_STD }, // cac
    
    { obcTxId, obcRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 10,  0 }, 0, ISOTP_STD }, // OBC
    { obcTxId, obcRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x02,{ 0, 10,  0 }, 0, ISOTP_STD }, // OBC
    { obcTxId, obcRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x03,{ 0, 10,  0 }, 0, ISOTP_STD }, // OBC
    
    { fmcuTxId, fmcuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 10, 10 }, 0, ISOTP_STD }, //Front Motor Control Unit FMCU
    { rmcuTxId, rmcuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 10, 10 }, 0, ISOTP_STD }, //Rear Motor Control Unit RMCU
    POLL_LIST_END
};

OvmsVehicleMitsubishiOutlander::OvmsVehicleMitsubishiOutlander()
{
    ESP_LOGI(TAG, "Start Mitsubishi Outlander vehicle module");
    
    StandardMetrics.ms_v_env_parktime->SetValue(0);
    StandardMetrics.ms_v_charge_type->SetValue("Standard");
    StandardMetrics.ms_v_charge_mode->SetValue("Type1");
    StandardMetrics.ms_v_charge_climit->SetValue("7");
    StandardMetrics.ms_v_bat_energy_used->SetValue(0);
    StandardMetrics.ms_v_bat_energy_recd->SetValue(0);
    StandardMetrics.ms_v_env_headlights->SetValue(false);
    StandardMetrics.ms_v_charge_duration_full->SetValue(0);
    StandardMetrics.ms_v_charge_kwh->SetValue(0);
    memset(m_vin,0,sizeof(m_vin));
    mi_start_time_utc = StandardMetrics.ms_m_timeutc->AsInt();
    StandardMetrics.ms_v_charge_inprogress->SetAutoStale(30);    //Set autostale to 30 second
    
    ms_v_charge_ac_kwh->SetValue(0);
    ms_v_charge_dc_kwh->SetValue(0);
    
    ms_v_trip_park_energy_recd->SetValue(0);
    ms_v_trip_park_energy_used->SetValue(0);
    ms_v_trip_park_heating_kwh->SetValue(0);
    ms_v_trip_park_ac_kwh->SetValue(0);
    
    
    RegisterCanBus(1,CAN_MODE_ACTIVE,CAN_SPEED_500KBPS);
    PollSetPidList(m_can1,obdii_polls);
    PollSetState(1);
    mi_SC = false;
    //mi_QC = false;
    cfg_newcell = false;
#ifdef CONFIG_OVMS_COMP_WEBSERVER
    //MyWebServer.RegisterPage("/bms/cellmon", "BMS cell monitor", OvmsWebServer::HandleBmsCellMonitor, PageMenu_Vehicle, PageAuth_Cookie);
    //MyWebServer.RegisterPage("/cfg/brakelight", "Brake Light control", OvmsWebServer::HandleCfgBrakelight, PageMenu_Vehicle, PageAuth_Cookie);
    WebInit();
#endif
}

OvmsVehicleMitsubishiOutlander::~OvmsVehicleMitsubishiOutlander()
{
    ESP_LOGI(TAG, "Shutdown Mitsubishi Outlander vehicle module");
}

void OvmsVehicleMitsubishiOutlander::vehicle_mitsubishi_car_on(bool isOn)
{
    StdMetrics.ms_v_env_awake->SetValue(isOn);
    
    if (isOn && !StdMetrics.ms_v_env_on->AsBool())
    {
        // Car is ON
        StdMetrics.ms_v_env_on->SetValue(isOn);
        //Reset trip variables so that they are updated as soon as they are available
        //mi_trip_start_odo = 0;
        StdMetrics.ms_v_env_charging12v->SetValue(true);
        //Reset energy calculation
        StdMetrics.ms_v_bat_energy_recd->SetValue(0);
        StdMetrics.ms_v_bat_energy_used->SetValue(0);
        ms_v_trip_park_energy_used->SetValue(0);
        ms_v_trip_park_energy_recd->SetValue(0);
        ms_v_trip_park_heating_kwh->SetValue(0);
        ms_v_trip_park_ac_kwh->SetValue(0);
        ms_v_trip_park_time_start->SetValue(StdMetrics.ms_m_timeutc->AsInt());
        if(has_odo == true && StandardMetrics.ms_v_bat_soc->AsFloat() > 0.0)
        {
            mi_park_trip_counter.Reset(ms_v_trip_B->AsFloat());
            ms_v_trip_park_soc_start->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
        }
        
        BmsResetCellStats();
        PollSetState(1);
    }
    else if ( !isOn && StdMetrics.ms_v_env_on->AsBool() )
    {
        // Car is OFF
        StdMetrics.ms_v_env_on->SetValue( isOn );
        StdMetrics.ms_v_pos_speed->SetValue(0);
        mi_park_trip_counter.Update(ms_v_trip_B->AsFloat());
        StdMetrics.ms_v_pos_trip->SetValue(mi_park_trip_counter.GetDistance());
        ms_v_pos_trip_park->SetValue(mi_park_trip_counter.GetDistance());
        
        StdMetrics.ms_v_env_charging12v->SetValue(false);
        ms_v_trip_park_soc_stop->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
        ms_v_trip_park_time_stop->SetValue(StdMetrics.ms_m_timeutc->AsInt());
        PollSetState(0);
    }
    
}

void OvmsVehicleMitsubishiOutlander::Ticker1(uint32_t ticker)
{
    // battery temp from battery pack avg
    StdMetrics.ms_v_bat_temp->SetValue(StdMetrics.ms_v_bat_pack_tavg->AsFloat());
    
    //Check only if 'transmission in park
    if (StandardMetrics.ms_v_env_gear->AsInt() == -1)
    {
        ////////////////////////////////////////////////////////////////////////
        // Charge state determination
        ////////////////////////////////////////////////////////////////////////
        
        if (mi_SC == true)
        {
            PollSetState(2);
            StandardMetrics.ms_v_env_charging12v->SetValue(true);
            if (! StandardMetrics.ms_v_charge_pilot->AsBool())
            {
                // Charge has just started
                ms_v_charge_ac_kwh->SetValue(0);
                ms_v_charge_dc_kwh->SetValue(0);
                //StandardMetrics.ms_v_charge_kwh->SetValue(0.0,kWh);
                mi_chargekwh = 0;      // Reset charge kWh
                v_c_time->SetValue(0); //Reser charge timer
                StandardMetrics.ms_v_charge_inprogress->SetValue(true);
                StandardMetrics.ms_v_charge_pilot->SetValue(true);
                StandardMetrics.ms_v_door_chargeport->SetValue(true);
                StandardMetrics.ms_v_charge_substate->SetValue("onrequest");
                ms_v_trip_charge_soc_stop->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
                v_c_soc_start->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
                StandardMetrics.ms_v_charge_duration_full->SetValue(0);
                BmsResetCellStats();
            }
            else //ms_v_charge_pilot = true and mi_SC = true *** CHARGING ****
            {
                if (StandardMetrics.ms_v_bat_current->AsFloat() > 7)
                {
                    StandardMetrics.ms_v_charge_climit->SetValue("16");
                } else {
                    StandardMetrics.ms_v_charge_climit->SetValue("7");
                }
                StandardMetrics.ms_v_charge_mode->SetValue("Type1");
                StandardMetrics.ms_v_charge_state->SetValue("charging");
                v_c_time->SetValue(StandardMetrics.ms_v_charge_time->AsInt());
                v_c_soc_stop->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
                
                // always calculate remaining charge time to full
                float full_soc           = 100.0;     // 100%
                int   minsremaining_full = calcMinutesRemaining(full_soc, StandardMetrics.ms_v_charge_power->AsFloat()*1000);
                StandardMetrics.ms_v_charge_duration_full->SetValue(minsremaining_full, Minutes);
                ESP_LOGV(TAG, "Time remaining: %d mins to full", minsremaining_full);
                
                // if charge and DC current is negative cell balancing is active. If charging battery current is negative, discharge is positive
                if ((StandardMetrics.ms_v_bat_current->AsFloat() < 0) && StandardMetrics.ms_v_bat_soc->AsInt() > 92 && StandardMetrics.ms_v_charge_mode->AsString() != "Balancing")
                {
                    StandardMetrics.ms_v_charge_mode->SetValue("Balancing");
                    StandardMetrics.ms_v_charge_state->SetValue("topoff");
                }
            }
        }
        else if (StandardMetrics.ms_v_bat_current->AsFloat() < 0)
            //((StandardMetrics.ms_v_charge_current->AsInt() == 0) && (StandardMetrics.ms_v_charge_voltage->AsInt() < 100))
        {
            // Car is not charging
            if (StandardMetrics.ms_v_charge_pilot->AsBool())
            {
                // Charge has stopped
                StandardMetrics.ms_v_charge_inprogress->SetValue(false);
                StandardMetrics.ms_v_charge_pilot->SetValue(false);
                StandardMetrics.ms_v_door_chargeport->SetValue(false);
                StandardMetrics.ms_v_env_charging12v->SetValue(false);
                StandardMetrics.ms_v_charge_type->SetValue("None");
                StandardMetrics.ms_v_charge_duration_full->SetValue(0);
                if (StandardMetrics.ms_v_bat_soc->AsInt() < 92)
                {
                    // Assume charge was interrupted
                    StandardMetrics.ms_v_charge_state->SetValue("stopped");
                    StandardMetrics.ms_v_charge_substate->SetValue("interrupted");
                }
                else
                {
                    // Charge done
                    StandardMetrics.ms_v_charge_state->SetValue("done");
                    StandardMetrics.ms_v_charge_substate->SetValue("scheduledstop");
                }
                
                v_c_power_ac->SetValue(0.0);  // Reset charge power meter
                //v_c_power_dc->SetValue(0.0);  // Reset charge power meter
                StandardMetrics.ms_v_charge_current->SetValue(0.0); //Reset charge current
                StandardMetrics.ms_v_charge_climit->SetValue(0.0); //Reset charge limit
                StandardMetrics.ms_v_charge_mode->SetValue("None"); //Set charge mode to NONE
                StandardMetrics.ms_v_bat_current->SetValue(0.0);  //Set battery current to 0
                ms_v_charge_ac_kwh->SetValue(StandardMetrics.ms_v_charge_kwh->AsFloat()); //save charge kwh to variable
                //StandardMetrics.ms_v_charge_kwh->SetValue(0); //reset charge kwh variable
                
                // Reset trip counter for this charge
                
                //mi_charge_trip_counter.Reset(POS_ODO);
                ms_v_trip_charge_energy_recd->SetValue(0);
                ms_v_trip_charge_energy_used->SetValue(0);
                ms_v_trip_charge_heating_kwh->SetValue(0);
                ms_v_trip_charge_ac_kwh->SetValue(0);
                ms_v_trip_charge_soc_start->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
                ms_v_trip_charge_soc_stop->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
                v_c_soc_stop->SetValue(StandardMetrics.ms_v_bat_soc->AsFloat());
                
            }
        }
        
        //Efficiency calculation AC/DC
        
        if ((v_c_power_dc->AsInt() <= 0) || (v_c_power_ac->AsInt() <= 0))
        {
            v_c_efficiency->SetValue(0,Percentage);
        }
        else
        {
            v_c_efficiency->SetValue((v_c_power_dc->AsFloat() / v_c_power_ac->AsFloat()) * 100, Percentage);
        }
    } //Car in P "if" close
    
    
    if (StandardMetrics.ms_v_bat_soc->AsFloat() <= 10)
    {
        StandardMetrics.ms_v_bat_range_ideal->SetValue(0);
    }
    else
    {
        int range_max = 52;
        float newCarAh = 40.0;
        float range_ideal = range_max * StdMetrics.ms_v_bat_soc->AsFloat() / 100.0;
        StdMetrics.ms_v_bat_range_ideal->SetValue(range_ideal);
        StdMetrics.ms_v_bat_soh->SetValue((StdMetrics.ms_v_bat_cac->AsFloat() / newCarAh) * 100);
    }
}

int OvmsVehicleMitsubishiOutlander::calcMinutesRemaining(float target_soc, float charge_power_w)
{
    float bat_soc = StandardMetrics.ms_v_bat_soc->AsFloat(100);
    if (bat_soc > target_soc)
    {
        ESP_LOGW(TAG, "Battery SOC %.1f > Target SOC %.1f", bat_soc, target_soc);
        
        return 0;   // Done!
    }
    /*
     if (charge_power_w <= 0.0f)
     {
     return 1440;
     }
     */
    float bat_cap_kwh     = 12; //m_battery_energy_capacity->AsFloat(24, kWh);
    float remaining_wh    = bat_cap_kwh * 1000.0 * (target_soc - bat_soc) / 100.0;
    float remaining_hours = remaining_wh / charge_power_w;
    float remaining_mins  = remaining_hours * 60.0;
    ESP_LOGW(TAG, "Charge Power: %.1f W Remaining: %.1f Wh Remaining: %.1f mins", charge_power_w, remaining_wh, remaining_mins);
    
    return MIN( 1440, (int)remaining_mins );
}

class OvmsVehicleMitsubishiOutlanderInit
{
public: OvmsVehicleMitsubishiOutlanderInit();
} OvmsVehicleMitsubishiOutlanderInit  __attribute__ ((init_priority (9000)));

OvmsVehicleMitsubishiOutlanderInit::OvmsVehicleMitsubishiOutlanderInit()
{
    ESP_LOGI(TAG, "Registering Vehicle: OBDII (9000)");
    
    MyVehicleFactory.RegisterVehicle<OvmsVehicleMitsubishiOutlander>("MO","Outlander");
}
