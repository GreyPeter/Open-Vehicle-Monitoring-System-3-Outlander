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
static const char *TAG = "v-mo";

#include <stdio.h>
#include "mo_obd_pids.h"
#include "vehicle_mitsubishi_outlander.h"
#include "metrics_standard.h"
#define VERSION "1.0.0"

// {txmoduleid,rxmoduleid,type,pid,{pid for additional payload, payload length, payload data},pollbus,protocol}
static const OvmsVehicle::poll_pid_t obdii_polls[] =
{
    { bmuTxId, bmuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 7, 7 }, 0, ISOTP_STD }, // cac
    { bmuTxId, bmuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x02,{ 0, 7, 7 }, 0, ISOTP_STD }, // cac
    { bmuTxId, bmuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x03,{ 0, 7, 7 }, 0, ISOTP_STD }, // cac
    
    { obcTxId, obcRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 7,  7 }, 0, ISOTP_STD }, // OBC
    { obcTxId, obcRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x02,{ 0, 7,  7 }, 0, ISOTP_STD }, // OBC
    { obcTxId, obcRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x03,{ 0, 7,  7 }, 0, ISOTP_STD }, // OBC
    
    { fmcuTxId, fmcuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 4, 4 }, 0, ISOTP_STD }, //Front Motor Control Unit FMCU
    { rmcuTxId, rmcuRxId, VEHICLE_POLL_TYPE_OBDIIGROUP,  0x01,{ 0, 4, 4 }, 0, ISOTP_STD }, //Rear Motor Control Unit RMCU
    POLL_LIST_END
};

OvmsVehicleMitsubishiOutlander::OvmsVehicleMitsubishiOutlander()
{
    ESP_LOGI(TAG, "Start Mitsubishi Outlander vehicle module");
    
    StandardMetrics.ms_v_type->SetValue("OUTLANDER");
    //StandardMetrics.ms_v_vin->SetValue("DEMODEMODEMO");
    
    StandardMetrics.ms_v_bat_soc->SetValue(0);
    StandardMetrics.ms_v_bat_soh->SetValue(0);
    StandardMetrics.ms_v_bat_cac->SetValue(0);
    StandardMetrics.ms_v_bat_voltage->SetValue(0);
    StandardMetrics.ms_v_bat_current->SetValue(0);
    StandardMetrics.ms_v_bat_power->SetValue(0);
    StandardMetrics.ms_v_bat_energy_used->SetValue(0);
    StandardMetrics.ms_v_bat_energy_recd->SetValue(0);
    StandardMetrics.ms_v_bat_range_full->SetValue(52);
    StandardMetrics.ms_v_bat_range_ideal->SetValue(0);
    StandardMetrics.ms_v_bat_range_est->SetValue(0);
    StandardMetrics.ms_v_bat_12v_voltage->SetValue(0);
    StandardMetrics.ms_v_bat_12v_current->SetValue(0);
    StandardMetrics.ms_v_bat_temp->SetValue(25);
    StandardMetrics.ms_v_charge_voltage->SetValue(0);
    StandardMetrics.ms_v_charge_current->SetValue(0);
    StandardMetrics.ms_v_charge_climit->SetValue(0);
    StandardMetrics.ms_v_charge_kwh->SetValue(0);
    StandardMetrics.ms_v_charge_mode->SetValue("standard");
    StandardMetrics.ms_v_charge_timermode->SetValue(false);
    StandardMetrics.ms_v_charge_timerstart->SetValue(0);
    StandardMetrics.ms_v_charge_state->SetValue("done");
    StandardMetrics.ms_v_charge_substate->SetValue("stopped");
    StandardMetrics.ms_v_charge_type->SetValue("type1");
    StandardMetrics.ms_v_charge_pilot->SetValue(false);
    StandardMetrics.ms_v_charge_inprogress->SetValue(false);
    StandardMetrics.ms_v_charge_limit_range->SetValue(0);
    StandardMetrics.ms_v_charge_limit_soc->SetValue(0);
    StandardMetrics.ms_v_charge_duration_full->SetValue(0);
    StandardMetrics.ms_v_charge_duration_range->SetValue(0);
    StandardMetrics.ms_v_charge_duration_soc->SetValue(0);
    StandardMetrics.ms_v_charge_temp->SetValue(22);
    StandardMetrics.ms_v_inv_temp->SetValue(22);
    StandardMetrics.ms_v_mot_rpm->SetValue(0);
    StandardMetrics.ms_v_mot_temp->SetValue(30);
    StandardMetrics.ms_v_door_fl->SetValue(false);
    StandardMetrics.ms_v_door_fr->SetValue(false);
    StandardMetrics.ms_v_door_rl->SetValue(false);
    StandardMetrics.ms_v_door_rr->SetValue(false);
    StandardMetrics.ms_v_door_chargeport->SetValue(false);
    StandardMetrics.ms_v_door_hood->SetValue(false);
    StandardMetrics.ms_v_door_trunk->SetValue(false);
    StandardMetrics.ms_v_env_drivemode->SetValue(0);
    StandardMetrics.ms_v_env_handbrake->SetValue(false);
    StandardMetrics.ms_v_env_awake->SetValue(false);
    StandardMetrics.ms_v_env_charging12v->SetValue(false);
    StandardMetrics.ms_v_env_cooling->SetValue(false);
    StandardMetrics.ms_v_env_heating->SetValue(false);
    StandardMetrics.ms_v_env_hvac->SetValue(false);
    StandardMetrics.ms_v_env_on->SetValue(false);
    StandardMetrics.ms_v_env_locked->SetValue(false);
    StandardMetrics.ms_v_env_valet->SetValue(false);
    StandardMetrics.ms_v_env_headlights->SetValue(false);
    StandardMetrics.ms_v_env_alarm->SetValue(false);
    StandardMetrics.ms_v_env_ctrl_login->SetValue(false);
    StandardMetrics.ms_v_env_ctrl_config->SetValue(false);
    StandardMetrics.ms_v_env_temp->SetValue(22);
    //  StandardMetrics.ms_v_pos_gpslock->SetValue(true);
    //  StandardMetrics.ms_v_pos_satcount->SetValue(12);
    //  StandardMetrics.ms_v_pos_latitude->SetValue(22.280868);
    //  StandardMetrics.ms_v_pos_longitude->SetValue(114.160598);
    //  StandardMetrics.ms_v_pos_direction->SetValue(10);
    //  StandardMetrics.ms_v_pos_altitude->SetValue(30);
    StandardMetrics.ms_v_pos_speed->SetValue(0);
    //StandardMetrics.ms_v_pos_odometer->SetValue(100000);
    StandardMetrics.ms_v_pos_trip->SetValue(0);
    /*
    StandardMetrics.ms_v_tpms_pressure->SetValue(std::vector<float>{ 0, 0, 0, 0 });
    StandardMetrics.ms_v_tpms_temp->SetValue(std::vector<float>{ 0, 0, 0, 0 });
    StandardMetrics.ms_v_tpms_health->SetValue(std::vector<float>{ 0, 0, 0, 0 });
    StandardMetrics.ms_v_tpms_alert->SetValue(std::vector<short>{ 0, 0, 0, 0 });
    */
    RegisterCanBus(1,CAN_MODE_ACTIVE,CAN_SPEED_500KBPS);
    PollSetPidList(m_can1,obdii_polls);
    PollSetState(1);
}

OvmsVehicleMitsubishiOutlander::~OvmsVehicleMitsubishiOutlander()
{
    ESP_LOGI(TAG, "Shutdown Mitsubishi Outlander vehicle module");
}

void OvmsVehicleMitsubishiOutlander::Ticker1(uint32_t ticker)
{
    OvmsVehicle::Ticker1(ticker);
    
    //Check only if 'transmission in park
    if(StandardMetrics.ms_v_env_gear->AsInt() == -1)
    {
        ////////////////////////////////////////////////////////////////////////
        // Charge state determination
        ////////////////////////////////////////////////////////////////////////
        if(StandardMetrics.ms_v_charge_inprogress->AsBool())
        {
            if(StandardMetrics.ms_v_charge_pilot->AsBool()) // Charging is ongoing
            {
                StandardMetrics.ms_v_charge_state->SetValue("charging");
                StandardMetrics.ms_v_charge_inprogress->SetValue(true);
                StandardMetrics.ms_v_charge_voltage->SetValue(240, Volts);
                StandardMetrics.ms_v_charge_current->SetValue(StandardMetrics.ms_v_bat_power->AsFloat()*1000 / 240 * -1.5, Amps);
                StandardMetrics.ms_v_charge_power->SetValue((StandardMetrics.ms_v_charge_voltage->AsFloat() * StandardMetrics.ms_v_charge_current->AsFloat()) / 1000, kW);
                // if charge and DC current is negative cell balancing is active.
                if ((StandardMetrics.ms_v_bat_current->AsFloat() < 0) && StandardMetrics.ms_v_bat_soc->AsInt() > 92 && StandardMetrics.ms_v_charge_mode->AsString() != "Balancing")
                {
                    StandardMetrics.ms_v_charge_mode->SetValue("Balancing");
                    StandardMetrics.ms_v_charge_state->SetValue("topoff");
                }
            } else { // Charge has just started
                StandardMetrics.ms_v_charge_inprogress->SetValue(true);
                StandardMetrics.ms_v_charge_pilot->SetValue(true);
                StandardMetrics.ms_v_door_chargeport->SetValue(true);
                StandardMetrics.ms_v_charge_substate->SetValue("onrequest");
                StandardMetrics.ms_v_charge_duration_full->SetValue(0);
            }
            
        } else { // Not charging
            if (StandardMetrics.ms_v_charge_pilot->AsBool())
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
        }
        
    }
    /*
     if (StandardMetrics.ms_v_env_on->AsBool())
     {
     // We are driving
     int speed = StandardMetrics.ms_v_pos_speed->AsInt();
     if (speed<0) speed = 0;
     //StandardMetrics.ms_v_pos_speed->SetValue(speed);
     StandardMetrics.ms_v_mot_rpm->SetValue(speed*112);
     }
     */
}

void OvmsVehicleMitsubishiOutlander::Ticker10(uint32_t ticker)
{
    OvmsVehicle::Ticker10(ticker);
    if (StandardMetrics.ms_v_bat_soc->AsFloat() <= 10)
    {
        StandardMetrics.ms_v_bat_range_ideal->SetValue(0);
    }
    else
    {
        //int range_max = 52;
        float newCarAh = 40.0;
        //float range_ideal = StandardMetrics.ms_v_bat_range_full->AsFloat() * StdMetrics.ms_v_bat_soc->AsFloat() / 100.0;
        StdMetrics.ms_v_bat_range_ideal->SetValue(StandardMetrics.ms_v_bat_range_full->AsFloat() * StdMetrics.ms_v_bat_soc->AsFloat() / 100.0);
        StdMetrics.ms_v_bat_soh->SetValue((StdMetrics.ms_v_bat_cac->AsFloat() / newCarAh) * 100);
    }
    /*
    if (StandardMetrics.ms_v_charge_inprogress->AsBool())
    {
        // We are charging
        float soc = StandardMetrics.ms_v_bat_soc->AsFloat();
        if (soc < 100)
        {
            soc += 0.1;
            StandardMetrics.ms_v_bat_soc->SetValue(soc);
        }
        else
        {
            // Car full
            StandardMetrics.ms_v_charge_inprogress->SetValue(false);
            StandardMetrics.ms_v_door_chargeport->SetValue(false);
            StandardMetrics.ms_v_charge_state->SetValue("done");
            StandardMetrics.ms_v_charge_substate->SetValue("stopped");
            StandardMetrics.ms_v_charge_pilot->SetValue(false);
            StandardMetrics.ms_v_charge_voltage->SetValue(0);
            StandardMetrics.ms_v_charge_current->SetValue(0);
        }
    }
    else if (StandardMetrics.ms_v_env_on->AsBool())
    {
        // We are driving
        float soc = StandardMetrics.ms_v_bat_soc->AsFloat();
        if (soc > 0)
        {
            int speed = StandardMetrics.ms_v_pos_speed->AsInt() + (rand()%3) -1;
            if (speed<0) speed = 0;
            else if (speed>100) speed = 100;
            soc -= 0.1;
            StandardMetrics.ms_v_bat_soc->SetValue(soc);
            StandardMetrics.ms_v_pos_speed->SetValue(speed);
            StandardMetrics.ms_v_mot_rpm->SetValue(speed*112);
        }
        else
        {
            // Battery is empty. Charge the car...
            StandardMetrics.ms_v_pos_speed->SetValue(0);
            StandardMetrics.ms_v_mot_rpm->SetValue(0);
            StandardMetrics.ms_v_env_on->SetValue(false);
            StandardMetrics.ms_v_charge_inprogress->SetValue(true);
            StandardMetrics.ms_v_door_chargeport->SetValue(true);
            StandardMetrics.ms_v_charge_state->SetValue("charging");
            StandardMetrics.ms_v_charge_substate->SetValue("onrequest");
            StandardMetrics.ms_v_charge_pilot->SetValue(true);
            StandardMetrics.ms_v_charge_voltage->SetValue(220);
            StandardMetrics.ms_v_charge_current->SetValue(32);
        }
    }
     */
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandSetChargeMode(vehicle_mode_t mode)
{
    return NotImplemented;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandSetChargeCurrent(uint16_t limit)
{
    StandardMetrics.ms_v_charge_climit->SetValue(limit);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandStartCharge()
{
    StandardMetrics.ms_v_pos_speed->SetValue(0);
    StandardMetrics.ms_v_mot_rpm->SetValue(0);
    StandardMetrics.ms_v_env_on->SetValue(false);
    StandardMetrics.ms_v_charge_inprogress->SetValue(true);
    StandardMetrics.ms_v_door_chargeport->SetValue(true);
    StandardMetrics.ms_v_charge_state->SetValue("charging");
    StandardMetrics.ms_v_charge_substate->SetValue("onrequest");
    StandardMetrics.ms_v_charge_pilot->SetValue(true);
    StandardMetrics.ms_v_charge_voltage->SetValue(220);
    StandardMetrics.ms_v_charge_current->SetValue(32);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandStopCharge()
{
    StandardMetrics.ms_v_charge_inprogress->SetValue(false);
    StandardMetrics.ms_v_door_chargeport->SetValue(false);
    StandardMetrics.ms_v_charge_state->SetValue("done");
    StandardMetrics.ms_v_charge_substate->SetValue("stopped");
    StandardMetrics.ms_v_charge_pilot->SetValue(false);
    StandardMetrics.ms_v_charge_voltage->SetValue(0);
    StandardMetrics.ms_v_charge_current->SetValue(0);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandSetChargeTimer(bool timeron, uint16_t timerstart)
{
    return NotImplemented;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandCooldown(bool cooldownon)
{
    return NotImplemented;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandWakeup()
{
    StandardMetrics.ms_v_charge_inprogress->SetValue(false);
    StandardMetrics.ms_v_door_chargeport->SetValue(false);
    StandardMetrics.ms_v_charge_state->SetValue("done");
    StandardMetrics.ms_v_charge_substate->SetValue("stopped");
    StandardMetrics.ms_v_charge_pilot->SetValue(false);
    StandardMetrics.ms_v_charge_voltage->SetValue(0);
    StandardMetrics.ms_v_charge_current->SetValue(0);
    StandardMetrics.ms_v_env_on->SetValue(true);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandLock(const char* pin)
{
    StandardMetrics.ms_v_env_locked->SetValue(true);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandUnlock(const char* pin)
{
    StandardMetrics.ms_v_env_locked->SetValue(false);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandActivateValet(const char* pin)
{
    StandardMetrics.ms_v_env_valet->SetValue(true);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandDeactivateValet(const char* pin)
{
    StandardMetrics.ms_v_env_valet->SetValue(false);
    
    return Success;
}

OvmsVehicle::vehicle_command_t OvmsVehicleMitsubishiOutlander::CommandHomelink(int button, int durationms)
{
    return NotImplemented;
}

class OvmsVehicleMitsubishiOutlanderInit
{
public: OvmsVehicleMitsubishiOutlanderInit();
} MyOvmsVehicleMitsubishiOutlanderInit  __attribute__ ((init_priority (9000)));

OvmsVehicleMitsubishiOutlanderInit::OvmsVehicleMitsubishiOutlanderInit()
{
    ESP_LOGI(TAG, "Registering Vehicle: Outlander (9000)");
    
    MyVehicleFactory.RegisterVehicle<OvmsVehicleMitsubishiOutlander>("MO","Outlander");
}
