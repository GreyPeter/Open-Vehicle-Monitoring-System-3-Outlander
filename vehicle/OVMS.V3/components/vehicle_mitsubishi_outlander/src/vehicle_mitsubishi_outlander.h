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

#ifndef __VEHICLE_DEMO_H__
#define __VEHICLE_DEMO_H__

#include "vehicle.h"

using namespace std;

class OvmsVehicleMitsubishiOutlander : public OvmsVehicle
  {
  public:
    OvmsVehicleMitsubishiOutlander();
    ~OvmsVehicleMitsubishiOutlander();
      
  public:
      void IncomingFrameCan1(CAN_frame_t* p_frame);
      void IncomingPollReply(canbus* bus, uint16_t type, uint16_t pid, uint8_t* data, uint8_t length, uint16_t mlremain);
      void IncomingObcPoll(uint16_t pid, uint8_t* data, uint8_t length, uint16_t remain);
      void OBCresponse1(uint16_t m_poll_ml_frame, uint8_t* data);
      void OBCresponse2(uint16_t m_poll_ml_frame, uint8_t* data);
      void OBCresponse3(uint16_t m_poll_ml_frame, uint8_t* data);
      void IncomingBmuPoll(uint16_t pid, uint8_t* data, uint8_t length, uint16_t remain);
      void BMUresponse1(uint16_t m_poll_ml_frame, uint8_t* data);
      void BMUresponse2(uint16_t m_poll_ml_frame, uint8_t* data, uint8_t length, uint16_t mlremain);
      void BMUresponse3(uint16_t m_poll_ml_frame, uint8_t* data, uint8_t length, uint16_t mlremain);
      
      unsigned int dcdcCount = 0;
      char cellVolts[160];
      char cellTemps[80];
      int voltCell = 0;
      int tempCell = 0;
      char m_vin[18];
      
  public:
      OvmsMetricFloat* ms_v_bat_cac_rem = MyMetrics.InitFloat("xmi.v.bat.cac.rem", 10, 0, AmpHours);
      OvmsMetricFloat* ms_v_bat_max_input = MyMetrics.InitFloat("xmi.v.bat.max.input", 10, 0, kW);
      OvmsMetricFloat* ms_v_bat_max_output = MyMetrics.InitFloat("xmi.v.bat.max.output", 10, 0, kW);

  public:
    virtual void Ticker1(uint32_t ticker);
    virtual void Ticker10(uint32_t ticker);

  public:
    virtual vehicle_command_t CommandSetChargeMode(vehicle_mode_t mode);
    virtual vehicle_command_t CommandSetChargeCurrent(uint16_t limit);
    virtual vehicle_command_t CommandStartCharge();
    virtual vehicle_command_t CommandStopCharge();
    virtual vehicle_command_t CommandSetChargeTimer(bool timeron, uint16_t timerstart);
    virtual vehicle_command_t CommandCooldown(bool cooldownon);
    virtual vehicle_command_t CommandWakeup();
    virtual vehicle_command_t CommandLock(const char* pin);
    virtual vehicle_command_t CommandUnlock(const char* pin);
    virtual vehicle_command_t CommandActivateValet(const char* pin);
    virtual vehicle_command_t CommandDeactivateValet(const char* pin);
    virtual vehicle_command_t CommandHomelink(int button, int durationms=1000);
  };

#endif //#ifndef __VEHICLE_DEMO_H__
