#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <Arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_RGB(void);
  void ApplicationFunctionSet_Tracking(void);  // Line follow mode
  void ApplicationFunctionSet_Sumo(void);      // Sumo mode
  void ApplicationFunctionSet_Standby(void);
  void ApplicationFunctionSet_KeyCommand(void);
  void ApplicationFunctionSet_SensorDataUpdate(void);

private:
  volatile float VoltageData_V;
  volatile uint16_t UltrasoundData_cm;
  volatile int TrackingData_L;
  volatile int TrackingData_M;
  volatile int TrackingData_R;

  boolean VoltageDetectionStatus = false;
  boolean TrackingDetectionStatus_R = false;
  boolean TrackingDetectionStatus_M = false;
  boolean TrackingDetectionStatus_L = false;

public:
  boolean Car_LeaveTheGround = true;

  const float VoltageDetection = 7.00;
  const uint8_t ObstacleDetection = 20;

public:
  /*
   * Surface profile (black shiny floor, white masking tape track/boundary, black electrical tape center/edge):
   *   White masking tape  → high IR reflection → LOW  analog value (~50–180)
   *   Black electrical tape → low IR reflection → HIGH analog value (~700–950)
   *   Black shiny floor   → mid IR reflection  → MID  analog value (~200–500)
   *
   * TrackingDetection_S: low edge — must be above white tape and shiny floor (~250)
   * TrackingDetection_E: high edge — must include pure black electrical tape (~930)
   * TrackingDetection_V: "off ground" threshold — all sensors above this = robot lifted
   */
  uint8_t TrackingDetection_S = 250;   // below this = white masking tape or shiny floor, ignore
  uint16_t TrackingDetection_E = 930;  // raised from 850 — black electrical tape can read up to ~950
  uint16_t TrackingDetection_V = 970;  // lifted off ground (all sensors above this)
};

extern ApplicationFunctionSet Application_FunctionSet;
#endif
