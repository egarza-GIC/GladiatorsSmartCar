#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <Arduino.h>
#include "RobotConfig.h"

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

  const float VoltageDetection = LOW_BATTERY_VOLTAGE;
  const uint8_t ObstacleDetection = OBSTACLE_DISTANCE_CM;

public:
  uint8_t TrackingDetection_S = TRACKING_THRESHOLD_LOW;
  uint16_t TrackingDetection_E = TRACKING_THRESHOLD_HIGH;
  uint16_t TrackingDetection_V = TRACKING_THRESHOLD_AIR;
};

extern ApplicationFunctionSet Application_FunctionSet;
#endif
