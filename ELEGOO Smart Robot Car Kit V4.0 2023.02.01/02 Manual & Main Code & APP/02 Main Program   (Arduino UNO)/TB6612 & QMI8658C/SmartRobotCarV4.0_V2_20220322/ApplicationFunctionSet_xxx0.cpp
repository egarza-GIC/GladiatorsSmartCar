#include <avr/wdt.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

ApplicationFunctionSet Application_FunctionSet;

/*Hardware objects*/
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;

/*Helper: check if x is in range [s, e]*/
static boolean function_xxx(long x, long s, long e)
{
  return (s <= x && x <= e);
}

static void delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

/*Motion direction enum*/
enum SmartRobotCarMotionControl
{
  Forward,
  Backward,
  Left,
  Right,
  LeftForward,
  LeftBackward,
  RightForward,
  RightBackward,
  stop_it
};

/*Mode enum — 3 modes only*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,
  TraceBased_mode,
  Sumo_mode
};

struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
};
Application_xxx Application_SmartRobotCarxxx0;

/*Check if car has been lifted off the ground*/
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}

/*Direct motor motion control — no gyro correction*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  uint8_t speed = is_speed;
  switch (direction)
  {
  case Forward:
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_just, speed, control_enable);
    break;
  case Backward:
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_back, speed, control_enable);
    break;
  case Left:
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_back, speed, control_enable);
    break;
  case Right:
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_just, speed, control_enable);
    break;
  case LeftForward:
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_just, speed / 2, control_enable);
    break;
  case LeftBackward:
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_back, speed / 2, control_enable);
    break;
  case RightForward:
    AppMotor.DeviceDriverSet_Motor_control(direction_just, speed / 2, direction_just, speed, control_enable);
    break;
  case RightBackward:
    AppMotor.DeviceDriverSet_Motor_control(direction_back, speed / 2, direction_back, speed, control_enable);
    break;
  case stop_it:
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
    break;
  default:
    break;
  }
}

/*Init*/
void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  Serial.begin(9600);
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

/*Sensor data update*/
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{
  /*Voltage status update*/
  static unsigned long VoltageData_time = 0;
  static int VoltageData_number = 1;
  if (millis() - VoltageData_time > 100)
  {
    VoltageData_time = millis();
    VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
    if (VoltageData_V < VoltageDetection)
    {
      VoltageData_number++;
      if (VoltageData_number == 50)
      {
        VoltageDetectionStatus = true;
        VoltageData_number = 0;
      }
    }
    else
    {
      VoltageDetectionStatus = false;
    }
  }

  /*Tracking sensor update*/
  TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
  TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
  TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
  TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
  TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
  TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
  ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
}

/*RGB LED status indicator*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);

  if (true == VoltageDetectionStatus)
  {
    if ((millis() - getAnalogue_time) > 3000)
    {
      getAnalogue_time = millis();
    }
  }

  unsigned long temp = millis() - getAnalogue_time;
  if (function_xxx((temp), 0, 500) && VoltageDetectionStatus == true)
  {
    /*Low voltage: rapid red blink*/
    switch (temp)
    {
    case 0 ... 49:   AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);   break;
    case 50 ... 99:  AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black); break;
    case 100 ... 149: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);  break;
    case 150 ... 199: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);break;
    case 200 ... 249: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);  break;
    case 250 ... 299: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);  break;
    case 300 ... 349: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);break;
    case 350 ... 399: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);  break;
    case 400 ... 449: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);break;
    case 450 ... 499: AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);  break;
    default: break;
    }
  }
  else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false)
  {
    switch (Application_SmartRobotCarxxx0.Functional_Mode)
    {
    case Standby_mode:
    {
      if (VoltageDetectionStatus == true)
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
        delay(30);
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Black);
        delay(30);
      }
      else
      {
        /*Pulsing violet*/
        static uint8_t setBrightness = 0;
        static boolean et = false;
        static unsigned long time = 0;
        if ((millis() - time) > 10)
        {
          time = millis();
          if (et == false)
          {
            setBrightness += 1;
            if (setBrightness == 100) et = true;
          }
          else
          {
            setBrightness -= 1;
            if (setBrightness == 0) et = false;
          }
        }
        AppRBG_LED.leds[0] = CRGB::Violet;
        FastLED.setBrightness(setBrightness);
        FastLED.show();
      }
    }
    break;
    case TraceBased_mode:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Green);
      break;
    case Sumo_mode:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Red);
      break;
    default:
      break;
    }
  }
}

/*Line follow mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void)
{
  static bool first_is = true;
  static bool timestamp = true;
  static bool BlindDetection = true;
  static unsigned long MotorRL_time = 0;

  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (first_is == true)
    {
      AppServo.DeviceDriverSet_Servo_control(90);
      first_is = false;
    }
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

    if (function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else
    {
      /*Line lost — start blind detection search*/
      if (timestamp == true)
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      if ((function_xxx((millis() - MotorRL_time), 0, 200) || function_xxx((millis() - MotorRL_time), 1600, 2000)) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      }
      else if ((function_xxx((millis() - MotorRL_time), 200, 1600)) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      }
      else if (function_xxx((millis() - MotorRL_time), 3000, 3500))
      {
        BlindDetection = false;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
    }
  }
  else
  {
    first_is = true;
    if (false == timestamp)
    {
      BlindDetection = true;
      timestamp = true;
      MotorRL_time = 0;
    }
  }
}

/*
  Arena surface layout (inside → outside):
    [black shiny floor]  [white masking tape boundary]  [black electrical tape outer edge]

  IR sensor readings (lower value = more reflection = brighter surface):
    Black shiny floor:    ~200–500  (interior of ring — robot should stay here)
    White masking tape:   ~50–180   (boundary — robot must reverse when it sees this)
    Black electrical tape: ~700–950 (outer edge — robot should never reach this)

  SUMO_BOUNDARY_THRESHOLD: sensors below this value = white masking tape detected = at boundary.
  Set well below the shiny floor (~200) to avoid false triggers on the floor.
  Tune lower (e.g. 100) if you get false positives; tune higher (e.g. 180) if it misses the boundary.
*/
#define SUMO_BOUNDARY_THRESHOLD 150

/*Sumo mode — search, charge, and avoid boundary*/
void ApplicationFunctionSet::ApplicationFunctionSet_Sumo(void)
{
  enum SumoState { SUMO_DELAY, SUMO_SEARCH, SUMO_CHARGE, SUMO_AVOID_BOUNDARY };
  static SumoState state = SUMO_DELAY;
  static unsigned long timer = 0;
  static bool first_is = true;
  static uint8_t servoAngle = 90;
  static int8_t servoDir = 1;
  static unsigned long servoTimer = 0;
  static SumoState stateBeforeBoundary = SUMO_SEARCH;

  if (Application_SmartRobotCarxxx0.Functional_Mode != Sumo_mode)
  {
    first_is = true;
    state = SUMO_DELAY;
    return;
  }

  if (first_is)
  {
    first_is = false;
    state = SUMO_DELAY;
    timer = millis();
    servoAngle = 90;
    AppServo.DeviceDriverSet_Servo_control(90);
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
  }

  uint16_t distance = 0;

  /*
    Boundary check: white arena border reflects IR strongly → low analog value.
    Check any sensor below threshold (skip during delay countdown).
  */
  bool boundaryLeft  = (TrackingData_L < SUMO_BOUNDARY_THRESHOLD);
  bool boundaryMiddle = (TrackingData_M < SUMO_BOUNDARY_THRESHOLD);
  bool boundaryRight = (TrackingData_R < SUMO_BOUNDARY_THRESHOLD);
  bool onBoundary = boundaryLeft || boundaryMiddle || boundaryRight;

  if (state != SUMO_DELAY && state != SUMO_AVOID_BOUNDARY && onBoundary)
  {
    stateBeforeBoundary = state;
    state = SUMO_AVOID_BOUNDARY;
    timer = millis();
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
  }

  switch (state)
  {
  case SUMO_DELAY:
    /*Motors off — wait 3 seconds before activating*/
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
    if (millis() - timer >= 3000)
    {
      state = SUMO_SEARCH;
      servoAngle = 90;
      servoDir = 1;
    }
    break;

  case SUMO_SEARCH:
    /*Creep forward slowly while sweeping servo to scan for opponent*/
    AppMotor.DeviceDriverSet_Motor_control(direction_just, 80, direction_just, 80, control_enable);
    if (millis() - servoTimer > 80)
    {
      servoTimer = millis();
      servoAngle += servoDir * 5;
      if (servoAngle >= 150) { servoAngle = 150; servoDir = -1; }
      if (servoAngle <= 30)  { servoAngle = 30;  servoDir = 1;  }
      AppServo.DeviceDriverSet_Servo_control(servoAngle);
    }
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance);
    if (distance > 0 && distance <= 60)
    {
      /*Opponent detected — center servo and charge*/
      AppServo.DeviceDriverSet_Servo_control(90);
      servoAngle = 90;
      state = SUMO_CHARGE;
    }
    break;

  case SUMO_CHARGE:
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance);
    if (distance > 0 && distance <= 60)
    {
      /*Full speed charge*/
      AppMotor.DeviceDriverSet_Motor_control(direction_just, 255, direction_just, 255, control_enable);
    }
    else
    {
      /*Lost opponent — return to searching*/
      state = SUMO_SEARCH;
    }
    break;

  case SUMO_AVOID_BOUNDARY:
    /*
      Phase 1 (0–350ms): reverse away from boundary.
      Phase 2 (350–700ms): spin to face back into ring.
        - If left sensor triggered → spin right (boundary is on left side)
        - If right sensor triggered → spin left (boundary is on right side)
        - Otherwise (middle only) → spin right by default
      After 700ms: resume previous state.
    */
    if (millis() - timer < 350)
    {
      AppMotor.DeviceDriverSet_Motor_control(direction_back, 200, direction_back, 200, control_enable);
    }
    else if (millis() - timer < 700)
    {
      if (boundaryLeft)
      {
        /*Boundary was on left — spin right to face back inward*/
        AppMotor.DeviceDriverSet_Motor_control(direction_back, 150, direction_just, 150, control_enable);
      }
      else
      {
        /*Boundary was on right or front — spin left*/
        AppMotor.DeviceDriverSet_Motor_control(direction_just, 150, direction_back, 150, control_enable);
      }
    }
    else
    {
      /*Recovery done — go back to searching*/
      AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
      state = SUMO_SEARCH;
    }
    break;
  }
}

/*Standby mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void)
{
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (true == is_ED)
    {
      static unsigned long timestamp;
      if (millis() - timestamp > 20)
      {
        timestamp = millis();
        if (ApplicationFunctionSet_SmartRobotCarLeaveTheGround())
        {
          cout += 1;
        }
        else
        {
          cout = 0;
        }
        if (cout > 10)
        {
          is_ED = false;
        }
      }
    }
  }
}

/*Button cycles: Standby → Line Follow → Sumo → Standby ...*/
void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
{
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue)
  {
    temp_keyValue = get_keyValue;
    switch (get_keyValue)
    {
    case 1:
      Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case 2:
      Application_SmartRobotCarxxx0.Functional_Mode = Sumo_mode;
      break;
    case 3:
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    case 4:
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    default:
      break;
    }
  }
}
