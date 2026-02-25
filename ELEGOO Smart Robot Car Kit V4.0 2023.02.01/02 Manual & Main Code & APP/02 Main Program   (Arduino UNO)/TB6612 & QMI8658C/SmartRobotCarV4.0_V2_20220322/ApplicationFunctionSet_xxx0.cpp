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

/*Mode enum*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,
  TraceBased_mode,
  Sumo_mode,
  Calibrate_mode
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
    case Calibrate_mode:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, 2, CRGB::Blue);
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
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, TRACK_SPEED_FORWARD);
    }
    else if (function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, TRACK_SPEED_TURN);
    }
    else if (function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, TRACK_SPEED_TURN);
    }
    else
    {
      /*Line lost — stop*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
  }
  else
  {
    first_is = true;
  }
}

// Sumo boundary threshold is defined in RobotConfig.h (SUMO_BOUNDARY_THRESHOLD)

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
  static uint8_t detectCount = 0;
  static bool avoidTurnRight = true; // captured at trigger time: true = spin right, false = spin left
  static unsigned long ultrasonicTimer = 0;
  static uint16_t distance = 0;
  static bool arcRight = true; // alternates each boundary recovery so the robot curves a different way

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

  /*Robot lifted — stop motors and bail before boundary check triggers on air readings*/
  if (Car_LeaveTheGround == false)
  {
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
    return;
  }

  /*
    Boundary check: read sensors fresh here rather than using cached TrackingData.
    The ultrasonic blocking read takes 5-30ms; reading fresh ensures we catch the
    tape even if the robot crossed it during the previous ultrasonic read.
  */
  int irL = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
  int irM = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
  int irR = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
  bool boundaryLeft   = (irL >= SUMO_BOUNDARY_L);
  bool boundaryMiddle = (irM >= SUMO_BOUNDARY_M);
  bool boundaryRight  = (irR >= SUMO_BOUNDARY_R);
  bool onBoundary = boundaryLeft || boundaryMiddle || boundaryRight;

  if (state != SUMO_DELAY && state != SUMO_AVOID_BOUNDARY && onBoundary)
  {
    /*Lock in turn direction now — sensors may leave the tape as robot reverses*/
    avoidTurnRight = !boundaryLeft; // boundary on left → turn right; otherwise → turn left
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
    if (millis() - timer >= SUMO_DELAY_MS)
    {
      state = SUMO_SEARCH;
      servoAngle = 90;
      servoDir = 1;
    }
    break;

  case SUMO_SEARCH:
    /*Arc forward slowly — one wheel faster than the other so the robot curves.
      Direction alternates each time boundary avoidance completes, covering the arena
      in a widening spiral rather than bouncing straight into opposite walls.*/
    if (arcRight)
      AppMotor.DeviceDriverSet_Motor_control(direction_just, SUMO_SEARCH_SPEED, direction_just, SUMO_SEARCH_INNER_SPEED, control_enable);
    else
      AppMotor.DeviceDriverSet_Motor_control(direction_just, SUMO_SEARCH_INNER_SPEED, direction_just, SUMO_SEARCH_SPEED, control_enable);
    if (millis() - servoTimer > SUMO_SERVO_INTERVAL_MS)
    {
      servoTimer = millis();
      servoAngle += servoDir * SUMO_SERVO_STEP;
      if (servoAngle >= SUMO_SERVO_MAX) { servoAngle = SUMO_SERVO_MAX; servoDir = -1; }
      if (servoAngle <= SUMO_SERVO_MIN) { servoAngle = SUMO_SERVO_MIN; servoDir = 1;  }
      AppServo.DeviceDriverSet_Servo_writeNoWait(servoAngle);
    }
    /*
      HC-SR04 needs ~60ms between pings — firing faster causes residual echoes to corrupt
      the next reading, making most returns 0 and breaking the consecutive-count logic.
    */
    if (millis() - ultrasonicTimer >= SUMO_ULTRASONIC_INTERVAL_MS)
    {
      ultrasonicTimer = millis();
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance);
      if (distance > 0 && distance <= SUMO_DETECT_DISTANCE_CM)
      {
        detectCount++;
        if (detectCount >= SUMO_CONFIRM_COUNT)
        {
          /*Confirmed opponent — center servo and charge*/
          detectCount = 0;
          AppServo.DeviceDriverSet_Servo_writeNoWait(90);
          servoAngle = 90;
          state = SUMO_CHARGE;
        }
      }
      else
      {
        detectCount = 0;
      }
    }
    break;

  case SUMO_CHARGE:
    if (millis() - ultrasonicTimer >= SUMO_ULTRASONIC_INTERVAL_MS)
    {
      ultrasonicTimer = millis();
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance);
    }
    if (distance > 0 && distance <= SUMO_DETECT_DISTANCE_CM)
    {
      /*Full speed charge*/
      AppMotor.DeviceDriverSet_Motor_control(direction_just, SUMO_CHARGE_SPEED, direction_just, SUMO_CHARGE_SPEED, control_enable);
    }
    else
    {
      /*Lost opponent — return to searching*/
      state = SUMO_SEARCH;
    }
    break;

  case SUMO_AVOID_BOUNDARY:
    /*
      Phase 1: reverse away from boundary.
      Phase 2: spin 180° to face back toward center.
        Turn direction was locked in at trigger time (avoidTurnRight).
      Tune SUMO_SPIN_MS in RobotConfig.h until the spin is a clean 180°.
    */
    if (millis() - timer < SUMO_REVERSE_MS)
    {
      AppMotor.DeviceDriverSet_Motor_control(direction_back, SUMO_REVERSE_SPEED, direction_back, SUMO_REVERSE_SPEED, control_enable);
    }
    else if (millis() - timer < (SUMO_REVERSE_MS + SUMO_SPIN_MS))
    {
      if (avoidTurnRight)
      {
        AppMotor.DeviceDriverSet_Motor_control(direction_back, SUMO_SPIN_SPEED, direction_just, SUMO_SPIN_SPEED, control_enable);
      }
      else
      {
        AppMotor.DeviceDriverSet_Motor_control(direction_just, SUMO_SPIN_SPEED, direction_back, SUMO_SPIN_SPEED, control_enable);
      }
    }
    else
    {
      /*Recovery done — flip arc direction so next search curves the other way, then search*/
      arcRight = !arcRight;
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

/*Calibration mode — motors off, print raw IR sensor values every 250ms*/
void ApplicationFunctionSet::ApplicationFunctionSet_Calibrate(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode != Calibrate_mode)
    return;

  /*Kill motors*/
  AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 250)
    return;
  lastPrint = millis();

  int L = TrackingData_L;
  int M = TrackingData_M;
  int R = TrackingData_R;

  Serial.print("L="); Serial.print(L);
  Serial.print("  M="); Serial.print(M);
  Serial.print("  R="); Serial.print(R);

  Serial.print("    |  TRACK("); Serial.print(TRACKING_THRESHOLD_LOW);
  Serial.print("-"); Serial.print(TRACKING_THRESHOLD_HIGH); Serial.print("): ");
  Serial.print("L="); Serial.print(function_xxx(L, TRACKING_THRESHOLD_LOW, TRACKING_THRESHOLD_HIGH) ? "IN " : "OUT");
  Serial.print(" M="); Serial.print(function_xxx(M, TRACKING_THRESHOLD_LOW, TRACKING_THRESHOLD_HIGH) ? "IN " : "OUT");
  Serial.print(" R="); Serial.print(function_xxx(R, TRACKING_THRESHOLD_LOW, TRACKING_THRESHOLD_HIGH) ? "IN " : "OUT");

  Serial.print("  |  SUMO_BOUNDARY(L>="); Serial.print(SUMO_BOUNDARY_L);
  Serial.print(" M>="); Serial.print(SUMO_BOUNDARY_M);
  Serial.print(" R>="); Serial.print(SUMO_BOUNDARY_R); Serial.print("): ");
  Serial.print("L="); Serial.print(L >= SUMO_BOUNDARY_L ? "YES" : "NO ");
  Serial.print(" M="); Serial.print(M >= SUMO_BOUNDARY_M ? "YES" : "NO ");
  Serial.print(" R="); Serial.print(R >= SUMO_BOUNDARY_R ? "YES" : "NO ");

  Serial.println();
}

/*Button cycles: Standby → Line Follow → Sumo → Calibrate → Standby ...*/
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
      Application_SmartRobotCarxxx0.Functional_Mode = Calibrate_mode;
      break;
    default:
      break;
    }
  }
}
