#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  Application_FunctionSet.ApplicationFunctionSet_Init();
  wdt_enable(WDTO_2S);
}

void loop()
{
  wdt_reset();
  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  Application_FunctionSet.ApplicationFunctionSet_Tracking();
  Application_FunctionSet.ApplicationFunctionSet_Sumo();
  Application_FunctionSet.ApplicationFunctionSet_Calibrate();
  Application_FunctionSet.ApplicationFunctionSet_Standby();
}
