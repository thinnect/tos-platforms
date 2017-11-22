#include "Timer.h"

configuration HilTimerMilliC
{
  provides 
  {
      interface Init;
      interface Timer<TMilli> as TimerMilli[ uint8_t num ];
      interface LocalTime<TMilli>;
  }
}

implementation
{
  enum {
    TIMER_COUNT = uniqueCount(UQ_TIMER_MILLI)
  };

  components AlarmCounterMilliP, new AlarmToTimerC(TMilli),
    new VirtualizeTimerC(TMilli, TIMER_COUNT),
    new CounterToLocalTimeC(TMilli);

  Init = AlarmCounterMilliP;

  TimerMilli = VirtualizeTimerC;
  VirtualizeTimerC.TimerFrom -> AlarmToTimerC;
  AlarmToTimerC.Alarm -> AlarmCounterMilliP.AlarmMilli32;

  LocalTime = CounterToLocalTimeC;
  CounterToLocalTimeC.Counter -> AlarmCounterMilliP.CounterMilli32;
}
