/**
 * @author Cory Sharp <cssharp@eecs.berkeley.edu>
 * @see  Please refer to TEP 102 for more information about this component and its
 *          intended use.
 */

configuration BusyWaitMicroC
{
  provides interface BusyWait<TMicro,uint16_t>;
}
implementation
{
  components new BusyWaitCounterC(TMicro,uint16_t);//, CounterTMicro16CF;

  BusyWait = BusyWaitCounterC;
  // BusyWaitCounterC.Counter -> CounterTMicro16C;
}
