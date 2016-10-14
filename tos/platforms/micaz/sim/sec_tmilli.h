/**
 * Platform specific macros to transform seconds to milliseconds and back.
 *
 * @author Raido Pahtma
 * @license MIT
*/
#ifndef SEC_TMILLI
#define SEC_TMILLI(seconds) ((seconds)*1000)
#define TMILLI_SEC(tmilli) ((tmilli)/1000)
#endif // SEC_TMILLI
