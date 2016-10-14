/*
 * RadioConfig for simulated micaz.
 *
 * @author Raido Pahtma
 * @license MIT
 */
#ifndef __RADIOCONFIG_H__
#define __RADIOCONFIG_H__

/**
 * The number of radio alarm ticks per one microsecond
 */
//#define RADIO_ALARM_MICROSEC 1

/**
 * The base two logarithm of the number of radio alarm ticks per one millisecond
 */
#define RADIO_ALARM_MILLI_EXP 0

typedef TMilli TRadio;
//typedef uint32_t tradio_size;

#endif//__RADIOCONFIG_H__
