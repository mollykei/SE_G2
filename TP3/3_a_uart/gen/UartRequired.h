/** Generated by YAKINDU Statechart Tools code generator. */

#ifndef UARTREQUIRED_H_
#define UARTREQUIRED_H_

#include "../inc/sc_types.h"
#include "Uart.h"

#ifdef __cplusplus
extern "C"
{
#endif 

/*! \file This header defines prototypes for all functions that are required by the state machine implementation.

This state machine makes use of operations declared in the state machines interface or internal scopes. Thus the function prototypes:
	- uartIface_opLed
are defined.

These functions will be called during a 'run to completion step' (runCycle) of the statechart. 
There are some constraints that have to be considered for the implementation of these functions:
	- never call the statechart API functions from within these functions.
	- make sure that the execution time is as short as possible.
 
*/
extern void uartIface_opLed(const Uart* handle);




#ifdef __cplusplus
}
#endif 

#endif /* UARTREQUIRED_H_ */
