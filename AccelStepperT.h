#ifndef _ACCELSTEPPERT_
#define _ACCELSTEPPERT_

#include "AccelStepper.h"

#define MAX_STEPPER_NUM 8
#define TIMER_FREQ_KHZ (50)

class AccelStepperT : public AccelStepper {
   public:
	AccelStepperT(uint8_t step, uint8_t dir);
	void runAsync(void);
	void runSpeedAsync(void);
	void runToPosition(void (*func)(void), volatile bool *keep_running);
	void runToPosition(void (*func)(void)) { return runToPosition(func, NULL); };
	void runToPosition(void) { return runToPosition(NULL, NULL); };
	void stopAsync(void) { _async_runtype = T_STOP; };

	static bool isTimerActive(void);

   private:
	enum { T_STOP,
		   T_RUN,
		   T_RUNSPEED,
	} _async_runtype;

	friend void __USER_ISR timer_isr(void);
	void step(long step);

	p32_ioport *iopStep;
	p32_ioport *iopDir;
	uint32_t bitStep;
	uint32_t bitDir;
};

#endif