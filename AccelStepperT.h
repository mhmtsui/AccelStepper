#ifndef _ACCELSTEPPERT_
#define _ACCELSTEPPERT_

#include "AccelStepper.h"

#define MAX_STEPPER_NUM 8
#define TIMER_FREQ_KHZ (50)

typedef enum {
	HOME_START = 0,
	HOME_TOWARD,
	HOME_AWAY,
	HOME_DONE
} _async_homestate_t;

typedef enum { 
	HOME_F = 1,
	HOME_R
} _async_hometype;

class AccelStepperT : public AccelStepper {
   private:
   	typedef struct {
		uint8_t pin;
		float toward_move;
		float leave_move; 
		bool invert;
		uint32_t position;
		uint8_t current;
		// uint8_t prev;
		// uint8_t prev1;
		// uint8_t prev2;
		// uint8_t current;
	} home_struct_t;
   public:
	AccelStepperT(uint8_t step, uint8_t dir);
	void runAsync(void);
	void runSpeedAsync(void);
	void runToPosition(void (*func)(void), volatile bool *keep_running);
	void runToPosition(void (*func)(void)) { return runToPosition(func, NULL); };
	void runToPosition(void) { return runToPosition(NULL, NULL); };
	void stopAsync(void) { _async_runtype = T_STOP; };
	static bool isTimerActive(void);
	void startHome(_async_hometype mode);
	uint8_t runHome(home_struct_t * home_struct);
	bool configHome(_async_hometype mode, uint8_t pin, float toward_move, float leave_move, bool invert, uint32_t position);

	_async_homestate_t Homestatus();

   private:
	enum { T_STOP,
		   T_RUN,
		   T_RUNSPEED,
		   T_HOME_F,
		   T_HOME_R
	} _async_runtype;

	friend void __USER_ISR timer_isr(void);
	void step(long step);

	p32_ioport *iopStep;
	p32_ioport *iopDir;
	uint32_t bitStep;
	uint32_t bitDir;

	_async_homestate_t home;//a flag for stepper homing process
	home_struct_t HomeF;
	home_struct_t HomeR;
};

#endif