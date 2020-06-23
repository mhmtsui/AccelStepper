#ifndef _ACCELSTEPPERT_
#define _ACCELSTEPPERT_

#include "../../robot.h"
#include "AccelStepper.h"

//#define MAX_STEPPER_NUM 12

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

typedef enum {
	VMODE,
	PMODE
} _stpr_mode_t;

class AccelStepperT : public AccelStepper {
   public:
   	typedef struct {
		p32_ioport *iopHome;
		uint32_t bitHome;   
		uint8_t pin;
		float toward_move;
		float leave_move; 
		bool invert;
		uint32_t position;
		uint32_t current;
		// uint8_t prev;
		// uint8_t prev1;
		// uint8_t prev2;
		// uint8_t current;
	} home_struct_t;
	enum { T_STOP,
		   T_RUN,
		   T_RUNSPEED,
		   T_RUNSPEEDACCEL,
		   T_HOME_F,
		   T_HOME_R
	} _async_runtype;
   //public:
	AccelStepperT(uint8_t step, uint8_t dir);
	void runAsync(void);
	void runSpeedAsync(void);
	void runSpeedwithAccelerationAsync(void);
	void runToPosition(void (*func)(void), volatile bool *keep_running);
	void runToPosition(void (*func)(void)) { return runToPosition(func, NULL); };
	void runToPosition(void) { return runToPosition(NULL, NULL); };
	void stopAsync(void) { _async_runtype = T_STOP; };
	static bool isTimerActive(void);
	void startHome(_async_hometype mode);
	uint8_t runHome(home_struct_t * home_struct);
	bool configHome(_async_hometype mode, uint8_t pin, float toward_move, float leave_move, bool invert, uint32_t position);
	void setMode(_stpr_mode_t mode) {_mode = mode;};
	bool isRunning();
	_async_homestate_t Homestatus();
	home_struct_t HomeF;
	home_struct_t HomeR;
   private:
	friend void __USER_ISR timer_isr(void);
	void step(long step);

	p32_ioport *iopStep;
	p32_ioport *iopDir;
	uint32_t bitStep;
	uint32_t bitDir;

	_async_homestate_t home;//a flag for stepper homing process
	_stpr_mode_t _mode;
};

//extern AccelStepperT *stepper_list[MAX_STEPPER_NUM];
//extern uint8_t stepper_list_num;
//extern bool timer_has_init;

static volatile bool timer_started = false;

extern void timer_init(void);
extern void timer_start(void);
extern void timer_stop(void);

#endif