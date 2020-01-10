#include "AccelStepperT.h"

#define PR (uint16_t)(2500UL / TIMER_FREQ_KHZ)

static AccelStepperT *stepper_list[MAX_STEPPER_NUM];
static uint8_t stepper_list_num = 0;

static bool timer_has_init = false;
static volatile bool timer_started = false;
static volatile bool timer_needstart = false;

void timer_start(void) {
	timer_needstart = true;
	if (!timer_started) {
		setIntEnable(_TIMER_4_IRQ);
		T4CONbits.TON = 1;
		timer_started = true;
	}
}

void timer_stop(void) {
	//clearIntEnable(_TIMER_4_IRQ);
	IEC0CLR = _IEC0_T4IE_MASK;
	T4CONbits.TON = 0;
	timer_started = false;
}

void __USER_ISR timer_isr(void) {
	IFS0CLR = _IFS0_T4IF_MASK;
	// clearIntFlag(_TIMER_4_IRQ);
	// _LATB14 = 1;
	uint8_t cnt = 0;
	for (int i = 0; i < stepper_list_num; i++) {
		if (stepper_list[i]->_async_runtype == AccelStepperT::T_RUN) {
			if (stepper_list[i]->run()) {
				cnt++;
			} else {
				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
			}
		} else if (stepper_list[i]->_async_runtype == AccelStepperT::T_RUNSPEED) {
			if (stepper_list[i]->speed() != 0.0) {
				stepper_list[i]->runSpeed();
				cnt++;
			} else {
				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
			}
		}
	}
	if (cnt == 0 && !(timer_needstart)) {
		// timer_stop();
		timer_started = false;
		IEC0CLR = _IEC0_T4IE_MASK;
		T4CONbits.TON = 0;
	}
	timer_needstart = false;
	// _LATB14 = 0;
}

void timer_init(void) {
	T4CONbits.TCKPS = 5;
	setIntVector(_TIMER_4_VECTOR, timer_isr);
	setIntPriority(_TIMER_4_VECTOR, 6, 3);
	clearIntFlag(_TIMER_4_IRQ);
	clearIntEnable(_TIMER_4_IRQ);
	PR4 = PR;

	// pinMode(19, OUTPUT);
}

AccelStepperT::AccelStepperT(uint8_t step, uint8_t dir) : AccelStepper(AccelStepper::DRIVER, step, dir, 255, 255) {
	if (stepper_list_num < MAX_STEPPER_NUM) {
		uint8_t port;
		if ((step >= NUM_DIGITAL_PINS) || ((port = digitalPinToPort(step)) == NOT_A_PIN)) {
			return;
		}
		iopStep = (p32_ioport *)portRegisters(port);
		bitStep = digitalPinToBitMask(step);

		if ((dir >= NUM_DIGITAL_PINS) || ((port = digitalPinToPort(dir)) == NOT_A_PIN)) {
			return;
		}
		iopDir = (p32_ioport *)portRegisters(port);
		bitDir = digitalPinToBitMask(dir);

		stepper_list[stepper_list_num++] = this;
		if (!timer_has_init) {
			timer_init();
			timer_has_init = true;
		}
	}
}

void AccelStepperT::runAsync(void) {
	_async_runtype = T_RUN;
	timer_start();
}

void AccelStepperT::runSpeedAsync(void) {
	_async_runtype = T_RUNSPEED;
	timer_start();
}

void AccelStepperT::runToPosition(void (*func)(void) = NULL, volatile bool *keep_running = NULL) {
	bool bptr = true;
	if (!keep_running) keep_running = &bptr;
	runAsync();
	do {
		delay(1);
		if (func) func();
	} while (isRunning() && isTimerActive() && *keep_running);
	if (!*keep_running) _async_runtype = T_STOP;
}

bool AccelStepperT::isTimerActive(void) {
	return timer_started;
}

void AccelStepperT::step(long step) {
	(void)step;
	if (_direction) {
		iopDir->lat.set = bitDir;
	} else {
		iopDir->lat.clr = bitDir;
	}
	asm volatile("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n ");
	if (_prev_direction != _direction){
		delayMicroseconds(5);
	}
	_prev_direction = _direction;
	iopStep->lat.set = bitStep;
	delayMicroseconds(5);
	iopStep->lat.clr = bitStep;
}