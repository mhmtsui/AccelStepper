#include "AccelStepperT.h"

#define PR (uint16_t)(3125UL / TIMER_FREQ_KHZ)

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

void __attribute__((nomips16,at_vector(_TIMER_4_VECTOR),interrupt(IPL4SRS))) timer_isr(void) {
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
		} else if (stepper_list[i]->_async_runtype == AccelStepperT::T_HOME_F) {
			if (stepper_list[i]->runHome(&(stepper_list[i]->HomeF))) {
				cnt++;
			} else {
				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
			}
		} else if (stepper_list[i]->_async_runtype == AccelStepperT::T_HOME_R) {
			if (stepper_list[i]->runHome(&(stepper_list[i]->HomeR))) {
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
	setIntPriority(_TIMER_4_VECTOR, 7, 1);
	clearIntFlag(_TIMER_4_IRQ);
	clearIntEnable(_TIMER_4_IRQ);
	PR4 = PR;
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

void AccelStepperT::startHome(_async_hometype mode) {
	if (mode == HOME_F){
		home = HOME_START;
		_async_runtype = T_HOME_F;
		timer_start();
	}else if (mode == HOME_R){
		home = HOME_START;
		_async_runtype = T_HOME_R;
		timer_start();
	}
}

uint8_t AccelStepperT::runHome(home_struct_t * home_struct) {
	home_struct->current = ((((home_struct->current & 0x07) << 1) & 0x0E) | ((digitalRead(home_struct->pin)) & 0x01));
	//Serial.println(home_struct->current);
	if (home == HOME_START){
		if (home_struct->invert == false){//active 1
			if (!(home_struct->current & 0x01)){
				home = HOME_TOWARD;
				if (speed() != home_struct->toward_move){
					setSpeed(home_struct->toward_move);
				}
				runSpeed();
				return 1;
			}else{
				home = HOME_AWAY;
				if (speed() != home_struct->leave_move){
					setSpeed(home_struct->leave_move);
				}
				runSpeed();
				return 1;
			}
		}else if (home_struct->invert == true){//active 0
			if (home_struct->current & 0x01){
				home = HOME_TOWARD;
				if (speed() != home_struct->toward_move){
					setSpeed(home_struct->toward_move);
				}
				runSpeed();
				return 1;
			}else{
				home = HOME_AWAY;
				if (speed() != home_struct->leave_move){
					setSpeed(home_struct->leave_move);
				}
				runSpeed();
				return 1;
			}
		}
	}else if (home == HOME_TOWARD){
		if (home_struct->invert == false){//active 1
			if (((home_struct->current) & 0x0F) != 0x0F){
				home = HOME_TOWARD;
				if (speed() != home_struct->toward_move){
					setSpeed(home_struct->toward_move);
				}
				runSpeed();
				return 1;
			}else{
				setSpeed(0.0f);
				_async_runtype = T_STOP;
				setCurrentPosition(home_struct->position);
				home = HOME_DONE;
			}
		}else if (home_struct->invert == true){//active 0
			if ((home_struct->current) & 0x0F){
				home = HOME_TOWARD;
				if (speed() != home_struct->toward_move){
					setSpeed(home_struct->toward_move);
				}
				runSpeed();
				return 1;
			}else{
				setSpeed(0.0f);
				_async_runtype = T_STOP;
				setCurrentPosition(home_struct->position);
				home = HOME_DONE;
			}
		}
	}else if (home == HOME_AWAY){
		if (home_struct->invert == false){//active 1
			if (((home_struct->current) & 0x0F) != 0x00){
				home = HOME_AWAY;
				if (speed() != home_struct->leave_move){
					setSpeed(home_struct->leave_move);
				}
				runSpeed();
				return 1;
			}else{
				home = HOME_TOWARD;
				return 1;
			}
		}else if (home_struct->invert == true){//active 0
			if (((home_struct->current) & 0x0F) != 0x0F){
				home = HOME_AWAY;
				if (speed() != home_struct->leave_move){
					setSpeed(home_struct->leave_move);
				}
				runSpeed();
				return 1;
			}else{
				home = HOME_TOWARD;
				return 1;
			}
		}
	}
	return 0;
}

bool AccelStepperT::configHome(_async_hometype mode, uint8_t pin, float toward_move, float leave_move, bool invert, uint32_t position) {
	if (mode == HOME_F){//Home F
		uint8_t port;
		if ((pin >= NUM_DIGITAL_PINS) || ((port = digitalPinToPort(pin)) == NOT_A_PIN)) {
			return false;
		}
		HomeF.pin = pin;
		HomeF.toward_move = toward_move;
		HomeF.leave_move = leave_move;
		HomeF.invert = invert;
		HomeF.position = position;
	}else if (mode == HOME_R){//Home R
		uint8_t port;
		if ((pin >= NUM_DIGITAL_PINS) || ((port = digitalPinToPort(pin)) == NOT_A_PIN)) {
			return false;
		}
		HomeR.pin = pin;
		HomeR.toward_move = toward_move;
		HomeR.leave_move = leave_move;
		HomeR.invert = invert;
		HomeR.position = position;
	}
	return true;
}

_async_homestate_t AccelStepperT::Homestatus(){
	return home;
}