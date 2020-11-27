#include <cpudefs.h>
#include <Streaming.h>
#include "AccelStepperT.h"

#define HOME_DEBUG

AccelStepperT::AccelStepperT(uint8_t step, uint8_t dir) : AccelStepper(AccelStepper::DRIVER, step, dir, 255, 255) {
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

}

void AccelStepperT::runAsync(void) {
	_async_runtype = T_RUN;
	timer_start();
}

void AccelStepperT::runSpeedAsync(void) {
	_async_runtype = T_RUNSPEED;
	timer_start();
}

void AccelStepperT::runSpeedwithAccelerationAsync(void) {
	_async_runtype = T_RUNSPEEDACCEL;
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
	if (_direction ^ _pinInverted[1]) {
		iopDir->lat.set = bitDir;
	} else {
		iopDir->lat.clr = bitDir;
	}
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

#define DEBOUNCE_CYCLE (14)
//#define MASK_1 (0xFE)
#define DEBOUNCE_BITMASK ((uint32_t) ((1<<DEBOUNCE_CYCLE)-1))
uint8_t AccelStepperT::runHome(home_struct_t * home_struct) {
	int s = ((home_struct->iopHome->port.reg & home_struct->bitHome) != 0)? 1: 0;
	home_struct->current = ((((home_struct->current) << 1) & (0xFFFFFFFE)) | ((s) & 0x01));
	//home_struct->current = ((((home_struct->current & 0x07) << 1) & 0x0E) | ((digitalRead(home_struct->pin)) & 0x01));
	//LOG.println(home_struct->current);
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
			if (((home_struct->current) & (DEBOUNCE_BITMASK)) != (DEBOUNCE_BITMASK)){
				home = HOME_TOWARD;
				if (speed() != home_struct->toward_move){
					setSpeed(home_struct->toward_move);
				}
				runSpeed();
				return 1;
			}else{
				setSpeed(0.0f);
				_async_runtype = T_STOP;
#ifdef HOME_DEBUG
				LOG << "setCurrentPosition" << home_struct->position << ":" ;
#endif
				setCurrentPosition(home_struct->position);
				LOG << currentPosition() << endl;
				home = HOME_DONE;
			}
		}else if (home_struct->invert == true){//active 0
			if ((home_struct->current) & (DEBOUNCE_BITMASK)){
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
			if (((home_struct->current) & (DEBOUNCE_BITMASK)) != 0x00){
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
			if (((home_struct->current) & (DEBOUNCE_BITMASK)) != (DEBOUNCE_BITMASK)){
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
		HomeF.iopHome = (p32_ioport *)portRegisters(port);
		HomeF.bitHome = digitalPinToBitMask(pin);
		HomeF.pin = pin;
		HomeF.toward_move = toward_move;
		HomeF.leave_move = leave_move;
		HomeF.invert = invert;
		HomeF.position = position;
#ifdef HOME_DEBUG
		LOG << "HomeF Position" << HomeF.position << endl;
#endif
	}else if (mode == HOME_R){//Home R
		uint8_t port;
		if ((pin >= NUM_DIGITAL_PINS) || ((port = digitalPinToPort(pin)) == NOT_A_PIN)) {
			return false;
		}
		HomeR.iopHome = (p32_ioport *)portRegisters(port);
		HomeR.bitHome = digitalPinToBitMask(pin);
		HomeR.pin = pin;
		HomeR.toward_move = toward_move;
		HomeR.leave_move = leave_move;
		HomeR.invert = invert;
		HomeR.position = position;
#ifdef HOME_DEBUG
		LOG << "HomeR Position" << HomeR.position <<endl;
#endif
	}
	return true;
}

_async_homestate_t AccelStepperT::Homestatus(){
	return home;
}

void AccelStepperT::resetHomestatus(){
	home = HOME_START;
}

bool AccelStepperT::isRunning(){
	if (_mode == VMODE){
		return !(speed() == 0.0);
	}else if (_mode == PMODE){
		return AccelStepper::isRunning();
	}
}