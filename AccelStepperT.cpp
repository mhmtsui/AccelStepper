#include <cpudefs.h>
#include <Streaming.h>
#include "AccelStepperT.h"
//#include "../SMC/SMC.h"

//AccelStepperT *stepper_list[MAX_STEPPER_NUM];
//uint8_t stepper_list_num = 0;

//bool timer_has_init = false;
//static volatile bool timer_started = false;
//static volatile bool timer_needstart = false;

// #if defined (__PIC32MX3XX__)
// void __attribute__((USER_ISR)) timer_isr(void) {
// #elif defined (__PIC32MZXX__)
// void __attribute__((nomips16,at_vector(_TIMER_4_VECTOR),interrupt(IPL4SRS))) timer_isr(void) {
// #else
// void __attribute__((USER_ISR)) timer_isr(void) {
// #endif
// 	IFS0CLR = _IFS0_T4IF_MASK;
// 	// clearIntFlag(_TIMER_4_IRQ);
// 	// _LATB14 = 1;
// 	uint8_t cnt = 0;
// 	for (int i = 0; i < stepper_list_num; i++) {
// 		if (stepper_list[i]->_async_runtype == AccelStepperT::T_RUN) {
// 			if (stepper_list[i]->run()) {
// 				cnt++;
// 			} else {
// 				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
// 			}
// 		} else if (stepper_list[i]->_async_runtype == AccelStepperT::T_RUNSPEED) {
// 			if (stepper_list[i]->speed() != 0.0) {
// 				stepper_list[i]->runSpeed();
// 				cnt++;
// 			} else {
// 				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
// 			}
// 		} else if (stepper_list[i]->_async_runtype == AccelStepperT::T_RUNSPEEDACCEL) {
// 			//if (stepper_list[i]->targetSpeed() != 0.0) {
// 			if (stepper_list[i]->runSpeedwithAcceleration()) {
// 				cnt++;
// 			} else {
// 				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
// 			}
// 		} else if (stepper_list[i]->_async_runtype == AccelStepperT::T_HOME_F) {
// 			if (stepper_list[i]->runHome(&(stepper_list[i]->HomeF))) {
// 				cnt++;
// 			} else {
// 				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
// 			}
// 		} else if (stepper_list[i]->_async_runtype == AccelStepperT::T_HOME_R) {
// 			if (stepper_list[i]->runHome(&(stepper_list[i]->HomeR))) {
// 				cnt++;
// 			} else {
// 				stepper_list[i]->_async_runtype = AccelStepperT::T_STOP;
// 			}
// 		} 
// 	}
// 	if (cnt == 0 && !(timer_needstart)) {
// 		// timer_stop();
// 		timer_started = false;
// 		IEC0CLR = _IEC0_T4IE_MASK;
// 		T4CONbits.TON = 0;
// 	}
// 	timer_needstart = false;
// 	// _LATB14 = 0;
// }

AccelStepperT::AccelStepperT(uint8_t step, uint8_t dir) : AccelStepper(AccelStepper::DRIVER, step, dir, 255, 255) {
	//if (stepper_list_num < MAX_STEPPER_NUM) {
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

	//	stepper_list[stepper_list_num++] = this;
	//	if (!timer_has_init) {
	//		timer_init();
	//		timer_has_init = true;
	//	}
	//}
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
	//Serial << "ACCEL Step" << endl;
	if (_direction ^ _pinInverted[1]) {
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

#define DEBOUNCE_CYCLE (8)
//#define MASK_1 (0xFE)
#define DEBOUNCE_BITMASK ((1<<DEBOUNCE_CYCLE)-1)
uint8_t AccelStepperT::runHome(home_struct_t * home_struct) {
	int s = ((home_struct->iopHome->port.reg & home_struct->bitHome) != 0)? 1: 0;
	home_struct->current = ((((home_struct->current & (DEBOUNCE_BITMASK)) << 1) & (DEBOUNCE_BITMASK << 1)) | ((s) & 0x01));
	//home_struct->current = ((((home_struct->current & 0x07) << 1) & 0x0E) | ((digitalRead(home_struct->pin)) & 0x01));
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
				Serial << "setCurrentPosition" << home_struct->position << ":" ;
				setCurrentPosition(home_struct->position);
				Serial << currentPosition() << endl;
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
		Serial << "HomeF Position" << HomeF.position << endl;
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
		Serial << "HomeR Position" << HomeR.position <<endl;
	}
	return true;
}

_async_homestate_t AccelStepperT::Homestatus(){
	return home;
}

bool AccelStepperT::isRunning(){
	if (_mode == VMODE){
		return !(speed() == 0.0);
	}else if (_mode == PMODE){
		return AccelStepper::isRunning();
	}
}