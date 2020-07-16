/*
 * stepper.h
 *
 *  Created on: 11 sty 2020
 *      Author: MSZ
 */

#ifndef STEPPER_H_
#define STEPPER_H_





/* SOME INFO
 *
 * If motor has 3.75 degree for pulse, one full turn
 * will be 360 / 3.75 = 96 pulses, but calling
 * function stepper_step(M1, 96) gives you half turn,
 * because this function takes half pulses as second argument.
 * So, to make one full turn, you have to call it like that:
 * stepper_step(M1, 2 * 96);
 * Of course this is when your driver is set up for one pulse,
 * otherwise:
 * stepper_step(M1, 2 * 96 * driver_pulse_factor);
 *
 *
 * */




#include <math.h>









#define forward 1
#define backward 0

typedef struct {
	GPIO step, dir, enable;
	int32_t steps;
	int32_t prescaler;
	uint32_t counter;
} _stepper;

typedef _stepper* stepper;


volatile stepper __steppers[10];
int __steppersCount = 0;
uint32_t __stepper_frequency = 0;


//stepper_initMotor(TIMER, DIR_PIN, STEP_PIN, ENABLE_PIN)
stepper stepper_createMotor(GPIO step, GPIO dir, GPIO enable) {
	io_init(step.port);
	io_init(dir.port);
	io_init(enable.port);
	io_out(step);
	io_out(dir);
	io_out(enable);
	io_low(dir);
	io_low(enable);
	io_speedHigh(step);
	io_speedHigh(dir);
	io_speedHigh(enable);
	stepper s = malloc(sizeof(_stepper));
	s->step = step;
	s->dir = dir;
	s->enable = enable;
	s->steps = 0;
	s->prescaler = 1;
	s->counter = 0;
	__steppers[__steppersCount++] = s;
	return s;
}


void __stepper_stepMotors() {
	for(int x = 0;x < __steppersCount;x++) {
		stepper s = __steppers[x];
		if(s->steps == 0) continue;
		if(s->counter > 0) {
			s->counter--;
			continue;
		}
		s->counter = s->prescaler - 1;
		s->steps--;
		io_toggle(s->step);
	}
}


//Function called with frequency of 1MHz / second argument whose name is "prescaler"
//recommended value of second argument is 10
void stepper_initTimer(TIM_TypeDef *timer, uint16_t prescaler) {
	__stepper_frequency = (uint32_t)round(1000000.0 / prescaler);
	TIM_repeat(timer, __F_CPU_MHZ, prescaler, __stepper_stepMotors);
}


void stepper_run(stepper motor, int32_t steps, uint32_t stepsPerSecond) {
	motor->prescaler = round(__stepper_frequency / (double)stepsPerSecond);
	steps < 0 ? io_high(motor->dir) : io_low(motor->dir);
	motor->counter = 0;
	motor->steps = abs(steps);
}

void stepper_setSpeed(stepper motor, uint32_t stepsPerSecond) {
	if(stepsPerSecond == 0) motor->prescaler = round(__stepper_frequency / (double)stepsPerSecond);
	else motor->prescaler = round(__stepper_frequency / (double)stepsPerSecond);
	motor->counter = motor->prescaler;
}

void stepper_hold(stepper motor) {
	io_low(motor->enable);
}

void stepper_release(stepper motor) {
	io_high(motor->enable);
}

int32_t stepper_getSteps(stepper motor) {
	return motor->steps;
}






/*
typedef struct {
	GPIO step, dir, enable;
	TIM_TypeDef *timer;
	int32_t steps;
	int32_t prescaler;
	uint32_t counter;
	uint8_t pwmAltFunction;
} _stepper;


void stepper_run(stepper motor, uint8_t direction, float ticksPerSecond) {
	direction == forward ? io_low(motor->dir) : io_high(motor->dir);
	io_altF(motor->step, motor->pwmAltFunction);
	motor->timer->PSC = __F_CPU_MHZ * 1000000 / (65535 * ticksPerSecond); //prescaler
	motor->timer->ARR = __F_CPU_MHZ * 1000000 / ((motor->timer->PSC + 1) * ticksPerSecond); //limit
	motor->timer->CCR1 = motor->timer->ARR / 2; //threshold when to change pin state to 0
	if(motor->timer == TIM1) motor->timer->EGR |= TIM_EGR_UG;
	motor->timer->CR1 |= TIM_CR1_CEN;
}


//stepper_initMotor(TIMER, DIR_PIN, STEP_PIN, ENABLE_PIN)
stepper stepper_createMotor(TIM_TypeDef *timer, uint8_t pwmAltFunction, GPIO step, GPIO dir, GPIO enable) {
	io_init(step.port);
	io_init(dir.port);
	io_init(enable.port);
	io_out(step);
	io_out(dir);
	io_out(enable);
	io_low(dir);
	io_low(enable);
	io_altF(step, pwmAltFunction);
	TIM_wakeUp(timer);
	timer->PSC = 0;
	timer->ARR = 0;
	timer->CCR1 = 0;
	timer->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
	timer->CCER = TIM_CCER_CC1E;
	timer->BDTR = TIM_BDTR_MOE;
	timer->CNT = 0;
	stepper s = malloc(sizeof(stepper));
	_stepper t = {step, dir, enable, timer, 1, 0, pwmAltFunction};
	*s = t;
	__steppers[__steppersCount++] = s;
	return s;
}


 */












#endif /* STEPPER_H_ */
