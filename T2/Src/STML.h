/*
 * STML.h
 *	VERSION 1.5.3
 *  Created on: Jul 6, 2019
 *  by MSZ!
 */

#ifndef STML_H_
#define STML_H_

//#define F_CPU_MHZ 8
//#define STM32F030x6
//#include <stm32f030x6.h>
//#include "STML.h"



















//Pins for UART and SPI

//UART_init(9600, 100, PA10, 1, PA9, 1, RXD);
//right RX - 3rd from up, TX - 4th from up

//UART_init(9600, 100, PB6, 0, PB7, 0, RXD);
//right RX - 4th from down, TX 3rd from down

//#define MOSI_PA7 {GPIOA, 7, 0}
//#define MISO_PA6 {GPIOA, 7, 0}
//#define SCK_PA5 {GPIOA, 7, 0}
//#define NSS_PA4 {GPIOA, 7, 0}






















typedef struct {
	GPIO_TypeDef* port;
	uint8_t pin;
} GPIO;

typedef GPIO_TypeDef* PORT;

PORT A = GPIOA;
PORT B = GPIOB;
PORT C = GPIOC;
PORT D = GPIOD;
PORT F = GPIOF;

//Thanks to this, you can write simply PA7 instead of A, 10 or worse GPIOA, 10 in function argument, I think it's a great improvement
GPIO PA0 = {GPIOA, 0};  GPIO PB0 = {GPIOB, 0};  GPIO PC0 = {GPIOC, 0};  GPIO PD0 = {GPIOD, 0};  GPIO PF0 = {GPIOF, 0};
GPIO PA1 = {GPIOA, 1};  GPIO PB1 = {GPIOB, 1};  GPIO PC1 = {GPIOC, 1};  GPIO PD1 = {GPIOD, 1};  GPIO PF1 = {GPIOF, 1};
GPIO PA2 = {GPIOA, 2};  GPIO PB2 = {GPIOB, 2};  GPIO PC2 = {GPIOC, 2};  GPIO PD2 = {GPIOD, 2};  GPIO PF2 = {GPIOF, 2};
GPIO PA3 = {GPIOA, 3};  GPIO PB3 = {GPIOB, 3};  GPIO PC3 = {GPIOC, 3};  GPIO PD3 = {GPIOD, 3};  GPIO PF3 = {GPIOF, 3};
GPIO PA4 = {GPIOA, 4};  GPIO PB4 = {GPIOB, 4};  GPIO PC4 = {GPIOC, 4};  GPIO PD4 = {GPIOD, 4};  GPIO PF4 = {GPIOF, 4};
GPIO PA5 = {GPIOA, 5};  GPIO PB5 = {GPIOB, 5};  GPIO PC5 = {GPIOC, 5};  GPIO PD5 = {GPIOD, 5};  GPIO PF5 = {GPIOF, 5};
GPIO PA6 = {GPIOA, 6};  GPIO PB6 = {GPIOB, 6};  GPIO PC6 = {GPIOC, 6};  GPIO PD6 = {GPIOD, 6};  GPIO PF6 = {GPIOF, 6};
GPIO PA7 = {GPIOA, 7};  GPIO PB7 = {GPIOB, 7};  GPIO PC7 = {GPIOC, 7};  GPIO PD7 = {GPIOD, 7};  GPIO PF7 = {GPIOF, 7};
GPIO PA8 = {GPIOA, 8};  GPIO PB8 = {GPIOB, 8};  GPIO PC8 = {GPIOC, 8};  GPIO PD8 = {GPIOD, 8};  GPIO PF8 = {GPIOF, 8};
GPIO PA9 = {GPIOA, 9};  GPIO PB9 = {GPIOB, 9};  GPIO PC9 = {GPIOC, 9};  GPIO PD9 = {GPIOD, 9};  GPIO PF9 = {GPIOF, 9};
GPIO PA10 = {GPIOA, 10};GPIO PB10 = {GPIOB, 10};GPIO PC10 = {GPIOC, 10};GPIO PD10 = {GPIOD, 10};GPIO PF10 = {GPIOF, 10};
GPIO PA11 = {GPIOA, 11};GPIO PB11 = {GPIOB, 11};GPIO PC11 = {GPIOC, 11};GPIO PD11 = {GPIOD, 11};GPIO PF11 = {GPIOF, 11};
GPIO PA12 = {GPIOA, 12};GPIO PB12 = {GPIOB, 12};GPIO PC12 = {GPIOC, 12};GPIO PD12 = {GPIOD, 12};GPIO PF12 = {GPIOF, 12};
GPIO PA13 = {GPIOA, 13};GPIO PB13 = {GPIOB, 13};GPIO PC13 = {GPIOC, 13};GPIO PD13 = {GPIOD, 13};GPIO PF13 = {GPIOF, 13};
GPIO PA14 = {GPIOA, 14};GPIO PB14 = {GPIOB, 14};GPIO PC14 = {GPIOC, 14};GPIO PD14 = {GPIOD, 14};GPIO PF14 = {GPIOF, 14};
GPIO PA15 = {GPIOA, 15};GPIO PB15 = {GPIOB, 15};GPIO PC15 = {GPIOC, 15};GPIO PD15 = {GPIOD, 15};GPIO PF15 = {GPIOF, 15};
GPIO PA16 = {GPIOA, 16};GPIO PB16 = {GPIOB, 16};GPIO PC16 = {GPIOC, 16};GPIO PD16 = {GPIOD, 16};GPIO PF16 = {GPIOF, 16};
GPIO PA17 = {GPIOA, 17};GPIO PB17 = {GPIOB, 17};GPIO PC17 = {GPIOC, 17};GPIO PD17 = {GPIOD, 17};GPIO PF17 = {GPIOF, 17};
GPIO PA18 = {GPIOA, 18};GPIO PB18 = {GPIOB, 18};GPIO PC18 = {GPIOC, 18};GPIO PD18 = {GPIOD, 18};GPIO PF18 = {GPIOF, 18};
GPIO PA19 = {GPIOA, 19};GPIO PB19 = {GPIOB, 19};GPIO PC19 = {GPIOC, 19};GPIO PD19 = {GPIOD, 19};GPIO PF19 = {GPIOF, 19};
GPIO PA20 = {GPIOA, 20};GPIO PB20 = {GPIOB, 20};GPIO PC20 = {GPIOC, 20};GPIO PD20 = {GPIOD, 20};GPIO PF20 = {GPIOF, 20};
GPIO PA21 = {GPIOA, 21};GPIO PB21 = {GPIOB, 21};GPIO PC21 = {GPIOC, 21};GPIO PD21 = {GPIOD, 21};GPIO PF21 = {GPIOF, 21};
GPIO PA22 = {GPIOA, 22};GPIO PB22 = {GPIOB, 22};GPIO PC22 = {GPIOC, 22};GPIO PD22 = {GPIOD, 22};GPIO PF22 = {GPIOF, 22};
GPIO PA23 = {GPIOA, 23};GPIO PB23 = {GPIOB, 23};GPIO PC23 = {GPIOC, 23};GPIO PD23 = {GPIOD, 23};GPIO PF23 = {GPIOF, 23};
GPIO PA24 = {GPIOA, 24};GPIO PB24 = {GPIOB, 24};GPIO PC24 = {GPIOC, 24};GPIO PD24 = {GPIOD, 24};GPIO PF24 = {GPIOF, 24};
GPIO PA25 = {GPIOA, 25};GPIO PB25 = {GPIOB, 25};GPIO PC25 = {GPIOC, 25};GPIO PD25 = {GPIOD, 25};GPIO PF25 = {GPIOF, 25};
GPIO PA26 = {GPIOA, 26};GPIO PB26 = {GPIOB, 26};GPIO PC26 = {GPIOC, 26};GPIO PD26 = {GPIOD, 26};GPIO PF26 = {GPIOF, 26};
GPIO PA27 = {GPIOA, 27};GPIO PB27 = {GPIOB, 27};GPIO PC27 = {GPIOC, 27};GPIO PD27 = {GPIOD, 27};GPIO PF27 = {GPIOF, 27};
GPIO PA28 = {GPIOA, 28};GPIO PB28 = {GPIOB, 28};GPIO PC28 = {GPIOC, 28};GPIO PD28 = {GPIOD, 28};GPIO PF28 = {GPIOF, 28};
GPIO PA29 = {GPIOA, 29};GPIO PB29 = {GPIOB, 29};GPIO PC29 = {GPIOC, 29};GPIO PD29 = {GPIOD, 29};GPIO PF29 = {GPIOF, 29};
GPIO PA30 = {GPIOA, 30};GPIO PB30 = {GPIOB, 30};GPIO PC30 = {GPIOC, 30};GPIO PD30 = {GPIOD, 30};GPIO PF30 = {GPIOF, 30};
GPIO PA31 = {GPIOA, 31};GPIO PB31 = {GPIOB, 31};GPIO PC31 = {GPIOC, 31};GPIO PD31 = {GPIOD, 31};GPIO PF31 = {GPIOF, 31};




//for malloc
#include <stdlib.h>

#ifdef F_CPU_MHZ
volatile uint8_t __F_CPU_MHZ = F_CPU_MHZ;
#else
volatile uint8_t __F_CPU_MHZ = 8;
#endif




//================================================ DELAY
/* STM has a system timer SysTick
 * counting from some number to 0
 * and when reaches 0 calls interrupt
 * void SysTick_Handler() {}
 * */
void SysTick_Handler() {SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;}

//Sleeps for given cycles
void sleep(uint16_t cycles) {
	if(cycles == 0) return;
	SysTick_Config(cycles);
	NVIC_SetPriority(SysTick_IRQn, 0);
	while(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk);
}

void delay_us(uint32_t dt_us) {
	if(dt_us == 0) return;
	SysTick_Config(__F_CPU_MHZ * dt_us);
	NVIC_SetPriority(SysTick_IRQn, 0);
	while(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk);
}

void delay_ms(uint32_t dt_ms) {
	if(dt_ms == 0) return;
	int one_ms = __F_CPU_MHZ * 1000;
	for(register int x = 0;x < dt_ms;x++) {
		SysTick_Config(one_ms);
		NVIC_SetPriority(SysTick_IRQn, 0);
		while(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk);
	}
}

void delay_s(uint32_t dt_s) {
	for(register int x = 0;x < dt_s;x++) delay_ms(1000);
}

uint32_t getTime() {return SysTick->VAL;}
//================================================ END OF DELAY




//========================================================================== CLOCK

/*Sets system clock to 48MHz using internal 8MHz RC multiplied by PLL*/
/*DON'T FORGET TO POWER VDDA (pin 28 k6)*/
void CLOCK_internal48mhz() {
	RCC->CFGR &= RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; //Set PLL as system clock
	RCC->CFGR &=~ RCC_CFGR_PLLMUL;
	RCC->CFGR |= RCC_CFGR_PLLMUL12; //Mul (HSI/2) by 12 to get 48MHz
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)); //Wait for PLL start
	__F_CPU_MHZ = 48;
}
/* Internal 8MHz oscillator frequency can be
 * multiplied by PLL, before it enters PLL,
 * it's divider by 2, then multiplied by 12 gives 48MHz
 * */

void CLOCK_default8mhz() {
	RCC->CFGR &= RCC_CFGR_SW;
	RCC->CFGR &=~ RCC_CFGR_PLLMUL;
	RCC->CR &=~ RCC_CR_PLLON;
	__F_CPU_MHZ = 8;
}

//========================================================================== CLOCK END




//========================================================================== IO
//================================================ RCC
void io_init(PORT port) {
	if(port == A) RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	else if(port == B) RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	else if(port == C) RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	else if(port == D) RCC->AHBENR |= RCC_AHBENR_GPIODEN;
	else if(port == F) RCC->AHBENR |= RCC_AHBENR_GPIOFEN;
}
void io_off(PORT port) {
	if(port == A) RCC->AHBENR &=~ RCC_AHBENR_GPIOAEN;
	else if(port == B) RCC->AHBENR &=~ RCC_AHBENR_GPIOBEN;
	else if(port == C) RCC->AHBENR &=~ RCC_AHBENR_GPIOCEN;
	else if(port == D) RCC->AHBENR &=~ RCC_AHBENR_GPIODEN;
	else if(port == F) RCC->AHBENR &=~ RCC_AHBENR_GPIOFEN;
}
//================================================ MODER
#define noPull   0
#define pullUp   1
#define pullDown 2
//io_in(PA0, pullUp); sets PA0 as input with pullup
void io_in(GPIO pin, uint8_t pull) {
	pin.port->MODER &=~ (3 << (pin.pin * 2));
	if(pull == noPull) pin.port->PUPDR &=~ (3 << (pin.pin * 2));
	else if(pull == pullDown) {pin.port->PUPDR &=~ (1 << (pin.pin * 2));pin.port->PUPDR |= 2 << (pin.pin * 2);}
	else if(pull == pullUp) {pin.port->PUPDR &=~ (2 << (pin.pin * 2));pin.port->PUPDR |= 1 << (pin.pin * 2);}
}
void io_out(GPIO pin) {pin.port->MODER &=~ (2 << (pin.pin * 2));pin.port->MODER |=  1 << (pin.pin * 2);}
//write directly into moder register, for example io_writeIO(A, 0xFFFF); makes all port A pins output
void io_writeIO(PORT port, uint16_t value) {
	for(int x = 0;x < 16;x++) {
		if(value & (1 << x)) port->MODER |= 1 << (x * 2);
		else port->MODER &=~ (1 << (x * 2));
	}
}
/**Analog mode*/
void io_analogMode(GPIO pin) {pin.port->MODER |= 3 << (pin.pin * 2);}
//================================================ ODR
/**Logic 0*/
void io_low(GPIO pin) {pin.port->ODR &=~ (1 << pin.pin);}
/**Logic 1*/
void io_high(GPIO pin) {pin.port->ODR |= (1 << pin.pin);}
void io_toggle(GPIO pin) {pin.port->ODR ^= (1 << pin.pin);}
/*Writes into the port*/
void io_writeLogic(PORT port, uint16_t value) {port->ODR = value;}
//================================================ IDR
uint16_t io_read(PORT port) {return port->IDR;}
uint8_t io_isHigh(GPIO pin) {return pin.port->IDR & (1 << pin.pin) ? 1 : 0;}
uint8_t io_isLow(GPIO pin) {return pin.port->IDR & (1 << pin.pin) ? 0 : 1;}
//================================================ OSPEEDR
void io_speedLow(GPIO pin) {pin.port->OSPEEDR &=~ (3 << (pin.pin * 2));}
void io_speedMedium(GPIO pin) {pin.port->OSPEEDR &=~ (2 << (pin.pin * 2));pin.port->OSPEEDR |= 1 << (pin.pin * 2);}
void io_speedHigh(GPIO pin) {pin.port->OSPEEDR |= 3 << (pin.pin * 2);}
//================================================ OTYPER
void io_pushPull(GPIO pin) {pin.port->OTYPER &=~ (1 << pin.pin);}
void io_openDrain(GPIO pin) {pin.port->OTYPER |= (1 << pin.pin);}
//================================================ LCKR
void io_lock(GPIO pin) {pin.port->LCKR |= 1 << pin.pin;}
void io_unlock(GPIO pin) {pin.port->LCKR &=~ (1 << pin.pin);}
//================================================ AFR
void io_altF(GPIO pin, uint8_t altFunction) {
	pin.port->MODER &=~ (1 << (pin.pin * 2));pin.port->MODER |=  2 << (pin.pin * 2);
	uint64_t *afr = (uint64_t*)(pin.port->AFR);
	*afr &=~ ((uint64_t)0xF << (pin.pin * 4));
	*afr |= (uint64_t)altFunction << (pin.pin * 4);
}
//================================================ INTERRUPTS
#define RISING 1
#define FALLING 2
#define FALLING_RISING 3
#define RISING_FALLING 3

//Function, that doesn't do anything
void __extIntDefaultFunction() {}
void (*__extIntFunction)() = __extIntDefaultFunction;

void io_setInterrupt(GPIO pin, uint8_t falling_rising, void (*function)()) {
	__extIntFunction = function;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[pin.pin / 4] &=~ (0xF << ((pin.pin % 4) * 4));
	//if(pin.port == A) SYSCFG->EXTICR[pin.pin / 4] &=~ (0x1 << ((pin.pin % 4) * 4));
	//if(pin.port == B) SYSCFG->EXTICR[pin.pin / 4] &=~ (0x2 << ((pin.pin % 4) * 4));
	//if(pin.port == C) SYSCFG->EXTICR[pin.pin / 4] &=~ (0x3 << ((pin.pin % 4) * 4));
	///if(pin.port == D) SYSCFG->EXTICR[pin.pin / 4] &=~ (0x4 << ((pin.pin % 4) * 4));
	//if(pin.port == F) SYSCFG->EXTICR[pin.pin / 4] &=~ (0x5 << ((pin.pin % 4) * 4));
	EXTI->IMR |= 1 << pin.pin;
	if(falling_rising & 1) EXTI->RTSR |= 1 << pin.pin; else EXTI->RTSR &=~ (1 << pin.pin);
	if(falling_rising & 2) EXTI->FTSR |= 1 << pin.pin; else EXTI->FTSR &=~ (1 << pin.pin);
	if(pin.pin < 2) {NVIC_SetPriority(EXTI0_1_IRQn, 2);NVIC_EnableIRQ(EXTI0_1_IRQn);}
	else if(pin.pin < 4) {NVIC_SetPriority(EXTI2_3_IRQn, 2);NVIC_EnableIRQ(EXTI2_3_IRQn);}
	else {NVIC_SetPriority(EXTI4_15_IRQn, 2);NVIC_EnableIRQ(EXTI4_15_IRQn);}
}

void io_disableInterrupt(GPIO pin) {EXTI->IMR &=~ (1 << pin.pin);}

uint8_t io_intFromPin(GPIO pin) {return EXTI->PR & (1 << pin.pin);}

#ifdef __cplusplus
extern "C" {
#endif
void EXTI0_1_IRQHandler() {__extIntFunction();EXTI->PR |= 0xFFFFFFFF;}
void EXTI2_3_IRQHandler() {__extIntFunction();EXTI->PR |= 0xFFFFFFFF;}
void EXTI4_15_IRQHandler() {__extIntFunction();EXTI->PR |= 0xFFFFFFFF;}
#ifdef __cplusplus
}
#endif

//========================================================================== IO END


//========================================================================== TIMERS

//It's used as default timers interrupt function
void __dummy() {}

/* Pointers to functions called on interrupt
 * Concept:
 *  Default functions called on interrupt ("interrupt handlers")
 *  are overrided and they call following special pointers to dummy function.
 *  If you want to make your own interrupt function, define it and change
 *  one of following pointers to point your defined function anywhere.
 *  EXAMPLE:
 *
 *    void myInterruptFunction() {}
 *
 *    int main() {
 *        TIM1_int = myInterruptFunction;
 *    }
 *
 * */
void (*TIM1_int)() = __dummy;
void (*TIM3_int)() = __dummy;
void (*TIM14_int)() = __dummy;
void (*TIM16_int)() = __dummy;
void (*TIM17_int)() = __dummy;

//INTERRUPT HANDLERS OVERRIDE


//C++ INTERRUPTS
/* If code is compiled in C++, there are some problems with
 * functions, that are called, when interrupt happens,
 * this is "interrupt handlers" or "callbacks"
 * So, we have to put those functions in special following braces:
 * extern "C" {}
 * */
#ifdef __cplusplus
extern "C" {
#endif

void TIM1_CC_IRQHandler() {
	TIM1_int();
	TIM1->SR = 0x0000;
}

void TIM1_BRK_UP_TRG_COM_IRQHandler() {
	TIM1_int();
	TIM1->SR = 0x0000;
}

void TIM3_IRQHandler() {
	TIM3_int();
	TIM3->SR = 0x0000;
}

void TIM14_IRQHandler() {
	TIM14_int();
	TIM14->SR = 0x0000;
}

void TIM16_IRQHandler() {
	TIM16_int();
	TIM16->SR = 0x0000;
}

void TIM17_IRQHandler() {
	TIM17_int();
	TIM17->SR = 0x0000;
}

#ifdef __cplusplus
}
#endif
//END OF INTERRUPT HANDLERS OVERRIDE

//FUNCTIONS THAT HAVE DIFFERENT GUTS FOR EVERY TIMER, BUT SAME PURPOSE

/* FUNCTION FOR CONNECTING TIMERS TO CLOCK
 * Unfortunetly, every timer has it's special formula for this purpose,
 * so, there is a need for creating one function, that wakes up desired timer.
 * */
void TIM_wakeUp(TIM_TypeDef *timer) {
	if     (timer == TIM1 ) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(timer == TIM3 ) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(timer == TIM14) RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	else if(timer == TIM16) RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	else if(timer == TIM17) RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
}

/* FUNCTION FOR ENABLING TIMER INTERRUPT
 * Every timer has it's own recipe for enabling interrupts,
 * so, following function detects given timer and uses
 * corresponding way to do it.
 *
 * ARGUMENTS:
 * void TIM_interruptEnable(TIM_TypeDef *timer, uint8_t priority, void (*function)())
 *
 * timer - timer, which interrupts have to be enabled, for example TIM1
 * priority - number from 0 to 3 (SysTick has 0, UART has 1)
 * function - name of function to be called on interrupt
 * */
void TIM_interruptEnable(TIM_TypeDef *timer, uint8_t priority, void (*function)()) {
	if(timer == TIM1) {
		TIM1_int = function;
		NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
		NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, priority);
		NVIC_EnableIRQ(TIM1_CC_IRQn);
		NVIC_SetPriority(TIM1_CC_IRQn, priority);
	}
	else if(timer == TIM3) {
		TIM3_int = function;
		NVIC_EnableIRQ(TIM3_IRQn);
		NVIC_SetPriority(TIM3_IRQn, priority);
	}
	else if(timer == TIM14) {
		TIM14_int = function;
		NVIC_EnableIRQ(TIM14_IRQn);
		NVIC_SetPriority(TIM14_IRQn, priority);
	}
	else if(timer == TIM16) {
		TIM16_int = function;
		NVIC_EnableIRQ(TIM16_IRQn);
		NVIC_SetPriority(TIM16_IRQn, priority);
	}
	else if(timer == TIM17) {
		TIM17_int = function;
		NVIC_EnableIRQ(TIM17_IRQn);
		NVIC_SetPriority(TIM17_IRQn, priority);
	}
}

//END OF FUNCTIONS THAT HAVE DIFFERENT GUTS FOR EVERY TIMER, BUT SAME PURPOSE



void TIM_countTo(TIM_TypeDef *timer, uint16_t limit, uint16_t prescaler) {
	TIM_wakeUp(timer);
	timer->PSC = prescaler - 1;
	timer->ARR = limit;
	timer->CNT = 0;
	timer->CR1 = TIM_CR1_CEN;
}

void TIM_countFrom(TIM_TypeDef *timer, uint16_t limit, uint16_t prescaler) {
	TIM_wakeUp(timer);
	timer->PSC = prescaler - 1;
	timer->ARR = limit;
	timer->CNT = limit;
	timer->CR1 = TIM_CR1_CEN | TIM_CR1_DIR;
}

void TIM_start(TIM_TypeDef *timer) {
	timer->CR1 &=~ TIM_CR1_CEN;
}

void TIM_stop(TIM_TypeDef *timer) {
	timer->CR1 |= TIM_CR1_CEN;
}

uint16_t TIM_getValue(TIM_TypeDef *timer) {
	return timer->CNT;
}

void TIM_setValue(TIM_TypeDef *timer, uint16_t value) {
	timer->CNT = value;
}



/* This is function, that tells timer to repeat funtion
 * at equal intervals of time.
 * It starts immediately.
 *
 * ARGUMENTS:
 * TIM_repeat(TIM_TypeDef *timer, uint16_t limit, uint16_t prescaler, void (*function)())
 *
 * timer - timer given for this purpose (example: TIM1)
 * limit - limit to which the timer has to count (0 - 65535)
 * prescaler - timer will count with frequency of cpu divided by prescaler value (0 - 65535)
 * function - name of function to be called
 * */
void TIM_repeat(TIM_TypeDef *timer, uint16_t limit, uint16_t prescaler, void (*function)()) {
	TIM_wakeUp(timer);
	TIM_interruptEnable(timer, 1, function);
	timer->PSC = prescaler - 1;
	timer->ARR = limit;
	timer->DIER = TIM_DIER_UIE;
	timer->CNT = 0;
	timer->CR1 = TIM_CR1_CEN;
}


//========================================================================== TIMERS END



//========================================================================== T1

void T1_countTo(uint16_t limit, uint16_t prescaler) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->PSC = prescaler - 1;
	TIM1->ARR = limit;
	TIM1->CNT = 0;
	TIM1->CR1 = TIM_CR1_CEN;
}

void T1_countFrom(uint16_t limit, uint16_t prescaler) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->PSC = prescaler - 1;
	TIM1->ARR = limit;
	TIM1->CNT = limit;
	TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_DIR;
}

void T1_resume() {
	TIM1->CR1 |= TIM_CR1_CEN;
}

void T1_pause() {
	TIM1->CR1 &=~ TIM_CR1_CEN;
}

uint16_t T1_getValue() {
	return TIM1->CNT;
}

void T1_setValue(uint16_t count) {
	TIM1->CNT = count;
}

uint8_t T1_isWorking() {
	return (TIM1->CR1 & TIM_CR1_CEN) ? 1 : 0;
}

void T1_repeat(uint16_t limit, uint16_t prescaler, void (*function)()) {
	TIM1_int = function;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->PSC = prescaler - 1;
	TIM1->ARR = limit;
	TIM1->DIER = TIM_DIER_UIE;
	TIM1->CNT = 0;
	TIM1->CR1 = TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
}

//========================================================================== T1 END



//========================================================================== T3

void T3_countTo(uint16_t limit, uint16_t prescaler) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = prescaler - 1;
	TIM3->ARR = limit;
	TIM3->CNT = 0;
	TIM3->CR1 = TIM_CR1_CEN;
}

void T3_countFrom(uint16_t limit, uint16_t prescaler) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = prescaler - 1;
	TIM3->ARR = limit;
	TIM3->CNT = limit;
	TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_DIR;
}

void T3_resume() {
	TIM3->CR1 |= TIM_CR1_CEN;
}

void T3_pause() {
	TIM3->CR1 &=~ TIM_CR1_CEN;
}

uint16_t T3_getValue() {
	return TIM3->CNT;
}

void T3_setValue(uint16_t count) {
	TIM3->CNT = count;
}

uint8_t T3_isWorking() {
	return (TIM3->CR1 & TIM_CR1_CEN) ? 1 : 0;
}


//function given in argument is called everytime when timer counts to limit, T3_stop() stops it
void T3_repeat(uint16_t limit, uint16_t prescaler, void (*function)()) {
	TIM3_int = function;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->CNT = 0;
	TIM3->PSC = prescaler - 1;
	TIM3->ARR = limit;
	TIM3->DIER = TIM_DIER_UIE;
	TIM3->CR1 = TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 3);
}

//========================================================================== T3 END




//========================================================================== T14

void T14_countTo(uint16_t limit, uint16_t prescaler) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->PSC = prescaler - 1;
	TIM14->ARR = limit;
	TIM14->CNT = 0;
	TIM14->CR1 = TIM_CR1_CEN;
}

void T14_countFrom(uint16_t limit, uint16_t prescaler) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->PSC = prescaler - 1;
	TIM14->ARR = limit;
	TIM14->CNT = limit;
	TIM14->CR1 = TIM_CR1_CEN | TIM_CR1_DIR;
}

void T14_resume() {
	TIM14->CR1 |= TIM_CR1_CEN;
}

void T14_pause() {
	TIM14->CR1 &=~ TIM_CR1_CEN;
}

uint16_t T14_getValue() {
	return TIM14->CNT;
}

void T14_setValue(uint16_t count) {
	TIM14->CNT = count;
}

uint8_t T14_isWorking() {
	return (TIM14->CR1 & TIM_CR1_CEN) ? 1 : 0;
}


//function given in argument is called everytime when timer counts to limit, T14_stop() stops it
void T14_repeat(uint16_t limit, uint16_t prescaler, void (*function)()) {
	TIM14_int = function;
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	TIM14->CNT = 0;
	TIM14->PSC = prescaler - 1;
	TIM14->ARR = limit;
	TIM14->DIER = TIM_DIER_UIE;
	TIM14->CR1 = TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM14_IRQn);
	NVIC_SetPriority(TIM14_IRQn, 0);
}

//========================================================================== T3 END




//========================================================================== UART1

char *UART_buffer = 0;
uint16_t UART_bufSize = 0;
uint16_t UART_dataSize = 0;
uint8_t UART_dataReceived = 0;
int __UART_dataRead = 0;

//Functions for interrupts

/*Data received*/
//Function, that doesn't do anything
void __UART_defaultRXDFunction(char* data, int size) {}

//This pointer is called, when data package is fully received
void (*__UART_RXDFunction)(char *data, int size) = __UART_defaultRXDFunction;


//extern "C" {} must be if code is compiled as C++, otherwise callbacks completely do not work :/
#ifdef __cplusplus
extern "C" {
#endif

void USART1_IRQHandler() {
	if(USART1->ISR & USART_ISR_RXNE) {	//One byte received
		UART_buffer[UART_dataSize++] = USART1->RDR;
		UART_buffer[UART_dataSize] = 0;	//For easy strings read
		UART_dataReceived = 0;
	}
	//Timeout happened, data stopped being received for a timeout time, so it must be an end of data block
	if(USART1->ISR & USART_ISR_RTOF) {UART_dataReceived = 1;__UART_RXDFunction(UART_buffer, UART_dataSize);}
	USART1->ICR = 0xFFFFF;//clear flags to dispose interrupt
}

#ifdef __cplusplus
}
#endif



void UART_init(uint32_t baud, uint16_t bufferSize, GPIO RX_pin, uint8_t RX_altF, GPIO TX_pin, uint8_t TX_altF, void (*RXDFunction)(char *data, int size)) {
	__UART_RXDFunction = RXDFunction != NULL ? RXDFunction : __UART_defaultRXDFunction;
	UART_buffer = malloc(bufferSize + 1);
	UART_bufSize = bufferSize;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = (__F_CPU_MHZ * 1e6) / baud;
	USART1->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RTOIE | USART_CR1_RXNEIE;
	USART1->CR2 = USART_CR2_RTOEN;
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 1);
	io_init(RX_pin.port);io_altF(RX_pin, RX_altF);
	io_init(TX_pin.port);io_altF(TX_pin, TX_altF);
	USART1->RTOR = 8; //timeout 8 bits by default
}

void UART_clearBuffer() {
	UART_dataSize = 0;
}

int UART_dataPacketAvailable() {
	return UART_dataReceived;
}

int UART_available() {
	return UART_dataSize - __UART_dataRead;
}

int UART_readNextByte() {
	int data = UART_dataSize == 0 ? -1 : UART_buffer[__UART_dataRead++];
	if(__UART_dataRead == UART_dataSize) {__UART_dataRead = 0;UART_clearBuffer();}
	return data;
}

void UART_writeByte(char data) {
	char sent = 0;
	while(!sent) if(USART1->ISR & USART_ISR_TXE) {USART1->TDR = data;sent = 1;}
	while(!(USART1->ISR & USART_ISR_TC));
}


void UART_flush() {
	while(!(USART1->ISR & USART_ISR_TXE));
	delay_ms(1);
}

void UART_print(char *data) {
	while(*data) if(USART1->ISR & USART_ISR_TXE) USART1->TDR = *data++;
}

void UART_println(char *data) {
	UART_print(data);
	UART_writeByte('\n');
}


//return number of digits written
int UART_num(int64_t a) {
	if(a == 0) {
		UART_writeByte('0');
		return 1;
	}
	uint8_t neg = a < 0;
	if(neg) {
		a *= -1;
		UART_writeByte('-');
	}
	int n = 1; //digits
	uint64_t c = 9; //compare
	//detects the number of digits
	while(a > c) {
		n++;
		c *= 10;
		c += 9;
	}
	char d[n]; //digits
	//fills d array with digits
	for(int x = 0;x < n;x++) {
		uint8_t b = a % 10;
		a /= 10;
		d[x] = b;
	}
	for(int x = n - 1;x >= 0;x--) UART_writeByte(d[x] + '0');
	if(neg) n++;
	return n;
}

int UART_numln(int64_t number) {
	int n = UART_num(number);
	UART_writeByte('\n');
	return n;
}

int UART_hex(int64_t a) {
	if(a == 0) {
		UART_writeByte('0');
		return 1;
	}
	uint8_t neg = a < 0;
	if(neg) {
		a *= -1;
		UART_writeByte('-');
	}
	int n = 1; //digits
	uint64_t c = 15; //compare
	//detects the number of digits
	while(a > c) {
		n++;
		c <<= 4;
		c += 15;
	}
	char d[n]; //digits
	//fills d array with digits
	for(int x = 0;x < n;x++) {
		uint8_t b = a % 16;
		a >>= 4;
		d[x] = b;
	}
	for(int x = n - 1;x >= 0;x--) {
		if(d[x] < 10) UART_writeByte(d[x] + '0');
		else UART_writeByte(d[x] - 10 + 'A');
	}
	if(neg) n++;
	return n;
}

int UART_hexln(int64_t number) {
	int n = UART_hex(number);
	UART_writeByte('\n');
	return n;
}

void UART_backspace(int times) {
	for(int x = 0;x < times;x++) UART_writeByte(8);
}

void UART_write(char *data, uint16_t size) {
	for(int x = 0;x < size;) if(USART1->ISR & USART_ISR_TXE) {USART1->TDR = *data++;x++;}
	while(!(USART1->ISR & USART_ISR_TC));
}

/**When data is received byte by byte,
 * uart can detect when data block transfer is done.
 * After each byte is received, there is nothing on the lines
 * for a moment. When this moment is longer than a time of
 * a few bits flow, it is a sign, that all bytes have been received.
 * Default timeout is a time corresponding to time of 8 bits flow.
 * UART_setTimeout(int timeout_bits) is used to change this value.
 * */
void UART_setTimeout(int timeout_bits) {
	USART1->RTOR = timeout_bits;
}

//========================================================================== UART1 END





//========================================================================== ADC

//This function has implemented io_init for desired pin
void ADC_init(GPIO pin) {
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->CR2 |= RCC_CR2_HSI14ON;
	while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
	ADC1->CFGR2 |= ADC_CFGR2_CKMODE;
	ADC1->CR |= ADC_CR_ADEN;
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	ADC1->CHSELR = 1 << pin.pin;
	ADC->CCR = ADC_CCR_VREFEN;
	ADC1->SMPR |= 0x7;
	io_init(pin.port);
	io_analogMode(pin);
}
void ADC_disable() {
	ADC1->CR &=~ ADC_CR_ADEN;
}

void ADC_setPin(GPIO pin) {
	ADC1->CHSELR = 1 << pin.pin;
	io_init(pin.port);
	io_analogMode(pin);
}

//max value returned is 4095
uint16_t ADC_sample() {
	ADC1->CR |= ADC_CR_ADSTART;
	while(!(ADC1->ISR & ADC_ISR_EOC));
	return ADC1->DR;
}
//========================================================================== ADC END





//========================================================================== SPI

#define SPI_1n2_fcpu 0
#define SPI_1n4_fcpu 1
#define SPI_1n8_fcpu 2
#define SPI_1n16_fcpu 3
#define SPI_1n32_fcpu 4
#define SPI_1n64_fcpu 5
#define SPI_1n128_fcpu 6
#define SPI_1n256_fcpu 7

#define SPI_MOSI_PA7 0
#define SPI_MISO_PA6 0
#define SPI_SCK_PA5 0
#define SPI_NSS_PA4 0



GPIO __SPI_NSS_PIN = {NULL, 0};
#define __NSS1 __SPI_NSS_PIN.port->ODR |= 1 << __SPI_NSS_PIN.pin;
#define __NSS0 __SPI_NSS_PIN.port->ODR &=~ (1 << __SPI_NSS_PIN.pin);



void SPI_on(uint8_t SPI_baud, uint8_t dataBits, GPIO MOSI_pin, uint8_t MOSI_altF, GPIO MISO_pin, uint8_t MISO_altF, GPIO SCK_pin, uint8_t SCK_altF, GPIO NSS_pin) {

	io_init(MOSI_pin.port);io_altF(MOSI_pin, MOSI_altF);
	io_init(MISO_pin.port);io_altF(MISO_pin, MISO_altF);
	io_init(SCK_pin.port);io_altF(SCK_pin, SCK_altF);
	io_init(NSS_pin.port);__SPI_NSS_PIN = NSS_pin;
	NSS_pin.port->MODER |= 1 << (2 * __SPI_NSS_PIN.pin);__NSS1;

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = SPI_CR1_MSTR | (SPI_baud << SPI_CR1_BR_Pos) | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA;
	SPI1->CR2 = ((dataBits - 1) << SPI_CR2_DS_Pos);
	SPI1->CR1 |= SPI_CR1_SPE;
}



uint16_t SPI_word(uint16_t data) {
	__NSS0;
	if(!(SPI1->SR & SPI_SR_TXE)) return 1 << 15;
	SPI1->DR = data;
	while((SPI1->SR & SPI_SR_RXNE) == 0);
	__NSS1;
	data = SPI1->DR;
	return data;
}

//========================================================================== SPI END


//========================================================================== WDT

//max 26208ms, min 25us
void WDT_init(int time_ms) {
	int p = 0;
	uint64_t limit = time_ms * 10.0;
	while(limit > 4095) {
		limit /= 2;
		p++;
	}
	if(p > 6) {
		p = 6;
		limit = 0xFFF;
	}

	RCC->CSR |= RCC_CSR_LSION;
	while((RCC->CSR & RCC_CSR_LSIRDY) == 0);	//LSI STARTUP 40kHz

	IWDG->KR = 0xCCCC; //turn on watchdog timer
	IWDG->KR = 0x5555; //unlock RLR and PR regs
	IWDG->PR = p;
	IWDG->RLR = limit; //20ms period between resets
	while(IWDG->SR == 0); //wait for watchdog to accept changes in RLR and PR
	IWDG->KR = 0xAAAA; //reload watchdog timer before serious work
}

void WDT_reload() {
	IWDG->KR = 0xAAAA;
}

//========================================================================== WDT END


//========================================================================== UTILS

uint8_t UTIL_charToHex(char c) {
	if((c >= '0') && (c <= '9')) return c - '0';
	else if((c >= 'A') && (c <= 'F')) return 10 + c - 'A';
	else if((c >= 'a') && (c <= 'f')) return 10 + c - 'a';
	else return 16;//ERROR
}

uint16_t UTIL_stringHexToWord(char *s, uint8_t length) {
	int result = 0;
	for(uint8_t x = 0;x < length;x++) {
        uint16_t digit = UTIL_charToHex(s[length - 1 - x]);
        for(int y = 0;y < x;y++) digit *= 16;
		result += digit;
	}
	return result;
}

//number must be ended with character, that is not a digit!!!, example 1234X or 1234_ or just space
//accepts decimal number as string of characters, returns 64 bit value
int64_t UTIL_stringToNum(char *s) {
	int n = 0;
	int64_t a = 0; //converted number
	uint8_t neg = 0;
	if(s[0] == '-') {
		neg = 1;
		s = &s[1];
	}
	while(s[n] >= '0' && s[n] <= '9') n++;
	for(int x = 0;x < n;x++){
		int d = s[x] - '0';
		a += d;
		if(x < n - 1) a *= 10;
	}
	if(neg) a *= -1;
	return a;
}


//number must be ended with character, that is not a digit, including hex digits!!! example 12CFX or 12CFG or just space
//accepts hex number as string of characters, returns 64 bit value
int64_t UTIL_stringToNumHex(char *s) {
	int n = 0;
	int64_t a = 0; //converted number
	uint8_t neg = 0;
	if(s[0] == '-') {
		neg = 1;
		s = &s[1];
	}
	if(s[0] == '0' && s[1] == 'x') s = &s[2];
	while((s[n] >= '0' && s[n] <= '9') || (s[n] >= 'A' && s[n] <= 'F') || (s[n] >= 'a' && s[n] <= 'f')) n++;
	for(int x = 0;x < n;x++){
		int d;
		if(s[x] >= '0' && s[x] <= '9') d = s[x] - '0';
		else if(s[x] >= 'A' && s[x] <= 'F') d = s[x] - 'A' + 10;
		else if(s[x] >= 'a' && s[x] <= 'f') d = s[x] - 'a' + 10;
		a += d;
		if(x < n - 1) a <<= 4;
	}
	if(neg) a *= -1;
	return a;
}

uint16_t UTIL_crc16(uint8_t *req, uint8_t req_length, uint16_t polynomial)
{
    uint8_t j;
    uint16_t crc;

    crc = 0xFFFF;
    while (req_length--) {
        crc = crc ^ *req++;
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ polynomial;
            else
                crc = crc >> 1;
        }
    }
    return (crc << 8 | crc >> 8);
}
//modbus polynomial = 0xA001

//========================================================================== UTILS END










/* HEX VALUES CONVERSIONS
 * If pc writes 0x1234CX (number must be ended with character, that is not a digit, including hex digits!!!)
 * to UART, STM will exchange this value to variable and send value in hex and dec
 *
#include <stm32f0xx.h>
#include "STML.h"

void RXD(char *data, int size) {
	int64_t a = UTIL_stringToNumHex(data);
	UART_hexln(a);
	UART_numln(a);
	UART_clearBuffer();
}

int main() {
	UART_init(115200, 100, PA10, 1, PA9, 1, RXD);
}
 * */


/* EXAMPLE WITH EXTERNAL INTERRUPT AND DIODE
 * W STM'ach przerwania zewnętrzne mają pewną wadę - nie da się rozpoznać portu, z którego dochodzą.
#define STM32F030x6
#include <stm32f030x6.h>
#include "STML.h"

void przerwanie() {							//ta funkcja zostanie wywołana, gdy nastąpi przerwanie
	if(io_intFromPin(PA5)) io_toggle(PB1);	//jeżeli przerwanie pochodzi z pinu PA5, to zmień stan PB1
	while(io_isLow(PA5));					//poczekaj, aż przycisk zostanie puszczony
	delay_ms(10);							//Nie dopuść do wystąpnienia ponownego przerwania przez drgania styków przycisku
}

int main() {

	CLOCK_internal48mhz();						//procesor na 48MHz

	io_init(B);									//włącz port B
	io_init(A);									//włącz port A
	io_out(PB1);								//dioda
	io_in(PA5, pullUp);							//włącza pullUp, czyli wewnętrzny rezystor do VDD na PA5
	io_setInterrupt(PA5, FALLING, przerwanie);	//włącz przerwanie dla podanej funkcji przy zboczu opadającym na PA5 można dać np. FALLING_RISING

	while(1);
}
*/


/* EXAMPLE WITH TIMER USED FOR REPEATING FUNCTION PERIODICALLY (DIODE BLINKING)
 * KIND OF ASYNCHRONOUS TASK

#define STM32F030x6
#include <stm32f030x6.h>
#include "STML.h"

void toggleDiode() {
	io_toggle(PB1);
}

int main() {
	io_init(B);
	io_out(PB1); //diode to PB1
	T3_repeat(1000, 8000, toggleDiode);
	while(1);
}

*/




/*EXAMPLE USE OF TIM1 INTERRUPT (blinking LED)
 *PA7 as GND for diode, PA6 as out for diode

void TIM1_BRK_UP_TRG_COM_IRQHandler(void) {
	io_toggle(PA6);
	TIM1->SR &=~ TIM_SR_UIF;
}

int main() {
	io_init(A);
	io_out(PA7);
	io_out(PA6);

	T1_countTo(10000, 500);
	T1->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	while(1);
}
*/



/*PWM EXAMPLE
 *


RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
io_out(PA6);
GPIOA->MODER &= ~(GPIO_MODER_MODER7_0);
GPIOA->MODER |= GPIO_MODER_MODER7_1;

io_altF(PA7, 2);


TIM1->PSC = 65535;
TIM1->ARR = 100;
TIM1->CCR1 = 10;
TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
TIM1->CCER |= TIM_CCER_CC1NE;//TIM_CCER_CC1E;
TIM1->BDTR |= TIM_BDTR_MOE;
TIM1->CR1 |= TIM_CR1_CEN;


	!!!!!!!!!!!!!!!!!!!!IMPORTANT NOTE!!!!!!!!!!!!!!!!!!!!
 * When TIM1->CNT is updated, you have to generate update event
 * This is done by writing 1 to TIM1->EGR in place of bit UG
 *
 * example
 * TIM1->CNT = 500
 * TIM1->EGR |= TIM_EGR_UG


*/



/* UART EXAMPLE
#define STM32F030x6
#include <stm32f030x6.h>
#include "STML.h"

void dataReceived(char *data, int size) {
	UART_print("\nOdebrano tekst: ");
	UART_write(data, size);
	UART_clearBuffer();
}

int main() {
	//RX pin PA10 with alt function 1 and TX pin PA9 with alt function 1
	UART_init(9600, 100, PA10, 1, PA9, 1, dataReceived);
	while(1);
}
*/




/*ADC EXAMPLE with diode brightness regulated with potentiometer
 * DONT FORGET TO POWER VDDA WHEN USING ADC!!!
#define STM32F030x6
#include <stm32f030x6.h>
#include "STML.h"




int main() {

	//lewa nóżka potencjometru do GND
	//prawa nóżka potencjometru do VCC
	ADC_init(PA5);	//ADC będzie próbkował napięcie na środkowej nóżce potencjometru
	T1_PWM(5000, 8, PB1);	//Wyjście regulacji jasności diody
	//5000, 8, PB1
	//limit = 5000 timer będzie zliczał tylko do 5000 i będzie się zerował
	//prescaler = 8 podziel częstotliwość procesora (8MHz) na 8, więc na timerze jest 1MHz
	//a potem przełączy się automatycznie na stan niski
	//wyjście PWM to pin PB1

	while(1) {
		double duty = ADC_sample();
		//ADC_sample zwraca wartość od 0 do 4095 w zależności od zmierzonego napięcia
		//ADC_sample() to 16 bitowa wartość (od 0 do 4095) napięcia, 4095 oznacza VDD, czyli 3.3V
		//Żeby uniknąć małych wahań napięcia, mniej znaczące bity ADC_sample można uciąć ACD_sample() & 0xFFF0
		T1_setPWM(duty);
		//Tutaj długość trwania wysokiego stanu na PWM jest regulowana zmierzonym napięciem
		//Efektem jest płynna regulacja jasności diody potencjometrem, zupełnie, jakby był do niej podłączony
		//i służył jako regulator napięcia
		delay_ms(50);
	}
}
*/




#define _____append(a, b) a ## b
#define ___append(a, b) _____append(a, b)
#define ___append3(a, b, c) ___append(___append(a, b), c)

/*
#define ____PWM_config(prescaler, limit, tim, pin, altf)\
		#ifdef ___append3(RCC_APB1ENR_TIM,tim,EN)\
		RCC->APB1ENR |= ___append3(RCC_APB1ENR_TIM,tim,EN);\
		#else
		RCC->APB2ENR |= ___append3(RCC_APB2ENR_TIM,tim,EN);\
		io_init(A);\
		io_altF(pin, altf);\
		___append(TIM,tim)->PSC = prescaler - 1;\
		___append(TIM,tim)->ARR = limit;\
		___append(TIM,tim)->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;\
		___append(TIM,tim)->CCER |= TIM_CCER_CC1E;\
		___append(TIM,tim)->BDTR |= TIM_BDTR_MOE;\
		___append(TIM,tim)->CR1 |= TIM_CR1_CEN;
*/
//#define PWM_config(prescaler, limit, tim, pin, altf) ____PWM_config(prescaler, limit, tim, pin, altf)

/* HARDWARE'owy PWM na T1
 *
void T1_PWMCH1Non(uint16_t prescaler, uint16_t limit, float dutyCycle) {
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	io_init(A);
	io_altFMode(PA7);
	io_altF(PA7, 2);
	TIM1->PSC = prescaler - 1;
	TIM1->ARR = limit;
	TIM1->CCR1 = dutyCycle / 100 * limit;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
	TIM1->CCER |= TIM_CCER_CC1NE;
	TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CR1 |= TIM_CR1_CEN;
}
void T1_PWMCH1Noff() {
	io_in(PA7);
	TIM1->CCMR1 &=~ (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE);
	TIM1->CCER &=~ TIM_CCER_CC1NE;
	TIM1->BDTR &=~ TIM_BDTR_MOE;
	TIM1->CR1 &=~ TIM_CR1_CEN;
}
void T1_PWMCH1Nduty(float dutyCycle) {
	TIM1->CCR1 = dutyCycle / 100 * TIM1->ARR;
}
*/


/* WATCHDOG DIODE BUTTON
#include <stm32f0xx.h>
#include "STML.h"


int main() {

	//8MHz STM32F030K6T6
	io_init(A); //RCC for GPIOA
	io_init(B); //RCC for GPIOB
	io_out(PB0); //diode on PB0
	io_in(PA0, pullUp); //switch button to gnd on PA0

	WDT_init(20);

	//BUTTON PRESSED - WDT reloaded, CPU works, diode blinks
	//BUTTON RELEASED - WDT not reloaded, every 20ms WDT reaches 0
	//and resets CPU, program can't reach point, where diode is toggled
	//so, diode is doesn't blink

	while(1) {
		for(int x = 0;x < 50;x++) {
			if(io_isLow(PA0)) WDT_reload();
			delay_ms(10);
		}
		io_toggle(PB0); //diode toggle
	}

}
 * */



//EXAMPLE USE OF TIMER EXTENDING
/*
 * To extend timer, set up two timers for the first and then
 * call T_extend(masterTimerNumber, slaveTimerNumber, ITR)
 * ITR is number specified in specification of any timer
 * available to be driven by other timer and is used for
 * telling slave timer which timer (overflow of timer) will
 * be used as clock source.
 *
#include <stm32f0xx.h>
#include "STML.h"


void fun() {
	io_toggle(PB0); //toggle diode
}

int main() {

	io_init(B);
	io_out(PB0); //diode on PB0

	T3_repeat(1000, 1, fun); //init timer 3 to call function fun on overflow (when reaches 1000)
	T1_countTo(1, 8000); //init timer 1 to count up with frequency of 1MHz only from 0 to 1
	T_extend(1, 3, 0); //extend timer 1, that timer 3 counts up, when T1 has overflow

	//diode is blinking 2x slower than if we comment T3_repeat, T1_countTo and T_extend
	//and use T3_repeat(1000, 8000, fun);


	while(1);
}
 * */


#define T_extend(master, slave, itr)\
	___append(TIM, master)->CR2 |= 2 << TIM_CR2_MMS_Pos;\
	___append(TIM, slave)->SMCR |= 7 << TIM_SMCR_SMS_Pos;\
	___append(TIM, slave)->SMCR |= itr << TIM_SMCR_TS_Pos


/* ENCODER MODE
#include <stm32f0xx.h>
#include "STML.h"

int main() {

	io_init(B);
	io_out(PB0); //diode on PB0

	T1_countTo(65535, 1);
	TIM1->SMCR |= 1 << TIM_SMCR_SMS_Pos; //encoder mode
	io_init(A);
	io_altF(PA8, 2);
	io_altF(PA9, 2);

	while(1);
}
 * */








#endif /* STML_H_ */
