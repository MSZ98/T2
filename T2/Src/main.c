


#include <stm32f0xx.h>
#include "STML.h"
#include "stepper.h"


// button wciśnięty = 1


stepper m1, m2;
#define button PB3
#define potSpeed PA7
#define potTime PA6

//LIMIT SWITCHES
#define ls1 PB5
#define ls2 PB4

//CONTACT SENSOR zakryty = 1, dioda nie świeci = 1, dioda świeci = 0
#define cs1 PA2
#define cs2 PA1

//STEPPER 1 dir, step, enable
#define m1_d PA5
#define m1_s PA4
#define m1_e PA3



#define turn 2 * 16 * 96


void initControls();


uint16_t speed = 0.1* 3398;


void updateSpeed() {
	ADC_setPin(potSpeed);
	speed = ADC_sample() / 4095.0 * turn * 10;
	stepper_setSpeed(m1, speed);
}



//Jeżeli wywołana została funkcja release, to trzeba wywołać hold, bo inaczej nie pojedzie




#define chowajGniazdo() stepper_hold(m1);stepper_run(m1, -turn * 5000, speed)
#define pokazGniazdo() stepper_hold(m1);stepper_run(m1, turn * 5000, speed)

#define zatrzymajGniazdo() stepper_run(m1, 0, 0);stepper_release(m1)


//K1 krańcówka stykająca, kiedy wysunięte jest gniazdko
//K2 styka kiedy gniazdko jest zasunięte
#define gniazdo_wysuniete io_isLow(ls1)  //Jeżeli to będzie 1, to gniazdko wysunięte
#define gniazdo_zasuniete io_isLow(ls2)  //Jeżeli to będzie 1, to gniazdko zasunięte


#define wtyczka_wlozona (io_isHigh(cs1) || io_isHigh(cs2))






//Gniazdo wysunięte k1 daje sygnał
//Gniazdo zasunięte k2 daje sygnał
//Gniazdo w podróży żadna nie daje sygnału






int main() {
	CLOCK_internal48mhz();


	initControls();

	zatrzymajGniazdo();

	//stepper_hold(m1);stepper_run(m1, turn * 5000, speed);




	uint16_t lim = 30;
	uint16_t bt = lim;
	while(1) {

		// CHOWANIE GNIZDA
		//if(io_isHigh(button) && gniazdo_wysuniete && wtyczka_wlozona) continue; //Jeżeli przycisk jest wciśnięty (zbocze) i gniazdko wysunięte i jeżeli wtyczka nie jest włożona
		if(io_isHigh(button) && gniazdo_wysuniete && !wtyczka_wlozona) { //Jeżeli przycisk jest wciśnięty (zbocze) i gniazdko wysunięte i jeżeli wtyczka nie jest włożona
			if(bt == lim) {
				chowajGniazdo();
				while(!gniazdo_zasuniete) delay_ms(100);             //poczekaj aż gniazdo się schowa
				zatrzymajGniazdo();
			}
			bt = 0;
		}

		// WYSUWANIE GNIAZDA
		else if(io_isHigh(button)) { //Jeżeli przycisk jest wciśnięty (zbocze) i gniazdko zasunięte
			if(bt == lim) {
				pokazGniazdo();
				while(!gniazdo_wysuniete) delay_ms(100);             //poczekaj aż gniazdo się wysunie
				zatrzymajGniazdo();
			}
			bt = 0;
		}

		if(bt < lim) bt++;
		delay_ms(10);
	}






}




void initControls() {

	//POT_SPEED
	ADC_init(potSpeed);

	//POT_TIME
	ADC_init(potTime);

	//BUTTON
	io_init(button.port);
	io_in(button, pullUp);

	//ls1
	io_init(ls1.port);
	io_in(ls1, pullUp);

	//ls2
	io_init(ls2.port);
	io_in(ls2, pullUp);

	//cs1
	io_init(cs1.port);
	io_in(cs1, pullUp);

	//cs2
	io_init(cs2.port);
	io_in(cs2, pullUp);


	//STEPPER MOTOR LIBRARY
	stepper_initTimer(TIM1, 20);


	//STEPPER MOTORS
	m1 = stepper_createMotor(m1_s, m1_d, m1_e);


	//TIM_repeat(TIM3, 48000, 50, updateSpeed);


}













