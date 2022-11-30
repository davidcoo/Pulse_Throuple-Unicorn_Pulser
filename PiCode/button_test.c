#include <wiringPi.h>
#include <stdio.h>

#define BUTTON_PIN 27

int main()
{ 
	wiringPiSetupGpio();
	pinMode(BUTTON_PIN, INPUT);

	while(1) printf("Pin Value: %d \n", digitalRead(BUTTON_PIN));
}
