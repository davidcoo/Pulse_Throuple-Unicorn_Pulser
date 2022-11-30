#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>


#define trigPin 5
#define echoPin 6


int main(void) {

	wiringPiSetupGpio();
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);

        digitalWrite(TRIG, LOW);

	while(1){
		long duration, distance;
		digitalWrite(trigPin, LOW);  // Added this line
		delayMicroseconds(2); // Added this line
		digitalWrite(trigPin, HIGH);
		//  delayMicroseconds(1000); - Removed this line
		delayMicroseconds(10); // Added this line
		digitalWrite(trigPin, LOW);
		duration = pulseIn(echoPin, HIGH);
		distance = (duration/2) / 29.1;
		if (distance >= 200 || distance <= 0){
			printf("Out of range\n");
		}
		else {
			printf("distance: %d", distance);
			printf(" cm\n");
 		}
  		delay(500);
	}
	printf("am done \n");
	return 0;
}
