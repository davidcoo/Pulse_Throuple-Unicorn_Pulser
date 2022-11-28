#include <stdio.h>
#include <wiringPi.h>

#define PIN_BUTTON 27

int main(int argc, char **argv)
{
    wiringPiSetupGpio();

    pinMode(PIN_BUTTON, INPUT);
    printf("button pins have been setup.\n");
    while (1)
    {
         printf("PIN VALUE: %d\n", digitalRead(PIN_BUTTON));
    }
}
