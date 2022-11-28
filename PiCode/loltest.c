#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "state.h"
#include "can_header_temp.h"
#include <time.h>
#include <math.h>

#define MAX_RANGE 32767
#define MIN_RANGE -32768
uint8_t map_throttle(int raw_throttle) {
    // divide
    int temp = (raw_throttle * 50)/MAX_RANGE;
    temp += 50;
    printf("raw: %d, mapped: %d\n", raw_throttle, temp);
    return (uint8_t)temp;
}

uint8_t map_brake(int raw_brake) { 
    raw_brake -= 20000;
    if (raw_brake < 0) {
        printf("raw: %d, mapped: %d\n", raw_brake, 0);
        return (uint8_t)0;
    }
    int temp = (raw_brake * 100)/ 12767;
    printf("%d\n", temp);
    printf("raw: %d, mapped: %d\n", raw_brake, temp);
    return (uint8_t)temp;

}

uint8_t map_steering(int raw_steering){
    int temp = (raw_steering * 255)/((2*MAX_RANGE)+1);
    temp += 128;
    printf("raw: %d, mapped: %d\n", raw_steering, temp);
    return (uint8_t)temp; 
    // output between 1-255
}

int main()
{
    map_steering(0);
    map_steering(MAX_RANGE);
    map_steering(MIN_RANGE);
    map_steering(pow(2, 14));
    map_steering(-(pow(2, 14)));
}