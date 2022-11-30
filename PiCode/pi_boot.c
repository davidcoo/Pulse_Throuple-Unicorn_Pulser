#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "pthread.h"
#include <inttypes.h>
#include "state.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include "can_header_temp.h"
#include <time.h>
#include <math.h>
#include <wiringPi.h>

#define BITRATE 500000
//#define LOCAL_HOST "169.254.81.79"
#define LOCAL_HOST "169.254.190.42"
#define R_PORT 13606

#define REMOTE_HOST "169.254.204.15"
//#define REMOTE_HOST "169.254.190.42"
#define S_PORT 1000

#define TX_INTERVAL_MS 300
#define HEARTBEAT_TO 200
#define STATE_SIZE sizeof(DIJOYSTATE2_t)

#define MAX_RANGE 32767
#define MIN_RANGE -32768

#define PIN_BUTTON 27

struct period_info {
    struct timespec next_period;
    long period_ns;
};


uint8_t map_throttle(int raw_throttle) {
    // divide
    int temp = (raw_throttle * 50)/MAX_RANGE;
    temp += 50;
   // printf("raw: %d, mapped: %d\n", raw_throttle, temp);
    return (uint8_t)(100-temp);
    //output between 0-100
}

uint8_t map_brake(int raw_brake) { 
    raw_brake -= 20000;
    if (raw_brake < 0) {
       // printf("raw: %d, mapped: %d\n", raw_brake, 0);
        return (uint8_t)100;
    }
    int temp = (raw_brake * 100)/ 12767;
    //printf("%d\n", temp);
    //printf("raw: %d, mapped: %d\n", raw_brake, temp);
    return (uint8_t)(100-temp);
    //output between 0-100

}

uint8_t map_steering(int raw_steering){
    int temp = (raw_steering * 50)/MAX_RANGE;
    temp += 50;
    //printf("raw: %d, mapped: %d\n", raw_steering, temp);
    return (uint8_t)temp; 
    // output between 1-255
}

typedef enum {
    BLINK_0, // previously unpressed
    BLINK_1, // pressed once
    BLINK_2 // pressed once and pressed again
} blink_button_state_e;

typedef enum {
    BUTTON_0,
    BUTTON_1,
    BUTTON_2,
} reset_button_state_e;

typedef struct {
    long heartbeat_rear;
    long heartbeat_front;
} heartbeat_tracker_t;



typedef struct {
    uint8_t enabled; // in collision, so we can still send heartbeat w/ 0 braking pos
    uint8_t sending_heartbeat;
    uint8_t front_enabled;
    uint8_t rear_enabled;
    uint8_t brake_pos;
    uint8_t throttle_pos;
    uint8_t steering_pos;
    uint8_t blink_both;
    uint8_t blink_right;
    blink_button_state_e blink_right_state;
    uint8_t blink_left;
    blink_button_state_e blink_left_state;
    uint16_t servo_current;
    uint16_t servo_pos; // tell which direction to apply the force
    int raw_steering;
    int raw_throttle;
    int raw_brake;
    reset_button_state_e button_state;
    pthread_mutex_t mux_pos; //  mutex for accessing mapped values
    pthread_mutex_t mux_blink; // mutex for accessing blinker values
    pthread_mutex_t mux_servo; // mutex for accesing servo values
    pthread_mutex_t mux_heartbeat;
} control_state_t;

typedef struct { 
    int sockfd;
    int s;
    struct sockaddr_in servaddr;
    control_state_t *control_state;
    heartbeat_tracker_t *heart_track;
} receive_position_info_t;

static void inc_period(struct period_info *pinfo) 
{
    pinfo->next_period.tv_nsec += pinfo->period_ns;
 
    while (pinfo->next_period.tv_nsec >= 1000000000) {
        /* timespec nsec overflow */
        pinfo->next_period.tv_sec++;
        pinfo->next_period.tv_nsec -= 1000000000;
    }
}
 
static void periodic_task_init(struct period_info *pinfo, long period_ns)
{
    // period in ns
    pinfo->period_ns = period_ns;
 
    // initialize next period to current time
    clock_gettime(CLOCK_MONOTONIC, &(pinfo->next_period));
}

static void wait_rest_of_period(struct period_info *pinfo)
{
    inc_period(pinfo);
 
    /* for simplicity, ignoring possibilities of signal wakes */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, NULL);
}

long time_ms(){
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);
    return round(spec.tv_sec/1.0e6);
}
 
void *send_force(void *args) {
    // UDP

    // if control_state->front_disabled, send 0 force feedback

    int8_t force = 0;
    int sockfd;
    struct sockaddr_in servaddr;
        // Init steering wheel force feedback

    receive_position_info_t *send1_args = (receive_position_info_t *)args;
    control_state_t *control_state = send1_args->control_state;
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(S_PORT);
    servaddr.sin_addr.s_addr = inet_addr(REMOTE_HOST);
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        exit(EXIT_FAILURE);
    }

    while (1) {
        usleep(TX_INTERVAL_MS * 1000);
       // printf("Send force %d\n", force);
        force += 5;
        if (!(control_state->front_enabled)) force = 0;
        if (control_state->enabled) sendto(sockfd, (char*) &force, 1, MSG_CONFIRM,
                (struct sockaddr *) &servaddr, sizeof(servaddr));

    }
}

void *receive_position(void *args){


    DIJOYSTATE2_t state;
    char recvbuf[sizeof(DIJOYSTATE2_t) + 4];
    receive_position_info_t *receive_args = (receive_position_info_t *)args;

    int sockfd = receive_args->sockfd;
    struct sockaddr_in servaddr = receive_args->servaddr;
    heartbeat_tracker_t *heart_track = (heartbeat_tracker_t *)(receive_args->heart_track);
    control_state_t *control_state = (control_state_t *)(receive_args->control_state);
    // need: sockfd,  servaddr, control_state
    while(control_state->enabled) {
        int n, len;
        n = recvfrom(sockfd, recvbuf, STATE_SIZE, MSG_WAITALL,
                    (struct sockaddr *) &servaddr, &len);
        uint32_t packet_ct = ((uint32_t*) recvbuf)[0];
        memcpy(&state, recvbuf + 4, sizeof(state));



        uint8_t brake_pos = map_brake(state.lRz);
        uint8_t throttle_pos = map_throttle(state.lY);
        uint8_t steering_pos = map_steering(state.lX);

        pthread_mutex_lock(&(control_state->mux_pos));
        control_state->brake_pos = brake_pos;
        control_state->throttle_pos = throttle_pos;
        control_state->steering_pos = steering_pos;
        pthread_mutex_unlock(&(control_state->mux_pos));
       
       // printf("rgb button 4: %u, rgb button 5: %d \n", state.rgbButtons[4], state.rgbButtons[5]);
        pthread_mutex_lock(&(control_state->mux_blink));
        if (state.rgbButtons[5] > 0) { // Left
            if (control_state->blink_left_state == BLINK_0) {
                // toggle blinker - using xor to toggle last bit
                control_state->blink_left = control_state->blink_left ^ 1;
                control_state->blink_left_state = BLINK_1;
            } else if (control_state->blink_left_state == BLINK_1) {
                // no toggle - button continuously pressed
                control_state->blink_left_state = BLINK_2;
            } // else do nothing - stay in BLINK_2
            // need to debounce this button 
            //control_state->blink_left = state.rgbButtons[4];
        } else {
            control_state->blink_left_state = BLINK_0;
        }
        if (state.rgbButtons[4] > 0) { // Right
            if (control_state->blink_right_state == BLINK_0) {
                    // toggle blinker - using xor to toggle last bit
                    control_state->blink_right = control_state->blink_right ^ 1;
                    control_state->blink_right_state = BLINK_1;
            } else if (control_state->blink_right_state == BLINK_1) {
                    // no toggle - button continuously pressed
                    control_state->blink_right_state = BLINK_2;
            } // else do nothing - stay in BLINK_2
            else {
                control_state->blink_right_state = BLINK_0;
            }
            pthread_mutex_unlock(&(control_state->mux_blink));
        }
    }
        // check the button in here too, might as well?
        // check the read of the button 
    if (digitalRead(PIN_BUTTON)){
            if (control_state->button_state == BUTTON_0){
                control_state->button_state = BUTTON_1;
                printf("BUTTON HIGH \n");
            }
    }
    else {
        if (control_state->button_state = BUTTON_1){
            control_state->button_state = BUTTON_0;
            if (control_state->enabled = 0) {
		pthread_mutex_lock(&(control_state->mux_heartbeat));
		heart_track->heartbeat_front = time_ms();
                heart_track->heartbeat_rear = time_ms(); // you should have just put these in control_state smhhhhhh
        	pthread_mutex_unlock(&(control_state->mux_heartbeat));    
	} 
            control_state->enabled = !(control_state->enabled); //disable if enabled, enable if disabled // might want to make this a functon to determine if heartbeat should be sent or not, with collision avoidance stuff
            printf("CONTROL ZONE = %d\n", control_state->enabled);
        }
    }

}
        //printf("Receive state (Pkt: %8X) :  Wheel: %d | Throttle: %d | Brake: %d\n", packet_ct, state.lX, state.lY, state.lRz);


void *send_can(void *args) {
    //make this periodic 
    receive_position_info_t *send_args = (receive_position_info_t *)args;
    control_state_t *control_state = send_args->control_state;
    int s = send_args->s;
    int nbytes;
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    frame.can_id = 0x100;
    frame.can_dlc = 8;
    struct period_info pinfo;
    periodic_task_init(&pinfo, 20000000); // Period 20ms

    while (1) {
        // might need to lock this guy
        if (control_state->enabled){
            pthread_mutex_lock(&(control_state->mux_pos));
            frame.data[0] = control_state->brake_pos;
            frame.data[1] = control_state->throttle_pos;
            frame.data[2] = control_state->steering_pos;
            pthread_mutex_unlock(&(control_state->mux_pos));

            pthread_mutex_lock(&(control_state->mux_blink));
            frame.data[3] = control_state->blink_both;
            frame.data[4] = control_state->blink_left;
            frame.data[5] = control_state->blink_right;
            pthread_mutex_unlock(&(control_state->mux_blink));

            frame.data[6] = 0;
            frame.data[7] = 0;
            printf("frame: %u %u %u %u %u %u  %u %u \n", frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
            nbytes = write (s, &frame, sizeof(frame));
            if (nbytes != sizeof(frame)) {
                printf("Send error frame[0]\r\n");
                system("sudo ifconfig can0 down");
            }
        }
        wait_rest_of_period(&pinfo);
    }
}

void *receive_can(void *args) {
    receive_position_info_t *receiver2_args = (receive_position_info_t *)args;
    heartbeat_tracker_t *heart_track = receiver2_args->heart_track;
 
    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    // need to memset frame
    struct can_filter rfilter[2];
    // need the control state here!
    int s = receiver2_args->s;
    control_state_t *control_state = receiver2_args->control_state;
    int nbytes;
    // Make periodic
    struct period_info pinfo;
    periodic_task_init(&pinfo, 10000000); //10 ms period

    rfilter[0].can_id = 0x200;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x300;
    rfilter[1].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    while(1){ 
        if (control_state->enabled) {
        nbytes = read(s, &frame, sizeof(frame));
        if (nbytes > 0){
            // received a front zone one: 
            if (frame.can_id == 0x200) {
                // front zone, 
                // save force value and
                pthread_mutex_lock(&(control_state->mux_servo));
                control_state->servo_current = ((uint16_t)frame.data[0] << 8) & frame.data[1];
                control_state->servo_pos = ((uint16_t)frame.data[2] << 8) & frame.data[1];
                pthread_mutex_unlock(&(control_state->mux_servo));
                
                pthread_mutex_lock(&(control_state->mux_heartbeat));
	        heart_track->heartbeat_front = time_ms();
                pthread_mutex_unlock(&(control_state->mux_heartbeat));
                // GET OUT OF ERROR STATE (done in main now, otherwise can mux and write here?)
            }
            else if (frame.can_id == 0x300){
                pthread_mutex_lock(&(control_state->mux_heartbeat));
		heart_track->heartbeat_rear = time_ms();
		pthread_mutex_unlock(&(control_state->mux_heartbeat));
              // GET OUT OF ERROR STATE
            }

          }
        memset(&frame, 0, sizeof(struct can_frame));
        }
        wait_rest_of_period(&pinfo);
     
     }
}

int main()
{
    // first set up CAN
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    control_state_t c_state;
    control_state_t *control_state = &c_state;
    
    wiringPiSetupGpio();
    pinMode(PIN_BUTTON, INPUT);
    int sockfd;
    char buffer[STATE_SIZE+1];
    struct sockaddr_in servaddr = { 0 };

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        exit(EXIT_FAILURE);
    }

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(R_PORT);
    servaddr.sin_addr.s_addr = inet_addr(LOCAL_HOST);

    if (bind(sockfd, (const struct sockaddr *) &servaddr, 
                sizeof(servaddr)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }


    memset(&frame, 0, sizeof(struct can_frame));

    // Bring up CAN bus
    //system("sudo ip link set can0 up type can bitrate %d", BITRATE);
    system("sudo ip link set can0 up type can bitrate 500000");
    system("sudo ifconfig can0 txqueuelen 65536");
    // probably want to write a check here to make sure that can0 was down and you're not getting one of those weird errors
    printf("pi is connected to can0\r\n");

    //1. Create Socket for CAN
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
        return 1;
    }
    //2. Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);

    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }

    //3. Bind the sockt to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
        return 1;
    }

    // Init state vars
    control_state->enabled = 1;
    control_state->sending_heartbeat = 1;
    control_state->front_enabled = 1;
    control_state->rear_enabled = 1;
    control_state->brake_pos = 0;
    control_state->throttle_pos = 0;
    control_state->steering_pos = 0;
    control_state->blink_both = 0;
    control_state->blink_right = 0;
    control_state->blink_right_state = BLINK_0;
    control_state->blink_left = 0;
    control_state->blink_left_state = BLINK_0;
    control_state->servo_current = 0;
    control_state->servo_pos = 0;
    control_state->button_state = BUTTON_0;
    
    // Init steering wheel force feedback
    receive_position_info_t s1_args;
    receive_position_info_t *send1_args = &s1_args;
    send1_args->control_state = control_state;
    pthread_t send_tid;
    pthread_create(&send_tid, NULL, send_force, send1_args);


    receive_position_info_t r_args;
    receive_position_info_t *receive_args = &r_args;
    receive_args->sockfd = sockfd;
    receive_args->servaddr = servaddr;
    receive_args->control_state = control_state;

    /* initialize a mutex to its default value */
    pthread_mutex_init(&(control_state->mux_pos), NULL);
    pthread_mutex_init(&(control_state->mux_blink), NULL);
    pthread_mutex_init(&(control_state->mux_servo), NULL);
    pthread_mutex_init(&(control_state->mux_heartbeat), NULL);

    // Init steering wheel communication thread
    pthread_t receive_tid;


    heartbeat_tracker_t h_track;
    heartbeat_tracker_t *heart_track = &h_track;

    heart_track->heartbeat_rear = time_ms();
    heart_track->heartbeat_front = time_ms();
    receive_args->heart_track = heart_track;
    
    pthread_create(&receive_tid, NULL, receive_position, (void *)receive_args);


    // Init CAN RX thread
    receive_position_info_t r2_args;
    receive_position_info_t *receive2_args = &r2_args;
    receive2_args->s = s;
    receive2_args->control_state = control_state;
    pthread_t receive_can_tid;
    pthread_create(&receive_can_tid, NULL, receive_can, (void*)receive2_args);

    // Init CAN TX thread
    receive_position_info_t s_args;
    receive_position_info_t *send_args = &s_args;
    send_args->s = s;
    send_args->control_state = control_state;
    pthread_t send_can_tid;
    pthread_create(&send_can_tid, NULL, send_can, (void *)send_args);
    
    while(1){
         if (!(control_state->enabled)) {
             control_state->front_enabled = 1;
             control_state->rear_enabled =1 ; // if control state is not enabled, pretend that everything else is working perfectly
         } 
         else {
         if (time_ms() - heart_track->heartbeat_front > HEARTBEAT_TO) {
             control_state->front_enabled = 0;
            // SET FRONT STATE FUCKED UP
            // GO INTO HAZARD STATE, UPDATE THE CONTROL VARIABLE
         } else {
             control_state->front_enabled = 1;
         }
         if (time_ms() - heart_track->heartbeat_rear > HEARTBEAT_TO){
             control_state->rear_enabled = 0;
         } else {
             control_state->rear_enabled = 1;
         }
	}
    }
}
