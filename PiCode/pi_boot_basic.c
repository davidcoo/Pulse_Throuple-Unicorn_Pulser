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
#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <errno.h>

#define BUF_SIZE 1024

#define SHM_KEY_RIGHT 0X1234
#define SHM_KEY_LEFT 0X5678
#define SHM_KEY_FRONT 0x1357

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

#define RESET_BUTTON 18

#define LEFT_STEER_TH1 35
#define LEFT_STEER_TH2 45
#define RIGHT_STEER_TH1 65
#define RIGHT_STEER_TH2 55

#define COLLISION_THRESH_CLOSE 5
#define COLLISION_THRESH_FAR 10

#define COLLISION_MAX_THROTTLE 20
#define COLLISION_MAX_LEFT 55
#define COLLISION_MAX_RIGHT 45

#define COLLISION_ITER 5

// do the thresholds for left and right turn 
// run safe exit!

// do the thresholds for left and right turn 
// run safe exit!
struct shmseg {
   int cnt;
   int complete;
   char buf[BUF_SIZE];
};

typedef enum {
    BUTTON_0,
    BUTTON_1
} reset_button_state_e;

long old_rounded = 0;
long time_ms(){
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);
    //printf("spec: %ld\n", spec.tv_nsec);
    long rounded = spec.tv_sec*1000 + round(spec.tv_nsec/1.0e6);
    if (rounded >= old_rounded) { 
        old_rounded = rounded;
    }
    else { printf("ERROR: decreased. new: %ld, old: %ld\n", rounded, old_rounded);}
    //printf("rounded: %ld\n", rounded);
    return rounded;
}

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
    BLINK_OFF,
    BLINK_ACTIVE,
    TURN_PASS_TH1
} blink_turn_state_e;

typedef struct {
    uint8_t enabled; // in collision, so we can still send heartbeat w/ 0 braking pos
    uint8_t collision;
    uint8_t front_enabled;
    uint8_t rear_enabled;
    uint8_t brake_pos;
    uint8_t throttle_pos;
    uint8_t steering_pos;
    uint8_t blink_both;
    uint8_t blink_right;
    blink_button_state_e right_trig_state;
    blink_turn_state_e right_turn_state;
    uint8_t blink_left;
    blink_button_state_e left_trig_state;
    blink_turn_state_e left_turn_state;
    reset_button_state_e reset_button_state;
    uint16_t servo_current;
    uint16_t servo_pos; // tell which direction to apply the force
    int raw_steering;
    int raw_throttle;
    int raw_brake;
    pthread_mutex_t mux_pos; //  mutex for accessing mapped values
    pthread_mutex_t mux_blink; // mutex for accessing blinker values
    pthread_mutex_t mux_servo; // mutex for accesing servo values
    long heartbeat_front;
    long heartbeat_rear;
} control_state_t;

typedef struct { 
    int sockfd;
    int s;
    struct sockaddr_in servaddr;
    control_state_t *control_state;

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
 

void *send_force(void *args) {
    // UDP
    int8_t force = 0;
    int sockfd;
    struct sockaddr_in servaddr;
    control_state_t *control_state = (control_state_t *)args;

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(S_PORT);
    servaddr.sin_addr.s_addr = inet_addr(REMOTE_HOST);
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        exit(EXIT_FAILURE);
    }

    while (1) { 
        //force = 0;
        usleep(TX_INTERVAL_MS * 100);
        if (!(control_state->front_enabled)){
            force = 0;
        }
        if (control_state->enabled && !(control_state->collision)) { 
            printf("froce: %d\n", force);
            sendto(sockfd, (char*) &force, 1, MSG_CONFIRM,
                (struct sockaddr *) &servaddr, sizeof(servaddr));
        }  
    }
}

void *receive_position(void *args){


    DIJOYSTATE2_t state;
    char recvbuf[sizeof(DIJOYSTATE2_t) + 4];
    receive_position_info_t *receive_args = (receive_position_info_t *)args;

    int sockfd = receive_args->sockfd;
    struct sockaddr_in servaddr = receive_args->servaddr;
    control_state_t *control_state = receive_args->control_state;
    // need: sockfd,  servaddr, control_state
    while(1) {
        if (control_state->enabled && !(control_state->collision)){
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
        
            pthread_mutex_lock(&(control_state->mux_blink));

            // should we be blinking?
            if (state.rgbButtons[5] > 0) { // Left
                switch(control_state->left_trig_state) {
                    case BLINK_0:
                        // toggle blinker - using xor to toggle last bit
                        control_state->blink_left = control_state->blink_left ^ 1;
                        control_state->left_trig_state = BLINK_1;
                        if (control_state->blink_right = 1) {
                            control_state->blink_right = 0;
                        }
                        break;
                    case BLINK_1:
                        control_state->left_trig_state = BLINK_2;
                        break;
                    case BLINK_2: // else do nothing - stay in BLINK_2
                    default:
                        break;
                }
            } else {
                control_state->left_trig_state = BLINK_0;
            }
            if (state.rgbButtons[4] > 0) { // Right
                switch(control_state->right_trig_state) {
                    case BLINK_0:
                        // toggle blinker - using xor to toggle last bit
                        control_state->blink_right = control_state->blink_right ^ 1;
                        control_state->right_trig_state = BLINK_1;
                        if (control_state->blink_left = 1) {
                            control_state->blink_left = 0;
                        }
                        break;
                    case BLINK_1:
                        control_state->right_trig_state = BLINK_2;
                        break;
                    case BLINK_2: // else do nothing - stay in BLINK_2
                    default:
                        break;
                }                
            } else {
                control_state->right_trig_state = BLINK_0;
            }
            // Turn and blinkers state machines - left
            blink_turn_state_e nextL = control_state->left_turn_state;
            switch(control_state->left_turn_state) {
                case BLINK_OFF:
                    if (control_state->blink_left) {
                        nextL = BLINK_ACTIVE;
                    } // stay off if not
                    break;
                case BLINK_ACTIVE:
                    if (control_state->steering_pos < LEFT_STEER_TH1) {
                        nextL = TURN_PASS_TH1;
                    }
                    break;
                case TURN_PASS_TH1:
                    if (control_state->steering_pos > LEFT_STEER_TH2) {
                        nextL = BLINK_OFF;
                        control_state->blink_left = 0;
                    }
                    break;
                default: break;
            }
            control_state->left_turn_state = nextL;
            // Turn and blinkers state machines - right
            blink_turn_state_e nextR = control_state->right_turn_state;
            switch(control_state->right_turn_state) {
                case BLINK_OFF:
                    if (control_state->blink_right) {
                        nextR = BLINK_ACTIVE;
                    } // stay off if not
                    break;
                case BLINK_ACTIVE:
                    if (control_state->steering_pos > RIGHT_STEER_TH1) {
                        nextR = TURN_PASS_TH1;
                    }
                    break;
                case TURN_PASS_TH1:
                    if (control_state->steering_pos < RIGHT_STEER_TH2) {
                        nextR = BLINK_OFF;
                        control_state->blink_right = 0;
                    }
                    break;
                default:
                    nextR = BLINK_OFF; 
                    break;
            }
            control_state->right_turn_state = nextR;
            pthread_mutex_unlock(&(control_state->mux_blink));
        }
        else if (control_state->collision == 2) {
            int n, len;
            n = recvfrom(sockfd, recvbuf, STATE_SIZE, MSG_WAITALL,
                        (struct sockaddr *) &servaddr, &len);
            uint32_t packet_ct = ((uint32_t*) recvbuf)[0];
            memcpy(&state, recvbuf + 4, sizeof(state));

            uint8_t brake_pos = map_brake(state.lRz);
            uint8_t throttle_pos = map_throttle(state.lY);

            pthread_mutex_lock(&(control_state->mux_pos));
            control_state->brake_pos = brake_pos;
            control_state->throttle_pos = throttle_pos;
            pthread_mutex_unlock(&(control_state->mux_pos));
        }
        if (digitalRead(RESET_BUTTON)) {
            // button is being pressed, where should it go? 
            if (control_state->reset_button_state == BUTTON_0) {
                control_state->reset_button_state = BUTTON_1; // being held down once
                if (control_state->collision == 1 ) {
                    // you need to remove obstacle before pressing button
                    // otherwise there would be a race condition 
                    // but i don't feel like writing in MORE mutexes right now lol
                    control_state->collision = 0;
                }
                else {
                    control_state->enabled = !(control_state->enabled);
                }
            }
        }
        else {
            if (control_state->reset_button_state == BUTTON_1){
                control_state->reset_button_state = BUTTON_0;
            }
        }

        //printf("Receive state (Pkt: %8X) :  Wheel: %d | Throttle: %d | Brake: %d\n", packet_ct, state.lX, state.lY, state.lRz);
    }

}


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
            //printf("(%ld) frame: %u %u %u %u %u %u  %u %u \n", time_ms(), frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
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
    printf("s : %d\n", s);

    while(1){
        if (control_state->enabled) {
            nbytes = read(s, &frame, sizeof(frame));

            if (nbytes > 0){
                // received a front zone one: 
                if (frame.can_id == 0x200) {

                    // front zone, 
                    // save force value and
                    pthread_mutex_lock(&(control_state->mux_servo));
                    //control_state->servo_current = ((uint16_t)frame.data[0] << 8) & frame.data[1];
                    printf("(%ld) frame: %u %u %u %u %u %u  %u %u \n", time_ms(), frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
                    control_state->servo_current = (uint16_t)frame.data[1];
                    printf("control state: %u \n", control_state->servo_current);
                    control_state->servo_pos = ((uint16_t)frame.data[2] << 8) & frame.data[1];
                    pthread_mutex_unlock(&(control_state->mux_servo));
                    // increment heartbeat
                    control_state->heartbeat_front = time_ms();
                }
                else if (frame.can_id == 0x300){
                    // keep track of the heartbeat
                    control_state->heartbeat_rear = time_ms();

                }
            }
            memset(&frame, 0, sizeof(struct can_frame));

        } else {

            control_state->heartbeat_front = time_ms();
            control_state->heartbeat_rear = time_ms(); // don't go into failure due to other zones, while this zone is disabled
        }
        wait_rest_of_period(&pinfo);
    }
}

int shmid_r, shmid_l, shmid_f;
struct shmseg *shmp_r, *shmp_l, *shmp_f;

void *collision_detection(void *args) {
    // basically the system read
   control_state_t *control_state = (control_state_t *)args;
   /* Transfer blocks of data from shared memory to stdout*/
    struct period_info pinfo;
    periodic_task_init(&pinfo, 30000000); //10 ms period

    // lol could probably make this a better cast but whatever
    int right, left, front;
    int right_iter, left_iter, front_iter, iter;
    while (shmp_r->complete != 1) {
        right = atoi(shmp_r->buf);
        left = atoi(shmp_l->buf);
        front = atoi(shmp_f->buf);

        int collision_10 = front <= 10 || left <= 10 || right <= 10;
        int coll_20_front = front > 10 && front <= 20;
        int coll_20_left = left > 10 && left <= 20;
        int coll_20_right = right > 10 && right <= 20;

        if (collision_10) {
            control_state->collision = 1;

            // turn on both blinkers
            pthread_mutex_lock(&(control_state->mux_blink));
            control_state->blink_left = 1;
            control_state->blink_right = 1;
            pthread_mutex_unlock(&(control_state->mux_blink));
            
            pthread_mutex_lock(&(control_state->mux_pos));
            control_state->throttle_pos = 0;
            control_state->brake_pos = 100;
            pthread_mutex_unlock(&(control_state->mux_pos));
                  
            printf("COLLISION 10: (%ld): f = %d, r = %d, l = %d \n", time_ms(), front, right, left);
        } else if (coll_20_front) {
            control_state->collision = 2;

            pthread_mutex_lock(&(control_state->mux_blink));
            control_state->blink_left = 1;
            control_state->blink_right = 1;
            pthread_mutex_unlock(&(control_state->mux_blink));

            pthread_mutex_lock(&(control_state->mux_pos));
            if (control_state->throttle_pos > COLLISION_MAX_THROTTLE)
                control_state->throttle_pos = COLLISION_MAX_THROTTLE;
            pthread_mutex_unlock(&(control_state->mux_pos)); 
            printf("COLLISION 20 FRONT (%ld): f = %d, r = %d, l = %d \n", time_ms(), front, right, left);


        } else if (coll_20_left) {
            // okay to else if because we assume only 1 collision in our scope
            control_state->collision = 2;

            pthread_mutex_lock(&(control_state->mux_blink));
            control_state->blink_left = 1;
            control_state->blink_right = 1;
            pthread_mutex_unlock(&(control_state->mux_blink));

            pthread_mutex_lock(&(control_state->mux_pos));
            if (control_state->throttle_pos > COLLISION_MAX_THROTTLE)
                control_state->throttle_pos = COLLISION_MAX_THROTTLE;
            if (control_state->steering_pos < COLLISION_MAX_LEFT)
                control_state->steering_pos = COLLISION_MAX_LEFT;
            pthread_mutex_unlock(&(control_state->mux_pos)); 
            printf("COLLISION 20 LEFT (%ld): f = %d, r = %d, l = %d \n", time_ms(), front, right, left);


        } else if (coll_20_right) {
            control_state->collision = 2;

            pthread_mutex_lock(&(control_state->mux_blink));
            control_state->blink_left = 1;
            control_state->blink_right = 1;
            pthread_mutex_unlock(&(control_state->mux_blink));

            pthread_mutex_lock(&(control_state->mux_pos));
            if (control_state->throttle_pos > COLLISION_MAX_THROTTLE)
                control_state->throttle_pos = COLLISION_MAX_THROTTLE;
            if (control_state->steering_pos > COLLISION_MAX_RIGHT)
                control_state->steering_pos = COLLISION_MAX_RIGHT;
            pthread_mutex_unlock(&(control_state->mux_pos)); 
            printf("COLLISION 20 RIGHT (%ld): f = %d, r = %d, l = %d \n", time_ms(), front, right, left);

        }
        else if (control_state->collision == 2) {
            iter++;
            if (iter > COLLISION_ITER) {
                control_state->collision = 0;
                pthread_mutex_lock(&(control_state->mux_blink));
                control_state->blink_left = 0;
                control_state->blink_right = 0;
                pthread_mutex_unlock(&(control_state->mux_blink));    
                iter = 0;      
                printf("EXIT COLLISION\n");
      
            }
        }

        wait_rest_of_period(&pinfo);
    }
}

void clean_up(){
    if (shmdt(shmp_r) == -1) {
        perror("shmdt");
    }

   if (shmctl(shmid_r, IPC_RMID, 0) == -1) {
      perror("shmctl");
   }
    if (shmdt(shmp_l) == -1) {
        perror("shmdt");
    }

   if (shmctl(shmid_l, IPC_RMID, 0) == -1) {
      perror("shmctl");
   }
    if (shmdt(shmp_f) == -1) {
        perror("shmdt");
    }

   if (shmctl(shmid_f, IPC_RMID, 0) == -1) {
      perror("shmctl");
   }
}
int s; 
pid_t pid_right, pid_left, pid_front;


void sigint_handler(int signum){
    // forward sigint, could use gpid but lol
    kill(pid_right, SIGINT);
    kill(pid_left, SIGINT);
    kill(pid_front, SIGINT);

    close(s);
    //system("sudo ifconfig can0 down");
    clean_up();
    exit(1);
}
int main()
{
    //signal(SIGINT, sigint_handler);
    // pid_right = fork();
    // pid_left = fork();
    // pid_front = fork();
    // if (pid_right == 0){
    //     execlp("python3", "python3", "ultra.py", "RIGHT", NULL);
    // }
    // if (pid_left == 0){
    //     execlp("python3", "python3", "ultra.py", "LEFT", NULL);
    // }
    // if (pid_front == 0){
    //     execlp("python3", "python3", "ultra.py", "FRONT", NULL);
    // }

    int ret;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    control_state_t c_state;
    control_state_t *control_state = &c_state;

    wiringPiSetupGpio();
    pinMode(RESET_BUTTON, HIGH);

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
    printf("socket: %d \n", s);
    if (s < 0) {
        perror("socket PF_CAN failed");
        return 1;
    }
    printf("after 1\n");
    //2. Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);

    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }
    printf("after 2\n");
    //3. Bind the sockt to can0
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
        return 1;
    }
    printf("after 3 \n");

    // Init state vars
    control_state->enabled = 1;
    control_state->collision = 0;
 
    control_state->brake_pos = 0;
    control_state->throttle_pos = 0;
    control_state->steering_pos = 0;
    control_state->blink_both = 0;
    control_state->blink_right = 0;
    control_state->right_trig_state = BLINK_0;
    control_state->right_turn_state = BLINK_OFF;
    control_state->blink_left = 0;
    control_state->left_trig_state = BLINK_0;
    control_state->left_turn_state = BLINK_OFF;
    control_state->servo_current = 0;
    control_state->servo_pos = 0;
    control_state->front_enabled = 1;
    control_state->rear_enabled = 1;
    control_state->heartbeat_front = time_ms();
    control_state->heartbeat_rear = time_ms();
    control_state->reset_button_state = BUTTON_0;
    
    printf("hi hit here \n");
    receive_position_info_t r_args;
    receive_position_info_t *receive_args = &r_args;
    receive_args->sockfd = sockfd;
    receive_args->servaddr = servaddr;
    receive_args->control_state = control_state;

    /* initialize a mutex to its default value */
    pthread_mutex_init(&(control_state->mux_pos), NULL);
    pthread_mutex_init(&(control_state->mux_blink), NULL);
    pthread_mutex_init(&(control_state->mux_servo), NULL);
   

    // Init steering wheel communication threads
    pthread_t receive_tid;
    pthread_create(&receive_tid, NULL, receive_position, (void *)receive_args);
    pthread_t send_tid;
    pthread_create(&send_tid, NULL, send_force, (void*)control_state);

    // Init CAN RX thread
    receive_position_info_t r2_args;
    receive_position_info_t *receive2_args = &r2_args;
    receive2_args->s = s;
    receive2_args->control_state = control_state;
    pthread_t receive_can_tid;
    pthread_create(&receive_can_tid, NULL, receive_can, (void *)receive2_args);

    // Init CAN TX thread
    receive_position_info_t s_args;
    receive_position_info_t *send_args = &s_args;
    send_args->s = s;
    send_args->control_state = control_state;
    pthread_t send_can_tid;
    pthread_create(&send_can_tid, NULL, send_can, (void *)send_args);

    shmid_r = shmget(SHM_KEY_RIGHT, sizeof(struct shmseg), 0644|IPC_CREAT);
    if (shmid_r == -1) {
        perror("Shared memory");
        return 1;
    }
    
    // Attach to the segment to get a pointer to it.
    // shmp_r = shmat(shmid_r, NULL, 0);
    // if (shmp_r == (void *) -1) {
    //     perror("Shared memory attach");
    //     return 1;
    // }

    // shmid_l = shmget(SHM_KEY_LEFT, sizeof(struct shmseg), 0644|IPC_CREAT);
    // if (shmid_l == -1) {
    //     perror("Shared memory");
    //     return 1;
    // }
    
    // // Attach to the segment to get a pointer to it.
    // shmp_l = shmat(shmid_l, NULL, 0);
    // if (shmp_l == (void *) -1) {
    //     perror("Shared memory attach");
    //     return 1;
    // }
    
    // shmid_f = shmget(SHM_KEY_FRONT, sizeof(struct shmseg), 0644|IPC_CREAT);
    // if (shmid_f == -1) {
    //     perror("Shared memory");
    //     return 1;
    // }
    
    // // Attach to the segment to get a pointer to it.
    // shmp_f = shmat(shmid_f, NULL, 0);
    // if (shmp_f == (void *) -1) {
    //     perror("Shared memory attach");
    //     return 1;
    // }

    pthread_t collision_tid;
    //pthread_create(&collision_tid, NULL, collision_detection, (void *)control_state);

    while(1){
        long count = time_ms() - control_state->heartbeat_front;
        if (time_ms() - control_state->heartbeat_front > HEARTBEAT_TO) {
            control_state->front_enabled = 0;
        }
        if (time_ms() - control_state->heartbeat_rear > HEARTBEAT_TO) {
            //control_state->rear_enabled = 0;
        }
    }
}
