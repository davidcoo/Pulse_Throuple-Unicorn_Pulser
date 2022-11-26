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

#define BITRATE 500000
//#define LOCAL_HOST "169.254.81.79"
#define LOCAL_HOST "169.254.190.42"
#define R_PORT 13606

#define REMOTE_HOST "169.254.81.79"
//#define REMOTE_HOST "169.254.190.42"
#define S_PORT 1000

#define TX_INTERVAL_MS 300
#define HEARTBEAT_INTERVAL 300
#define STATE_SIZE sizeof(DIJOYSTATE2_t)

typedef struct {
    uint8_t enabled; // in collision, so we can still send heartbeat w/ 0 braking pos
    uint8_t sending_heartbeat;
    uint8_t brake_pos;
    uint8_t throttle_pos;
    uint8_t steering_pos;
    uint8_t blink_both;
    uint8_t blink_right;
    uint8_t blink_left;
    uint16_t raw_steering;
    uint16_t raw_throttle;
    uint16_t raw_brake;
    pthread_mutex_t mux_raw; // mutex
} control_state_t;

typedef struct { 
    int sockfd;
    int s;
    struct sockaddr_in servaddr;
    control_state_t *control_state;

} receive_position_info_t;

void *send_force(void *args) {
    int8_t force = 0;
    int sockfd;
    struct sockaddr_in servaddr;

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(S_PORT);
    servaddr.sin_addr.s_addr = inet_addr(REMOTE_HOST);
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("failed to create socket");
        exit(EXIT_FAILURE);
    }

    while (1) {
        usleep(TX_INTERVAL_MS * 1000);
        printf("Send force %d\n", force);
        force += 5;
        sendto(sockfd, (char*) &force, 1, MSG_CONFIRM,
                (struct sockaddr *) &servaddr, sizeof(servaddr));

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
    while(control_state->enabled) {
        int n, len;
        n = recvfrom(sockfd, recvbuf, STATE_SIZE, MSG_WAITALL,
                    (struct sockaddr *) &servaddr, &len);
        uint32_t packet_ct = ((uint32_t*) recvbuf)[0];
        memcpy(&state, recvbuf + 4, sizeof(state));
        pthread_mutex_lock(&(control_state->mux_raw));
        control_state->raw_brake = state.lRz;
        control_state->raw_steering = state.lX;
        contorl_state->raw_throttle = state.lY;
        pthread_mutex_unlock(&(control_state->mux-raw));
        //printf("Receive state (Pkt: %8X) :  Wheel: %d | Throttle: %d | Brake: %d\n", packet_ct, state.lX, state.lY, state.lRz);
    }
}

void *processor(void *args) {
    control_state_t *control_state = (control_state_t *)args;


    // convert position into 0-255 (0-127 left, 129-255 right, 128 is middle) // create some mapping function
    // convert throttle into 1-100
    // convert brake into 1-100
    
    // save back into control state

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


    uint8_t turn_on = 0;
    while (1) {
        if (control_state->sending_heartbeat){
            turn_on = !turn_on;
            frame.data[0] = control_state->brake_pos;
            frame.data[1] = control_state->throttle_pos;
            frame.data[2] = control_state->steering_pos;
            frame.data[3] = control_state->blink_both;
            frame.data[4] = control_state->blink_left;
            frame.data[5] = control_state->blink_right;
            frame.data[6] = 0;
            frame.data[7] = 0;
            nbytes = write (s, &frame, sizeof(frame));
            if (nbytes != sizeof(frame)) {
                printf("Send error frame[0]\r\n");
                system("sudo ifconfig can0 down");
            }
        }
    }
}

void *receive_can(void *args) {

    struct can_frame frame;
    memset(&frame, 0, sizeof(struct can_frame));
    // need to memset frame
    struct can_filter rfilter[2];
    int s = *((int*)args);
    int nbytes;

    rfilter[0].can_id = 0x200;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x300;
    rfilter[1].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    while(1){
        nbytes = read(s, &frame, sizeof(frame));
        if (nbytes > 0){
            printf("can_id = 0x%X\r\ncan_dlc = %d \r\n", frame.can_id, frame.can_dlc);
            int i =0;
            for(i = 0; i < 8; i++)
                printf("data[%d] = %d\r\n", i, frame.data[i]);
        }
        memset(&frame, 0, sizeof(struct can_frame));

    }

    // option 1: check off the heartbeat categories

    // option 2: forward the force
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
    pthread_t send_tid;
    pthread_create(&send_tid, NULL, send_force, NULL);


    memset(&frame, 0, sizeof(struct can_frame));

    //system("sudo ip link set can0 up type can bitrate %d", BITRATE);
    system("sudo ip link set can0 up type can bitrate 500000");
    system("sudo ifconfig can0 txqueuelen 65536");
    // probably want to write a check here to make sure that can0 was down and you're not getting one of those weird errors
    printf("pi is connected to can0\r\n");

    //1. Create Socket
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
    control_state->enabled = 1;
    control_state->sending_heartbeat = 1;
    control_state->brake_pos = 0;
    control_state->throttle_pos = 0;
    control_state->steering_pos = 0;
    control_state->blink_both = 0;
    control_state->blink_right = 0;
    control_state->blink_left = 0;

    pthread_t receive_tid;
    
    printf("hi hit here \n");
    receive_position_info_t r_args;
    receive_position_info_t *receive_args = &r_args;
    receive_args->sockfd = sockfd;
    receive_args->servaddr = servaddr;
    receive_args->control_state = control_state;


    /* initialize a mutex to its default value */
    pthread_mutex_init(&(control_state->mux_raw), NULL);


    pthread_create(&receive_tid, NULL, receive_position, (void *)receive_args);

    pthread_t receive_can_tid;
    pthread_create(&receive_can_tid, NULL, receive_can, &s);
 
    receive_position_info_t s_args;
    receive_position_info_t *send_args = &s_args;
    send_args->s = s;
    send_args->control_state = control_state;
    printf("s before thread: %d \n", s);
    pthread_t send_can_tid;
    pthread_create(&send_can_tid, NULL, send_can, (void *)send_args);

    printf("yuh \n");
    while(1){

    }
}
