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
#define STATE_SIZE sizeof(DIJOYSTATE2_t)

typedef struct {
    uint8_t enabled;
    uint8_t sending_heartbeat;
} control_state_t;


void *send_force(void *arg) {
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

int main()
{
    // first set up CAN
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    control_state_t control_state;

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


    DIJOYSTATE2_t state;
    char recvbuf[sizeof(DIJOYSTATE2_t) + 4];
    pthread_t send_tid;
    pthread_create(&send_tid, NULL, send_force, NULL);


    memset(&frame, 0, sizeof(struct can_frame));

    //system("sudo ip link set can0 up type can bitrate %d", BITRATE);
    system("sudo ip link set can0 up type can bitrate 500000");
    system("sudo ifconfig can0 txqueuelen 65536");
    //system("sudo ifconfig can0 up");
    // probably want to write a check here to make sure that can0 was down and you're not getting one of those weird errors
    printf("pi is connected to can0\r\n");

    //1. Create Socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    printf("socket: %d \n", s);
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

    control_state.enabled = 1;
    control_state.sending_heartbeat = 1;
    while(control_state.enabled){
        int n, len;
        n = recvfrom(sockfd, recvbuf, STATE_SIZE, MSG_WAITALL,
                    (struct sockaddr *) &servaddr, &len);
        uint32_t packet_ct = ((uint32_t*) recvbuf)[0];
        memcpy(&state, recvbuf + 4, sizeof(state));
        printf("Receive state (Pkt: %8X) :  Wheel: %d | Throttle: %d | Brake: %d\n", packet_ct, state.lX, state.lY, state.lRz);
            /*
        if (state.sending_heartbeat) {
            // make this a function to send the heartbeat 
            frame.can_id = 0x446;
            frame.can_dlc = 8;
            frame.data[0] = 0;
            frame.data[1] = 0;
            frame.data[2] = 0;
            frame.data[3] = 0;
            frame.data[4] = 0;
            frame.data[5] = 0;
            frame.data[6] = 0;
            frame.data[7] = 0;
            nbytes = write (s, &frame, sizeof(frame));
            if (nbytes != sizeof(frame)) {
                printf("Send error frame[0]\r\n");
                system("sudo ifconfig can0 down");
            }
        }
	break; */
    }
}
