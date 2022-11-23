#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <can_header_temp.h>

typedef struct {
    uint8_t enabled;
    uint8_t sending_heartbeat;
} control_state_t;
int main()
{
    // first set up CAN
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct control_state_t state;

    memset(&frame, 0, sizeof(struct can_frame));


    //1. Create Socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
        return 1;
    }

    //2. Specify can0 device
    strcpy(if.ifr_name, "can0");
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

    //4. don't disable filtering rules
    // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    state.enabled = 1;
    state.sending_heartbeat = 1;
    while(state.enabled){
        if (state.sending_heartbeat) {
            // make this a function to send the heartbeat 
            frame.can_id = 0x100;
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
            } )
        }
    }
}