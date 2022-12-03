/* Filename: shm_write.c */
#include<stdio.h>
#include<sys/ipc.h>
#include<sys/shm.h>
#include<sys/types.h>
#include<string.h>
#include<errno.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>

#define BUF_SIZE 1024

#define SHM_KEY_RIGHT 0X1234
#define SHM_KEY_LEFT 0X5678
#define SHM_KEY_FRONT 0x1357

struct shmseg {
   int cnt;
   int complete;
   char buf[BUF_SIZE];
};
int fill_buffer(char * bufptr, char *values, int size);

int shmid;
struct shmseg *shmp;
int enabled = 1;
int set_up(long shm_key){
   shmid = shmget(shm_key, sizeof(struct shmseg), 0644|IPC_CREAT);
   if (shmid == -1) {
      perror("Shared memory");
      return 1;
   }
   
   shmp = shmat(shmid, NULL, 0);
   if (shmp == (void *) -1) {
      perror("Shared memory attach");
      return 1;
   } 
   return 0;

}

int test_vals(){
   printf("shmid: %d \n", shmid);
   (void)(shmp->cnt);
   (void)(shmp->complete);
   (void)(shmp->buf);
   return shmp->cnt;
}
int write_vals(char *values[]) {
   char bufvals[16];
   int numtimes;
   char *bufptr;
   int spaceavailable;
   
   //printf("values: %s\n", values);
   /* Transfer blocks of data from buffer to shared memory */
   memset(bufvals, 0, 16);
   memcpy(bufvals, values, 16);
   bufptr = shmp->buf;
   spaceavailable = 16;
   shmp->cnt = fill_buffer(bufptr, bufvals, spaceavailable);
   shmp->complete = 0;
   bufptr = shmp->buf;
   spaceavailable = 16;   
   return 0;
}

void cleanup() {
   shmp->complete = 1;

   if (shmdt(shmp) == -1) {
      perror("shmdt");
   }

   if (shmctl(shmid, IPC_RMID, 0) == -1) {
      perror("shmctl");
   }
   printf("shmid: %d\n", shmid);
   printf("Writing Process: Complete\n");
}

int fill_buffer(char * bufptr, char *values, int size) {
   int filled_count;
   char src[] = "what's up dogs!";
   memcpy(bufptr, values, size-1);
   bufptr[size-1] = '\0';
   filled_count = strlen(bufptr);
   return filled_count;
}