#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h> // share memory
#include <sys/shm.h> // share memory
#include <sys/types.h>
#include <unistd.h>

#define BUFF_SIZE 1024 // bytes

int main(int argc, char *argv[]) {
  int shm_id;     // 共享内存标识符
  key_t key;      // 进程通信键值
  char *shm_addr; // 共享内存映射地址

  // 创建key值
  key = ftok("../", 2025);
  if (key == -1) {
    perror("ftok");
  }

  // 创建共享内存, IPC_CREAT 不存在则创建, 0666 读写权限
  shm_id = shmget(key, BUFF_SIZE, IPC_CREAT | 0666);
  if (shm_id < 0) {
    perror("shmget");
    exit(-1);
  }

  // shmat 将一个共享内存段映射到调用进程的数据段中,
  // 让进程和共享内存建立一种联系，让进程某个指针指向此共享内存
  // 成功则返回共享内存映射地址，失败返回-1
  // void* 是通用指针类型，可指向任何类型数据，转成char*
  // 要注意类型安全和内存对齐
  shm_addr = (char *)shmat(shm_id, NULL, 0);
  if (shm_addr < (char *)0) {
    perror("shmat");
    _exit(-1);
  }

  // int shmdt(const void *shmaddr); 解除共享内存映射
  // 将共享内存和当前进程分离(仅仅是断开联系并不删除共享内存，相当于让之前的指向此共享内存的指针，不再指向)

  //拷贝数据至共享内存区
  printf("copy data to shared-memory\n");
  bzero(shm_addr, BUFF_SIZE); // 共享内存清空
  // bzero 也可换成 memset(shm_addr,0,BUFF_SIZE);
  strcpy(shm_addr, "ding dong ji, da gou jiao\n");

  pause();

  return 0;
}
