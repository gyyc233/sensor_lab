#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>

#define BUFF_SIZE 1024

int main(int argc, char *argv[]) {
  int shm_id; // 共享内存标识符
  int ret;
  key_t key;      // 进程通讯键值
  char *shm_addr; // 共享内存映射地址

  //创建key值
  key = ftok("../", 2025);
  if (key == -1) {
    perror("ftok");
  }

  // 查看系统中的共享存储段
  system("ipcs -m"); //查看共享内存

  //打开共享内存
  shm_id = shmget(key, BUFF_SIZE, IPC_CREAT | 0666);
  if (shm_id < 0) {
    perror("shmget");
    exit(-1);
  }

  // 获取共享内存映射地址
  shm_addr = (char *)shmat(shm_id, NULL, 0);
  if (shm_addr < (char *)0) {
    perror("shmat");
    exit(-1);
  }

  // 读共享内存区数据
  printf("data = [%s]\n", shm_addr);

  // 分离共享内存和当前进程
  ret = shmdt(shm_addr);
  if (ret < 0) {
    perror("shmdt");
    exit(1);
  } else {
    printf("deleted shared-memory\n");
  }

  // clang-format off

    // 共享内存控制 int shmctl(int shmid, int cmd, struct shmid_ds *buf);
    // cmd:
    //  IPC_RMID: 删除
    //  IPC_SET: 设置 shmid_ds 参数，相当于把共享内存原来的属性值替换为 buf 里的属性值
    //  IPC_STAT: 保存 shmid_ds 参数，把共享内存原来的属性值备份到 buf 里
    //  SHM_LOCK: 锁定共享内存段(超级用户)
    //  SHM_UNLOCK: 解锁共享内存段
    //  SHM_LOCK: 用于锁定内存，禁止内存交换,
    //      并不代表共享内存被锁定后禁止其它进程访问。其真正的意义是：被锁定的内存不允许被
    //      交换到虚拟内存中。这样做的优势在于让共享内存一直处于内存中，从而提高程序性能

  // clang-format on
  shmctl(shm_id, IPC_RMID, NULL);

  system("ipcs -m"); //查看共享内存

  pause();

  return 0;
}
