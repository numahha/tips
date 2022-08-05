
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/shm.h>
#include <pthread.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/stat.h>
int
main()
{
  pthread_mutex_t *m;
  pthread_mutexattr_t mat;
  int shmid;
  pid_t pid;
  FILE *proxima_shm_mutex_fp;

  char proxima_shm_mutex_key_path[64] = "/home/khi01/proxima_shm_mutex_key";


  const int mutex_id = 40;
  proxima_shm_mutex_fp = fopen(proxima_shm_mutex_key_path, "w");
  fclose(proxima_shm_mutex_fp);
  const key_t mutex_key = ftok(proxima_shm_mutex_key_path, mutex_id);
  shmid = shmget(mutex_key, sizeof(pthread_mutex_t), IPC_CREAT | IPC_EXCL| S_IRUSR | S_IWUSR);
  //if (shmid < 0) {
  //  perror("shmget");
  //  return 1;
  //}


  m = (pthread_mutex_t*)shmat(shmid, 0, 0);
  //m = (pthread_mutex_t*)mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANON, -1, 0);

  /* mutexのattributeを設定する準備 */
  pthread_mutexattr_init(&mat);

  /* mutexをプロセス間で利用する設定を行う */
  /* これを行わないとプロセス内でのみ有効のmutexになります */
  pthread_mutexattr_setpshared(&mat, PTHREAD_PROCESS_SHARED);

  printf("1");
  pthread_mutex_init(m, &mat);

  printf("press enter\n");

  getchar();

  printf("2");
  pthread_mutex_lock(m);

  printf("press enter\n");

  getchar();

  pthread_mutex_unlock(m);

  shmdt(m);

  if (shmctl(shmid, IPC_RMID, NULL) != 0) {
    perror("shmctl");
    return 1;
  }


  return 0;
}
