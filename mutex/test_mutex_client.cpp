
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
  FILE *proxima_shm_mutex_fp;

  char proxima_shm_mutex_key_path[64] = "/home/khi01/proxima_shm_mutex_key";


  const int mutex_id = 40;
  //proxima_shm_mutex_fp = fopen(proxima_shm_mutex_key_path, "w");
  //fclose(proxima_shm_mutex_fp);
  const key_t mutex_key = ftok(proxima_shm_mutex_key_path, mutex_id);
  shmid = shmget(mutex_key, 0, 0);


  printf("0\n");
  m = (pthread_mutex_t*)shmat(shmid, 0, 0);
  printf("1\n");
  //m = (pthread_mutex_t*)mmap(NULL, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANON, -1, 0);
/*
  pthread_mutexattr_init(&mat);

  pthread_mutexattr_setpshared(&mat, PTHREAD_PROCESS_SHARED);

  printf("1");
  pthread_mutex_init(m, &mat);
*/
  printf("2\n");

  pthread_mutex_lock(m);

  printf("press enter\n");

  getchar();

  pthread_mutex_unlock(m);

  shmdt(m);


  return 0;
}
