#ifndef SHM_HOST_GUEST_H
#define SHM_HOST_GUEST_H

#include <iostream>
#include <string>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <string.h>
#include <vector>

#define USE_MUTEX 0

#define SHM_DIR_NAME "shm"

#define TYPE_RAW_PV int
#define TYPE_RAW_MV int
#define TYPE_FILTERED_PV double


#if USE_MUTEX
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/wait.h>
#endif


const int pv_num=8;
const int mv_num=4;

const int id_raw_pv = 11;
const int id_raw_mv = 12;
const int id_filtered_pv = 13;
const std::string filename_raw_pv = "raw_pv.dat";
const std::string filename_raw_mv = "raw_mv.dat";
const std::string filename_filtered_pv = "filtered_pv.dat";

#if USE_MUTEX
const int mutex_id_raw_pv      = 1001;
const int mutex_id_raw_mv      = 1002;
const int mutex_id_filtered_pv = 1003;
const std::string mutex_filename_raw_pv = "mutex_raw_pv.dat";
const std::string mutex_filename_raw_mv = "mutex_raw_mv.dat";
const std::string mutex_filename_filtered_pv = "mutex_filtered_pv.dat";
#endif


class ShMHost{
	private:
		int segid_raw_pv;
		int segid_raw_mv;
		int segid_filtered_pv;
		TYPE_RAW_PV* shm_raw_pv;
		TYPE_RAW_MV* shm_raw_mv;
		TYPE_FILTERED_PV* shm_filtered_pv;
#if USE_MUTEX
		pthread_mutex_t *m_raw_pv;
		pthread_mutexattr_t mat_raw_pv;
		int mshmid_raw_pv;
		pid_t pid_raw_pv;
#endif

	public:
		ShMHost();
		~ShMHost();
		void write_raw_pv     (std::vector<TYPE_RAW_PV> raw_pv);
		void write_filtered_pv(std::vector<TYPE_FILTERED_PV> filtered_pv);
		std::vector<TYPE_RAW_MV> read_raw_mv();
		void view_raw_pv();
		void view_filtered_pv();
};


ShMHost::ShMHost()
{
	// prepare shared memory
	const char *homepath = std::getenv("HOME");
	std::string homepath_s = &homepath[0];
	const std::string filepath_dir = homepath_s + "/" + SHM_DIR_NAME;
	struct stat st;
	if(stat(filepath_dir.c_str(), &st) != 0)
	{
		mkdir(filepath_dir.c_str(), 0775);
	}

	const std::string filepath_raw_pv      = filepath_dir + "/" + filename_raw_pv;
        const std::string filepath_raw_mv      = filepath_dir + "/" + filename_raw_mv;
        const std::string filepath_filtered_pv = filepath_dir + "/" + filename_filtered_pv;

        FILE *fp;
        fp = fopen(filepath_raw_pv.c_str(), "w");
        fclose(fp);
        key_t key_raw_pv = ftok(filepath_raw_pv.c_str(), id_raw_pv);
        fp = fopen(filepath_raw_mv.c_str(), "w");
        fclose(fp);
        key_t key_raw_mv = ftok(filepath_raw_mv.c_str(), id_raw_mv);
        fp = fopen(filepath_filtered_pv.c_str(), "w");
        fclose(fp);
        key_t key_filtered_pv = ftok(filepath_filtered_pv.c_str(), id_filtered_pv);
        if(key_raw_pv == -1 || key_raw_mv == -1 || key_filtered_pv == -1 )
	{
        	std::cerr << "Failed to acquire key for shared memory host" << std::endl;
		exit(1);
        }

        segid_raw_pv      = shmget(key_raw_pv,      pv_num*sizeof(TYPE_RAW_PV),    IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR);
        segid_raw_mv      = shmget(key_raw_mv,      mv_num*sizeof(TYPE_RAW_MV),    IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR);
        segid_filtered_pv = shmget(key_filtered_pv, pv_num*sizeof(TYPE_FILTERED_PV), IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR);
        if(segid_raw_pv == -1 || segid_raw_mv == -1|| segid_filtered_pv == -1)
	{
		std::cerr << "Failed to acquire segment for shared memory host" << std::endl;
		exit(1);
        }

        shmdt(shm_raw_pv);
        shm_raw_pv = (TYPE_RAW_PV*)(shmat(segid_raw_pv, 0, 0));
        shmdt(shm_raw_mv);
        shm_raw_mv = (TYPE_RAW_MV*)(shmat(segid_raw_mv, 0, 0));
        shmdt(shm_filtered_pv);
        shm_filtered_pv = (TYPE_FILTERED_PV*)(shmat(segid_filtered_pv, 0, 0));

	for(int i=0;i<pv_num;i++) shm_raw_pv[i] = 0;
	for(int i=0;i<mv_num;i++) shm_raw_mv[i] = 0;
	for(int i=0;i<pv_num;i++) shm_filtered_pv[i] = 0.;

#if USE_MUTEX
	const std::string mutex_filepath_raw_pv      = filepath_dir + "/" + mutex_filename_raw_pv;
        const std::string mutex_filepath_raw_mv      = filepath_dir + "/" + mutex_filename_raw_mv;
        const std::string mutex_filepath_filtered_pv = filepath_dir + "/" + mutex_filename_filtered_pv;

	mshmid_raw_pv = shmget(IPC_PRIVATE, sizeof(pthread_mutex_t), 0600);
	m_raw_pv = shmat(mshmid_raw_pv, NULL, 0);
	pthread_mutexattr_init(&mat_raw_pv);
	pthread_mutexattr_setpshared(&mat_raw_pv, PTHREAD_PROCESS_SHARED)
	pthread_mutex_init(m_raw_pv, &mat_raw_pv);
#endif
        printf("shared memory host constructor finishes.\n");
}
    

ShMHost::~ShMHost(){
	shmdt(shm_raw_pv);
	shmdt(shm_raw_mv);
	shmdt(shm_filtered_pv);

        shmctl(segid_raw_pv,      IPC_RMID, NULL);
        shmctl(segid_raw_mv,      IPC_RMID, NULL);
        shmctl(segid_filtered_pv, IPC_RMID, NULL);
        printf("shared memory host destructor finishes.\n");
};

void ShMHost::write_raw_pv(std::vector<TYPE_RAW_PV> raw_pv)
{
	for(int i=0;i<pv_num;i++) shm_raw_pv[i] = raw_pv[i];
}

void ShMHost::write_filtered_pv(std::vector<TYPE_FILTERED_PV> filtered_pv)
{
	for(int i=0;i<pv_num;i++) shm_filtered_pv[i] = filtered_pv[i];
}

std::vector<TYPE_RAW_MV> ShMHost::read_raw_mv()
{
	std::vector<TYPE_RAW_MV> ret(mv_num);
	for(int i=0;i<mv_num;i++) ret[i] = shm_raw_mv[i];
	return ret;
}

void ShMHost::view_raw_pv()
{
	printf("raw_pv = ");
	for(int i=0;i<pv_num;i++) printf("%d\t",shm_raw_pv[i]);
	printf("\n");
}

void ShMHost::view_filtered_pv()
{
	printf("filtered_pv = ");
	for(int i=0;i<pv_num;i++) printf("%f\t",shm_filtered_pv[i]);
	printf("\n");
}



class ShMClient{
	private:
		int segid_raw_pv;
		int segid_raw_mv;
		int segid_filtered_pv;
		TYPE_RAW_PV* shm_raw_pv;
		TYPE_RAW_MV* shm_raw_mv;
		TYPE_FILTERED_PV* shm_filtered_pv;
	public:
		ShMClient();
		std::vector<TYPE_RAW_PV>  read_raw_pv();
		std::vector<TYPE_FILTERED_PV> read_filtered_pv();
		void write_raw_mv(std::vector<TYPE_RAW_MV> raw_mv);
		void view_raw_mv();
};

ShMClient::ShMClient()
{
        const char *homepath   = std::getenv("HOME");
        std::string homepath_s = &homepath[0];
	const std::string filepath_dir = homepath_s + "/" + SHM_DIR_NAME;

        const std::string filepath_raw_pv      = filepath_dir + "/" + filename_raw_pv;
        const std::string filepath_raw_mv      = filepath_dir + "/" + filename_raw_mv;
        const std::string filepath_filtered_pv = filepath_dir + "/" + filename_filtered_pv;

        const key_t key_raw_pv = ftok(filepath_raw_pv.c_str(), id_raw_pv);
        const int segid_raw_pv = shmget(key_raw_pv, 0, 0);

        const key_t key_raw_mv = ftok(filepath_raw_mv.c_str(), id_raw_mv);
        const int segid_raw_mv = shmget(key_raw_mv, 0, 0);

        const key_t key_filtered_pv = ftok(filepath_filtered_pv.c_str(), id_filtered_pv);
        const int segid_filtered_pv = shmget(key_filtered_pv, 0, 0);

        if(segid_raw_pv == -1 || segid_raw_mv == -1 || segid_filtered_pv == -1)
	{
        	std::cerr << "Failed to acquire segment for shared memory client" << std::endl;
		exit(1);
        }

	shm_raw_pv      = (TYPE_RAW_PV*)(shmat(segid_raw_pv, 0, 0));
	shm_raw_mv      = (TYPE_RAW_MV*)(shmat(segid_raw_mv, 0, 0));
	shm_filtered_pv = (TYPE_FILTERED_PV*)(shmat(segid_filtered_pv, 0, 0));
        printf("shared memory client constructor finishes.\n");
}

void ShMClient::write_raw_mv(std::vector<TYPE_RAW_MV> raw_mv)
{
	for(int i=0;i<mv_num;i++) shm_raw_mv[i] = raw_mv[i];
}

std::vector<TYPE_RAW_PV> ShMClient::read_raw_pv()
{
	std::vector<TYPE_RAW_PV> ret(pv_num);
	for(int i=0;i<pv_num;i++) ret[i] = shm_raw_pv[i];
	return ret;
}

std::vector<TYPE_FILTERED_PV> ShMClient::read_filtered_pv()
{
	std::vector<TYPE_FILTERED_PV> ret(pv_num);
	for(int i=0;i<pv_num;i++) ret[i] = shm_filtered_pv[i];
	return ret;
}


void ShMClient::view_raw_mv()
{
	printf("raw_mv = ");
	for(int i=0;i<mv_num;i++) printf("%d\t",shm_raw_mv[i]);
	printf("\n");
}


#endif
