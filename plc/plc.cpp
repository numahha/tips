/*
 This code implements PLC communication referring to https://github.com/senrust/pymcprotocol/blob/main/src/pymcprotocol/type3e.py

 Attributes written in type3e.py:
        commtype(str):          communication type. "binary" or "ascii". (Default: "binary") 
        subheader(int):         Subheader for mc protocol
        network(int):           network No. of an access target. (0<= network <= 255)
        pc(int):                network module station No. of an access target. (0<= pc <= 255)
        dest_moduleio(int):     When accessing a multidrop connection station via network, 
                                specify the start input/output number of a multidrop connection source module.
                                the CPU module of the multiple CPU system and redundant system.
        dest_modulesta(int):    accessing a multidrop connection station via network, 
                                specify the station No. of aaccess target module
        timer(int):             time to raise Timeout error(/250msec). default=4(1sec)
                                If PLC elapsed this time, PLC returns Timeout answer.
                                Note: python socket timeout is always set timer+1sec. To recieve Timeout answer.

 We assume using type3e.py's defalut attribute values.

*/

#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <vector>
#include <regex>
#include <time.h> 

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>

#include <chrono>

// user-define parameters
#define AVE_FILTER_WINDOW 25
#define SAMPLING_TIME_MS 200 // seems to be more than 25 mili sec.
#define PLC_IP_ADDRESS "192.168.3.36"
#define PV_PORT 3001
#define MV_PORT 3002

// constant
#define _COMMTYPE_BINARY 1
#define _SOCKBUFSIZE 4096

#if _COMMTYPE_BINARY
#define _WORDSIZE 2
#define _DATA_INDEX 11 // result of data_index = self._get_answerdata_index()
#else 
#define _WORDSIZE 4
#define _DATA_INDEX 22
#endif


#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC 1000000000L
#endif

std::vector<unsigned char> _encode_value(short value )
{
	unsigned char* buffer = new unsigned char(2);
	buffer = (unsigned char*) &value;
	std::vector<unsigned char> value_byte(buffer, buffer+2);
	return value_byte;		
}

std::vector<unsigned char> _make_commanddata(int command , int subcommand)
{
	unsigned char* buffer = new unsigned char(2);
	std::vector<unsigned char> command_data(4);

	buffer = (unsigned char*) &command;
	for(int i=0; i<2; i++) command_data[i] = buffer[i];
	buffer = (unsigned char*) &subcommand;
	for(int i=0; i<2; i++) command_data[i+2] = buffer[i];	
	return command_data;
}

std::pair<char, int> get_binary_devicecode(std::string device)
{
	if(device[0] == 'D')
	{
		std::pair<char, int> devicecode_devicebase(0xA8, 10);
		return devicecode_devicebase;
	} 
	else if(device[0] == 'W')
	{
		std::pair<char, int> devicecode_devicebase(0xB4, 16);
		return devicecode_devicebase;
	} 
	else
	{
		std::cout << "Haven't been implemented." << std::endl;
		std::exit(1);
	}
}

std::string get_device_number(std::string device)
{
	std::smatch results;
	std::regex re(R"(\d.*)");
	std::regex_search(device, results, re);
	return results[0].str();
}

std::vector<unsigned char> _make_devicedata(std::string device)
{
	std::pair<short, int> devicecode_devicebase;
	char devicecode;
	int devicebase;
	std::string devicenum_str;
	int devicenum;	
	unsigned char* buffer = new unsigned char(4);
	std::vector<unsigned char> device_data(4);

	devicecode_devicebase = get_binary_devicecode(device);
	devicecode = devicecode_devicebase.first;
	devicebase = devicecode_devicebase.second;
	devicenum_str = get_device_number(device);
	devicenum = std::stoi(devicenum_str, nullptr, devicebase);
		
	buffer = (unsigned char*) &devicenum; //iQR_SERIES以外
	
	for(int i=0; i<3; i++) device_data[i] = buffer[i];
	device_data[3] = devicecode;
	return device_data;
}


std::vector<unsigned char> _make_senddata(std::vector<unsigned char> requestdata)
{
#if _COMMTYPE_BINARY
        std::vector<unsigned char> ret{0x50,0x00}; // result from subheader = 0x0050 and self.subheader.to_bytes(2, "big")
#else
        std::vector<unsigned char> ret{0x35,0x30,0x30,0x30}; // result from subheader = 0x0050 and format(self.subheader, "x").ljust(4, "0").upper().encode()
#endif
        std::vector<unsigned char> temp5 = _encode_value(requestdata.size()+_WORDSIZE);
        ret.emplace_back(0x00); // result from network = 0
        ret.emplace_back(0xff); // result from pc = 0xFF
        ret.emplace_back(0xff); // result from dest_moduleio = 0X3FF
        ret.emplace_back(0x03); // result from dest_moduleio = 0X3FF
        ret.emplace_back(0x00); // result from dest_modulesta = 0X0
        ret.insert(ret.end(), temp5.begin(), temp5.end());
        ret.emplace_back(0x04); // result from timer=4
        ret.emplace_back(0x00); // result from timer=4
        ret.insert(ret.end(), requestdata.begin(), requestdata.end());

	//for(int i=0; i<ret.size(); i++) printf("%02x",ret[i]); printf("\n"); // view for debug
	return ret;
}


void batchwrite_wordunits(int sockfd, std::string headdevice, std::vector<short> values)
{
	//melsec Qシリーズ(write)
	const short command = 0x1401;
	const short subcommand = 0x0000;

	std::vector<unsigned char> request_data;
	std::vector<unsigned char> value_bin;

	std::vector<unsigned char> command_data = _make_commanddata(command, subcommand);
	std::vector<unsigned char> device_data = _make_devicedata(headdevice);
        std::vector<unsigned char> write_size_bin = _encode_value(values.size());

	request_data.insert(request_data.end(), command_data.begin(), command_data.end());
	request_data.insert(request_data.end(), device_data.begin(), device_data.end());
	request_data.insert(request_data.end(), write_size_bin.begin(), write_size_bin.end());

	for(short value: values)
	{
		value_bin = _encode_value(value);
		request_data.insert(request_data.end(), value_bin.begin(), value_bin.end());
	}

	std::vector<unsigned char> temp_vec = _make_senddata(request_data);
        send(sockfd, temp_vec.data(), temp_vec.size(), 0);
}


std::vector<short> batchread_wordunits(int sockfd, std::string headdevice, int readsize)
{
	//melsec Qシリーズ(read)
	const short command = 0x0401;
	const short subcommand = 0x0000;

	unsigned char r_str[1024]={0};

	std::vector<unsigned char> command_data  = _make_commanddata(command, subcommand);
	std::vector<unsigned char> device_data   = _make_devicedata(headdevice);
        std::vector<unsigned char> read_size_bin = _encode_value(readsize);

	std::vector<unsigned char> request_data;
	request_data.insert(request_data.end(), command_data.begin(), command_data.end());
	request_data.insert(request_data.end(), device_data.begin(), device_data.end());
	request_data.insert(request_data.end(), read_size_bin.begin(), read_size_bin.end());

	std::vector<unsigned char> temp_vec = _make_senddata(request_data);
        send(sockfd, temp_vec.data(), temp_vec.size(), 0);

	recv(sockfd, r_str, _SOCKBUFSIZE, 0);
        //for(int i=0; i<20; i++) printf("%02x",r_str[i]); printf("\n"); // view for debug

        unsigned char temp_val[_WORDSIZE];
        std::vector<short> ret(readsize);
        for(int i=0;i<readsize;i++)
	{
	        for(int j=0;j<_WORDSIZE;j++) temp_val[j] = r_str[ _DATA_INDEX + i*_WORDSIZE + j ];
		ret[i] = (*(short *)&temp_val);
        }
	return  ret;
}


int kbhit(void)
{
    struct termios oldt, newt;
    int ch, oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

int isInterrupt()
{
    if ( kbhit() )
    {
        if ( getchar() == 'q' ) return 1;
    }
    return 0;
}

class AveFilterPV
{
	public:
		AveFilterPV()
		{
			for(int i=0;i<8;i++) sum[i]=0;
		}

		void add_rawdata(std::vector<short> values)
		{
			for(int i=0;i<8;i++)
			{
				sum[i] += values[i];
				queue[i].push(values[i]);
			}
			if (queue[0].size()>AVE_FILTER_WINDOW)
			{
				for(int i=0;i<8;i++){
					sum[i] -= queue[i].front();
					queue[i].pop();
				}
			}
		}

		std::vector<double> get_filtered_data()
		{
			std::vector<double> ret(8);
			for(int i=0;i<8;i++) ret[i] = 1.*sum[i] / queue[i].size();
			return ret;
		}

	private:
		std::queue<short> queue[8];
		short sum[8];
};

static void timespec_add_ns(struct timespec *a, unsigned int ns)
{
	ns += a->tv_nsec;
	while(ns >= NSEC_PER_SEC)
	{
		ns -= NSEC_PER_SEC;
		a->tv_sec++;
	}
	a->tv_nsec = ns;
}

static int timespec_compare(const struct timespec *lhs, const struct timespec *rhs)
{
	if (lhs->tv_sec < rhs->tv_sec) return -1;
	if (lhs->tv_sec > rhs->tv_sec) return 1;
	return lhs->tv_nsec - rhs->tv_nsec;
}


int connect_socket(int port_num)
{
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if(sockfd < 0)
	{
		std::cout << "Error socket:" << std::strerror(errno);
		exit(1);
	}
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port_num); // port number
	addr.sin_addr.s_addr = inet_addr(PLC_IP_ADDRESS); // IP address
	connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
	return sockfd;
}

int main(){

	// time sheduling
	struct timespec tsperiod, tsnow;
	unsigned int control_period_ns = SAMPLING_TIME_MS*1000*1000; // [nano sec] 
	/*
	// real time setting: if not using real time computing, comment out here.

	//struct sched_param param;
	//int policy = SCHED_FIFO;

	memset(&param, 0, sizeof(param));
	param.sched_priority = sched_get_priority_max(policy);
	if (sched_setscheduler(0, policy, &param) < 0)
	{
		printf("sched_setscheduler: %s\n", strerror(errno));
		printf("Please check you are setting /etc/security/limits.conf\n");
		exit (EXIT_FAILURE);
	}
	if (mlockall(MCL_CURRENT|MCL_FUTURE) < 0)
	{
		printf("mlockall: %s\n", strerror(errno));
		exit (EXIT_FAILURE);
	}
	//*/ 

	int sockfd_pv = connect_socket(PV_PORT);
	int sockfd_mv = connect_socket(MV_PORT);

	// read-write loop
	std::vector<short> raw_mv(4,0);
	std::vector<short> raw_pv;
	std::vector<double> filtered_pv;
	AveFilterPV avefilterpv;

	std::chrono::system_clock::time_point  start; // debug time scheduling

	std::cout<<"Press q to exit"<<std::endl;
	clock_gettime(CLOCK_MONOTONIC, &tsperiod);
        while(1){

		start = std::chrono::system_clock::now(); // debug time scheduling
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tsperiod,0); 

		// read
        	raw_pv = batchread_wordunits(sockfd_pv, "W001010", 8);
		avefilterpv.add_rawdata(raw_pv);
		filtered_pv = avefilterpv.get_filtered_data();

		// control
                raw_mv[0] = -raw_pv[0];
                raw_mv[1] = -raw_pv[1];
                raw_mv[2] = -raw_pv[2];
                raw_mv[3] = -raw_pv[3];


		// write
		batchwrite_wordunits(sockfd_mv, "D008510", raw_mv);

		// break if "q" is pressed
        	if(isInterrupt()) break;

		// time scheduling
		timespec_add_ns(&tsperiod, control_period_ns);
		clock_gettime(CLOCK_MONOTONIC, &tsnow);
		if (timespec_compare(&tsperiod, &tsnow)<0)
		{
			printf("overruning!\n");
			tsperiod = tsnow;
			timespec_add_ns(&tsperiod, control_period_ns);
		}

        	printf("pv ="); for(int i=0;i<raw_pv.size();i++) printf("\t%d",raw_pv[i]);
        	printf("\tmv ="); for(int i=0;i<raw_mv.size();i++) printf("\t%d",raw_mv[i]);
		std::cout << "\tduration[ms] = " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now()-start).count()/1000.; // debug time scheduling
        	printf("\r");

        }

	std::cout<<"\nFinish!"<<std::endl;
	close(sockfd_pv);
	close(sockfd_mv);
	return 0;
}

