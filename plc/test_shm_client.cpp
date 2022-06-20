
#include <iostream>
#include <vector>
#include "shm_proxima.h"

int main()
{
	ShMClient* shm_client = new ShMClient;

	std::vector<TYPE_RAW_PV> temp_pv;
	std::vector<TYPE_FILTERED_PV> temp_filteredpv;

	temp_pv = shm_client->read_raw_pv();
	temp_filteredpv = shm_client->read_filtered_pv();
	printf("raw_pv = "); for(int i=0;i<temp_pv.size();i++) printf("%d\t",temp_pv[i]); printf("\n");
	printf("filtered_pv = "); for(int i=0;i<temp_filteredpv.size();i++) printf("%f\t",temp_filteredpv[i]); printf("\n");
	shm_client->view_raw_mv();

	printf("Press Enter\n");
	getchar();
	
	std::vector<TYPE_RAW_MV> temp3{2,0,2,1};
	shm_client->write_raw_mv(temp3);

	temp_pv = shm_client->read_raw_pv();
	temp_filteredpv = shm_client->read_filtered_pv();
	printf("raw_pv = "); for(int i=0;i<temp_pv.size();i++) printf("%d\t",temp_pv[i]); printf("\n");
	printf("filtered_pv = "); for(int i=0;i<temp_filteredpv.size();i++) printf("%f\t",temp_filteredpv[i]); printf("\n");
	shm_client->view_raw_mv();
	

	delete shm_client;
	
	return 0;
}
