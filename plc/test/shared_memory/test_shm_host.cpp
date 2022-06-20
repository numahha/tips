#include <iostream>
#include "shm_proxima.h"

int main()
{
	ShMHost* shm_host = new ShMHost;
	std::vector<TYPE_RAW_MV> temp_mv;
	std::vector<TYPE_RAW_PV> temp_pv(8,7);
	std::vector<TYPE_FILTERED_PV> temp_filteredpv(8,9.8);

	shm_host->write_raw_pv(temp_pv);
	shm_host->write_filtered_pv(temp_filteredpv);
	shm_host->view_raw_pv();
	shm_host->view_filtered_pv();
	temp_mv = shm_host->read_raw_mv();
	printf("raw_mv = "); for(int i=0;i<temp_mv.size();i++) printf("%d\t",temp_mv[i]); printf("\n");

	printf("Press Enter\n");
	getchar();

	shm_host->view_raw_pv();
	shm_host->view_filtered_pv();
	temp_mv = shm_host->read_raw_mv();
	printf("raw_mv = "); for(int i=0;i<temp_mv.size();i++) printf("%d\t",temp_mv[i]); printf("\n");


	delete shm_host;
	return 0;
}
