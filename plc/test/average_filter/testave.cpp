
#include <iostream>
#include <vector>
#include <queue>

#define AVE_FILTER_WINDOW 2

class AveFilterPV
{
	public:
		AveFilterPV(){
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

int main()
{
	AveFilterPV avefilterpv;
	for(int i=0;i<20;i++)
	{
		std::vector<short> raw_pv(8,i);
		avefilterpv.add_rawdata(raw_pv);
		std::vector<double> temp = avefilterpv.get_filtered_data();
		for(int j=0;j<4;j++) printf("\t%d",raw_pv[j]);
		for(int j=0;j<4;j++) printf("\t%f",temp[j]);
		printf("\n");
	}


	return 0;
}
