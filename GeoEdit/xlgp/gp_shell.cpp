#include <fstream>
#include <pthread.h>
#include <NJUST_IP_comm.h>
#include <NJUST_MC_data.h>
#include <NJUST_MC_proc.h>
#include <NJUST_MAP_data.h>
#include <protocol_gp.h>
#include <unistd.h>
#include <sys/time.h>
#include <gpmodule.h>
using namespace std;


#define 	MODULE_NAME 	"MAP"
#define 	TIME_SLICE_MSEC		50		//ms
#define		M_PI			3.1415926535898

double 			longitude_degree;
double 			latitude_degree;
double			yaw_radius;
static 			pthread_mutex_t	gMutex	=	PTHREAD_MUTEX_INITIALIZER;
static 			pthread_cond_t	cond	=	PTHREAD_COND_INITIALIZER;

int NJUST_MAP_Encode_IP_Data(const void* pUnknow,int date,char globle[])
{
    char *pdata=(char *)pUnknow;
    switch (date)
    {
	case 0://road信息
		{
			globle[0]='0';
			memcpy(&globle[1],pdata,sizeof(NJUST_MAP_INFO_ROAD));
			break;
		}
	case 1://node信息
		{
			globle[0]='1';
			memcpy(&globle[1],pdata,sizeof(NJUST_MAP_INFO_NODE));
			break;
		}

	case 2://方向导引
	    {
			globle[0] = '2';
	//		memcpy(&globle[1], pdata, sizeof(NJUST_MAP_INFO_DIRECTION));
			break;
	    }
    }
	return 0;

}

int MCCallBack(void *mc_to_map,size_t size,void *args)
{
	NJUST_MC_STATE_INFO *pState;
	NJUST_MC_NAV_INFO 	*pNav;
	NJUST_MC_DRIVE_INFO *pDrive;
	NJUST_MC_Decode_IP_Data(mc_to_map,size,&pState,&pNav,&pDrive);
	if(pNav)
	{
		pNav->Yaw_rad=2*3.14159265358979-pNav->Yaw_rad;
		pthread_mutex_lock(&gMutex);
		longitude_degree=pNav->Longitude_degree;
		latitude_degree=pNav->Latitude_degree;
		yaw_radius=pNav->Yaw_rad;
	
	
		pthread_cond_signal(&cond);
		pthread_mutex_unlock(&gMutex);
	}
	return 0;
}
int MOCallBack( void* mo_to_pl, size_t size, void* args)
{
	return 0;
}
void* Processing(void* ptr)
{
	double lon,lat;

	timeval cur,lst;
	gettimeofday(&lst,NULL);
	
	GP_INFO output;
	output.header.id=0;
	MAP gp;
	while(1)
	{
		gettimeofday(&cur,NULL);
		int time_cost=(cur.tv_sec-lst.tv_sec)*1000+(cur.tv_usec-lst.tv_usec)/1000;
		if(time_cost<TIME_SLICE_MSEC)
			usleep((TIME_SLICE_MSEC-time_cost)*1000);
		lst=cur;
		pthread_mutex_lock(&gMutex);	
		//@warning:  there may be error here,at using of cond
		pthread_cond_wait(&cond,&gMutex);
		lon=longitude_degree;
		lat=latitude_degree;

		pthread_mutex_unlock(&gMutex);
		//log
		
		printf("\tprocessing:\n\tlon=%f,lat=%f\n",lon,lat);
		output.header.id++;
		
		double gps[3];
		gps[2]=180.0*yaw_radius/M_PI;
		gps[0]=lon;
		gps[1]=lat;
		gp.context(gps,&output);
		//output
		NJUST_IP_tcp_send_to("PL",&output,sizeof(output));	//send frame
		NJUST_IP_tcp_send_to("FU",&output,sizeof(output));
		NJUST_MAP_INFO_NODE node;
		node.FrameID=output.header.id;
		node.synTime = NJUST_IP_get_time();
		int count=node.GPSPointQueuelength=output.scene.gls.valid;
		for(int i=0;i<count;i++)
		{
			node.nextGPSPointQueue[i].longtitude=output.scene.gls.gps[i].x;
			node.nextGPSPointQueue[i].latitude	=output.scene.gls.gps[i].y;
		}
		char buff[1024];
		NJUST_MAP_Encode_IP_Data(&node, 1, buff);
		NJUST_IP_udp_send_to("MO", buff, 1024);

	}
}

int main()
{

	
	/* step 1
	 * @brief: globle var initial
	 */

	/* step 1 
	*  @brief: register gp module
	*/
	printf("aaa");
	if(NJUST_IP_moduleName_exist(MODULE_NAME))
	{
		NJUST_IP_set_moduleName(MODULE_NAME,0);
		printf("a");
	}
	printf("register success\n");
	/* step 3
	 * @brief: set callback function
	 */
	if(-1==NJUST_IP_set_tcp_callBack("MO",MOCallBack,NULL))
	{
		printf("can't connect MO\n");
		return -1;
	}//recieve instruction from MO

	if(-1==NJUST_IP_set_broadcast_callBack(MCCallBack,NULL))
	{
		printf("can't connect MC\n");
		return -1;
	}//recieve GPS data from broadcast

	pthread_t processing;
	pthread_create(&processing,NULL,Processing,NULL);
	void *result;
	pthread_join(processing,&result);
	return 0;
}
