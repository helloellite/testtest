#pragma once


#include <QObject>
#include <QTime>
#include <QByteArray>
#include <QUdpSocket>

#include <cmath>
#include <fstream>

#include "gp_linux/robix4/protocols/protocol_head.h"

//#define XL_LINUX

#include <gp_linux/NJUST_ALV_BYD_H/NJUST_MC_proc.h>
#include <gp_linux/NJUST_ALV_BYD_H/NJUST_IP_comm.h>
#include <gp_linux/NJUST_ALV_BYD_H/NJUST_MC_data.h>


#pragma pack(push)
#pragma pack(1)
struct INS_RECV_DATA
{
    unsigned char 	head;
    unsigned char 	flag;
    unsigned char 	mode;
    unsigned int	timer;
    int 	  		position[3];
    int 			velocity[3];
    short 			attitude[3];
    short 			gyro[3];
    short 			acclerate[3];
    unsigned char	sum;
};
#pragma pack(pop)
 
const unsigned int SECONDS_PER_WEEK = 604800;
const unsigned int GPS_WEEK_OFFSET = 345600;
#define X_OFFSET_DY 2056580900;
#define Y_OFFSET_DY 361405900;


class ImuSvc : public QObject
{
	Q_OBJECT
public:
	ImuSvc();
	~ImuSvc();
	void setRoadType(unsigned int type);
	static unsigned int roadtype;		//涉及：初始化、设置值、输出到文本中;将POSE_INFO中的checksum字段作为roadtype利用
	static std::ofstream *gps_log;

	public slots:
		
		void enableSvc();
		void disableSvc();
		void startLog();
		void stopLog();
		static int MCCallBack(void *mc_to_map,size_t size,void *args);
signals:
		void dataArrived(POSE_INFO _pose);
	
private:
	//新增
	
	//原有

   

   // void blh2xy(double lat, double lon, double alt, double & x, double & y, double & z);

};
