#include "imusvc.h"
unsigned int ImuSvc::roadtype;
std::ofstream *ImuSvc::gps_log;
ImuSvc::ImuSvc()
{

#ifdef XL_LINUX
	if(NJUST_IP_moduleName_exist("SMP"))
		NJUST_IP_set_moduleName("SMP",0);
	NJUST_IP_set_broadcast_callBack(MCCallBack,NULL);
#endif
	roadtype=0;
}

ImuSvc::~ImuSvc()
{
	if(gps_log!=NULL)
	{ 
		gps_log->flush();
		gps_log->close();
		gps_log=NULL;
	}
}

void ImuSvc::enableSvc()
{
	return;
}

void ImuSvc::startLog()
{
	if(gps_log!=NULL) return;
	gps_log=new std::ofstream;
	QDateTime::currentDateTime();
	gps_log->open((QDateTime::currentDateTime().toString("yy_MM_dd_hh_mm_ss")+".log").toLocal8Bit().data(), std::ios_base::out | std::ios_base::trunc);
}
void ImuSvc ::stopLog()
{
	if(gps_log==NULL) return;
	gps_log->flush();
	gps_log->close();
	delete gps_log;
	gps_log=NULL;
}

void ImuSvc::disableSvc()
{
	
    return;
}
int ImuSvc::MCCallBack(void *mc_to_map,size_t size,void *args)
{
	NJUST_MC_STATE_INFO *pState;
	NJUST_MC_NAV_INFO 	*pNav;
	NJUST_MC_DRIVE_INFO *pDrive;
	POSE_INFO pi;
	NJUST_MC_Decode_IP_Data(mc_to_map,size,&pState,&pNav,&pDrive);
	if(pNav)
	{
		pNav->Yaw_rad=2*3.14159265358979-pNav->Yaw_rad;
		//@warinng : 涉及坐标转换
		memset(&pi, 0, sizeof(POSE_INFO));
		pi.ins_coord.x = 100*pNav->EarthRefCoord[0];
		pi.ins_coord.y = 100*pNav->EarthRefCoord[1];
		pi.z = 0;
		pi.roll = int(pNav->Roll_rad*1e8);
		pi.pitch = int(pNav->Pitch_rad*1e8);
		pi.yaw = int(pNav->Yaw_rad*1e8);
		pi.spd = pNav->Speed_cm_ps;
		pi.acc.x = pNav->IMU_A_xyz_mm_ps[0]/10;
		pi.acc.y = pNav->IMU_A_xyz_mm_ps[1]/10;
		pi.acc.z = pNav->IMU_A_xyz_mm_ps[2]/10;
		pi.checksum=roadtype;		//新增，tips:将checksum暂时作为roadtype
	//	emit dataArrived(pi);
		if(gps_log!=NULL)
		{
			QString lat = QString::number(pNav->Latitude_degree, 'f', 7);
			QString lon = QString::number(pNav->Longitude_degree, 'f', 7);
			QString alt = QString::number(pNav->Altitude_m, 'f', 3);
			QString xx = QString::number((int)pi.ins_coord.x);
			QString yy = QString::number((int)pi.ins_coord.y);
			QString yaw = QString::number(pNav->Yaw_rad*180/3.1415926535898, 'f', 7);
			QString pitch = QString::number(pNav->Pitch_rad*180/3.141592653589, 'f', 7);
			QString roll = QString::number(pNav->Roll_rad*180/3.141592653589, 'f', 7);
			QString speed = QString::number(pi.spd);
			QString time = QTime::currentTime().toString("hh:mm:ss:zzz");
			QString log = QString("LAT=%1, LON=%2, ALT=%3, X=%4, Y=%5, YAW=%6, TYPE=%7, PITCH=%8, ROLL=%9, SPEED=%10, TIME=%11").arg(lat).arg(lon).arg(alt).arg(xx).arg(yy).arg(yaw).arg(roadtype).arg(pitch).arg(roll).arg(speed).arg(time);
			(*gps_log) << log.toStdString() << std::endl;
		}

		
	}
	return 0;
}


/*

void ImuSvc::blh2xy(double lat, double lon, double alt, double & x, double & y, double & z)
{
	int n, L0;
	double X, N54, W54, t, m, a54, e54, e_54;
	double iptr;
	double t_2 = 0, t_4 = 0, yita_2 = 0, yita_4 = 0;
	double lp = 0, lp_2 = 0;
	double SinL, CosL, CosL_2, SinL_2;
	double SinG, CosG;
	double daa, df, db2p, dl2p, dahm;
	double deltabo, deltalo;
	double w84, n84, m84, a84, e842, f84, f54, dx, dy, dz;
	double lati, logi, hegt;
	double pi = 3.1415926535;
	lati = lat;
	logi = lon;
	hegt = alt;
	lati = lati*pi / 180;
	logi = logi*pi / 180;
	SinL = sin(lati);
	CosL = cos(lati);
	SinG = sin(logi);
	CosG = cos(logi);
	CosL_2 = CosL * CosL;
	SinL_2 = SinL * SinL;
	a84 = 6378137.0;
	e842 = 0.00669437999014132;
	f84 = 1.0 / 298.257223563;
	a54 = 6378245.0;
	f54 = 1.0 / 298.3;
	dx = -16.0;
	dy = 147.0;
	dz = 77.0;
	w84 = sqrt(1 - e842*SinL_2);
	n84 = a84 / w84;
	m84 = a84*(1 - e842) / (w84*w84*w84);
	daa = a54 - a84;
	df = f54 - f84;
	db2p = (-dx*SinL*CosG - dy*SinL*SinG + dz*CosL + (a84*df + f84*daa)*sin(2 * lati)) / (m84*sin(1 / 3600.0*pi / 180));
	dl2p = (-dx*SinG + dy*CosG) / (n84*CosL*sin(1 / 3600.0*pi / 180));
	dahm = dx*CosL*CosG + dy*CosL*SinG + dz*SinL + (a84*df + f84*daa)*SinL_2 - daa;
	deltabo = (db2p / 3600.0)*pi / 180.0;
	deltalo = (dl2p / 3600.0)*pi / 180.0;
	logi = logi + deltalo;
	lati = lati + deltabo;
	hegt = hegt + dahm;
	SinL = sin(lati);
	CosL = cos(lati);
	CosL_2 = CosL * CosL;
	SinL_2 = SinL * SinL;
	a54 = 6378245.0;
	e54 = 0.0066934274898192;
	W54 = sqrt(1.0 - e54*SinL_2);
	N54 = a54 / W54;
	e_54 = 0.0067385254147;
	logi = logi * 180 / pi;
	modf(logi / 6.0, &iptr);
	n = (int)iptr + 1;
	L0 = n * 6 - 3;
	lp = (logi - L0)*pi / 180;
	lp_2 = lp*lp;
	m = CosL_2*lp_2;
	yita_2 = e_54*CosL_2;
	yita_4 = yita_2 * yita_2;
	t = tan(lati);
	t_2 = t*t;
	t_4 = t_2*t_2;
	X = 111134.8611*lati * 180 / pi
		- SinL*CosL*(32005.7799 + 133.9238*SinL_2 + 0.6973*SinL_2*SinL_2 + 0.0039*SinL_2*SinL_2*SinL_2);
	y = X + N54*t*m*(0.5 + 1.0 / 24.0*(5.0 - t_2 + 9.0*yita_2 + 4.0*yita_4)*m
		+ 1.0 / 720.0*(61.0 - 58.0*t_2 + t_4)*m*m);
	x = N54*CosL*lp*(1.0 + 1.0 / 6.0*(1 - t_2 + yita_2)*m
		+ 1.0 / 120.0*(5.0 - 18.0*t_2 + t_4 + 14.0*yita_2 - 58.0*yita_2*t_2)*m*m);
	x = x + 1000000 * n + 500000;
	x = x*100 - X_OFFSET_DY;
	y = y*100 - Y_OFFSET_DY;
	z = alt*100;
	return;
}
*/

void ImuSvc::setRoadType(unsigned int type)
{
	roadtype=type;
	qDebug("type=%d",roadtype);
}
