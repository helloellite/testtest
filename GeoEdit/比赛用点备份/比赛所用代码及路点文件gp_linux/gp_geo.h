#include <vector>
#include <cmath>
#include<fstream>
#include<iostream>



using namespace std;
#define M_PI    3.14159265358979323846
#define  TURN_THRESHOLD 3000		//控制拐弯点提前多少米触发
#define  SINGULAR_THRESHOLD 400		
#define SINGULAR_ANGEL_EQUAL (M_PI/9.0)		//20度，视为同方向 即方向已经与任务点的方向（未必是道路的方向）一致
#ifndef USING_ROBIX4
#include "robix4/protocols/app_gp.h"
#include <QDebug>
#define MBUG qDebug
#define RECOVERY_PATH "recovery"
#define  MAP_PATH "map.mp"
#define  TASK_PATH "task"
#define  TASKXY_PATH "task.xy"
#else
#include "protocols/app_gp.h"
#include "rbx4api.h"
#define RECOVERY_PATH "/home/alv/ugv/bin/config/recovery"
#define  MAP_PATH "/home/alv/ugv/bin/config/map.mp"
#define  TASK_PATH "/home/alv/ugv/bin/config/task"
#define  TASKXY_PATH "/home/alv/ugv/bin/config/task.xy"

#endif


#define NEAR_THREASHOLD 1500		//单位 cm ,判断道线是否在点周围时的距离阈值
#define  EQUATOR_RADIUS 538855318.87441504946122116593191   // 63781370/0.11836455494812478	//地球赤道周长
#define X_OFFSET_DY 2056580900;
#define Y_OFFSET_DY 361405900;
struct GLPoint		//guideline point
{
	double x,y,yaw;
	unsigned int roadtype;		//道路类型
};
struct GuideLine			//
{
	vector<GLPoint> points; //组成GuideLine的点
	vector<int> lineID;	//
	vector<int> index;			//points定位到i，对应的具象层的line为line中的(2i，2i+1）表示 	
	double bound[4];
};
struct junction{
	vector<int> adjacent;	//相邻路口，逆时针排列
	vector<int> lines;		//拐弯线，与adjacent的下标匹配
	vector<int> roads_code;		//通往下一个路口的道路码。 编码方式： roads_code=roadID*10+line_ID_in_road

	int center[2];			//记录junction圆心位置。 junction是以center（x,y）为圆心，radius（定值）为半径的圆。
};
struct road{
	vector<int> lines;		//道路中的引导线，通常只有两条：来回方向各一条
	vector<int> start;		//引导线起点；下标与lines匹配
	vector<int> terminal;	//引导线终点；下标与lines匹配
};
enum RNDF_STATUS
{
	RNDF_RAW    = 0,    /* 路网读取 */
	RNDF_BUILD  = 1,    /* 路网有效 */
	RNDF_INI    = 2,    /* 路网初始化 */
	RNDF_RUN    = 3,    /* 路网运行 */
	RNDF_END    = 4,    /* 路网完成 */
	RNDF_ERROR  = 6     /* 路网出错 */
};
//tips：junction与road都用index寻址。
//优点：方便错误日志的输出与错误的定位。 若用指针寻址，则无法知道错误发生在第几个线
//缺点：当pLines内发生变动时，整个pLines与junctions、roads都需要发生变动
class MAP	//带任务的地图
/************************************************************************/
/*本地图约束：两个路口具有唯一的简单道路；                              */
/************************************************************  m m m   m m m m       m, ,m************/
{
	


protected:
	vector<GuideLine*> pLineArray;	//引导线（路线）集，包含所有的引导线
	vector<GuideLine*> pNavigatorArray;	//导航线，与pLinesArray对应。 每个引导线有对应的一个Navigator（双射关系）。
	vector<junction> junctionArray;	//拓扑-路口集，地图两个结构之一，包含所有的路口。	  两个路口之间存在唯一的简单道路
	vector<road> roadListArray;		//拓扑-简单道路集，地图双结构之一，包含所有简单道路。  简单道路：道路上没有路口
	GuideLine task;
	double bound[4];
	//task.points   任务文件。非Map必须。 可从Map继承出TaskMap，用TaskMap包含本变量。此处仅为了方便，比赛后可做模块细分。与任务有关的都加task前缀
							//任务点4要素：经纬度，高度，属性  分别用GPPoint的x,y,yaw,type四个属性替代。
	//task.line task.index 表示task.point[i]到task.point[i+1]中间对应的引导线码。 
	

	/************************************************************************/
	/* 全局定位过程描述：task→navigator→line                              */
	/************************************************************************/
	int locationInfo[4];	//从0-2分别为：在task.points中的下标，在navigator[j]中的下标，在Line[k]中的下标  并未直接记录各条线本身的ID，但可通过一定方法寻找到
							//3中存放引导线的线ID，也就是说(3,2)表示在线中的位置
	int nextTaskID;
public:
	MAP();
	~MAP();
	double lastPos[2];
	bool isSingularTaskPoint(int i);
	bool taskAtrributeComplete();
	void taskLineComplete();		//补全task中的line结构。
	bool locate(double x,double y,double yaw);			//定位，根据x，y，yaw设置当前info。
	bool taskLocate(double x,double y,double yaw);
	RNDF_STATUS stat(){
		return status;
	};
    void intit(unsigned int id,double gps[3]);
    void context(double gps[3],GP_INFO* output);
	//文件读写
	bool loadTask(char* filename);
	bool loadTaskXY(char* filename);
	void saveTaskXY(char *filename);
	bool loadMap(char *filename);
	void saveMap(char* filename);



protected:
	int taskLine2GuideLine(int p1,int p2);			//最重要的函数之一，根据确定任务点(p1,p2)段所对应的引导线。
	RNDF_STATUS status;
    bool globleLocate(double x,double y,double yaw);	//全局定位,设置的结果放在locationInfo中
	bool localLocate(double x,double y,double yaw);		//利用locationInfo信息定位，并更新locationInfo
	
    void updateNextTaskID();
    int isInSegment(double x,double y,double sx,double sy,double ex,double ey);		//判断点(x,y)是否垂直投影于线段(sx,sy)-(ex,ey)上（包含顶点）。
	bool isDirectEqual(double yaw,double sx,double sy,double ex,double ey);		//line_dir为道路的法向量的角度		//判断当前航向角yaw与定位线向量dir是否同向：-夹角在90~90内为同向
	double distance(double x,double y,double sx,double sy,double ex,double ey);
	static	double distanceP2P(double x1,double y1,double x2,double y2)		//return the distance of point (x1,y1) to point (x2,y2).
	{
		return sqrt((y1-y2)*(y1-y2)+(x1-x2)*(x1-x2));
	};
	bool isInVision(double x,double y,double gps[3]);
	int getExitPoint(int entrance){
        int tcount=task.points.size();
        for(int i=entrance+1;i<tcount;i++)
            if(task.points[i].roadtype/10==2)
                return i;
         return -1;
    };	//根据拐弯入口点的位置，获得相应的出口点
	
	//以下代码可能很简单，仅为了增加灵活性以适应修改

	void disableLocation();
	bool isLocationAva();
//	int coor2Junction(double x,double y){return -1;}; //根据坐标点的位置，返回其所在的Junction下标
//	int coor2Road(double x,double y){return -1;};	  //根据坐标点的位置，返回其所在的道路的下标
//	bool isCoorInJunciton(double x,double y,int junction){return true;};		//判断(x,y)是否在junction里

	int lineLocate(double x,double y,double yaw,const GuideLine* line,double thre=NEAR_THREASHOLD,int t1=-1,int t2=-1);	//定位(x,y,yaw)在引导线line[t1,t2]部分的位置，若不在该引导线上则返回1。0<=t1<t2<=num-1，t1（t2）若不在范围内，则强制改为0(num-1)。
																							//不指定t1，t2时，则默认全线搜索
																							//注意处理盲区部分（参考GuideLine的locate里的初次定位）。
																							//无先验信息定位（即仅根据这三个，不考虑前一帧的位置）。 约束强度：___（待填TODO？）
																							//使用了点线位置判定法

	vector<int> searchNearLine(double x,double y,double yaw,double threashold);	//搜索(x,y)垂直距离不大于threashold 且 方向与yaw "同向" 的所有道路ID
																				//使用了点线位置判定法
																				
	//正规性检查
	bool taskAttributeVerify();			//检查任务点的属性是否合适。 主要检查入口点与出口点的完整性
	bool connectiveVerify();			//验证在地图上，任务点之间是否可达

public:
static	double latToY(double lat)
	{
		/*
		lat*=M_PI/180.0;		//转换为弧度
				return 2*M_PI*EQUATOR_RADIUS*log(tan(lat)+1.0/cos(lat));*/
		double x,y,z;
		blh2xy(lat,0,0,x,y,z);
		return y;
	};
static	double lonToX(double lon)
	{
		double x,y,z;
		blh2xy(0,lon,0,x,y,z);
		return x;
	};
static	double xToLon(double x)
	{
		double lon=120.7743707;
		lon*=M_PI/180.0;
		double lx=EQUATOR_RADIUS*lon-1106864141 ;

		double dx=lx-72287107.7270920;
		x+=dx;
		x+=1106864141;
		x/=EQUATOR_RADIUS;		//转换为弧度	
		return x*180.0/M_PI;
	};
static	double yToLat(double y)
	{
		double lat=31.59290948;
		double yy=log(tan((lat+90)/360*M_PI));
		double ly=yy*EQUATOR_RADIUS-324964598;

		double dy=ly+11513980.3467803;
		y+=dy;
		y+=324964598;
		y/=EQUATOR_RADIUS;
		return 2.0*atan(exp(y))*180/M_PI-90;
	};


static void blh2xy(double lat, double lon, double alt, double & x, double & y, double & z)
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
};

