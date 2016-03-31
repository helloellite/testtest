#include <vector>
#include <cmath>
#include<fstream>
#include<iostream>
#include <memory.h>
/*
@info:当无任务点文件或当前所在点任务点匹配失败时时，跟随当前附近的线行动；
	  
*/


using namespace std;
#define		M_PI					3.14159265358979323846
#define		TURN_THRESHOLD			3000		//控制拐弯点提前多少米触发
#define		SINGULAR_THRESHOLD		400		
#define		SINGULAR_ANGEL_EQUAL	(M_PI/9.0)		//20度，视为同方向 即方向已经与任务点的方向（未必是道路的方向）一致
//#define USING_ROBIX4
#ifndef USING_ROBIX4
    #include  <app_gp.h>
	#include <QDebug>
	#define		MBUG				qDebug
	#define		RECOVERY_PATH		"recovery"
	#define		MAP_PATH			"map.mp"
	#define		TASK_PATH			"task"
	#define		TASKXY_PATH			"task.xy"
#else
	#define		MBUG			printf	
	#include "protocols/app_gp.h"
	#define		RECOVERY_PATH	"recovery"
	#define		MAP_PATH		"map.mp"
	#define		TASK_PATH		"task"
	#define		TASKXY_PATH		"task.xy"
#endif


#define		NEAR_THREASHOLD		1500		//单位 cm ,判断道线是否在点周围时的距离阈值
#define		EQUATOR_RADIUS		2003750834//538855318.87441504946122116593191   // 63781370/0.11836455494812478	//地球赤道周长

struct GLPoint		//guide point
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

enum RNDF_STATUS
{
	RNDF_RAW    = 0,    /* 路网读取 */
	RNDF_BUILD  = 1,    /* 路网有效 */
	RNDF_INI    = 2,    /* 路网初始化 */
	RNDF_RUN    = 3,    /* 路网运行 */
	RNDF_NO_TASK= 4,
	RNDF_END    = 5,    /* 路网完成 */
	RNDF_ERROR  = 6     
};

class MAP	//带任务的地图
{
public:
	vector<GuideLine*> pLineArray;	//引导线（路线）集，包含所有的引导线
	vector<GuideLine*> pNavigatorArray;	//导航线，与pLinesArray对应。 每个引导线有对应的一个Navigator（双射关系）。
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
	RNDF_STATUS status;


	static void MercatorProjCal( double longitude, double latitude, double *X, double *Y );
	static void MercatorProjInvCal( double X, double Y, double *longitude, double *latitude );
	
	//预处理函数
	void taskLineComplete();		//补全task中的line结构。
	int taskLine2GuideLine(int p1,int p2);			//最重要的函数之一，根据确定任务点(p1,p2)段所对应的引导线。
	vector<int> searchNearLine(double x,double y,double yaw,double threashold);	//搜索(x,y)垂直距离不大于threashold 且 方向与yaw "同向" 的所有道路ID

	//使用了点线位置判定法


	//定位函数
	bool locate(double x,double y,double yaw);			//定位，根据x，y，yaw设置当前info。
	bool globleLocate(double x,double y,double yaw);	//全局定位,设置的结果放在locationInfo中
	bool localLocate(double x,double y,double yaw);		//利用locationInfo信息定位，并更新locationInfo
	int lineLocate(double x,double y,double yaw,const GuideLine* line,double thre=NEAR_THREASHOLD,int t1=-1,int t2=-1);	//定位(x,y,yaw)在引导线line[t1,t2]部分的位置，若不在该引导线上则返回1。0<=t1<t2<=num-1，t1（t2）若不在范围内，则强制改为0(num-1)。
	bool taskLocate(double x,double y,double yaw);
	bool isSingularTaskPoint(int i);
	
	//文件读写
	bool loadTask(char* filename);
	bool loadTaskXY(char* filename);
	void saveTaskXY(char *filename);
	bool loadMap(char *filename);
	void saveMap(char* filename);

	void intit(unsigned int id,double gps[3]);
	void context(double gps[3],GP_INFO* output);



	//protected:
	
	//辅助函数
	void disableLocation();
	bool isLocationAva();
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
	RNDF_STATUS stat(){
		return status;
	};


	
	//正规性检查
	bool taskAttributeVerify();			//检查任务点的属性是否合适。 主要检查入口点与出口点的完整性
	bool connectiveVerify();			//验证在地图上，任务点之间是否可达

};

