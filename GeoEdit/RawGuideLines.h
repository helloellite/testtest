
#pragma once
#include <vector>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <cfloat>
 #include<iomanip>
using namespace std;

#include "gp_linux/gpmodule.h"
#include "gp_linux/robix4/protocols/app_gp.h"
#define _USE_MATH_DEFINES
#define TASK_TURN_ENTER 5000



class LineTool :public MAP			//处理GuideLine的线
{
public:
	#define TURN_EXTEND 0		//4*1.5=6米，前后加长6米
	#define  MAX_INTEGER (DBL_MAX)
	#define  MIN_INTEGER (-DBL_MAX) 
	#define NOISE_ANGLETHRESHOLD 60
	
	//预处理函数
	LineTool();
	void taskTurnComplete();
	void noiseDelete(GuideLine* line);
	void reduceSample(GuideLine* line,double LINE_FACTOR=-1,double TURN_FACTOR=-1);
	GuideLine* vehiclePartition(GuideLine* line,const int ,double ,int LINE_LEAST_NUM=-1,int TURN_LEAST_NUM=-1);
	//TASK相关
	
	bool taskAtrributeComplete();
	
	//文件读写
	GuideLine* loadGuideLine(char *filname);		//从filename中加载一条新的引导线，返回该引导线的下标
	//void saveTaskXYReverse(char* filename);
	
	
	
	//增删改查
	void appendAtLine(double x,double y,double yaw)
	{
		GuideLine* line=pLineArray[pLineArray.size()-1];
		GLPoint p={x,y,yaw};
		line->points.push_back(p);
	}
	void appendLine()
	{
		pLineArray.push_back(new GuideLine);
	}
	void deletePoint(GuideLine* line ,int pointID)
	{
		line->points.erase(line->points.begin()+pointID);
		if(line->lineID.size()!=0)
		{
			line->lineID.clear();
			line->index.clear();
		}
	};


	

	

	vector<GuideLine*>* getLines(){return &pLineArray;};
	vector<GuideLine*>* getNavigator(){return &pNavigatorArray;};
	double* getBound(){return bound;};
	int getLinesCount(){return pLineArray.size();};
	GuideLine* getTask(){return &task;};
	

	
	//功能函数
	
	void angleSmooth(double *da,int length);

	double getEquatorRADIUS(){	//只是为了保证参数统一
		return EQUATOR_RADIUS;
	};
};
