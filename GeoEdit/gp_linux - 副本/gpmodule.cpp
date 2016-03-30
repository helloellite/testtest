#include "gpmodule.h"
#define EXTEND_NUM 5

using namespace std;
MAP::MAP()
{
	nextTaskID=locationInfo[0]=locationInfo[1]=locationInfo[2]=locationInfo[3]=-1;
	status = RNDF_RAW;
	if(!loadMap(MAP_PATH))
	{
		status=RNDF_ERROR;
		return;
	}
	if(!loadTaskXY(TASKXY_PATH)&&!loadTask(TASK_PATH))
	{
		status=RNDF_ERROR;	
		return;
	}
	status=RNDF_RUN;

}
MAP::~MAP()
{
	int pcount=pLineArray.size();
	for(int i=0;i<pcount;i++)
		delete pLineArray[i];
	pLineArray.clear();
	int ncount=pNavigatorArray.size();
	for(int i=0;i<ncount;i++)
		delete pNavigatorArray[i];
	pNavigatorArray.clear();
}
void MAP::MercatorProjCal( double longitude, double latitude, double *X, double *Y )
{
	*X = longitude *EQUATOR_RADIUS/180;  
	*Y = log(tan((90+latitude)*M_PI/360))/M_PI;  
	(*Y) *= EQUATOR_RADIUS;  
}
void MAP::MercatorProjInvCal( double X, double Y, double *longitude, double *latitude )
{	
	*longitude = (X / EQUATOR_RADIUS) * 180;
	*latitude = (Y / EQUATOR_RADIUS) * 180;
	*latitude = 180/M_PI * (2 * atan(exp(*latitude * M_PI / 180)) - M_PI / 2);
}

void MAP::taskLineComplete()
{
	task.lineID.clear();
	task.index.clear();
	int tnum=task.points.size();

	for(int i=0;i<tnum-1;i++)
	{
		int id=taskLine2GuideLine(i,i+1);
		task.lineID.push_back(id);

		//以下代码可以简略，因为index在taskLine2GuideLine搜索过程中就可以确定了
		if(id!=-1)
		{
			double x1=task.points[i].x,
				x2=task.points[i+1].x,
				y1=task.points[i].y,
				y2=task.points[i+1].y;
			double yaw= 180.0*atan2(y2-y1,x2-x1)/M_PI+270;
			if(yaw>=360) yaw-=360;
			double yaw1,yaw2=yaw1=yaw;
			int p1=i,p2=i+1;
			if(task.points[p1].roadtype/10==1&&task.points[p2].roadtype/10==2)
			{
				yaw1=180*atan2(y1-task.points[p1-1].y,x1-task.points[p1-1].x)/M_PI+270;
				if(yaw1>=360) yaw1-=360;
				yaw2=180*atan2(task.points[p2+1].y-y2,task.points[p2+1].x-x2)/M_PI+270;
				if(yaw2>=360) yaw2-=360;
			}

			int li=lineLocate(x1,y1,yaw1,pLineArray[id]);
			li-=EXTEND_NUM;
			if(li<0) 
				li=0;
			task.index.push_back(li);

			li=lineLocate(x2,y2,yaw2,pLineArray[id]);
			li+=EXTEND_NUM;
			if(li>pLineArray[id]->points.size()-1)
				li=pLineArray[id]->points.size()-1;
			task.index.push_back(li);
		}
		else
		{
			task.index.push_back(-1);		//表示整条线范围
			task.index.push_back(-1);		//表示整条线范围
		}

	}
}
int MAP::taskLine2GuideLine(int p1,int p2)
{
	double x1=task.points[p1].x,
		x2=task.points[p2].x,
		y1=task.points[p1].y,
		y2=task.points[p2].y;

	double yaw= 180.0*atan2(y2-y1,x2-x1)/M_PI+270;
	if(yaw>=360) yaw-=360;
	double yaw1,yaw2=yaw1=yaw;
	if(task.points[p1].roadtype/10==1&&task.points[p2].roadtype/10==2)
	{
		yaw1=180*atan2(y1-task.points[p1-1].y,x1-task.points[p1-1].x)/M_PI+270;
		if(yaw1>=360) yaw1-=360;
		yaw2=180*atan2(task.points[p2+1].y-y2,task.points[p2+1].x-x2)/M_PI+270;
		if(yaw2>=360) yaw2-=360;
	}

	vector<int> l1=searchNearLine(x1,y1,yaw1,NEAR_THREASHOLD),
		l2=searchNearLine(x2,y2,yaw2,NEAR_THREASHOLD),
		l12;
	int c1=l1.size()/2,
		c2=l2.size()/2;
	for(int i=0;i<c1;i++)
	{
		for(int j=0;j<c2;j++)
		{
			if(l1[2*i]==l2[2*j])
			{
				l12.push_back(l1[2*i]);
				l12.push_back(l1[2*i+1]+l2[2*j+1]);
			}
		}
	}
	int c12=l12.size()/2;
	int ans=-1,v=6000;
	for(int i=0;i<c12;i++)
		if(v>l12[2*i+1])
		{
			ans=l12[2*i];
			v=l12[2*i+1];
		}
		if(ans!=-1)
			return ans;
		else
		{
			MBUG("Task from %d to %d multimatched or no match,please check\n",p1,p2);
			return -1;
		}
}
vector<int> MAP::searchNearLine(double x,double y,double yaw,double threashold)
{
	vector<int> ans;
	vector<GuideLine*> * accord=&pLineArray;
	int lnum=accord->size();//pNavigatorArray.size();
	for(int j=0;j<lnum;j++)
	{
		GuideLine* line=(*accord)[j];
		int pnum=line->points.size(),t1=0,t2=pnum-1;
		if(pnum<2) continue;
		for(int i=t1,last_seg_info=isInSegment(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);i<t2;i++)
		{
			 
			//点线位置判定法 （搜索该词可找到所有用到本法的代码，以进行统一修改）
			int seg_info=isInSegment(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
			if(seg_info!=0 )
			{
				if(seg_info!=last_seg_info)
					;/*用注释的方法是为了使本法的代码结构保持一致 */// MBUG("blind zone appear at (x,y,x1,x2,x2,y2,x3,y3)=(%f,%f,%f,%f,%f,%f,%f,%f),\n \twhen locateInfo=(%d,%d,%d)",x,y,line->points[i-1].x,line->points[i-1].y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y,locationInfo[0],locationInfo[1],locationInfo[2]);
				else
					continue;
			}
			if(!isDirectEqual(yaw,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y))
			{
				if(last_seg_info!=seg_info)
				{
					/*用注释的方法是为了使本法的代码结构保持一致 *///MBUG("and blind zone unmatched for director constraint ");	
					last_seg_info=seg_info;
				}
				continue;
			}
			double dis=distance(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
			if(threashold>0&&dis<threashold)
			{//找到则直接搜索下一条
				ans.push_back(j);
				ans.push_back(dis);
				break;		//结束内存循环
			}

			if(last_seg_info!=seg_info)
			{
				/*用注释的方法是为了使本法的代码结构保持一致 *///MBUG("and blind zone distance is %f",dis);
				last_seg_info=seg_info;
			}

			//最后一个距离的判定，要怎么搞？
		}
	}
	return ans;
}

bool MAP::isSingularTaskPoint(int i)	//未考虑出现mid point的情况
{
	int pcount=task.points.size();
	if(i<=0||i>=pcount-1) return false;		//奇异点只能出现在[1,n-2]中——即存在拐角的地方
	double yaw1=atan2(task.points[i].y-task.points[i-1].y,task.points[i].x-task.points[i-1].x),
		yaw2=atan2(task.points[i+1].y-task.points[i].y,task.points[i+1].x-task.points[i].x);
	double dyaw=abs(yaw2-yaw1);
	while(dyaw>2*M_PI) dyaw-=2*M_PI;
	if(dyaw<0) dyaw+=2*M_PI;

	if(dyaw>M_PI) dyaw=2*M_PI-dyaw;

	if(dyaw>M_PI*5.0/12)		//任务点夹角＞75度，则判断为奇异点
		return true;
	else return false;
}
bool MAP::taskLocate(double x,double y,double yaw)
{
	if(task.points.size()==0) return false;
	//如果没有任务线，则生成之。
	if(task.lineID.size()!=task.points.size()-1)
	{
		MBUG("## task line complete\n");
		taskLineComplete();
	}


	if(locationInfo[0]==-1)
	{
		locationInfo[0]=lineLocate(x,y,yaw,&task);
		updateNextTaskID();
	}
	
	int ti=locationInfo[0];
	if(ti==-1) return  false; //ti不见，直接返回，不进行任务点上的全局搜索

	if(task.points[nextTaskID].roadtype/10==1)		//路口入点
	{
		GLPoint p=task.points[nextTaskID];
		if(distanceP2P(x,y,p.x,p.y)<TURN_THRESHOLD)
			ti=nextTaskID;
	}
	/*
	else if(task.points[nextTaskID].roadtype/10==2&&task.points[nextTaskID].roadtype%10==4)
		{
			GLPoint p=task.points[nextTaskID];
			int U_TURN_THRESHOLD=500;
			if(distanceP2P(x,y,p.x,p.y)<U_TURN_THRESHOLD)
				ti=nextTaskID;
	
		}*/
	
	else if(isSingularTaskPoint(nextTaskID))	//下一个点是奇异点
	{
		double cur_dis=distanceP2P(x,y,task.points[nextTaskID].x,task.points[nextTaskID].y);
		if(cur_dis<SINGULAR_THRESHOLD)		//
		{
			int i=nextTaskID;
			double yaw1=(yaw+90)*M_PI/180.0,
				yaw2=atan2(task.points[i+1].y-task.points[i].y,task.points[i+1].x-task.points[i].x);
			double dyaw=abs(yaw2-yaw1);
			while(dyaw>2*M_PI) dyaw-=2*M_PI;
			if(dyaw<0) dyaw+=2*M_PI;
			if(dyaw>M_PI) dyaw=2*M_PI-dyaw;
			if(dyaw<SINGULAR_ANGEL_EQUAL)		//方向已经切对了  此时与真实道路的夹角 最大为 SINGULAR_ANGEL_EQUAL+道路与任务线的夹角
				ti=nextTaskID;
			else if(0==isInSegment(x,y,task.points[nextTaskID].x,task.points[nextTaskID].y,task.points[nextTaskID+1].x,task.points[nextTaskID+1].y)&&isDirectEqual(yaw,task.points[nextTaskID].x,task.points[nextTaskID].y,task.points[nextTaskID+1].x,task.points[nextTaskID+1].y))
				ti=nextTaskID;
		}
	}
	else		//下一个是标准点
	{
		if(1==isInSegment(x,y,task.points[ti].x,task.points[ti].y,task.points[nextTaskID].x,task.points[nextTaskID].y))
			ti=nextTaskID;
		if(ti==task.points.size()-1) 
		{
			status=RNDF_END;
			locationInfo[0]=-1;
			locationInfo[3]=-1;
			nextTaskID=-1;
			return false;
		}
	}
	if(ti!=locationInfo[0])
	{
		locationInfo[0]=ti;
		locationInfo[3]=-1;
		updateNextTaskID();
	}
	return true;
}

//[定位函数
bool MAP::locate(double x,double y,double yaw)
{
	if(!taskLocate(x,y,yaw))
		MBUG("Task Locate default");//return false;		//以后可以采用直接进行全局定位来做
	if(locationInfo[3]!=-1&&!localLocate(x,y,yaw))
		locationInfo[3]=-1;
	if(locationInfo[3]==-1&&!globleLocate(x,y,yaw))			//第一次局部定位失败，开启全局定位
		locationInfo[3]=-1;
	MBUG("locate in line %d at %d",locationInfo[3],locationInfo[2]);
	return locationInfo[3]!=-1;
}
bool MAP::globleLocate(double x,double y,double yaw)
{	
	MBUG("start globleLocate");
	int stt,ed,istt,ied;
	int ti=locationInfo[0];
	if(ti!=-1&&task.lineID[ti]!=-1)	//在任务点层次上找不到，则直接判断定位失败。
	{
		//MBUG("Globle locate faulte,return at task layer.\n");
		stt=task.lineID[ti],ed=task.lineID[ti];
	}
	else
	{
		stt=0,ed=pNavigatorArray.size()-1;
	}
	//以下为通用全局匹配算法，可与task无关
	for(int i=stt;i<=ed;i++)
	{
		GuideLine* navigator=pNavigatorArray[i];	//index in navigator
		int ni=lineLocate(x,y,yaw,navigator);
		locationInfo[1]=ni;
		if(ni==-1)
		{
			locationInfo[3]=-1;
			continue;
		}
		locationInfo[3]=navigator->lineID[ni];		//navigator无可能为0，值为本身
		GuideLine* line=pLineArray[locationInfo[3]];
		int li=lineLocate(x,y,yaw,line,NEAR_THREASHOLD,navigator->index[2*ni],navigator->index[2*ni+1]);		//index in line
		locationInfo[2]=li;
		if(li==-1)
		{
			locationInfo[3]=-1;
			continue;
		}
		return true;			//找到就立马返回
	}
	return false;
}
bool MAP::localLocate(double x,double y,double yaw)
{
	MBUG("start localLocate");
	if(locationInfo[3]==-1)
		return false;
	else	
	{//@tip 需要特殊处理 盲区 的问题
		int li=locationInfo[2];       
		GuideLine* line=pLineArray[locationInfo[3]];
        int pcount=line->points.size();
        int ind=isInSegment(x,y,line->points[li].x,line->points[li].y,line->points[li+1].x,line->points[li+1].y);
		if(ind==0)	//段未发生变化，直接返回
			return true;
		while(	li<locationInfo[2]+10*ind &&
				(li<pcount-1&&li>=0) &&
				ind==isInSegment(x,y,line->points[li].x,line->points[li].y,line->points[li+1].x,line->points[li+1].y)
			)
			li+=ind;
		if(li<locationInfo[2]+10*ind&&li<pcount-1&&li>=0)		//找到了
		{				
			locationInfo[2]=li;
			return true;
		}
		else		//向前没找到
		{
			locationInfo[2]=locationInfo[3]=-1;
			return false;
		}
	}
	
}
int MAP::lineLocate(double x,double y,double yaw,const GuideLine* line,double threshold,int t1/* =-1 */,int t2/* =-1 */)
{
	MBUG("start lineLocate");
	int pnum=line->points.size();
	if(t1<0||t1>=pnum) t1=0;
	if(t2<0||t2>=pnum) t2=pnum-1;
	if(t1>=t2) 
	{
		MBUG("error in lineLocate,t1(%d) should  less than t2(%d)\n",t1,t2);
		return -1;
	}

	int rank[3]={-1,-1,-1};		//前3名
	double Z[3]={100000,100000,100000};				//对应的前三目标函数值Z

	for(int i=t1,last_seg_info=isInSegment(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);i<t2;i++)
	{
		//点线位置判定法（搜索该词可找到所有用到本法的代码，以进行统一修改）
		int seg_info=isInSegment(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
		if(seg_info!=0)
		{
			if(seg_info==-1&&last_seg_info==1)
				MBUG("blind zone appear at (x,y,x1,x2,x2,y2,x3,y3)=(%f,%f,%f,%f,%f,%f,%f,%f),\n \twhen locateInfo=(%d,%d,%d)\n",x,y,line->points[i-1].x,line->points[i-1].y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y,locationInfo[0],locationInfo[1],locationInfo[2]);
			else
				continue;
		}
		if(!isDirectEqual(yaw,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y))
		{
			if(last_seg_info!=seg_info)
			{
				MBUG("and blind zone unmatched for director constraint\n");
				last_seg_info=seg_info;
			}
			continue;
		}
		double dis=distance(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
		if(dis>threshold) return -1; 
		int k=0;
		while(rank[k]!=-1&&Z[k]<dis&&k<3) k++;
		if(k<3)
		{
			int m=2;
			while(m>k)
			{
				Z[m]=Z[m-1];
				rank[m]=rank[m-1];
				m--;
			}
			Z[k]=dis;
			rank[k]=i;
		}
		if(last_seg_info!=seg_info)
		{
			//MBUG("and blind zone distance is %f\n",dis);
			last_seg_info=seg_info;
		}

		//最后一个距离的判定，要怎么搞？
	}

	//TODO important :对distance无阈值约束，而是直接取最小距离。
	return rank[0];
}
//定位函数]

//[辅助函数
int MAP::isInSegment(double x,double y,double sx,double sy,double ex,double ey)
{
	double vx0=ex-sx,vy0=ey-sy,
		vx1=sx-x, vy1=sy-y,
		vx2=ex-x, vy2=ey-y,
		a1=(vx0*vx1+vy0*vy1),
		a2=(vx0*vx2+vy0*vy2);
	if(a1*a2<=0)	return 0;	//在段中
	else if(a1>0) return -1;	//在段的前方
	else return 1;				//在段的后方
}
bool MAP::isDirectEqual(double yaw,double sx,double sy,double ex,double ey)	//yaw是0,360度
{
	yaw=(yaw+90)*M_PI/180;
	double lx=ex-sx,
		ly=ey-sy,
		wx=cos(yaw),
		wy=sin(yaw);
	return lx*wx+ly*wy>0;
}
bool MAP::isInVision(double x,double y,double gps[3])
{
	double orgx=gps[0],orgy=gps[1],yaw=gps[2];
	yaw=(yaw+90)*M_PI/180;
	double vx=cos(yaw),		//竖直方向vertical
		vy=sin(yaw),
		hx=cos(yaw-M_PI/2.0),
		hy=sin(yaw-M_PI/2.0);
	x-=orgx,
		y-=orgy;
	double projx=hx*x+hy*y,
		projy=vx*x+vy*y;
	projy-=1500;
	if(abs(projx)<1500&&abs(projy)<1500)
		return true;
	return false;
}
double MAP::distance(double x,double y,double sx,double sy,double ex,double ey)
{
	double vx1=ex-sx,vx2=x-sx,vy1=ey-sy,vy2=y-sy;
	double vl1=vx1*vx1+vy1*vy1,		//向量1的模的平方
		vl2=vx2*vx2+vy2*vy2,		//向量2模的平方
		vl3=vx1*vx2+vy1*vy2;
	vl3=vl3*vl3;					//向量1、2点乘的平方
	if(vl1==0) 
		vl1=0;
	return sqrt(abs(vl2-vl3/vl1));
}
void MAP::disableLocation()
{
	locationInfo[3]=-1;
}
bool MAP::isLocationAva()
{
	return locationInfo[0]!=-1&&locationInfo[3]!=-1;
}
void MAP::updateNextTaskID()
{
	if(locationInfo[0]==-1||nextTaskID>=(int)task.points.size())		//可以取最后一个值num-1，但是不能比他大
	{
		nextTaskID=-1;
		return;
	}
	if(task.points[locationInfo[0]].roadtype/10==1)	//当前点为拐弯入点
	{
		nextTaskID=getExitPoint(locationInfo[0]);
	}
	else
		nextTaskID=locationInfo[0]+1;


	ofstream ofile(RECOVERY_PATH,std::ios_base::out|std::ios_base::trunc);
	ofile<<nextTaskID+1;
	ofile.close();
}

//辅助函数]


//[读写函数
bool MAP::loadTask(char* filename)
{
	if(task.points.size()!=0) 
	{
		task.points.clear();
		task.lineID.clear();
		task.index.clear();
	}
		ifstream file(filename,ios::in);
		if(file.fail())
			return false;
		GuideLine* line=&task;
		GLPoint point;
		int i=0;  
    	point.roadtype=0;		//用roadtype记录点的属性
		double tip;
		double lon;
		double lat;
		double alt = 0.0;
		double z = 0.0;
		while(!file.eof())
		{
			file>>tip;
			switch(i)
			{
			case 1:
				lon=tip;
				break;
			case 2:
				lat=tip;
				break;
			case 3:
				alt = 0.0;
				MercatorProjCal(lon,lat,&point.x,&point.y);
				break;
			case 4:
				point.roadtype=tip*10;
				break;
			case 5:
	            point.roadtype+=tip;
                line->points.push_back(point);
				i=-1;
			}
			
			i++;	
		}
		file.close();
		return true;
	
}
bool MAP::loadTaskXY(char* filename)
{
	if(task.points.size()!=0) 
	{
		task.points.clear();
		task.lineID.clear();
		task.index.clear();
	}
	ifstream file(filename,ios::in);
	if(file.fail())
		return false;
	GuideLine* line=&task;
	GLPoint point;
	int i=0;  
	point.roadtype=0;		//用roadtype记录点的属性
	while(!file.eof())
	{
		double tip;
		file>>tip;
		switch(i)
		{
		case 1:
			point.x=tip;
			break;
		case 2:
			point.y=tip;
			break;
		case 4:
			point.roadtype=tip*10;
			break;
		case 5:
			point.roadtype+=tip;
			line->points.push_back(point);
			i=-1;
		}
		i++;	
	}
	return true;

}
void MAP::saveTaskXY(char *filename)
{
	GuideLine* line=&task;
	ofstream file(filename);
	if(file.fail())
		return ;

	file.setf(ios::fixed);
	file.precision(7);
	int pcount=line->points.size();
    for(int i=1;i<=pcount;i++)
	{
		GLPoint point=line->points[i-1];
		file<<i<<'\t'<<point.x<<'\t'<<point.y<<'\t'<<'0'<<'\t'<<point.roadtype/10<<'\t'<<point.roadtype%10<<endl;
	}
	file.close();
	
}
bool MAP::loadMap(char* filename)
{
	ifstream file(filename);
	if(file.fail())
		return false;
	//清空清数据
	int lcount=pLineArray.size();
	for(int i=0;i<lcount;i++)
		delete pLineArray[i];
	pLineArray.clear();
	int ncount= pNavigatorArray.size();
	for(int i=0;i<ncount;i++)
		delete pNavigatorArray[i];
	pNavigatorArray.clear();

	//读取
	int lnum;
	file>>bound[0]>>bound[1]>>bound[2]>>bound[3];
	file>>lnum;
	for(int i=0;i<lnum;i++)
	{
		GuideLine* line=new GuideLine;
		file>>line->bound[0] >>line->bound[1] >>line->bound[2] >>line->bound[3] ;

		int pnum;
		file>>pnum ;
		for(int j=0;j<pnum;j++)
		{
			GLPoint point;
			file>>point.x >>point.y >>point.yaw >>point.roadtype ;
			line->points.push_back(point);
		}

		int nnum;
		file>>nnum ;
		for(int j=0;j<nnum;j++)
		{
			int lineID;
			file>>lineID ;
			line->lineID.push_back(lineID);
		}

		int inum=line->index.size();
		file>>inum ;
		for(int j=0;j<inum;j++)
		{
			int idx;
			file>>idx;
			line->lineID.push_back(idx);
		}
		if(pnum!=0)
			pLineArray.push_back(line);
	}

	file>>lnum ;
	for(int i=0;i<lnum;i++)
	{
		GuideLine* line=new GuideLine;
		file>>line->bound[0] >>line->bound[1] >>line->bound[2] >>line->bound[3] ;

		int pnum;
		file>>pnum ;
		for(int j=0;j<pnum;j++)
		{
			GLPoint point;
			file>>point.x >>point.y >>point.yaw >>point.roadtype ;
			line->points.push_back(point);
		}

		int nnum=line->lineID.size();
		file>>nnum ;
		for(int j=0;j<nnum;j++)
		{
			int lineID;
			file>>lineID ;
			line->lineID.push_back(lineID);
		}

		int inum=line->index.size();
		file>>inum ;
		for(int j=0;j<inum;j++)
		{
			int idx;
			file>>idx;
			line->index.push_back(idx);
		}
		pNavigatorArray.push_back(line);
	}

	return true;
}
void MAP::saveMap(char* filename)
{
	ofstream file(filename);
	if(file.fail())
		return;
	file.setf(ios::fixed);
	file.precision(1);
	int lnum=pLineArray.size();
	file<<bound[0]<<' '<<bound[1]<<' '<<bound[2]<<' '<<bound[3]<<' ';
	file<<lnum<<' ';
	for(int i=0;i<lnum;i++)
	{
		
		GuideLine* line=pLineArray[i];
		file<<line->bound[0]<<' '<<line->bound[1]<<' '<<line->bound[2]<<' '<<line->bound[3]<<' ';
		
		int pnum=line->points.size();
		file<<pnum<<' ';
		for(int j=0;j<pnum;j++)
		{
			GLPoint point=line->points[j];
			file<<point.x<<' '<<point.y<<' '<<point.yaw<<' '<<point.roadtype<<' ';
		}

		int nnum=line->lineID.size();
		file<<nnum<<' ';
		for(int j=0;j<nnum;j++)
		{
			int lineID=line->lineID[j];
			file<<lineID<<' ';
		}

		int inum=line->index.size();
		file<<inum<<' ';
		for(int j=0;j<inum;j++)
		{
			file<<line->index[j]<<' ';
		}
	}

	lnum=pNavigatorArray.size();
	file<<lnum<<' ';
	for(int i=0;i<lnum;i++)
	{
		
		GuideLine* line=pNavigatorArray[i];
		file<<1<<' '<<1<<' '<<1<<' '<<1<<' ';

		int pnum=line->points.size();
		file<<pnum<<' ';
		for(int j=0;j<pnum;j++)
		{
			GLPoint point=line->points[j];
			file<<point.x<<' '<<point.y<<' '<<point.yaw<<' '<<point.roadtype<<' ';
		}

		int nnum=line->lineID.size();
		file<<nnum<<' ';
		for(int j=0;j<nnum;j++)
		{
			int lineID=line->lineID[j];
			file<<lineID<<' ';
		}

		int inum=line->index.size();
		file<<inum<<' ';
		for(int j=0;j<inum;j++)
		{
			file<<line->index[j]<<' ';
		}
		
	}

	file.close();
}
//读写函数]


void MAP::intit(unsigned int id,double gps[3])	//id有效范围：【1，最后一个点的id】
{
	id--;
	if(id >= 1 && id <task.points.size())
	{
		//正常情况找到相应编号的任务项
		locationInfo[0] = id - 1;
		if(task.points[locationInfo[0]+1].roadtype/10==1)		//下一个点是拐弯点，判断是否进入拐弯半径，若是，则把该拐弯点当做当前点
		{
			GLPoint p=task.points[locationInfo[0]+1];
			if(distanceP2P(gps[0],gps[1],p.x,p.y)<TURN_THRESHOLD)
				locationInfo[0]++;
		}
		updateNextTaskID();
	}
	else
	{
		//超出上下边界的情况
		if(!taskLocate(gps[0],gps[1],gps[2]))
		{
			locationInfo[0] = -1;
			nextTaskID=-1;
		}
	}
}
void MAP::context(double gps[3],GP_INFO* output)
{
	MercatorProjCal(gps[0],gps[1],&gps[0],&gps[1]);		//车上使用的大地坐标系
	locate(gps[0],gps[1],gps[2]);	//定位失败，则返回

	//填写gls
	if(locationInfo[3]!=-1)
	{
		GuideLine* current_line=pLineArray[locationInfo[3]];
		int clnum=current_line->points.size();
		int maxnum=50+locationInfo[2];
		if(maxnum>clnum) maxnum=clnum;
		bool founded=false;
		int num=0;
		MBUG("## output gls\n");
		for(int i=locationInfo[2]+1;i<maxnum;i++)
		{
			if(!isInVision(current_line->points[i].x,current_line->points[i].y,gps))
				if(!founded)
					continue;
				else break;
			else
			{
				founded=true;
				output->scene.gls.gps[num].x=current_line->points[i].x;
				output->scene.gls.gps[num].y=current_line->points[i].y;
				num++;
			}
		}
		output->scene.gls.valid=num;
		output->scene.road_level=R_LEVEL5;	//结构化道路
	}
	else{
		output->scene.road_level=R_LEVEL5;	//未知
		output->scene.gls.valid=0;
	}
	//填写tips
	if(locationInfo[0]!=-1)
	{
		MBUG("## output tps\n");
		int idx=locationInfo[0];
		output->tps[0].id=idx+1;
		output->tps[0].x=task.points[idx].x;
		output->tps[0].y=task.points[idx].y;
		output->tps[0].type=task.points[idx].roadtype/10;
		output->tps[0].direction=task.points[idx].roadtype%10;

		idx=nextTaskID;
		int tnum=task.points.size();
		for(unsigned int i=1;i<4;i++)
		{
			if(idx>=tnum)
				memset(&output->tps[i],0,sizeof(TASK_POINT_XY));
			else
			{
				output->tps[i].id=idx+1;
				output->tps[i].x=task.points[idx].x;
				output->tps[i].y=task.points[idx].y;
				output->tps[i].type=task.points[idx].roadtype/10;
				output->tps[i].direction=task.points[idx].roadtype%10;
				output->mid[i].rsvd=0;
			}
			idx++;
		}
		int mnum=0;
		MBUG("## output mid,%d,nextTaskID=%d\n",locationInfo[2],nextTaskID);
		for(int i=locationInfo[0]+1;i<nextTaskID;i++)
		{
			output->mid[mnum].id=i+1;
			output->mid[mnum].x=task.points[i].x;
			output->mid[mnum].y=task.points[i].y;
			output->mid[mnum].type=task.points[i].roadtype/10;
			output->mid[mnum].direction=task.points[i].roadtype%10;
			output->mid[mnum].rsvd=0;
			mnum++;
		}
		output->valid_mid=mnum;
	}
	else
	{
		output->valid_mid=0;
		memset(output->tps,0,4*sizeof(TASK_POINT_XY));
	}

	MBUG("status=%d",status);
};


