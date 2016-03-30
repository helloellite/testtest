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

		//���´�����Լ��ԣ���Ϊindex��taskLine2GuideLine���������оͿ���ȷ����
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
			task.index.push_back(-1);		//��ʾ�����߷�Χ
			task.index.push_back(-1);		//��ʾ�����߷�Χ
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
			 
			//����λ���ж��� �������ôʿ��ҵ������õ������Ĵ��룬�Խ���ͳһ�޸ģ�
			int seg_info=isInSegment(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
			if(seg_info!=0 )
			{
				if(seg_info!=last_seg_info)
					;/*��ע�͵ķ�����Ϊ��ʹ�����Ĵ���ṹ����һ�� */// MBUG("blind zone appear at (x,y,x1,x2,x2,y2,x3,y3)=(%f,%f,%f,%f,%f,%f,%f,%f),\n \twhen locateInfo=(%d,%d,%d)",x,y,line->points[i-1].x,line->points[i-1].y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y,locationInfo[0],locationInfo[1],locationInfo[2]);
				else
					continue;
			}
			if(!isDirectEqual(yaw,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y))
			{
				if(last_seg_info!=seg_info)
				{
					/*��ע�͵ķ�����Ϊ��ʹ�����Ĵ���ṹ����һ�� *///MBUG("and blind zone unmatched for director constraint ");	
					last_seg_info=seg_info;
				}
				continue;
			}
			double dis=distance(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);
			if(threashold>0&&dis<threashold)
			{//�ҵ���ֱ��������һ��
				ans.push_back(j);
				ans.push_back(dis);
				break;		//�����ڴ�ѭ��
			}

			if(last_seg_info!=seg_info)
			{
				/*��ע�͵ķ�����Ϊ��ʹ�����Ĵ���ṹ����һ�� *///MBUG("and blind zone distance is %f",dis);
				last_seg_info=seg_info;
			}

			//���һ��������ж���Ҫ��ô�㣿
		}
	}
	return ans;
}

bool MAP::isSingularTaskPoint(int i)	//δ���ǳ���mid point�����
{
	int pcount=task.points.size();
	if(i<=0||i>=pcount-1) return false;		//�����ֻ�ܳ�����[1,n-2]�С��������ڹսǵĵط�
	double yaw1=atan2(task.points[i].y-task.points[i-1].y,task.points[i].x-task.points[i-1].x),
		yaw2=atan2(task.points[i+1].y-task.points[i].y,task.points[i+1].x-task.points[i].x);
	double dyaw=abs(yaw2-yaw1);
	while(dyaw>2*M_PI) dyaw-=2*M_PI;
	if(dyaw<0) dyaw+=2*M_PI;

	if(dyaw>M_PI) dyaw=2*M_PI-dyaw;

	if(dyaw>M_PI*5.0/12)		//�����нǣ�75�ȣ����ж�Ϊ�����
		return true;
	else return false;
}
bool MAP::taskLocate(double x,double y,double yaw)
{
	if(task.points.size()==0) return false;
	//���û�������ߣ�������֮��
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
	if(ti==-1) return  false; //ti������ֱ�ӷ��أ�������������ϵ�ȫ������

	if(task.points[nextTaskID].roadtype/10==1)		//·�����
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
	
	else if(isSingularTaskPoint(nextTaskID))	//��һ�����������
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
			if(dyaw<SINGULAR_ANGEL_EQUAL)		//�����Ѿ��ж���  ��ʱ����ʵ��·�ļн� ���Ϊ SINGULAR_ANGEL_EQUAL+��·�������ߵļн�
				ti=nextTaskID;
			else if(0==isInSegment(x,y,task.points[nextTaskID].x,task.points[nextTaskID].y,task.points[nextTaskID+1].x,task.points[nextTaskID+1].y)&&isDirectEqual(yaw,task.points[nextTaskID].x,task.points[nextTaskID].y,task.points[nextTaskID+1].x,task.points[nextTaskID+1].y))
				ti=nextTaskID;
		}
	}
	else		//��һ���Ǳ�׼��
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

//[��λ����
bool MAP::locate(double x,double y,double yaw)
{
	if(!taskLocate(x,y,yaw))
		MBUG("Task Locate default");//return false;		//�Ժ���Բ���ֱ�ӽ���ȫ�ֶ�λ����
	if(locationInfo[3]!=-1&&!localLocate(x,y,yaw))
		locationInfo[3]=-1;
	if(locationInfo[3]==-1&&!globleLocate(x,y,yaw))			//��һ�ξֲ���λʧ�ܣ�����ȫ�ֶ�λ
		locationInfo[3]=-1;
	MBUG("locate in line %d at %d",locationInfo[3],locationInfo[2]);
	return locationInfo[3]!=-1;
}
bool MAP::globleLocate(double x,double y,double yaw)
{	
	MBUG("start globleLocate");
	int stt,ed,istt,ied;
	int ti=locationInfo[0];
	if(ti!=-1&&task.lineID[ti]!=-1)	//������������Ҳ�������ֱ���ж϶�λʧ�ܡ�
	{
		//MBUG("Globle locate faulte,return at task layer.\n");
		stt=task.lineID[ti],ed=task.lineID[ti];
	}
	else
	{
		stt=0,ed=pNavigatorArray.size()-1;
	}
	//����Ϊͨ��ȫ��ƥ���㷨������task�޹�
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
		locationInfo[3]=navigator->lineID[ni];		//navigator�޿���Ϊ0��ֵΪ����
		GuideLine* line=pLineArray[locationInfo[3]];
		int li=lineLocate(x,y,yaw,line,NEAR_THREASHOLD,navigator->index[2*ni],navigator->index[2*ni+1]);		//index in line
		locationInfo[2]=li;
		if(li==-1)
		{
			locationInfo[3]=-1;
			continue;
		}
		return true;			//�ҵ���������
	}
	return false;
}
bool MAP::localLocate(double x,double y,double yaw)
{
	MBUG("start localLocate");
	if(locationInfo[3]==-1)
		return false;
	else	
	{//@tip ��Ҫ���⴦�� ä�� ������
		int li=locationInfo[2];       
		GuideLine* line=pLineArray[locationInfo[3]];
        int pcount=line->points.size();
        int ind=isInSegment(x,y,line->points[li].x,line->points[li].y,line->points[li+1].x,line->points[li+1].y);
		if(ind==0)	//��δ�����仯��ֱ�ӷ���
			return true;
		while(	li<locationInfo[2]+10*ind &&
				(li<pcount-1&&li>=0) &&
				ind==isInSegment(x,y,line->points[li].x,line->points[li].y,line->points[li+1].x,line->points[li+1].y)
			)
			li+=ind;
		if(li<locationInfo[2]+10*ind&&li<pcount-1&&li>=0)		//�ҵ���
		{				
			locationInfo[2]=li;
			return true;
		}
		else		//��ǰû�ҵ�
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

	int rank[3]={-1,-1,-1};		//ǰ3��
	double Z[3]={100000,100000,100000};				//��Ӧ��ǰ��Ŀ�꺯��ֵZ

	for(int i=t1,last_seg_info=isInSegment(x,y,line->points[i].x,line->points[i].y,line->points[i+1].x,line->points[i+1].y);i<t2;i++)
	{
		//����λ���ж����������ôʿ��ҵ������õ������Ĵ��룬�Խ���ͳһ�޸ģ�
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

		//���һ��������ж���Ҫ��ô�㣿
	}

	//TODO important :��distance����ֵԼ��������ֱ��ȡ��С���롣
	return rank[0];
}
//��λ����]

//[��������
int MAP::isInSegment(double x,double y,double sx,double sy,double ex,double ey)
{
	double vx0=ex-sx,vy0=ey-sy,
		vx1=sx-x, vy1=sy-y,
		vx2=ex-x, vy2=ey-y,
		a1=(vx0*vx1+vy0*vy1),
		a2=(vx0*vx2+vy0*vy2);
	if(a1*a2<=0)	return 0;	//�ڶ���
	else if(a1>0) return -1;	//�ڶε�ǰ��
	else return 1;				//�ڶεĺ�
}
bool MAP::isDirectEqual(double yaw,double sx,double sy,double ex,double ey)	//yaw��0,360��
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
	double vx=cos(yaw),		//��ֱ����vertical
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
	double vl1=vx1*vx1+vy1*vy1,		//����1��ģ��ƽ��
		vl2=vx2*vx2+vy2*vy2,		//����2ģ��ƽ��
		vl3=vx1*vx2+vy1*vy2;
	vl3=vl3*vl3;					//����1��2��˵�ƽ��
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
	if(locationInfo[0]==-1||nextTaskID>=(int)task.points.size())		//����ȡ���һ��ֵnum-1�����ǲ��ܱ�����
	{
		nextTaskID=-1;
		return;
	}
	if(task.points[locationInfo[0]].roadtype/10==1)	//��ǰ��Ϊ�������
	{
		nextTaskID=getExitPoint(locationInfo[0]);
	}
	else
		nextTaskID=locationInfo[0]+1;


	ofstream ofile(RECOVERY_PATH,std::ios_base::out|std::ios_base::trunc);
	ofile<<nextTaskID+1;
	ofile.close();
}

//��������]


//[��д����
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
    	point.roadtype=0;		//��roadtype��¼�������
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
	point.roadtype=0;		//��roadtype��¼�������
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
	//���������
	int lcount=pLineArray.size();
	for(int i=0;i<lcount;i++)
		delete pLineArray[i];
	pLineArray.clear();
	int ncount= pNavigatorArray.size();
	for(int i=0;i<ncount;i++)
		delete pNavigatorArray[i];
	pNavigatorArray.clear();

	//��ȡ
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
//��д����]


void MAP::intit(unsigned int id,double gps[3])	//id��Ч��Χ����1�����һ�����id��
{
	id--;
	if(id >= 1 && id <task.points.size())
	{
		//��������ҵ���Ӧ��ŵ�������
		locationInfo[0] = id - 1;
		if(task.points[locationInfo[0]+1].roadtype/10==1)		//��һ�����ǹ���㣬�ж��Ƿ�������뾶�����ǣ���Ѹù���㵱����ǰ��
		{
			GLPoint p=task.points[locationInfo[0]+1];
			if(distanceP2P(gps[0],gps[1],p.x,p.y)<TURN_THRESHOLD)
				locationInfo[0]++;
		}
		updateNextTaskID();
	}
	else
	{
		//�������±߽�����
		if(!taskLocate(gps[0],gps[1],gps[2]))
		{
			locationInfo[0] = -1;
			nextTaskID=-1;
		}
	}
}
void MAP::context(double gps[3],GP_INFO* output)
{
	MercatorProjCal(gps[0],gps[1],&gps[0],&gps[1]);		//����ʹ�õĴ������ϵ
	locate(gps[0],gps[1],gps[2]);	//��λʧ�ܣ��򷵻�

	//��дgls
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
		output->scene.road_level=R_LEVEL5;	//�ṹ����·
	}
	else{
		output->scene.road_level=R_LEVEL5;	//δ֪
		output->scene.gls.valid=0;
	}
	//��дtips
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


