#include "RawGuideLines.h"
#include <QFile>
#include <QTextStream>

LineTool::LineTool()
{
	if(pLineArray.size()==0)
	{	bound[1]=bound[3]=MIN_INTEGER;
	bound[2]=bound[0]=MAX_INTEGER;}
}

void LineTool::taskTurnComplete()
{
	int pcount=task.points.size();
	for(int i=1;i<pcount;i++)
	{
		 if(task.points[i].roadtype/10==1)	//只给路口点增加
		 {
			 double dy=task.points[i-1].y-task.points[i].y,
				 dx=task.points[i-1].x-task.points[i].x;
			 if(dx==0&&dy==0) continue;
			 if(dx*dx+dy*dy<(TASK_TURN_ENTER+1)*(TASK_TURN_ENTER+1))
			 {
				 qDebug("Point %d has a close prior,no adding excute",i);
				 continue;
			 }
			double  agl=atan2(dy,dx);
			 double x=task.points[i].x+TASK_TURN_ENTER*cos(agl),
				 y=task.points[i].y+TASK_TURN_ENTER*sin(agl);
			 GLPoint pt;
			 pt.x=x;
			 pt.y=y;
			 pt.roadtype=31;
			 task.points.insert(task.points.begin()+i,pt);
			 i++;
			 pcount++;
		 }
	}
}
void LineTool::noiseDelete(GuideLine* line)
{
	int length=line->points.size();
	double *dir=new double[length];
	for(int i=0;i<length-1;i++)
	{
		double dx=line->points[i+1].x-line->points[i].x,
			dy=line->points[i+1].y-line->points[i].y;
		dir[i]=atan2(dy,dx)*180.0/M_PI;
	}
	for(int i=length-2;i>0;i--)
	{
		double da=dir[i]-dir[i-1];
		if(da<0)
			da=-da;
		if(da>180.0)
			da=360-da;
		if(da>NOISE_ANGLETHRESHOLD)
			deletePoint(line,i);
	}
}
void LineTool::reduceSample(GuideLine* line,double LINE_FACTOR,double TURN_FACTOR)//降采样仅仅更新line中的点信息，不会动线段映射关系lineID与index
{
	noiseDelete(line);
	if(LINE_FACTOR<0||TURN_FACTOR<0)
		LINE_FACTOR=150,TURN_FACTOR=150;
	GuideLine *new_line=new GuideLine;

	int pcount=line->points.size();		//点数
	new_line->points.push_back(line->points[0]);//先放入第一个点
	double factor=150,dis=0;
	for(int pi=0;pi<pcount-1;pi++)
	{
		double current_dis=distanceP2P(line->points[pi].x,line->points[pi].y,line->points[pi+1].x,line->points[pi+1].y);
		dis+=current_dis;

		GLPoint last=line->points[pi];
		while(dis>factor)
		{
			double rate=(dis-factor)/current_dis;		//折pi点A，pi+1点B，插值点C  rate表示CB/AB

			GLPoint np;
			//统统线性插值

			np.x=(1-rate)*line->points[pi+1].x+rate*line->points[pi].x;
			np.y=(1-rate)*line->points[pi+1].y+rate*line->points[pi].y;
			np.yaw=(1-rate)*line->points[pi+1].yaw+rate*line->points[pi].yaw;	
			np.roadtype=last.roadtype;
			//插入新的点								
			last=np;
			new_line->points.push_back(np);
			dis-=factor;			
		}
	}
	if(dis>factor/2.0)
		new_line->points.push_back(line->points[pcount-1]);
    line->points.clear();
    int nlcount=new_line->points.size();
    for(int i=0;i<nlcount;i++)
		line->points.push_back(new_line->points[i]);
	delete new_line;
}
GuideLine* LineTool::vehiclePartition(GuideLine* line,int lineID,double factor,int LINE_LEAST_NUM,int TURN_LEAST_NUM)	//staihtline的结构：[s[2i],s[2i+1]]为直线，[s[2i+1],s[2i+2]]为弯道。线段的起点是上一个线段的重点 最后一个元素永远为道路最后一个点的下表
{
	int p=0,pnum=line->points.size();
	double x=line->points[p].x,
		y=line->points[p].y,
		agl=(line->points[p].yaw+90)/180.0*M_PI,
		x1=x+cos(agl)*1000,
		y1=y+sin(agl)*1000;
	vector<int> nvg;
	nvg.push_back(p);
	for(int i=0;i<pnum;i++)
	{
		double dis=distance(line->points[i].x,line->points[i].y,x,y,x1,y1);
		if(2*dis>100+10*factor)
		{
			nvg.push_back(i);
			p=i;
			x=line->points[p].x,
				y=line->points[p].y,
				agl=(line->points[p].yaw+90)/180.0*M_PI,
				x1=x+cos(agl)*1000,
				y1=y+sin(agl)*1000;
		}
	}
	if(p!=pnum-1)
		nvg.push_back(pnum-1);
	GuideLine* navigator=new GuideLine;
	int vnum=nvg.size();
	for(int i=0;i<vnum;i++)
	{
		GLPoint point;
		int idx=nvg[i];
		point.x=line->points[idx].x;
		point.y=line->points[idx].y;
		navigator->points.push_back(point);
		if(i<vnum-1)
		{
			navigator->lineID.push_back(lineID);
			navigator->index.push_back(idx);
			navigator->index.push_back(nvg[i+1]);
		}
	}


	//处理结果分析
	double min_dis=0;
	qDebug("\n\nline %d compress by %f",lineID,1.0*navigator->points.size()/pLineArray[lineID]->points.size());
	for(int i=0;i<vnum-1;i++)
	{
		for(int j=navigator->index[2*i];j<=navigator->index[2*i+1];j++)
		{
			double dis=distance(pLineArray[lineID]->points[j].x,pLineArray[lineID]->points[j].y,navigator->points[i].x,navigator->points[i].y,navigator->points[i+1].x,navigator->points[i+1].y);
			navigator->points[i].yaw=dis;
			if(dis>min_dis) 
				min_dis=dis;
		}
	}
	qDebug("\nwith max_dis %f",min_dis);

	return navigator;
}
void LineTool::angleSmooth(double *da,int length)
{
	int differ=0;
	double pi=atan2(0,-1.0) ;
	for(int i=1;i<length;i++)
	{
		da[i]+=differ;
		if(da[i]-da[i-1]>11.0/6*pi)
		{
			differ-=2*pi;
			da[i]-=2*pi;
		}
		else if(da[i]-da[i-1]<-11.0/6*pi)
		{
			differ+=2*pi;
			da[i]+=2*pi;
		}
	}
}
bool LineTool::taskAtrributeComplete()
{
	int pcount=task.points.size();
	for(int i=1;i<pcount-2;i++)		//最后一个70 倒数第二个至少为2x  不可能为入口点
		if(task.points[i].roadtype/10==1&&!(task.points[i].roadtype%10>=1&&task.points[i].roadtype%10<=4))
		{
			task.points[i].roadtype-=task.points[i].roadtype%10;
			MBUG("point %d attr missing with dir %d",i,task.points[i].roadtype%10);
			double yaw1=atan2(task.points[i].y-task.points[i-1].y,task.points[i].x-task.points[i-1].x),
				yaw2=atan2(task.points[i+2].y-task.points[i+1].y,task.points[i+2].x-task.points[i+1].x);
			double dyaw=yaw2-yaw1;
			while(dyaw>M_PI) dyaw-=2*M_PI;
			while(dyaw<-M_PI)	dyaw+=2*M_PI;
			if(abs(dyaw)<=M_PI/6)	//直行  拐角在正负30之间
				task.points[i].roadtype+=1;
			else if(dyaw>M_PI/6.0&&dyaw<M_PI*5.0/6)		//左拐： 方向左打[30,150]之间
				task.points[i].roadtype+=3;
			else if(dyaw<-M_PI/6.0&&dyaw>-M_PI*5.0/6)
				task.points[i].roadtype+=2;
			else
				task.points[i].roadtype+=4;
			MBUG("and complete with %d",task.points[i].roadtype%10);                
		}
		return true;
}
//[预处理函数，非实时执行




GuideLine* LineTool::loadGuideLine(char *filname)
{
	bool isXY=false;
	ifstream ifile(filname);
	if(ifile.fail()) 
		return NULL;
	char buff[300],digit[20];
	double xmax,xmin,ymax,ymin;
	xmax=ymax=MIN_INTEGER;
	xmin=ymin=MAX_INTEGER;
	GuideLine* line=new GuideLine();
	GLPoint point;
	
	while(!ifile.eof())
	{
		ifile.getline(buff,290);
		int i=0;
		if(buff[0]==0) continue;		//最后一行会读两遍，读取到一个空串，原因以后再谈
		
		for(int k=0;k<7;k++)	//提取=号与,号之间的数字
		{
			while(buff[i++]!='=') ;	//此时该点
			int j=0;
			while(buff[i]!=',')
				digit[j++]=buff[i++];
			digit[j]=0;				//数字get
			double num=atof(digit);
			if((k==4&&isXY)||(k==0&&!isXY))
				point.y=num;
			if((k==3&&isXY)||(k==1&&!isXY))
				point.x=num;
			if(k==5)     //YAW，记录
				point.yaw=num;
			if(k==6)	//roadtype
				point.roadtype=num;
		}
		if(!isXY)
		{
			double x,y;
			MercatorProjCal(point.x,point.y,&x,&y);
			point.x=x;
			point.y=y;
			
		}
		if(point.y>ymax) ymax=point.y;
		if(point.y<ymin) ymin=point.y;
		if(point.x>xmax) xmax=point.x;
		if(point.x<xmin) xmin=point.x;
		line->points.push_back(point);
		
	}
	line->bound[0]=xmin;
	line->bound[1]=xmax;
	line->bound[2]=ymin;
	line->bound[3]=ymax;
	

	if(bound[0]>xmin) bound[0]=xmin;
	if(bound[1]<xmax) bound[1]=xmax;
	if(bound[2]>ymin) bound[2]=ymin;
	if(bound[3]<ymax) bound[3]=ymax;

	pLineArray.push_back(line);
	qDebug("bound width=%f,height=%f",bound[1]-bound[0],bound[3]-bound[2]);
	return line;

}
