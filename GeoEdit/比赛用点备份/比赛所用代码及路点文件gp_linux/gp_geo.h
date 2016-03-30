#include <vector>
#include <cmath>
#include<fstream>
#include<iostream>



using namespace std;
#define M_PI    3.14159265358979323846
#define  TURN_THRESHOLD 3000		//���ƹ������ǰ�����״���
#define  SINGULAR_THRESHOLD 400		
#define SINGULAR_ANGEL_EQUAL (M_PI/9.0)		//20�ȣ���Ϊͬ���� �������Ѿ��������ķ���δ���ǵ�·�ķ���һ��
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


#define NEAR_THREASHOLD 1500		//��λ cm ,�жϵ����Ƿ��ڵ���Χʱ�ľ�����ֵ
#define  EQUATOR_RADIUS 538855318.87441504946122116593191   // 63781370/0.11836455494812478	//�������ܳ�
#define X_OFFSET_DY 2056580900;
#define Y_OFFSET_DY 361405900;
struct GLPoint		//guideline point
{
	double x,y,yaw;
	unsigned int roadtype;		//��·����
};
struct GuideLine			//
{
	vector<GLPoint> points; //���GuideLine�ĵ�
	vector<int> lineID;	//
	vector<int> index;			//points��λ��i����Ӧ�ľ�����lineΪline�е�(2i��2i+1����ʾ 	
	double bound[4];
};
struct junction{
	vector<int> adjacent;	//����·�ڣ���ʱ������
	vector<int> lines;		//�����ߣ���adjacent���±�ƥ��
	vector<int> roads_code;		//ͨ����һ��·�ڵĵ�·�롣 ���뷽ʽ�� roads_code=roadID*10+line_ID_in_road

	int center[2];			//��¼junctionԲ��λ�á� junction����center��x,y��ΪԲ�ģ�radius����ֵ��Ϊ�뾶��Բ��
};
struct road{
	vector<int> lines;		//��·�е������ߣ�ͨ��ֻ�����������ط����һ��
	vector<int> start;		//��������㣻�±���linesƥ��
	vector<int> terminal;	//�������յ㣻�±���linesƥ��
};
enum RNDF_STATUS
{
	RNDF_RAW    = 0,    /* ·����ȡ */
	RNDF_BUILD  = 1,    /* ·����Ч */
	RNDF_INI    = 2,    /* ·����ʼ�� */
	RNDF_RUN    = 3,    /* ·������ */
	RNDF_END    = 4,    /* ·����� */
	RNDF_ERROR  = 6     /* ·������ */
};
//tips��junction��road����indexѰַ��
//�ŵ㣺���������־����������Ķ�λ�� ����ָ��Ѱַ�����޷�֪���������ڵڼ�����
//ȱ�㣺��pLines�ڷ����䶯ʱ������pLines��junctions��roads����Ҫ�����䶯
class MAP	//������ĵ�ͼ
/************************************************************************/
/*����ͼԼ��������·�ھ���Ψһ�ļ򵥵�·��                              */
/************************************************************  m m m   m m m m       m, ,m************/
{
	


protected:
	vector<GuideLine*> pLineArray;	//�����ߣ�·�ߣ������������е�������
	vector<GuideLine*> pNavigatorArray;	//�����ߣ���pLinesArray��Ӧ�� ÿ���������ж�Ӧ��һ��Navigator��˫���ϵ����
	vector<junction> junctionArray;	//����-·�ڼ�����ͼ�����ṹ֮һ���������е�·�ڡ�	  ����·��֮�����Ψһ�ļ򵥵�·
	vector<road> roadListArray;		//����-�򵥵�·������ͼ˫�ṹ֮һ���������м򵥵�·��  �򵥵�·����·��û��·��
	GuideLine task;
	double bound[4];
	//task.points   �����ļ�����Map���롣 �ɴ�Map�̳г�TaskMap����TaskMap�������������˴���Ϊ�˷��㣬���������ģ��ϸ�֡��������йصĶ���taskǰ׺
							//�����4Ҫ�أ���γ�ȣ��߶ȣ�����  �ֱ���GPPoint��x,y,yaw,type�ĸ����������
	//task.line task.index ��ʾtask.point[i]��task.point[i+1]�м��Ӧ���������롣 
	

	/************************************************************************/
	/* ȫ�ֶ�λ����������task��navigator��line                              */
	/************************************************************************/
	int locationInfo[4];	//��0-2�ֱ�Ϊ����task.points�е��±꣬��navigator[j]�е��±꣬��Line[k]�е��±�  ��δֱ�Ӽ�¼�����߱����ID������ͨ��һ������Ѱ�ҵ�
							//3�д�������ߵ���ID��Ҳ����˵(3,2)��ʾ�����е�λ��
	int nextTaskID;
public:
	MAP();
	~MAP();
	double lastPos[2];
	bool isSingularTaskPoint(int i);
	bool taskAtrributeComplete();
	void taskLineComplete();		//��ȫtask�е�line�ṹ��
	bool locate(double x,double y,double yaw);			//��λ������x��y��yaw���õ�ǰinfo��
	bool taskLocate(double x,double y,double yaw);
	RNDF_STATUS stat(){
		return status;
	};
    void intit(unsigned int id,double gps[3]);
    void context(double gps[3],GP_INFO* output);
	//�ļ���д
	bool loadTask(char* filename);
	bool loadTaskXY(char* filename);
	void saveTaskXY(char *filename);
	bool loadMap(char *filename);
	void saveMap(char* filename);



protected:
	int taskLine2GuideLine(int p1,int p2);			//����Ҫ�ĺ���֮һ������ȷ�������(p1,p2)������Ӧ�������ߡ�
	RNDF_STATUS status;
    bool globleLocate(double x,double y,double yaw);	//ȫ�ֶ�λ,���õĽ������locationInfo��
	bool localLocate(double x,double y,double yaw);		//����locationInfo��Ϣ��λ��������locationInfo
	
    void updateNextTaskID();
    int isInSegment(double x,double y,double sx,double sy,double ex,double ey);		//�жϵ�(x,y)�Ƿ�ֱͶӰ���߶�(sx,sy)-(ex,ey)�ϣ��������㣩��
	bool isDirectEqual(double yaw,double sx,double sy,double ex,double ey);		//line_dirΪ��·�ķ������ĽǶ�		//�жϵ�ǰ�����yaw�붨λ������dir�Ƿ�ͬ��-�н���90~90��Ϊͬ��
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
    };	//���ݹ�����ڵ��λ�ã������Ӧ�ĳ��ڵ�
	
	//���´�����ܼܺ򵥣���Ϊ���������������Ӧ�޸�

	void disableLocation();
	bool isLocationAva();
//	int coor2Junction(double x,double y){return -1;}; //����������λ�ã����������ڵ�Junction�±�
//	int coor2Road(double x,double y){return -1;};	  //����������λ�ã����������ڵĵ�·���±�
//	bool isCoorInJunciton(double x,double y,int junction){return true;};		//�ж�(x,y)�Ƿ���junction��

	int lineLocate(double x,double y,double yaw,const GuideLine* line,double thre=NEAR_THREASHOLD,int t1=-1,int t2=-1);	//��λ(x,y,yaw)��������line[t1,t2]���ֵ�λ�ã������ڸ����������򷵻�1��0<=t1<t2<=num-1��t1��t2�������ڷ�Χ�ڣ���ǿ�Ƹ�Ϊ0(num-1)��
																							//��ָ��t1��t2ʱ����Ĭ��ȫ������
																							//ע�⴦��ä�����֣��ο�GuideLine��locate��ĳ��ζ�λ����
																							//��������Ϣ��λ������������������������ǰһ֡��λ�ã��� Լ��ǿ�ȣ�___������TODO����
																							//ʹ���˵���λ���ж���

	vector<int> searchNearLine(double x,double y,double yaw,double threashold);	//����(x,y)��ֱ���벻����threashold �� ������yaw "ͬ��" �����е�·ID
																				//ʹ���˵���λ���ж���
																				
	//�����Լ��
	bool taskAttributeVerify();			//��������������Ƿ���ʡ� ��Ҫ�����ڵ�����ڵ��������
	bool connectiveVerify();			//��֤�ڵ�ͼ�ϣ������֮���Ƿ�ɴ�

public:
static	double latToY(double lat)
	{
		/*
		lat*=M_PI/180.0;		//ת��Ϊ����
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
		x/=EQUATOR_RADIUS;		//ת��Ϊ����	
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

