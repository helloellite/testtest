#include <vector>
#include <cmath>
#include<fstream>
#include<iostream>
#include <memory.h>
/*
@info:����������ļ���ǰ���ڵ������ƥ��ʧ��ʱʱ�����浱ǰ���������ж���
	  
*/


using namespace std;
#define		M_PI					3.14159265358979323846
#define		TURN_THRESHOLD			3000		//���ƹ������ǰ�����״���
#define		SINGULAR_THRESHOLD		400		
#define		SINGULAR_ANGEL_EQUAL	(M_PI/9.0)		//20�ȣ���Ϊͬ���� �������Ѿ��������ķ���δ���ǵ�·�ķ���һ��
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


#define		NEAR_THREASHOLD		1500		//��λ cm ,�жϵ����Ƿ��ڵ���Χʱ�ľ�����ֵ
#define		EQUATOR_RADIUS		2003750834//538855318.87441504946122116593191   // 63781370/0.11836455494812478	//�������ܳ�

struct GLPoint		//guide point
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

enum RNDF_STATUS
{
	RNDF_RAW    = 0,    /* ·����ȡ */
	RNDF_BUILD  = 1,    /* ·����Ч */
	RNDF_INI    = 2,    /* ·����ʼ�� */
	RNDF_RUN    = 3,    /* ·������ */
	RNDF_NO_TASK= 4,
	RNDF_END    = 5,    /* ·����� */
	RNDF_ERROR  = 6     
};

class MAP	//������ĵ�ͼ
{
public:
	vector<GuideLine*> pLineArray;	//�����ߣ�·�ߣ������������е�������
	vector<GuideLine*> pNavigatorArray;	//�����ߣ���pLinesArray��Ӧ�� ÿ���������ж�Ӧ��һ��Navigator��˫���ϵ����
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
	RNDF_STATUS status;


	static void MercatorProjCal( double longitude, double latitude, double *X, double *Y );
	static void MercatorProjInvCal( double X, double Y, double *longitude, double *latitude );
	
	//Ԥ������
	void taskLineComplete();		//��ȫtask�е�line�ṹ��
	int taskLine2GuideLine(int p1,int p2);			//����Ҫ�ĺ���֮һ������ȷ�������(p1,p2)������Ӧ�������ߡ�
	vector<int> searchNearLine(double x,double y,double yaw,double threashold);	//����(x,y)��ֱ���벻����threashold �� ������yaw "ͬ��" �����е�·ID

	//ʹ���˵���λ���ж���


	//��λ����
	bool locate(double x,double y,double yaw);			//��λ������x��y��yaw���õ�ǰinfo��
	bool globleLocate(double x,double y,double yaw);	//ȫ�ֶ�λ,���õĽ������locationInfo��
	bool localLocate(double x,double y,double yaw);		//����locationInfo��Ϣ��λ��������locationInfo
	int lineLocate(double x,double y,double yaw,const GuideLine* line,double thre=NEAR_THREASHOLD,int t1=-1,int t2=-1);	//��λ(x,y,yaw)��������line[t1,t2]���ֵ�λ�ã������ڸ����������򷵻�1��0<=t1<t2<=num-1��t1��t2�������ڷ�Χ�ڣ���ǿ�Ƹ�Ϊ0(num-1)��
	bool taskLocate(double x,double y,double yaw);
	bool isSingularTaskPoint(int i);
	
	//�ļ���д
	bool loadTask(char* filename);
	bool loadTaskXY(char* filename);
	void saveTaskXY(char *filename);
	bool loadMap(char *filename);
	void saveMap(char* filename);

	void intit(unsigned int id,double gps[3]);
	void context(double gps[3],GP_INFO* output);



	//protected:
	
	//��������
	void disableLocation();
	bool isLocationAva();
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
	RNDF_STATUS stat(){
		return status;
	};


	
	//�����Լ��
	bool taskAttributeVerify();			//��������������Ƿ���ʡ� ��Ҫ�����ڵ�����ڵ��������
	bool connectiveVerify();			//��֤�ڵ�ͼ�ϣ������֮���Ƿ�ɴ�

};

