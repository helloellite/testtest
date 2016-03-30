#ifndef EDITVIEW_H
#define EDITVIEW_H

#include <QGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QTimer>
#include <gl/GLU.h>
#include <QInputDialog>
#include <QMessageBox>
#include <QAction>


#include "RawGuideLines.h"
#include "satellitemap.h"
#include "imusvc.h"
#define UGV_OPEN
#define  SELECT_ROAD_LINE_OPEN
class EditView : public QGLWidget
{

	Q_OBJECT
	friend class GeoEditor;
#define MODE_NAME {"IDLE_MODE","GUIDELINE_APPEND_MODE","POINT_MOVE_MODE","POINT_DELETE_MODE","POINT_UPDATE_MODE","MAP_MOVE_MODE","COOR_SHOW_MODE","MEASURE_MODE","SATELLITE_SELECT_MODE","SATELITE_MOVE_MODE","MONI_MODE"}
	enum State{
		IDLE_MODE,GUIDELINE_APPEND_MODE,POINT_MOVE_MODE,POINT_DELETE_MODE,POINT_UPDATE_MODE,MAP_MOVE_MODE,COOR_SHOW_MODE,MEASURE_MODE,SATELLITE_SELECT_MODE,SATELITE_MOVE_MODE,MONI_MODE
	};
	vector<QString> modes;
signals:
	void updateStatusBar(QString);
	void maniModeChanged(QString mode);			//��ǰģʽState�ı�ʱ�����͸��ź�
	void taskHit(int,int,bool);
public slots:
	void locate();

	void saveTask()
	{
		lineMap->saveTaskXY("task.xy");
	}

	void displayOrNot(bool flag)
	{
		QString buff[]={"line point","line","navigator point","navigator line","task point","task line","task"};
		QAction* btn = (QAction*)sender();
		int i=0;
		while(i<7&&btn->text()!=buff[i]) i++;
		dis_flag[i]=flag;
	};
public:
	QString getPosString(){
		QString str="x="+QString::number(currentPos[0],'f',1)+"  y="+QString::number(currentPos[1],'f',1)+"  yaw="+QString::number(currentPos[2]);
		return str;
	};

	int taskInputID;
	EditView(QWidget *parent = 0);
	~EditView();
	void paintGL();
	void initializeGL();
	void resizeGL(int,int);
	void mousePressEvent(QMouseEvent*);
	void mouseMoveEvent(QMouseEvent *);
	void mouseDoubleClickEvent(QMouseEvent* event);
	void mouseReleaseEvent(QMouseEvent *);
	void wheelEvent(QWheelEvent *);
	void keyPressEvent(QKeyEvent *);
	void keyReleaseEvent(QKeyEvent *);

	//��ʾģ��
	void displayAll();
	void drawLine(GuideLine* line,int color=255255000,int lcolor=255255255,int width=1,int stt=-1,int ed=-1);
	void drawTaskLine(int psize);
	void drawHit();
	

	//����ģ��
	void mapMove(int x,int y,bool is_finished=false);			//��lastPoint�ƶ���(x,y)
	void pointMove(int x,int y);		//��lastPoint�ƶ���(x,y)
	void calculateGrandResolution();
	void screenCorToXY(int x,int y,double* ans);		//������Ļ����x,y�ľ�γ��
	void selectPoints();
	void getPointID(int x,int y,int *ID);		//������Ļ����㣨x,y����ID�����ޣ��򷵻�{-1��-1}
	bool isEyeInMap(double center_x,double center_y);


	//Ԥ����ģ��
	void nvgtGen(double threshold);
	void reduceSample();
	void save(){
		lineMap->saveMap("map.mp");
	}
	void loadLine(QString filename);
	void loadMap(QString filename);
	void loadTaskXY(QString filename);
	void loadTask(QString filename);
	void setHitPointType(int type)
	{
		if(hitID[0]==plines->size()&&hitID[1]<task->points.size())
		{
			task->points[hitID[1]].roadtype=type;
			update();
		}
	}
	 

	
	
	int selectNum;		//��������ɾ���߶�

private:
	GuideLine fuzhu;		//���������Լ��������ߵĲο���
	GP_INFO opt;			//ģ�����
	LineTool *lineMap;
	vector<GuideLine*>* plines;
	vector<GuideLine*>* navigator;
	GuideLine* task;
	
	
	
	double groundResolution;		//��Ҫ�������ر�ֱ��ʡ���ʾһ�����ش����ʵ�ʳ���
	State currentMode;
	double centerPoint[2];

	int hitID[2],lastHit[2];		//��ѡ�е��ID
	int lastPoint[2];	//��һ����Ļ�����
	
	bool dis_flag[7];


	//@function-����ģ��
	double currentPos[3];		//��ǰ��������
	double rect[8];		//��ǰ���Ŀ�
	//double sendPoints[100];
	void setRec(double orgx,double orgy,double yaw,double*rec)
	{

		yaw=(yaw+90)*M_PI/180;
		double vx=cos(yaw),		//��ֱ����vertical
			vy=sin(yaw),
			hx=cos(yaw-M_PI/2),
			hy=sin(yaw-M_PI/2);
		rec[0]=1500*hx+orgx;
		rec[1]=1500*hy+orgy;
		rec[2]=rec[0]+vx*3000;
		rec[3]=rec[1]+vy*3000;

		rec[6]=orgx-1500*hx;
		rec[7]=orgy-1500*hy;
		rec[4]=rec[6]+vx*3000;
		rec[5]=rec[7]+vy*3000;

	}

	//@function-1 ����ͼ���
	SatelliteMap *satellitMap;
	double satellitMapBound[4];		
	double stltSelectedRect[4];			
	double stltK;
	double stltOffSet_cm[2];	//��ͼƫ�ƣ���γ��
	GLuint texture;	
	void mapUpdateRequest();
	void mapUpdateSelected(int);
	void drawMap();		//������ͼ������xx��
public slots:
	void bindTexture();
	
	//@function-�ɼ�
public:
	ImuSvc *imusvc;
	bool isTracking;  //�Ƿ�ʼ�ɼ�����
	void openImu()
	{
		imusvc->enableSvc();
	}
	void closeImu()
	{
		imusvc->disableSvc();
	}
	void startTracking()
	{
		GuideLine* line=new GuideLine;
		plines->push_back(line);
		imusvc->startLog();
		isTracking=true;

	}
	void stopTracking()
	{
		imusvc->stopLog();
		isTracking=false;
	}
	void setRoadTypeInImu(int);
public slots:
	void dataArrived(POSE_INFO pi);
	//@fucntion-ģ��
public:
	vector<GP_INFO> DBData;
	int db_data_count;
	QTimer *timer;
	ifstream ifile;
	int initialFromDB(char* file_name)
	{
/*

		DBData.clear();

		char sql_ct[] = "SELECT COUNT(*) FROM data_gp_info";
		char sql_fu[] = "SELECT * FROM data_gp_info";

		sqlite3* db=NULL;
		sqlite3_stmt * stmt_ct=NULL;
		sqlite3_stmt * stmt_fu(NULL);

		char *DB_name=DB_name=file_name;
		if(SQLITE_OK != sqlite3_open_v2(DB_name, &db, SQLITE_OPEN_READONLY, NULL))
		{
			//	cout<<"���ݿ��޷���\n";
			return 0;
		}
		//��ü�¼������ ��������ջ���

		if(SQLITE_OK != sqlite3_prepare_v2(db, sql_ct, (int)strlen(sql_ct), &stmt_ct, NULL))
		{
			//	cout<<"��ȡ��¼��ʧ��\n";
			sqlite3_close_v2(db);
			return 0;
		}
		if(SQLITE_ROW == sqlite3_step(stmt_ct))
		{
			int count = sqlite3_column_int(stmt_ct,0);
			DBData.reserve(count);
			DBData.resize(count);
			//	cout<<"��¼����"<<record_count;
		}
		sqlite3_finalize(stmt_ct);



		//����data_pl_inspector�ļ�¼��Ԥ��ȡ����д�뻺��


		if(SQLITE_OK != sqlite3_prepare_v2(db, sql_fu, (int)strlen(sql_fu), &stmt_fu, NULL))
		{
			//cout<<"fu��ѯʧ��\n";
			sqlite3_close_v2(db);
			return 0;
		}
		int idx(0);//���û���д��λ��ָʾ��
		while(SQLITE_DONE != sqlite3_step(stmt_fu))
		{
			//���������ʼ��

			//��ȡ��ѯ����2�������ݿ�
			DBData[idx] =*((GP_INFO*)sqlite3_column_blob(stmt_fu, 1));
			idx++;
		}
		sqlite3_finalize(stmt_fu);
		//����Ϊ��Ӧ�滮֡����fu����
		sqlite3_close_v2(db);
*/

		return 1;

	}
	void startSim(char* filename)
	{
#ifdef MONITXT
		ifile.open(filename);
		if(!ifile)
		{
			qDebug("file open error");
			return;
		}
#else
		//lineMap->addEmptyLineAtLast();
		initialFromDB(filename);
		db_data_count=0;
#endif
		timer->start(10);

	}
	void stopSim()
	{
#ifdef MONITXT
		ifile.close();
#else
		DBData.clear();
#endif
		timer->stop();
	}
public slots:
	void moni();
};

#endif // EDITVIEW_H


/*
1.��ͼ

*/


/*
CONTROL	�ƶ���
SHIFT	��࣬�����������µ㵽��ǰ���λ��
SPACE	��ȡ����ͼ����ΧΪ��ǰ��Ļ������ǰ�ֱ����Զ����õ�ͼZ	    
ALT		ѡȡ��Χ���ɿ����ȡ�÷�Χ�̶��ȼ�������ͼ
B		�ƶ�����ͼ

M		ģ����ܴ���GPS���ݡ�

D		ɾ��һ���켣���е�ָ��ĳһ��
P		�ָǰ�켣�� �Ե�ǰ��Ϊ�е� Ϊ�����켣�ߣ�
C		�������켣�ߺϲ�Ϊһ���켣��
I		�ڵ�ǰѡ�е�ǰ����һ���㣨��ǰ���λ�ã�
T		����������еĹ����ǰ���������㣨���⴦�������xxx�����
Q		����ID��λ��

Z		�Ŵ�����ϵ
X		��С����ϵ
*/
