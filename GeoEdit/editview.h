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
	void maniModeChanged(QString mode);			//当前模式State改变时，发送该信号
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

	//显示模块
	void displayAll();
	void drawLine(GuideLine* line,int color=255255000,int lcolor=255255255,int width=1,int stt=-1,int ed=-1);
	void drawTaskLine(int psize);
	void drawHit();
	

	//交互模块
	void mapMove(int x,int y,bool is_finished=false);			//从lastPoint移动到(x,y)
	void pointMove(int x,int y);		//从lastPoint移动到(x,y)
	void calculateGrandResolution();
	void screenCorToXY(int x,int y,double* ans);		//计算屏幕坐标x,y的经纬度
	void selectPoints();
	void getPointID(int x,int y,int *ID);		//返回屏幕坐标点（x,y）的ID。若无，则返回{-1，-1}
	bool isEyeInMap(double center_x,double center_y);


	//预处理模块
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
	 

	
	
	int selectNum;		//用来控制删除线段

private:
	GuideLine fuzhu;		//用来纠正自己的任务线的参考线
	GP_INFO opt;			//模拟输出
	LineTool *lineMap;
	vector<GuideLine*>* plines;
	vector<GuideLine*>* navigator;
	GuideLine* task;
	
	
	
	double groundResolution;		//重要参数，地标分辨率。表示一个像素代表的实际长度
	State currentMode;
	double centerPoint[2];

	int hitID[2],lastHit[2];		//被选中点的ID
	int lastPoint[2];	//上一个屏幕坐标点
	
	bool dis_flag[7];


	//@function-调试模块
	double currentPos[3];		//当前车体坐标
	double rect[8];		//车前方的框
	//double sendPoints[100];
	void setRec(double orgx,double orgy,double yaw,double*rec)
	{

		yaw=(yaw+90)*M_PI/180;
		double vx=cos(yaw),		//竖直方向vertical
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

	//@function-1 卫星图相关
	SatelliteMap *satellitMap;
	double satellitMapBound[4];		
	double stltSelectedRect[4];			
	double stltK;
	double stltOffSet_cm[2];	//地图偏移，经纬度
	GLuint texture;	
	void mapUpdateRequest();
	void mapUpdateSelected(int);
	void drawMap();		//将卫星图绘制在xx上
public slots:
	void bindTexture();
	
	//@function-采集
public:
	ImuSvc *imusvc;
	bool isTracking;  //是否开始采集数据
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
	//@fucntion-模拟
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
			//	cout<<"数据库无法打开\n";
			return 0;
		}
		//获得记录个数， 构建并清空缓存

		if(SQLITE_OK != sqlite3_prepare_v2(db, sql_ct, (int)strlen(sql_ct), &stmt_ct, NULL))
		{
			//	cout<<"获取记录数失败\n";
			sqlite3_close_v2(db);
			return 0;
		}
		if(SQLITE_ROW == sqlite3_step(stmt_ct))
		{
			int count = sqlite3_column_int(stmt_ct,0);
			DBData.reserve(count);
			DBData.resize(count);
			//	cout<<"记录数："<<record_count;
		}
		sqlite3_finalize(stmt_ct);



		//遍历data_pl_inspector的记录，预提取数据写入缓存


		if(SQLITE_OK != sqlite3_prepare_v2(db, sql_fu, (int)strlen(sql_fu), &stmt_fu, NULL))
		{
			//cout<<"fu查询失败\n";
			sqlite3_close_v2(db);
			return 0;
		}
		int idx(0);//设置缓存写入位置指示器
		while(SQLITE_DONE != sqlite3_step(stmt_fu))
		{
			//公共缓存初始化

			//提取查询到的2进制数据块
			DBData[idx] =*((GP_INFO*)sqlite3_column_blob(stmt_fu, 1));
			idx++;
		}
		sqlite3_finalize(stmt_fu);
		//尝试为对应规划帧填入fu数据
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
1.地图

*/


/*
CONTROL	移动点
SHIFT	测距，从鼠标左键按下点到当前鼠标位置
SPACE	获取卫星图，范围为当前屏幕，按当前分辨率自动设置地图Z	    
ALT		选取范围，松开后获取该范围固定等级的卫星图
B		移动卫星图

M		模拟接受串口GPS数据、

D		删除一个轨迹线中的指定某一段
P		分割当前轨迹线 以当前带为切点 为两条轨迹线，
C		将两个轨迹线合并为一个轨迹线
I		在当前选中点前插入一个点（当前鼠标位置）
T		增加任务点中的拐弯点前增加引导点（特殊处理以面对xxx情况）
Q		根据ID定位点

Z		放大坐标系
X		缩小坐标系
*/
