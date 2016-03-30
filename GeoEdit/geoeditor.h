#ifndef GEOEDITOR_H
#define GEOEDITOR_H
#include "editview.h"
#include <QTableView>
#include <QPushButton>
#include <QLineEdit>
#include <QFileDialog>
#include <QLabel>
#include <QCheckBox>
#include <QSlider>
#include <QMainWindow>
#include <QRadioButton>
#include <QLayout>
#include "ui_geoeditor.h"
#include <QComboBox>
#include <QAction>
#define  LANDMARK_NUM 3
#define LAND_MARK_NAME {"none","zebra","stop line"}
#define QStringLiteral  codec->toUnicode
class GeoEditor : public QMainWindow
{
	Q_OBJECT
	
public slots:
	void taskHit(int type,int dir,bool flag)
	{
		for(int i=0;i<2;i++)
			attr[i]->setEnabled(flag);
		if(flag)
		{
			attr[0]->setText(QString::number(type));
			attr[1]->setText(QString::number(dir));
		}
	}
	void attributeChanged();
	void setLandmark(bool flag)
	{
		if(flag)
		{
			QString sd=((QRadioButton*)sender())->text();
			QString mark_name[]=LAND_MARK_NAME;
			int i=0;
			while(i<LANDMARK_NUM&&sd!=mark_name[i]) i++;
			if(i!=LANDMARK_NUM)
			{
				landmark_roadtype%=100;
				landmark_roadtype+=100*i;
				mapView->setRoadTypeInImu(landmark_roadtype);
			}
			qDebug("landmark=%d,rodetype=%d",landmark_roadtype/100,landmark_roadtype%100);
		}
	}
	void setLandmark(int i)
	{
		landmark_roadtype/=100;
		landmark_roadtype*=100;
		landmark_roadtype+=i+3;
		mapView->setRoadTypeInImu(landmark_roadtype);
		qDebug("landmark=%d,rodetype=%d",landmark_roadtype/100,landmark_roadtype%100);
	}
	void buttonClicked();
	void turnPointRec(int threshold);
	void updateStatusBar(QString str)
	{
		posInfo->setText(str);
	}
	void modeChanged(QString mode)
	{
		modeInfo->setText(mode);
	}
	void loadMap();
	void loadLog();
	void loadLogs();
	void loadTask();
	void loadTaskXY();
private slots:
	void isTracking(int state)
	{
		if(state==Qt::Checked)
			mapView->startTracking();
		else if(state==Qt::Unchecked)
			mapView->stopTracking();
	}
	
public:
	GeoEditor(QWidget *parent = 0);
	~GeoEditor();
	 typedef void (GeoEditor::*Fun)(void);
	
	QTextCodec *codec;
	void turnRec();
	void reduceSample();
	void analyze();
	void save();

	void startSimulate(){
		/*
		QString fileName = QFileDialog::getOpenFileName(this, tr("open file")," ",  tr("logfile(*.db);;logfile(*.log);;Allfile(*.*)"));  
		if(fileName=="") return;
		mapView->startSim(fileName.toLocal8Bit().data());*/
		mapView->moni();
		mapView->setFocus();
	};
	void stopSimulate(){
		mapView->stopSim();
		mapView->setFocus();
	};
	void UdpPortOperation(){
		
		if(buttons[UdpPortButton]->text()==QStringLiteral("open UDP port"))
		{
			if(QMessageBox::Yes==QMessageBox::question(this,"point delete",QString("clean map and task exsiting already in memory ?"),QMessageBox::Yes|QMessageBox::No,QMessageBox::No))
			{
				for(int i=0;i<mapView->plines->size();i++)
					delete (*mapView->plines)[i];
				mapView->plines->clear();
				for(int i=0;i<mapView->navigator->size();i++)
					delete (*mapView->navigator)[i];
				mapView->navigator->clear();
				mapView->task->points.clear();
				mapView->task->lineID.clear();
				mapView->task->index.clear();
				mapView->lineMap->bound[1]=mapView->lineMap->bound[3]=MIN_INTEGER;
				mapView->lineMap->bound[2]=mapView->lineMap->bound[0]=MAX_INTEGER;
			}
			for(int i=0;i<buttonNum;i++)
				buttons[i]->setDisabled(true);
			buttons[UdpPortButton]->setEnabled(true);

			buttonName[UdpPortButton]=codec->toUnicode("close UDP port");
			buttons[UdpPortButton]->setText(codec->toUnicode("close UDP port"));
			roadTypeComboBox->setCurrentIndex(2);

			mapView->openImu();
		}
		else
		{
			for(int i=0;i<buttonNum;i++)
				buttons[i]->setEnabled(true);

			buttonName[UdpPortButton]=QStringLiteral("open UDP port");
			buttons[UdpPortButton]->setText(QStringLiteral("open UDP port"));
			mapView->closeImu();

			manipulateWidgets[lastButtonID]->setVisible(false);
			lastButtonID=0;
			manipulateWidgets[lastButtonID]->setVisible(true);

			trackCheckbBox->setChecked(false);
		}
		mapView->setFocus();
	};
	void createStatusBar(){
		posInfo=new QLabel;
		modeInfo=new QLabel;
		modeInfo->setIndent(10);
		modeInfo->setAlignment(Qt::AlignRight);
		statusBar()->addWidget(posInfo);
		statusBar()->addWidget(modeInfo,10);
		connect(mapView,SIGNAL(updateStatusBar(QString)),this,SLOT(updateStatusBar(QString)));
		connect(mapView,SIGNAL(maniModeChanged(QString)),this,SLOT(modeChanged(QString)));

		
		
		QMenu* file_menu=menuBar()->addMenu("&File");	//一级菜单
			QMenu *open_menu=file_menu->addMenu("&Open File");			//二级菜单
			//以下为二级菜单项open_menu的菜单项
				QAction* action=new QAction(tr("open &map"),this);		
				action->setStatusTip("open map file *.txt");
				connect(action,SIGNAL(triggered()),this,SLOT(loadMap()));
				open_menu->addAction(action);

				QMenu *open_log=open_menu->addMenu("open log");
				action=new QAction("by &file",this);
				action->setStatusTip("open rawdata file *.log");
				connect(action,SIGNAL(triggered()),this,SLOT(loadLog()));
				open_log->addAction(action);
				action=new QAction("by &direcotry",this);
				action->setStatusTip("open all logs in directory");
				connect(action,SIGNAL(triggered()),this,SLOT(loadLogs()));
				open_log->addAction(action);

				action=new QAction("open &task",this);
				action->setStatusTip("open task file");
				connect(action,SIGNAL(triggered()),this,SLOT(loadTask()));
				open_menu->addAction(action);
				action=new QAction("open &task.xy",this);
				action->setStatusTip("open task.xy file");
				connect(action,SIGNAL(triggered()),this,SLOT(loadTaskXY()));
				open_menu->addAction(action);

			QMenu *save_menu=file_menu->addMenu("&Save File");			//二级菜单
			//以下为二级菜单 save_menu的项v
				action=new QAction("save &task",this);
				action->setStatusTip("save task file at current dir as task.txt");
				connect(action,SIGNAL(triggered()),mapView,SLOT(saveTask()));
				save_menu->addAction(action);
				

		QMenu* display_menu=menuBar()->addMenu("Display");
		QString buff[]={"line point","line","navigator point","navigator line","task point","task line","task"};
		for(int i=0;i<7;i++)
		{
			action=new QAction(buff[i],this);
			action->setCheckable(true);
			if(i==1||i==5||i==6)
				action->setChecked(true);
			else
				action->setChecked(false);
			connect(action,SIGNAL(toggled(bool)),mapView,SLOT(displayOrNot(bool)));
			display_menu->addAction(action);
		}
		
	}
	void taskAttrComplete()
	{
		mapView->lineMap->taskAtrributeComplete();
		taskHit(0,0,false);
	}

private:
	enum{loadFileButton/*,turnPointRecButton*/,reduceSplButton,turnRecButton,analyzeButton,tskAttrCmpButton,saveButton,loadMapButton,startSimulateButton,UdpPortButton,buttonNum};//加载文件 转弯点识别 弯道识别 降采样  分析，生成道线描述 文件保存
	vector<QString> buttonName;
	QHBoxLayout *mainLayout;
	QVBoxLayout *interactLayout;
	QHBoxLayout *manipulateLayout;			//操作布局，包含按钮与参数设置控件
	QVBoxLayout *buttonLayout;				//功能按钮布局
	QVBoxLayout *parameterLayout;			//参数设置布局，包含参数设置控件
    QVBoxLayout *attributeLayout;
    QCheckBox *trackCheckbBox;
	QComboBox  *roadTypeComboBox;
	QSlider *angleThresholdSlider;
	QWidget* manipulateWidgets[buttonNum];
	Ui::GeoEditorClass ui;
	QWidget *centerWidget;
	EditView *mapView;
	QTableView *tableView;
	QLineEdit* attr[2];

	vector<QPushButton*> buttons;
	int lastButtonID;
	int landmark_roadtype;
	void initialManipulateWidgets();
	vector<QRadioButton*> landmarks;
	//状态栏标签
	QLabel* posInfo;
	QLabel* modeInfo;
};

#endif // GEOEDITOR_H
