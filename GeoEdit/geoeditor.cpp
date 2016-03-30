#include "geoeditor.h"

GeoEditor::GeoEditor(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	centerWidget=new QWidget;
	landmark_roadtype=0;
	setCentralWidget(centerWidget);
	mainLayout=new QHBoxLayout;
	
	tableView=new QTableView;
	



	mapView=new EditView;
    attributeLayout=new QVBoxLayout;
	QSizePolicy policy(QSizePolicy::Expanding,QSizePolicy::Expanding);
	mapView->setSizePolicy(policy);
	policy.setHorizontalPolicy(QSizePolicy::Fixed);
	policy.setVerticalPolicy(QSizePolicy::Expanding);
	tableView->setSizePolicy(policy);
	interactLayout=new QVBoxLayout;
	buttonLayout=new QVBoxLayout;				//功能按钮布局
	parameterLayout=new QVBoxLayout;			//参数设置布局，包含参数设置控件
	manipulateLayout=new QHBoxLayout;			//操作布局，包含按钮与参数设置控件
	mainLayout->addWidget(mapView);
	mainLayout->addLayout(interactLayout);
	interactLayout->addLayout(manipulateLayout);
	interactLayout->addWidget(tableView);
    interactLayout->addLayout(attributeLayout);
	manipulateLayout->addLayout(buttonLayout);
	manipulateLayout->addLayout(parameterLayout);
	centerWidget->setLayout(mainLayout);

	char buffname[][30]={"load log file"/*,"拐弯点识别"*/,"desampling","navigator generate","task match","task attribute complete","save map","load map","simulate","open UDP port"};

	codec = QTextCodec::codecForName("GB18030");
	for(int i=0;i<buttonNum;i++)
	{
		buttonName.push_back(codec->toUnicode(buffname[i]));
		buttons.push_back(new QPushButton(buttonName[i]));
		buttonLayout->addWidget(buttons[i]);
		connect(buttons[i],SIGNAL(clicked()),this,SLOT(buttonClicked()));
	}
	initialManipulateWidgets();
	connect(mapView,SIGNAL(taskHit(int,int,bool)),this,SLOT(taskHit(int,int,bool)));

	createStatusBar();
}
GeoEditor::~GeoEditor()
{
	
}
void GeoEditor::initialManipulateWidgets()
{
	for(int i=0;i<buttonNum;i++)
	{
		manipulateWidgets[i]=new QWidget;
		manipulateLayout->addWidget(manipulateWidgets[i]);
		manipulateWidgets[i]->setVisible(false);
	}
	//分别部署各个widgets的内容

/************************************************************************/
int index;
QVBoxLayout *mly;
QHBoxLayout *hly;
QLabel *lable;
QSlider* slider;
/************************************************************************/
/*降采样																*/
/************************************************************************/
	index=reduceSplButton;
	mly=new QVBoxLayout;
	//line factor
	hly=new QHBoxLayout;
	lable=new QLabel(QStringLiteral("line"));
	slider=new QSlider(Qt::Horizontal);
	slider->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	hly->addWidget(lable);
	hly->addWidget(slider);
	mly->addLayout(hly);
	//turn factor
	hly=new QHBoxLayout;
	lable=new QLabel(QStringLiteral("turn"));
	slider=new QSlider(Qt::Horizontal);
	slider->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	hly->addWidget(lable);
	hly->addWidget(slider);
	mly->addLayout(hly);
	manipulateWidgets[index]->setLayout(mly);

/*转弯点识别                                                                     */
/************************************************************************/
/*
	int index=turnPointRecButton;
	QVBoxLayout* mly=new QVBoxLayout;
	QHBoxLayout* hly=new QHBoxLayout;
	QLabel* lable=new QLabel(QStringLiteral("yaw'\n阈\n值"));
	QSlider *slider=new QSlider(Qt::Horizontal);
	connect(slider,SIGNAL(valueChanged(int)),this,SLOT(turnPointRec()));
	slider->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	hly->addWidget(lable);
	hly->addWidget(slider);
	mly->addLayout(hly);
	manipulateWidgets[index]->setLayout(mly);*/


/************************************************************************/
/*弯道识别                                                                     */
/************************************************************************/
	index=turnRecButton;
	mly=new QVBoxLayout;
	//line factor
	hly=new QHBoxLayout;
	lable=new QLabel(QStringLiteral("角阈值"));
	angleThresholdSlider=new QSlider(Qt::Horizontal);
	angleThresholdSlider->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	connect(angleThresholdSlider,SIGNAL(valueChanged(int)),this,SLOT(turnPointRec(int)));
	hly->addWidget(lable);
	hly->addWidget(angleThresholdSlider);
	mly->addLayout(hly);

	
	manipulateWidgets[index]->setLayout(mly);


	/************************************************************************/

	/*打开端口															*/
	/************************************************************************/
	index=UdpPortButton;
	mly=new QVBoxLayout;
	QPushButton *bff=new QPushButton(QStringLiteral("location"));
	connect(bff,SIGNAL(clicked()),mapView,SLOT(locate()));
	mly->addWidget(bff);
	
	
/*
		butt=new QPushButton(QStringLiteral("关闭端口"));
		connect(butt,SIGNAL(clicked()),this,SLOT(closeUdpPort()));
		mly->addWidget(butt);*/

	roadTypeComboBox=new QComboBox;
	roadTypeComboBox->addItem(QStringLiteral("off road"));
	roadTypeComboBox->addItem(QStringLiteral("country road"));
	roadTypeComboBox->addItem(QStringLiteral("structed road"));
	connect(roadTypeComboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(setLandmark(int)));
	mly->addWidget(roadTypeComboBox);

	QString mark_name[]=LAND_MARK_NAME;
	

	for(int i=0;i<LANDMARK_NUM;i++)
	{
		QRadioButton* rb=new QRadioButton(this);
		rb->setText(mark_name[i]);
		connect(rb,SIGNAL(toggled(bool)),this,SLOT(setLandmark(bool)));
		mly->addWidget(rb);
		landmarks.push_back(rb);
	}

	trackCheckbBox=new QCheckBox;
	trackCheckbBox->setText(QStringLiteral("start tracking"));
	connect(trackCheckbBox,SIGNAL(stateChanged(int)),this,SLOT(isTracking(int)));
	mly->addWidget(trackCheckbBox);

	

	manipulateWidgets[index]->setLayout(mly);

	lastButtonID=0;
	manipulateWidgets[lastButtonID]->setVisible(true);

	QString att_name[]={"type=","dir="};
	QHBoxLayout* line=new QHBoxLayout;
	for(int i=0;i<2;i++)
    {
		QLabel *tip=new QLabel(att_name[i]);
		tip->setFixedSize(30,20);
		QLineEdit *edit=new QLineEdit();
		attr[i]=edit;
		connect(edit,SIGNAL(editingFinished(void)),this,SLOT(attributeChanged(void)));
		edit->setFixedSize(60,20);
		line->addWidget(tip);
		line->addWidget(edit);
		if(i%2==1)
		{
			attributeLayout->addLayout(line);
			line=new QHBoxLayout;
		}
		
	}
	delete line;
   
	
}
void GeoEditor::attributeChanged()
{
	int i=0;
	while(i<2&&(QLineEdit*)sender()!=attr[i]) i++;
	int type=10*attr[0]->text().toInt()+attr[1]->text().toInt();
	qDebug("type edited to %d",type);
	mapView->setHitPointType(type);
}

void GeoEditor::buttonClicked()
{
	Fun slot[]={&GeoEditor::loadLog/*,&GeoEditor::turnPointRec*/,&GeoEditor::reduceSample,&GeoEditor::turnRec,&GeoEditor::analyze,&GeoEditor::taskAttrComplete,&GeoEditor::save,&GeoEditor::loadMap,&GeoEditor::startSimulate,&GeoEditor::UdpPortOperation};
	
	
	if (QPushButton* btn = dynamic_cast<QPushButton*>(sender())){
		for(int i=0;i<buttonNum;i++)
			if(btn->text()==buttonName[i])
			{
				manipulateWidgets[lastButtonID]->setVisible(false);
				lastButtonID=i;
				manipulateWidgets[lastButtonID]->setVisible(true);
				(this->*slot[i])();
			}
    }
}

//文件打开、保存操作
void GeoEditor::loadMap()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file")," ",  tr("mapfile(*.mp);;Allfile(*.*)"));
	if(fileName=="") return;
	mapView->loadMap(fileName);
	mapView->setFocus();
}

void GeoEditor::loadLog()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open file")," ",  tr("logfile(*.log);;Allfile(*.*)"));  
	if(fileName=="") return;
	mapView->loadLine(fileName);
	mapView->setFocus();
	mapView->update();
}
void GeoEditor::loadLogs()
{
	QString dirName=QFileDialog::getExistingDirectory(this,tr("open dir"));
	QDir dir(dirName);
	QStringList filter;
	filter<<"*.log";
	QStringList filenames=dir.entryList(filter);
	for(int i=0;i<filenames.size();i++)
	{
		mapView->loadLine(dirName+"/"+filenames[i]);
		qDebug((dirName+filenames[i]).toLocal8Bit().data());
	}
}
void GeoEditor::loadTask()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open file")," ",  tr("Allfile(*.*)"));  
	if(fileName=="") return;
	mapView->loadTask(fileName);
	mapView->setFocus();
}
void GeoEditor::loadTaskXY()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open file")," ",  tr("taskfile(*.xy);;Allfile(*.*)"));  
	if(fileName=="") return;
	mapView->loadTaskXY(fileName);
	mapView->setFocus();
}

void GeoEditor::save()
{
	mapView->save();
	mapView->setFocus();
}

//原始数据处理操作
void GeoEditor::turnPointRec(int threshold)
{
	qDebug("%d",threshold);
	mapView->nvgtGen(threshold);
	mapView->update();
}
void GeoEditor::turnRec()
{
	angleThresholdSlider->setValue(10);
	
}
void GeoEditor::reduceSample()
{
	

	mapView->reduceSample();
	mapView->update();
	mapView->setFocus();
}
void GeoEditor::analyze()
{
	mapView->lineMap->taskLineComplete();
	mapView->setFocus();
}



