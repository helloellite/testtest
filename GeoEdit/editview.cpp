#include "editview.h"
#define CLARITY_FACTOR	1		//地图清晰度，越高则地图越清晰（0-1）,下载的地图的大小是窗口大小的（1+CLARITY）倍 太高没用
#define SCALE_FACTOR 0.9		//鼠标滚动缩放比例
#include<cstdlib>
EditView::EditView(QWidget *parent)
	: QGLWidget(parent)
{	
	memset(stltSelectedRect,0,sizeof(double)*4);
	stltK=1;
	taskInputID=-1;
		isTracking=false;
		timer=new QTimer;
		connect(timer,SIGNAL(timeout()),this,SLOT(moni()));
		QString modebuff[]=MODE_NAME;
		int modecount=sizeof(modebuff)/sizeof(QString);
		for(int i=0;i<modecount;i++)
			modes.push_back(modebuff[i]);
		currentMode=IDLE_MODE;
		maniModeChanged(modes[currentMode]);
		hitID[0]=hitID[1]=-1;
		currentPos[0]=currentPos[1]=currentPos[2]=0;
		lineMap=new LineTool;
		plines=lineMap->getLines();
		navigator=lineMap->getNavigator();
		task=lineMap->getTask();
	satellitMap=new SatelliteMap((1+CLARITY_FACTOR)*width(),(1+CLARITY_FACTOR)*height());
	imusvc=new ImuSvc;
	connect(imusvc,SIGNAL(dataArrived(POSE_INFO)),this,SLOT(dataArrived(POSE_INFO)));
	lineMap->taskLineComplete();
	rect[0]=0;
	currentPos[0]=currentPos[1]=currentPos[2]=0;
	
	int w=width();
	w=height();
	connect(satellitMap,SIGNAL(update()),SLOT(bindTexture()));
	groundResolution=1000;
	ifstream infile("STLTOFFSET");
	if(!infile)
		stltOffSet_cm[0]=stltOffSet_cm[1]=0;
	else
	{
		infile>>stltOffSet_cm[0]>>stltOffSet_cm[1];
		infile.close();
	}
	memset(&opt,0,sizeof(opt));
/*
#ifdef UGV_OPEN
	GP_FUNC_INPUT ipt;
	ipt.reco_point.recovery=2;
	GP_LOCAL_DATA evt;
	UGV_GP::Global_Plan(&ipt,&opt,&evt);
	for(int i=0;i<4;i++)
		qDebug("tps=%d",opt.tps[i].id);
	UGV_GP::Global_Plan(&ipt,&opt,&evt);
	for(int i=0;i<4;i++)
		qDebug("tps=%d",opt.tps[i].id);
#endif*/


	for(int i=0;i<7;i++)
		dis_flag[i]=(i==1||i==5||i==6);

	
}

void EditView::loadLine(QString filename)
{
	lineMap->loadGuideLine(filename.toLocal8Bit().data());	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const double *bound=lineMap->getBound();	//采用整张地图范围
	double dx=(bound[1]-bound[0])/width(),dy=(bound[3]-bound[2])/height();
	groundResolution=dx>dy? dx:dy;
	glOrtho((bound[0]+bound[1]-groundResolution*width())/2,(bound[0]+bound[1]+groundResolution*width())/2,(bound[2]+bound[3]-groundResolution*height())/2,(bound[2]+bound[3]+groundResolution*height())/2,-1,1);

	//初始化groundResolution
//	screenCorToXY(width()/2,height()/2,centerPoint);
    centerPoint[0]=(bound[0]+bound[1])/2;
    centerPoint[1]=(bound[3]+bound[2])/2;
	qDebug("groundResolution initial with %f",groundResolution);

	update();
	//mapUpdateRequest();
};

void EditView::loadMap(QString filename)
{
	LineTool* buff=lineMap;//new LineTool;
	buff->loadMap(filename.toLocal8Bit().data());
	qDebug("map contains %d lines",buff->getLinesCount());
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const double *bound=buff->getBound();	//采用整张地图范围
	double dx=(bound[1]-bound[0])/width(),dy=(bound[3]-bound[2])/height();
	groundResolution=dx>dy? dx:dy;
	glOrtho((bound[0]+bound[1]-groundResolution*width())/2,(bound[0]+bound[1]+groundResolution*width())/2,(bound[2]+bound[3]-groundResolution*height())/2,(bound[2]+bound[3]+groundResolution*height())/2,-1,1);
	
    centerPoint[0]=(bound[0]+bound[1])/2;
    centerPoint[1]=(bound[3]+bound[2])/2;
	qDebug("groundResolution initial with %f",groundResolution);


	update();
	
	//mapUpdateRequest();
}

void EditView::loadTaskXY(QString filename)
{
	if(!lineMap->loadTaskXY(filename.toLocal8Bit().data()))
		QMessageBox::about(this,"load erro","loadError");

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const double *bound=lineMap->getBound();	//采用整张地图范围
	double dx=(bound[1]-bound[0])/width(),dy=(bound[3]-bound[2])/height();
	groundResolution=dx>dy? dx:dy;
	glOrtho((bound[0]+bound[1]-groundResolution*width())/2,(bound[0]+bound[1]+groundResolution*width())/2,(bound[2]+bound[3]-groundResolution*height())/2,(bound[2]+bound[3]+groundResolution*height())/2,-1,1);

	//初始化groundResolution
	centerPoint[0]=(bound[0]+bound[1])/2;
	centerPoint[1]=(bound[3]+bound[2])/2;
	qDebug("groundResolution initial with %f",groundResolution);

	update();
}

void EditView::loadTask(QString filename)
{
	if(!lineMap->loadTask(filename.toLocal8Bit().data()))
		QMessageBox::about(this,"load erro","loadError");

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const double *bound=lineMap->getBound();	//采用整张地图范围
	double dx=(bound[1]-bound[0])/width(),dy=(bound[3]-bound[2])/height();
	groundResolution=dx>dy? dx:dy;
	glOrtho((bound[0]+bound[1]-groundResolution*width())/2,(bound[0]+bound[1]+groundResolution*width())/2,(bound[2]+bound[3]-groundResolution*height())/2,(bound[2]+bound[3]+groundResolution*height())/2,-1,1);

	//初始化groundResolution
    centerPoint[0]=(bound[0]+bound[1])/2;
    centerPoint[1]=(bound[3]+bound[2])/2;
	qDebug("groundResolution initial with %f",groundResolution);

	update();
}

EditView::~EditView()
{
	if(lineMap!=NULL) delete lineMap;
	if(satellitMap!=NULL) delete satellitMap;
	ofstream offset_log("STLTOFFSET");
	if(!offset_log)
		qDebug("offset log open error\n");
	else
	{
		offset_log<<stltOffSet_cm[0]<<' '<<stltOffSet_cm[1];
		offset_log.close();
	}
}

void EditView::resizeGL(int w,int h)
{
	
	satellitMap->SetRect((1+CLARITY_FACTOR)*w,(1+CLARITY_FACTOR)*h);
	glViewport(0,0,w,h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	qDebug("%d,%d",w,width());
	glOrtho((2*centerPoint[0]-groundResolution*width())/2,(2*centerPoint[0]+groundResolution*width())/2,(2*centerPoint[1]-groundResolution*height())/2,(2*centerPoint[1]+groundResolution*height())/2,-1,1);
	
	qDebug("groundResolution=%f,center=(%f,%f)",groundResolution,centerPoint[0],centerPoint[1]);
	//mapUpdateRequest();	
}
/************************************************************************/
/* GLWidget  响应函数                                                                     */
/************************************************************************/





void EditView::moni()
{
	
		double x,y,yaw;
		x=QInputDialog::getDouble(this,"input x","input x(cm)");
		y=QInputDialog::getDouble(this,"input y","input x(cm)");
		yaw=QInputDialog::getDouble(this,"input yaw","input yaw(degree)");
		POSE_INFO pi;
		pi.yaw=yaw/(180.0*1e-8)*3.1415926535898;
		pi.ins_coord.x=x;
		pi.ins_coord.y=y;
		dataArrived(pi);
	
	/*
double x,y,yaw;
#ifdef MONITXT
	bool isXY=true;
	char buff[300],digit[20];;
	if(!ifile.eof())
	{
		ifile.getline(buff,290);
		int i=0;
		if(buff[0]==0) return;		//最后一行会读两遍，读取到一个空串，原因以后再谈
		for(int k=0;k<6;k++)	//提取=号与,号之间的数字
		{
			while(buff[i++]!='=') ;	//此时该点
			int j=0;
			while(buff[i]!=',')
				digit[j++]=buff[i++];
			digit[j]=0;				//数字get
			double num=atof(digit);
			if(k==(isXY? 4:0))		//LAT,转为y
			{	
				
				if(isXY)
					y=num;
				else
				{
				
					y=lineMap->latToY(num);
				}
				
			}
			if(k==(isXY? 3:1))		//LON,转为x
			{
				
				if(isXY)
					x=num;
				else
				{
					x=lineMap->lonToX(num);
				}
				
			}
			if(k==5)     //YAW，记录
				yaw=num;
		}
	}


#else
	x=DBData[db_data_count].state.pos.ins_coord.x;
	y=DBData[db_data_count].state.pos.ins_coord.y;
	yaw=DBData[db_data_count].state.pos.yaw;
	POSE_INFO pi;
	pi.ins_coord.x=x;
	pi.ins_coord.y=y;
	pi.yaw=yaw;//(int)((yaw)*3.1415926535898/(180.0*1e-8));
	
	dataArrived(pi);
		
	db_data_count++;
	if(db_data_count==DBData.size())
		stopSim();
#endif*/

}

void EditView::locate()		//读取当前的pos信息，从而获得当前坐标点
{
	centerPoint[0]=currentPos[0];
	centerPoint[1]=currentPos[1];

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho((2*centerPoint[0]-groundResolution*width())/2,(2*centerPoint[0]+groundResolution*width())/2,(2*centerPoint[1]-groundResolution*height())/2,(2*centerPoint[1]+groundResolution*height())/2,-1,1);

	this->setFocus();
}






void EditView::initializeGL()
{
	glViewport(0,0,width(),height());
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,1,0,1,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClearColor(0,0,0,0);	
	
	glGenTextures(1,&texture);
	glBindTexture(GL_TEXTURE_2D,texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);//当所显示的纹理比加载进来的纹理小时，采用GL_NEAREST的方法来处理  
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);//当所显示的纹理比加载进来的纹理大时，采用GL_NEAREST的方法来处理  
	

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	const double *bound=lineMap->getBound();	//采用整张地图范围
	double dx=(bound[1]-bound[0])/width(),dy=(bound[3]-bound[2])/height();
	groundResolution=dx>dy? dx:dy;
	glOrtho((bound[0]+bound[1]-groundResolution*width())/2,(bound[0]+bound[1]+groundResolution*width())/2,(bound[2]+bound[3]-groundResolution*height())/2,(bound[2]+bound[3]+groundResolution*height())/2,-1,1);

	//初始化groundResolution
	centerPoint[0]=(bound[0]+bound[1])/2;
    centerPoint[1]=(bound[2]+bound[3])/2;
    qDebug("groundResolution initial with %f",groundResolution);
}

void EditView::paintGL()
{
	displayAll();
}

void EditView::mousePressEvent(QMouseEvent* event)
{
	setFocus();
	lastPoint[0]=event->x();
	lastPoint[1]=height()-event->y();
	if(currentMode== COOR_SHOW_MODE&&event->button()==Qt::LeftButton)
	{	//此处累加处理,，退出COOR_SHOW_MODE，进入LINE_MOVE_MODE
		lastPoint[0]=event->x();
		lastPoint[1]=height()-event->y();
		setMouseTracking(false);
		maniModeChanged(modes[currentMode]);
	}
	if(currentMode==SATELLITE_SELECT_MODE)
	{
		int mx=event->x(),my=height()-event->y();
		screenCorToXY(mx,my,&stltSelectedRect[0]);
	}
	if(currentMode==POINT_MOVE_MODE)
	{
		if(hitID[0]!=-1) lastHit[0]=hitID[0],lastHit[1]=hitID[1];
		getPointID(event->x(),height()-event->y(),hitID);
		int t,d;
		if(hitID[0]==plines->size())
		{
			t=task->points[hitID[1]].roadtype/10;
			d=task->points[hitID[1]].roadtype%10;
		}
		emit taskHit(t,d,hitID[0]==plines->size());
		update();
	}
	
	if(currentMode!=IDLE_MODE) return;	//仅在IDLE模式下可进入其他模式

	if(  event->button() ==Qt::LeftButton )
	{
		lastPoint[0]=event->x();
		lastPoint[1]=height()-event->y();
		currentMode=MAP_MOVE_MODE;
		maniModeChanged(modes[currentMode]);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		update();
	}
	else if(event->button()==Qt::RightButton)
	{
		lastPoint[0]=event->x();
		lastPoint[1]=height()-event->y();
	}
}

void EditView::mouseMoveEvent(QMouseEvent *event)
{
	switch(currentMode)
	{
	case SATELITE_MOVE_MODE:
		{	
			double dx=(event->x()-lastPoint[0])*groundResolution,
				dy=(height()-event->y()-lastPoint[1])*groundResolution;
			stltOffSet_cm[0]+=dx;
			stltOffSet_cm[1]+=dy;
			lastPoint[0]=event->x();
			lastPoint[1]=height()-event->y();
			update();
			break;

		}
	case SATELLITE_SELECT_MODE:
		{
			int mx=event->x(),my=height()-event->y();
			screenCorToXY(mx,my,&stltSelectedRect[2]);
			update();
			break;
		}
	case MAP_MOVE_MODE:
		mapMove(event->x(),height()-event->y());
		update();
		break;
	case POINT_MOVE_MODE:
		if(hitID[0]!=-1&&hitID[1]!=-1)
		{
			pointMove(event->x(),height()-event->y());
			update();
		}
		break;
	case COOR_SHOW_MODE:
		double coor[2],lat,lon;
		screenCorToXY(event->x(),height()-event->y(),coor);
		lineMap->MercatorProjInvCal(coor[0],coor[1],&lon,&lat);
		double lat_du,lat_fen,lat_miao,lon_du,lon_fen,lon_miao;
		lat_du=floor(lat);
		lat_fen=lat-lat_du;
		lat_fen*=60;
		lat_miao=lat_fen-floor(lat_fen);
		lat_miao*=60;
		lat_fen=floor(lat_fen);

		lon_du=floor(lon);
		lon_fen=lon-lon_du;
		lon_fen*=60;
		lon_miao=lon_fen-floor(lon_fen);
		lon_miao*=60;
		lon_fen=floor(lon_fen);
		qDebug("(%f,%f),(%d`%d``%f,%d`%d``%f)",lat,lon,(int)lat_du,(int)lat_fen,lat_miao,(int)lon_du,(int)lon_fen,lon_miao);
		break;
	case MEASURE_MODE:
		{
			double last[2],current[2];
			screenCorToXY(lastPoint[0],lastPoint[1],last);
			screenCorToXY(event->x(),height()-event->y(),current);
			double dis=LineTool::distanceP2P(current[0],current[1],last[0],last[1]);
			updateStatusBar(QString("distance is %0 meters").arg(dis/100));
			break;

		}
	}
}

void EditView::mouseReleaseEvent(QMouseEvent* event)
{
	if(event->button()==Qt::RightButton)
	{
		double cx=event->x(),cy=height()-event->y();

		cx-=lastPoint[0];
		cy-=lastPoint[1];
		if(cy==0&&cx==0) return;
		double yaw=atan2(cy,cx)/M_PI*180+270;
		if(yaw>=360) yaw-=360;
		double org[2];

		screenCorToXY(lastPoint[0],lastPoint[1],org);
		int type;
		
		if(currentMode==MONI_MODE)
		{
			POSE_INFO pi;
			pi.yaw=yaw/(180.0*1e-8)*3.1415926535898;
			pi.ins_coord.x=org[0];
			pi.ins_coord.y=org[1];
			dataArrived(pi);
		}
		else{
			GLPoint p;
			p.x=org[0];
			p.y=org[1];
			task->points.push_back(p);
			lineMap->taskLineComplete();
		}
		update();
		return;
	}

	switch(currentMode)
	{
	case MAP_MOVE_MODE:
		mapMove(event->x(),height()-event->y(),true);
		currentMode=IDLE_MODE;
		maniModeChanged(modes[currentMode]);
		break;
	case GUIDELINE_APPEND_MODE:
		double cor[2];
		screenCorToXY(event->x(),height()-event->y(),cor);
		lineMap->appendAtLine(cor[0],cor[1],0);
		update();
		break;
		
	}

	update();
}

void EditView::mouseDoubleClickEvent(QMouseEvent* event)
{
	if(currentMode!=IDLE_MODE) return;
	getPointID(event->x(),height()-event->y(),hitID);
	if(hitID[0]!=-1&&hitID[1]!=-1)
	{
		if(QMessageBox::Yes==QMessageBox::question(this,"point delete",QString("delete point in line %1 at %2").arg(hitID[0]).arg(hitID[1]),QMessageBox::Yes|QMessageBox::No,QMessageBox::No))
		{
			if(hitID[0]<(int)plines->size())
			{
				int ct=(*plines)[hitID[0]]->lineID.size();
				lineMap->deletePoint((*plines)[hitID[0]],hitID[1]);
				if(ct!=0)
					lineMap->vehiclePartition((*plines)[hitID[0]],hitID[0],10);	
			}
            else if(hitID[0]==(int)plines->size())
			{
				task->points.erase(task->points.begin()+hitID[1]);
				lineMap->taskLineComplete();
			}
			hitID[0]=hitID[1]=-1;
		}
		return;
	}
}

void EditView::wheelEvent(QWheelEvent *event)
{
	//地图缩放包括两部分： 1.更新模型坐标   2.更新卫星图
	
	if (event->orientation() == Qt::Vertical)		//卫星图缩放
		if(currentMode==SATELITE_MOVE_MODE)
		{
			int numDegrees = event->delta() / 8;
			double scale_factor=-0.001;
			if(numDegrees>0)
				scale_factor=-scale_factor;
			double mx=event->x(),
				my=height()-event->y(),xy[2];
			screenCorToXY(mx,my,xy);
			//[0,2]是左下角，[1 3] 右上角
			for(int i=0;i<4;i++)
				satellitMapBound[i]=(satellitMapBound[i]-xy[i/2])/stltK*(stltK+scale_factor)+xy[i/2];
			stltK+=scale_factor;
			qDebug("satellite map scale");
			update();
		}
		else{
			int numDegrees = event->delta() / 8;
			//int numSteps = numDegrees / 15;
			double scale_factor,dx,dy;
			dx=event->x()-width()/2.0;
			dy=height()/2.0-event->y();
			dx/=width()/2.0;
			dy/=height()/2.0;
			if(numDegrees<0)
			{
				scale_factor=SCALE_FACTOR;
				dx*=1-scale_factor;
				dy*=1-scale_factor;
			}
			else
			{
				scale_factor=1.0/SCALE_FACTOR;
				dx*=1-scale_factor;
				dy*=1-scale_factor;
			}

			GLdouble proj_matrix[16];
			glGetDoublev(GL_PROJECTION_MATRIX,proj_matrix);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glTranslated(dx,dy,0);
			glScaled(scale_factor,scale_factor,1);
			glMultMatrixd(proj_matrix);
			glMatrixMode(GL_MODELVIEW);
			update();
			groundResolution/=scale_factor;
			screenCorToXY(width()/2,height()/2,centerPoint);
			qDebug("groundResolution=%f,center=(%f,%f)",groundResolution,centerPoint[0],centerPoint[1]);
			//mapUpdateRequest();
		}
	
}

void EditView::keyPressEvent(QKeyEvent *event)
{
	if(currentMode!=IDLE_MODE) return;
	int key=event->key();
	double len=1000;
#ifndef FUZHU_CLOSE
	if(event->key()==Qt::Key_F)
	{
		char filename[]="task";
		if(fuzhu.points.size()!=0) 
		{
			fuzhu.points.clear();
			fuzhu.lineID.clear();
			fuzhu.index.clear();
		}
		ifstream file(filename,ios::in);
		if(file.fail())
			return;
		GuideLine* line=&fuzhu;
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
				lineMap->MercatorProjCal(lat, lon, &point.x, &point.y);
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
		update();
	}
	if(event->key()==Qt::Key_G)
	{
		if(fuzhu.points.size()!=0) 
		{
			fuzhu.points.clear();
			fuzhu.lineID.clear();
			fuzhu.index.clear();
		}
		update();
	}
#endif
	
	if(event->key()==Qt::Key_Alt)
	{
		currentMode=SATELLITE_SELECT_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_B)	//移动卫星图
	{
		currentMode=SATELITE_MOVE_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_Space)	//更新坐标
		mapUpdateRequest();
	if(event->key()==Qt::Key_W&&!event->isAutoRepeat())
	{
		lineMap->appendLine();
		currentMode=GUIDELINE_APPEND_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_Control)	//移动点
	{	
		currentMode=POINT_MOVE_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_G)
	{
		QPoint p=QCursor::pos();
		p=QWidget::mapFromGlobal(p);
		double cor[2];
		screenCorToXY(p.x(),height()-p.y(),cor);
		double lon,lat;
		lineMap->MercatorProjInvCal(cor[0],cor[1],&lon,&lat);
		updateStatusBar(QString("(lon,lat)=(%0,%1)").arg(lon).arg(lat));
	}
	if(event->key()==Qt::Key_Shift)	
	{
		currentMode=MEASURE_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_M)		//模拟接受串口数据模式
	{
		currentMode=MONI_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_Z)		//放大
    {
		double scale_factor;
    	scale_factor=1.0/SCALE_FACTOR;
		GLdouble proj_matrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX,proj_matrix);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glScaled(scale_factor,scale_factor,1);
		glMultMatrixd(proj_matrix);
		glMatrixMode(GL_MODELVIEW);
		update();
		groundResolution/=scale_factor;
		qDebug("groundResolution=%f",groundResolution);
    }
    if(event->key()==Qt::Key_X)		//缩小
    {
        
		double scale_factor;
    	scale_factor=SCALE_FACTOR;
		GLdouble proj_matrix[16];
		glGetDoublev(GL_PROJECTION_MATRIX,proj_matrix);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glScaled(scale_factor,scale_factor,1);
		glMultMatrixd(proj_matrix);
		glMatrixMode(GL_MODELVIEW);
		update();
		groundResolution/=scale_factor;
		qDebug("groundResolution=%f",groundResolution);
    }	   
	if(event->key()==Qt::Key_D)	//delete
	{
		if(hitID[0]!=-1&&hitID[0]==lastHit[0])	//同一条线，则执行删除操作
		{
			if(QMessageBox::Yes==QMessageBox::question(this,"point delete",QString("delete point in line %1 between %2 and %3").arg(hitID[0]).arg(hitID[1]).arg(lastHit[1]),QMessageBox::Yes|QMessageBox::No,QMessageBox::No))
			{
				GuideLine* line=(*plines)[hitID[0]];
				int e=hitID[1]>lastHit[1]? hitID[1]:lastHit[1],s=hitID[1]<lastHit[1]? hitID[1]:lastHit[1];
				for(int i=s;i<=e;i++)
				{
					lineMap->deletePoint(line,s);
					qDebug("%d",s);
				}
				lastHit[0]=lastHit[1]=hitID[0]=hitID[1]=-1;
				
			}
		}
		else
			QMessageBox::warning(this,"two point located in different lines","two point located in different lines",QMessageBox::Ok);
		
	}
	if(event->key()==Qt::Key_P)	//part from current point
	{
		if(hitID[0]>=0&&hitID[0]<plines->size())
		{
			if(QMessageBox::Yes==QMessageBox::question(this,"line parting",QString("parting line %1 at point %2").arg(hitID[0]).arg(hitID[1]),QMessageBox::Yes|QMessageBox::No,QMessageBox::No))
			{
				GuideLine* line=(*plines)[hitID[0]];
				GuideLine* nl=new GuideLine;
				for(int i=0;i<hitID[1];i++)
				{
					nl->points.push_back(line->points[0]);
					lineMap->deletePoint(line,0);
				}
				plines->push_back(nl);
			}
		}
	}
	if(event->key()==Qt::Key_C)	//connect
	{
		if(hitID[0]>=0&&hitID[0]<plines->size()&&lastHit[0]>=0&&lastHit[0]<plines->size())
		{
			GuideLine* first,* last;
			int secd;
			if(hitID[1]==0&&lastHit[1]==(*plines)[lastHit[0]]->points.size()-1)
			{
				first=(*plines)[lastHit[0]];
				last=(*plines)[hitID[0]];
				secd=hitID[0];
			}
			else
			{
				first=(*plines)[hitID[0]];
				last=(*plines)[lastHit[0]];
				secd=lastHit[0];
			}
			int ct=last->points.size();
			for(int i=0;i<ct;i++)
				first->points.push_back(last->points[i]);
			first->lineID.clear();
			first->index.clear();
			delete last;
			plines->erase(plines->begin()+secd);
		}
	}
	if(event->key()==Qt::Key_I)	//insert，在hit前
	{
		QPoint p=QCursor::pos();
		p=QWidget::mapFromGlobal(p);
		double cor[2];
		screenCorToXY(p.x(),height()-p.y(),cor);
		if(hitID[0]>=0&&hitID[0]==plines->size())//&&lastHit[0]==hitID[0])
		{
			int sm,bg=hitID[1];
/*
			if(hitID[1]-lastHit[1]==1)
				sm=lastHit[1],bg=hitID[1];
			else
				if(hitID[1]-lastHit[1]==-1)
					sm=hitID[1],bg=lastHit[1];
*/
			GuideLine* line;
			if(hitID[0]==plines->size())
				line=task;
			else
				line=(*plines)[hitID[0]];
			GLPoint pt;
			pt.x=cor[0];
			pt.y=cor[1];
			pt.yaw=0;	//如果需要增加道线的点，yaw需要计算以赋值
			pt.roadtype=30;	//roadtype需要手动设置
			line->points.insert(line->points.begin()+bg,pt);//加点
		}
		update();
	}
	if(event->key()==Qt::Key_T)	//turn前引导点
	{
        if(QMessageBox::Yes==QMessageBox::question(this,"point delete",QString(QStringLiteral("adding point?")),QMessageBox::Yes|QMessageBox::No,QMessageBox::No))
		{
			lineMap->taskTurnComplete();
			update();
		}
	}
	if(event->key()==Qt::Key_Q)	//根据id定位点
	{
		taskInputID=QInputDialog::getInt(this,"task point id","input task ID:",-1,0,task->points.size()-1);
		update();
	}
	
}

void EditView::keyReleaseEvent(QKeyEvent *event)
{
	if(currentMode==SATELLITE_SELECT_MODE&&event->key()==Qt::Key_Alt)
	{
		if(stltSelectedRect[0]!=0)
		{
			int Z=QInputDialog::getInt(this,"satellite map level","input satellite level【17-1m】:",18,1,23);
			if(Z!=-1)
				mapUpdateSelected(Z);
		}
		maniModeChanged(modes[currentMode]);
		memset(stltSelectedRect,0,sizeof(double)*4);
		currentMode=IDLE_MODE;
		update();
	}
	if(currentMode==SATELITE_MOVE_MODE&&event->key()==Qt::Key_B)
	{
		currentMode=IDLE_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_Control && currentMode==POINT_MOVE_MODE)
	{
		currentMode=IDLE_MODE;
		maniModeChanged(modes[currentMode]);
	}

	if(event->key()==Qt::Key_Shift && currentMode==MEASURE_MODE)
	{
		currentMode=IDLE_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_M && currentMode==MONI_MODE)
	{
		currentMode=IDLE_MODE;
		maniModeChanged(modes[currentMode]);
	}
	if(event->key()==Qt::Key_W&&!event->isAutoRepeat())
	{
		currentMode=IDLE_MODE;
		maniModeChanged(modes[currentMode]);
	}
}


/************************************************************************/
/* GLWidget  功能实现函数                                                */
/************************************************************************/




void EditView::mapMove(int x,int y,bool is_finished)		//要移动坐标还是移视角呢
{

	double dx=2.0*(x-lastPoint[0])/width(),
		dy=2.0*(y-lastPoint[1])/height();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	if(!is_finished)
	{
		glPushMatrix();
	}
	GLdouble proj_matrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX,proj_matrix);
	glLoadIdentity();
	glTranslated(dx,dy,0);
	glMultMatrixd(proj_matrix);
	glMatrixMode(GL_MODELVIEW);
	//screenCorToXY(width()/2,height()/2,centerPoint);
	if(is_finished)
	{
		dx=x-lastPoint[0];
		dy=y-lastPoint[1];
		centerPoint[0]-=dx*groundResolution;
		centerPoint[1]-=dy*groundResolution;
		qDebug("groundResolution=%f,center=(%f,%f)",groundResolution,centerPoint[0],centerPoint[1]);
	}

}

void EditView::pointMove(int x,int y)
{
	double coor[2];
	screenCorToXY(x,y,coor);
	if(hitID[0]==-1)
		return;
	GuideLine* line;
	if(hitID[0]<(int)plines->size())
		line=(*plines)[hitID[0]];
	else 
		line=task;
	line->points[hitID[1]].x=coor[0];
	line->points[hitID[1]].y=coor[1];
	update();
}



//【绘制
void EditView::drawTaskLine(int psize)
{
	glColor3f(0,1,0);	//原始颜色
	glLineWidth(1);
	glBegin(GL_LINE_STRIP);
    int tpcount=task->points.size();
    for(int i=0;i<tpcount;i++)
	{
		glVertex2d(task->points[i].x,task->points[i].y);	
	}
	glEnd();
	
	glPointSize(psize);		//设置点的大小
	glBegin(GL_POINTS);
    for(int i=0;i<tpcount;i++)
	{
		if(task->points[i].roadtype/10==1||task->points[i].roadtype/10==2)
			glColor3f(1,0,0);
		else
			glColor3f(0,1,0);
		glVertex2d(task->points[i].x,task->points[i].y);	
	}
	glEnd();
	glPointSize(1);
}
int colorMap(int i)
{
	int k[][3]={0,0,1,0,1,0,1,0,0,0,1,1,1,0,1,1,1,0};
	int r=255*k[i][0],
	g=255*k[i][1],b=255*k[i][2];
	return r*1000000+g*1000+b;
}

void EditView::displayAll()
{

	glClear(GL_COLOR_BUFFER_BIT);
	drawMap();
	if(currentMode==SATELLITE_SELECT_MODE)
	{
		glColor3f(1,0,0);
		int index[]={0,1,2,1,2,3,0,3};
		glBegin(GL_LINE_LOOP);
		for(int i=0;i<4;i++)
			glVertex2d(stltSelectedRect[index[2*i]],stltSelectedRect[index[2*i+1]]);
		glEnd();
	}
	glColor3f(0.7,0.3,0.6);
	glBegin(GL_POINTS);
	glVertex2d((satellitMapBound[1]+satellitMapBound[0])/2,(satellitMapBound[2]+satellitMapBound[3])/2);
	glEnd();
	//画任务规划
	if(dis_flag[6])
		for(int i=0;i<task->lineID.size();i++)
		{
			if(task->lineID[i]!=-1)
				drawLine((*plines)[task->lineID[i]],0,000255000,4,task->index[2*i],task->index[2*i+1]);
		}
	int lcount=lineMap->getLinesCount();
	if(dis_flag[1])
	{
		int color=0;
		if(dis_flag[0])
			color=255255000;
		for(int i=0;i<lcount;i++)
			drawLine((*plines)[i],color);
	}
	if(dis_flag[3])
	{
		int color=0;
		if(dis_flag[2])
			color=231112200;
		int ncount=navigator->size();
		for(int i=0;i<ncount;i++)
			drawLine((*navigator)[i],color,255000255);
	}
	
	/*
	//绘制生成导航线navigator时的比较线
	glColor3f(0,1,0);
	glLineWidth(4);
	for(int i=0;i<ncount;i++)
	{
		for(int j=0;j<(*navigator)[i]->points.size()-1;j++)
		{	 
			GLPoint pt=(*plines)[(*navigator)[i]->lineID[j]]->points[(*navigator)[i]->index[2*j]];
			double x=pt.x,
			y=pt.y,
			agl=(pt.yaw+90)/180.0*M_PI,
			x1=x+cos(agl)*10000,
			y1=y+sin(agl)*10000;
			glBegin(GL_LINES);
				glVertex2d(x,y);
				glVertex2d(x1,y1);
			glEnd();
		}
	}	
*/

	//画task point
	if(dis_flag[5])
		drawTaskLine(6);
#ifndef FUZHU_CLOSE
	{
		glColor3f(0,1,1);	//原始颜色
		glLineWidth(1);
		glBegin(GL_LINE_STRIP);
		int tpcount=fuzhu.points.size();
		for(int i=0;i<tpcount;i++)
		{
			glVertex2d(fuzhu.points[i].x,fuzhu.points[i].y);	
		}
		glEnd();

		glPointSize(5);		//设置点的大小
		glBegin(GL_POINTS);
		for(int i=0;i<tpcount;i++)
		{
			if(fuzhu.points[i].roadtype/10==1||fuzhu.points[i].roadtype/10==2)
				glColor3f(1,0,1);
			else
				glColor3f(1,1,0);
			glVertex2d(fuzhu.points[i].x,fuzhu.points[i].y);	
		}
		glEnd();
		glPointSize(1);
	}

#endif

	drawHit();	
	
	//画鼠标选中的点
	if(hitID[0]>=0&&hitID[0]<=plines->size())
	{
		GuideLine* line;
		if(hitID[0]==plines->size())	line=task;
		else line=(*plines)[hitID[0]];
		glColor3f(0,1,1);
		glBegin(GL_POINTS);
			glVertex2d(line->points[hitID[1]].x,line->points[hitID[1]].y);
		glEnd();
	}

	setRec(currentPos[0],currentPos[1],currentPos[2],rect);
	glColor3f(1,0,1);
	glPointSize(4);
	glBegin(GL_POINTS);
	glVertex2d(currentPos[0],currentPos[1]);
	glEnd();

	glBegin(GL_LINE_LOOP);
	for(int i=0;i<8;i+=2)
		glVertex2d(rect[i],rect[i+1]);
	glEnd();

	if(taskInputID>=0&&taskInputID<task->points.size())
	{
		glColor3f(0,1,1);
		glPointSize(5);
		glBegin(GL_POINTS);
		glVertex2d(task->points[taskInputID].x,task->points[taskInputID].y);
		glEnd();
	}
	

}

void EditView::drawLine(GuideLine* line,int pcolor,int lcolor,int line_width,int stt,int ed)
{
	if(stt==-1&&ed==-1)
        stt=0,ed=line->points.size()-1;
glPushMatrix();							//保存原坐标系
	glTranslated(lineMap->bound[0],lineMap->bound[2],0);			//转换到局部坐标系

	glColor3f(lcolor/1000/1000/255.0,lcolor/1000%1000/255.0,lcolor%1000/255.0);
	glLineWidth(line_width);
	glBegin(GL_LINE_STRIP);
	
	for(int i=stt;i<=ed;i++)
	{
		glVertex2d((line->points[i].x-lineMap->bound[0]),(line->points[i].y-lineMap->bound[2]));
	}
	glEnd();
	if(pcolor==0)
	{
		glPopMatrix();
		return;
	}
	glPointSize(5);		//设置点的大小


	glBegin(GL_POINTS);	
	glColor3f(pcolor/1000/1000/255.0,pcolor/1000%1000/255.0,pcolor%1000/255.0);			//点的颜色
	for(int j=stt;j<=ed;j++)
	{
		glVertex2d((line->points[j].x-lineMap->bound[0]),(line->points[j].y-lineMap->bound[2]));
	}
	glEnd();	

glPopMatrix();
}

void EditView::drawHit()
{
#ifdef UGV_OPEN
	glColor3f(0,1,0);
	glPointSize(5);
	glBegin(GL_POINTS);
	for(int i=0;i<opt.scene.gls.valid;i++)
	{
		double rx=opt.scene.gls.gps[i].x,ry=opt.scene.gls.gps[i].y,x=currentPos[0],y=currentPos[1];
		x+=rx*cos(currentPos[2])-ry*sin(currentPos[2]);
		y+=rx*sin(currentPos[2])+ry*cos(currentPos[2]);
		glVertex2d(x,y);
	}
	glEnd();

	glColor3f(1,0,1);
	glPointSize(5);
	glBegin(GL_POINTS);
	for(int i=0;i<4;i++)
		if(opt.tps[i].id!=0)
			glVertex2d(opt.tps[i].x,opt.tps[i].y);
	for(int i=0;i<opt.valid_mid;i++)
		glVertex2d(opt.mid[i].x,opt.mid[i].y);
	glEnd();
#endif
}
//绘制】


void EditView::selectPoints()
{	
	vector<GuideLine*>& pl=*plines;
#ifdef SELECT_ROAD_LINE_OPEN
	for(int i=0;i<pl.size();i++)
	{
		GuideLine* line=pl[i];
		glPushName(i);
		for(int j=0;j< line->points.size() ;j++)
		{
			glPushName(j);
			glBegin(GL_POINTS);	
			glVertex2d(line->points[j].x,line->points[j].y);
			glEnd();
			glPopName();
		}
		glPopName();
	}
#endif
	GuideLine* line=task;
	glPushName(pl.size());
	for(int j=0;j< line->points.size() ;j++)
	{
		glPushName(j);
		glBegin(GL_POINTS);	
		glVertex2d(line->points[j].x,line->points[j].y);
		glEnd();
		glPopName();
	}
	glPopName();


}

void EditView::getPointID(int x,int y,int *ID)
{
#ifdef __glu_h__
	GLint viewport[4];
	GLdouble projMatrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
	glGetIntegerv(GL_VIEWPORT,viewport);
	GLuint pickStack[62];
	glSelectBuffer(62,pickStack);

	glRenderMode(GL_SELECT);
	glInitNames();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluPickMatrix(x,y,10,10,viewport);
	glMultMatrixd(projMatrix);
	glMatrixMode(GL_MODELVIEW);

	selectPoints();

	glMatrixMode(GL_PROJECTION);



	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	int hits=glRenderMode(GL_RENDER);
	qDebug("hit=%d,%d",pickStack[3],pickStack[4]);

	ID[0]=ID[1]=-1;
	if(hits>0)	//选中了点，执行更改点操作
		ID[1]=pickStack[4],ID[0]=pickStack[3];	//此处是默认操作，未进行数据合法性检测
#else
	qDebug("No GLU,getPoint useless");
#endif
}
/************************************************************************/
/* 计算函数（槽函数）                                                                     */
/************************************************************************/
void EditView::screenCorToXY(int x,int y,double* ans)
{
#ifdef __glu_h__
    GLdouble modelMatrix[16],projMatrix[16],objx,objy,objz;
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);
	glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
	glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
	gluUnProject(x,y,0,modelMatrix,projMatrix,viewport,&objx,&objy,&objz);
	
	ans[0]=objx;
	ans[1]=objy;
#else
	qDebug("No glu,screenCorToXY useless");
#endif
}

void EditView::nvgtGen(double threshold)
{
	int lnum=plines->size(),nnum=navigator->size();
	for(int i=0;i<nnum;i++)
		delete (*navigator)[i];
	navigator->clear();
	for(int i=0;i<lnum;i++)
		navigator->push_back(lineMap->vehiclePartition((*plines)[i],i,threshold));

}
void EditView::reduceSample()
{
	int lnum=plines->size();
	for(int i=0;i<lnum;i++)
		lineMap->reduceSample((*plines)[i]);
}



//地图
void EditView::mapUpdateRequest()		//根据当前窗口中心的坐标与窗口大小获取对应的地图
{
	double center_x=width()/2.0,center_y=height()/2;
	double org[2];
	screenCorToXY(center_x,center_y,centerPoint);
	screenCorToXY(0,0,org);
	double lon,lat,buff,buf;
	lineMap->MercatorProjInvCal(centerPoint[0]-stltOffSet_cm[0],centerPoint[1]-stltOffSet_cm[1],&lon,&lat);
	lineMap->MercatorProjInvCal(org[0],org[1],&buff,&buf);


	groundResolution=2*(centerPoint[0]-org[0])/width();
	qDebug("q=%f",groundResolution);
	double	ground_resolution=2*(lon-buff)/width();	//度/像素
	int z=CLARITY_FACTOR+log(360.0/ground_resolution)/log(2.0)-8;	//向下取整
	qDebug("z=%d",z);
	if(!(z==satellitMap->getZ()&&isEyeInMap(centerPoint[0],centerPoint[1])))
		satellitMap->loadMap(lon,lat,z);
	
}

void EditView::mapUpdateSelected(int stltZ)
{
	double lon,lat,buff,buf;
	lineMap->MercatorProjInvCal((stltSelectedRect[0]+stltSelectedRect[2])/2-stltOffSet_cm[0],(stltSelectedRect[1]+stltSelectedRect[3])/2-stltOffSet_cm[1],&lon,&lat);
	double scl=1.2;
	int z=stltZ;
	double gr=lineMap->getEquatorRADIUS()/pow(2.0,z+7.0);	
	int w=scl*abs(stltSelectedRect[2]-stltSelectedRect[0])/gr,h=scl*abs(stltSelectedRect[3]-stltSelectedRect[1])/gr;
	satellitMap->SetRect(w,h);
	satellitMap->loadMap(lon,lat,z);
}

bool EditView::isEyeInMap(double x,double y)
{
	return (x-groundResolution*width()/2>=satellitMapBound[0] &&
		x+groundResolution*width()/2<=satellitMapBound[1] &&
		y-groundResolution*height()/2>=satellitMapBound[2] &&
		y+groundResolution*height()/2<=satellitMapBound[3] );
	
}

void EditView::bindTexture()
{
	QImage image=satellitMap->getMap().toImage();
	glTexImage2D(GL_TEXTURE_2D, 0,3, image.width(), image.height(), 0, GL_BGRA_EXT , GL_UNSIGNED_BYTE, image.bits());//开始真正创建纹理数据  
	double width=satellitMap->getWidth()/2,height=satellitMap->getHeight()/2;
	/*
	satellitMapBound[0]=centerPoint[0]-groundResolution*width;
	satellitMapBound[1]=centerPoint[0]+groundResolution*width;
	satellitMapBound[2]=centerPoint[1]-groundResolution*height;
	satellitMapBound[3]=centerPoint[1]+groundResolution*height;

		*/
	const double* bound=satellitMap->getBound();
	lineMap->MercatorProjCal(bound[0],bound[2],&satellitMapBound[0],&satellitMapBound[2]);
	lineMap->MercatorProjCal(bound[1],bound[3],&satellitMapBound[1],&satellitMapBound[3]);


	//qDebug("return map,center(%f\t%f),size (%f\t%f)\n",(satellitMapBound[0]+satellitMapBound[1])/2.0,(satellitMapBound[2]+satellitMapBound[3])/2.0,satellitMapBound[1]-satellitMapBound[0],satellitMapBound[3]-satellitMapBound[2]);
	update();
}

void EditView::drawMap()
{
	glColor3f(1,1,1);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);  
	glTexCoord2f( 0.0, 1.0 ); glVertex2f(satellitMapBound[0]+stltOffSet_cm[0],satellitMapBound[2]+stltOffSet_cm[1]);  
	glTexCoord2f( 1.0, 1.0 ); glVertex2f(satellitMapBound[1]+stltOffSet_cm[0],satellitMapBound[2]+stltOffSet_cm[1]);  
	glTexCoord2f( 1.0, 0.0 ); glVertex2f(satellitMapBound[1]+stltOffSet_cm[0],satellitMapBound[3]+stltOffSet_cm[1]);  
	glTexCoord2f( 0.0, 0.0 ); glVertex2f(satellitMapBound[0]+stltOffSet_cm[0],satellitMapBound[3]+stltOffSet_cm[1]);  
	glEnd(); 
	glDisable(GL_TEXTURE_2D);
}




//采集
void EditView::dataArrived(POSE_INFO pi)
{
	double x,y,yaw;
	x=pi.ins_coord.x;
	y=pi.ins_coord.y;
	yaw=pi.yaw*(180.0*1e-8)/3.1415926535898;

	unsigned int type=pi.checksum;
	currentPos[0]=x;
	currentPos[1]=y;
	currentPos[2]=yaw;

#ifdef UGV_OPEN
	GP_FUNC_INPUT ipt;
	GP_LOCAL_DATA evt;
	ipt.reco_point.recovery=1;
	ipt.state.pos.yaw=pi.yaw;
	ipt.state.pos.com_coord.x=x;
	ipt.state.pos.com_coord.y=y;
	UGV_GP::Global_Plan(&ipt,&opt,&evt);
#endif
	if(isTracking)
	{	
		GuideLine* line=(*plines)[plines->size()-1];
		GLPoint pt;
		pt.x=x;
		pt.y=y;
		pt.yaw=yaw;
		pt.roadtype=type;
		line->points.push_back(pt);
		//第一个点才确定方向位置
		if(line->points.size()==1)
		{
			double dx=x-centerPoint[0],dy=y-centerPoint[1];
			glTranslated(-dx,-dy,0);
			centerPoint[0]+=dx;
			centerPoint[1]+=dy;
		}
	}
	emit updateStatusBar(QString("x=%1,y=%2,yaw=%3").arg(x).arg(y).arg(yaw));
	update();


}

void EditView::setRoadTypeInImu(int state)
{
	imusvc->setRoadType(state);
}
