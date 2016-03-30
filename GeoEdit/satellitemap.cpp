#include "satellitemap.h"
#include "RawGuideLines.h"
SatelliteMap::SatelliteMap(void)
{
	SatelliteMap(1024,1024);
}
SatelliteMap::SatelliteMap(int w,int h)
{
	Z=-1;
	bound[0]=bound[1]=bound[2]=bound[3]=0;
	pMap=NULL;
	SetRect(w,h);
	tileBuff.initial();
	pNetMng=new QNetworkAccessManager(this);
	connect(pNetMng,SIGNAL(finished(QNetworkReply*)),this,SLOT(httpFinished(QNetworkReply*)));
	LLOffset[0]=LLOffset[1]=0;
	QDir dir;
	if(!dir.exists(tr(TILE_TEMP_DIR)))
	{
		if(!dir.mkdir(tr(TILE_TEMP_DIR)))
			qDebug("无法创建文件夹\n");
	}
}
SatelliteMap::~SatelliteMap(void)
{
	tileBuff.destroy();
	if(pNetMng!=NULL)
		delete pNetMng;
	if(pMap!=NULL)
		delete pMap;
}
void SatelliteMap::loadMap(double lon,double lat,int z)	//提供需要下载地图的中心经纬度、宽高、地面分辨率
{
	if(z<0) z=0;
	if(z>20) z=20;
	
	const double len=pow(2.0,z);
	{		//计算包围矩形(在opengl中放地图的地方）
		double x=lon/180.0*M_PI,
			y=lat/180.0*M_PI,
			r=M_PI/len/128.0;		//地表分表率，每个像素代表的距离
		y=log(tan(y)+1/cos(y));
		int px=getWidth()/2,		//图像中的目标点的像素坐标x，y
			py=getHeight()/2;
		//	qDebug("asked for center (%f,%f)",x,y);
		//笛卡尔坐标
		bound[0]=x-px*r;
		bound[1]=bound[0]+getWidth()*r;
		bound[3]=y+py*r;
		bound[2]=bound[3]-getHeight()*r;
		//	qDebug("return with center (%f,%f)",(bound[0]+bound[1])/2,(bound[2]+bound[3])/2);
		//	qDebug("with deviation　(%f,%f)",((bound[0]+bound[1])/2-x)/(bound[1]-bound[0]),((bound[3]+bound[2])/2-y)/(bound[3]-bound[2]));
		//转为弧度
		bound[2]=2*atan(exp(bound[2]))-M_PI/2;
		bound[3]=2*atan(exp(bound[3]))-M_PI/2;
		//角度表示
		bound[0]=bound[0]*180.0/M_PI+LLOffset[0];
		bound[1]=bound[1]*180.0/M_PI+LLOffset[0];
		bound[2]=bound[2]*180.0/M_PI+LLOffset[1];
		bound[3]=bound[3]*180.0/M_PI+LLOffset[1];
	}

	//计算下载的哪一部分地图

//	lon+=dlon;
//	lat-=dlat;

	int dx=1+ceil((width/2.0-128)/256),		//填满地图横向至少需要2*dx+1块瓦片
		dy=1+ceil((height/2.0-128)/256);		//纵向至少需要2*dy+1块瓦片

	//以下几行代码实现：分辨率获取精度
	
	

	double tx=len/2.0*(lon/180+1),		//中心点对应的x
		ty=len/2.0*(1-(log(tan(M_PI*lat/180)+1.0/cos(M_PI*lat/180)))/M_PI);		//中心点的y
	int center_x=floor(tx),			//中心所在瓦片的x
		center_y=floor(ty);			//中心所在瓦片的y
	centerTileCor[0]=center_x;
	centerTileCor[1]=center_y;
	centerPixelCor[0]=(tx-center_x)*256-128;			//中心在瓦片内的像素偏移
	centerPixelCor[1]=(ty-center_y)*256-128;
//	qDebug("center tile(%d,%d),target at (%d,%d)",center_x,center_y,centerPixelCor[0],centerPixelCor[1]);
	int xmax,xmin,ymax,ymin;
	xmin=center_x-dx>0? center_x-dx:0;
	xmax=center_x+dx<len-1? center_x+dx:len-1;
	ymin=center_y-dy>0? center_y-dy:0;
	ymax=center_y+dy<len-1? center_y+dy:len-1;

	
	


	requestCount=(xmax-xmin+1)*(ymax-ymin+1);

	for(int x=xmin;x<=xmax;x++)
		for(int y=ymin;y<=ymax;y++)
		{	 
			getTile(x,y,z);
		}

}
void SatelliteMap::getTile(int x,int y,int z)	//从内存，文件或服务器获取瓦片
{
	QUrl url(getTileURL(x,y,z));
	qDebug(getTileURL(x,y,z).toLatin1().data());
	QFileInfo file_info(url.path());
	QString file_name = file_info.fileName();
	QPixmap *pTile=tileBuff.getTile(file_name);
	if(pTile)													//瓦片已在内存中
	{
		tileStick(file_name,*pTile);
	}
	else if(QFile::exists(QString(TILE_TEMP_DIR)+file_name))	//瓦片已经存在于文件系统中
	{
		pTile=new QPixmap(QString(TILE_TEMP_DIR)+file_name);
		tileStick(file_name,*pTile);
		tileBuff.insert(file_name,pTile);
	}
	else
		pNetMng->get(QNetworkRequest(url));
}
void SatelliteMap::httpFinished(QNetworkReply *reply)
{
	QUrl url=reply->url();
	QFileInfo file_info(url.path());
	QString file_name = file_info.fileName();
	QByteArray ba=reply->readAll();
	QFile file(tr(TILE_TEMP_DIR)+file_name);
	if(!file.open(QIODevice::WriteOnly))
	{	//文件无法打开。
    //	qDebug("%s open error\n",file_name);
		reply->deleteLater();				
		return;
	}
	file.write(ba);
	file.flush();
	file.close();
	QPixmap *pTile=new QPixmap();
	pTile->loadFromData(ba);
	//提取文件中的x y z信息 并填充到对应位置
	tileStick(file_name,*pTile);
	tileBuff.insert(file_name,pTile);
	reply->deleteLater();
	return;
}
void SatelliteMap::tileStick(QString tilename,const QPixmap& tile)
{
	int x,y;
	parseFileName(tilename,x,y);
	int dx=centerTileCor[0]-x,dy=centerTileCor[1]-y;
	int origin_x=pMap->width()/2-128-dx*256-centerPixelCor[0],		//以中心为标准向四周贴图
		origin_y=pMap->height()/2-128-dy*256-centerPixelCor[1];
	QPainter p(pMap);
	p.drawPixmap(origin_x,origin_y,256,256,tile);
	requestCount--;
	if(requestCount==0)
	{
		save("map.png");
		emit update();
	}
}
bool SatelliteMap::save(QString filename)	
{
	return pMap->save(filename,"PNG");
}
void SatelliteMap::mapRectify(double dlon,double dlat)
{
	LLOffset[0]+=dlon;
	LLOffset[1]+=dlat;
	bound[0]+=dlon;
	bound[1]+=dlon;
	bound[2]+=dlat;
	bound[3]+=dlat;
	update();
}
QString SatelliteMap::getTileURL(int x,int y,int z)
{
	return tr(URL_SCHEMA).arg(x).arg(y).arg(z);
}
QString SatelliteMap::getTileFileName(int x,int y,int z)
{
	return tr(FILE_NAME_SCHEMA).arg(x).arg(y).arg(z);
}
void SatelliteMap::parseFileName(QString filename,int& x,int& y)
{
	QByteArray b1=filename.toLatin1();
	char *buff=b1.data();
	int i=24,j=0;
	char digits[10];
	while(buff[i]!='&') digits[j++]=buff[i++];
	digits[j]=0;
	x=QString(digits).toInt();
	j=0;
	i+=3;
	while(buff[i]!='&') digits[j++]=buff[i++];
	digits[j]=0;
	y=QString(digits).toInt();
}


