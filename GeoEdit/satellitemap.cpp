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
			qDebug("�޷������ļ���\n");
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
void SatelliteMap::loadMap(double lon,double lat,int z)	//�ṩ��Ҫ���ص�ͼ�����ľ�γ�ȡ���ߡ�����ֱ���
{
	if(z<0) z=0;
	if(z>20) z=20;
	
	const double len=pow(2.0,z);
	{		//�����Χ����(��opengl�зŵ�ͼ�ĵط���
		double x=lon/180.0*M_PI,
			y=lat/180.0*M_PI,
			r=M_PI/len/128.0;		//�ر�ֱ��ʣ�ÿ�����ش���ľ���
		y=log(tan(y)+1/cos(y));
		int px=getWidth()/2,		//ͼ���е�Ŀ������������x��y
			py=getHeight()/2;
		//	qDebug("asked for center (%f,%f)",x,y);
		//�ѿ�������
		bound[0]=x-px*r;
		bound[1]=bound[0]+getWidth()*r;
		bound[3]=y+py*r;
		bound[2]=bound[3]-getHeight()*r;
		//	qDebug("return with center (%f,%f)",(bound[0]+bound[1])/2,(bound[2]+bound[3])/2);
		//	qDebug("with deviation��(%f,%f)",((bound[0]+bound[1])/2-x)/(bound[1]-bound[0]),((bound[3]+bound[2])/2-y)/(bound[3]-bound[2]));
		//תΪ����
		bound[2]=2*atan(exp(bound[2]))-M_PI/2;
		bound[3]=2*atan(exp(bound[3]))-M_PI/2;
		//�Ƕȱ�ʾ
		bound[0]=bound[0]*180.0/M_PI+LLOffset[0];
		bound[1]=bound[1]*180.0/M_PI+LLOffset[0];
		bound[2]=bound[2]*180.0/M_PI+LLOffset[1];
		bound[3]=bound[3]*180.0/M_PI+LLOffset[1];
	}

	//�������ص���һ���ֵ�ͼ

//	lon+=dlon;
//	lat-=dlat;

	int dx=1+ceil((width/2.0-128)/256),		//������ͼ����������Ҫ2*dx+1����Ƭ
		dy=1+ceil((height/2.0-128)/256);		//����������Ҫ2*dy+1����Ƭ

	//���¼��д���ʵ�֣��ֱ��ʻ�ȡ����
	
	

	double tx=len/2.0*(lon/180+1),		//���ĵ��Ӧ��x
		ty=len/2.0*(1-(log(tan(M_PI*lat/180)+1.0/cos(M_PI*lat/180)))/M_PI);		//���ĵ��y
	int center_x=floor(tx),			//����������Ƭ��x
		center_y=floor(ty);			//����������Ƭ��y
	centerTileCor[0]=center_x;
	centerTileCor[1]=center_y;
	centerPixelCor[0]=(tx-center_x)*256-128;			//��������Ƭ�ڵ�����ƫ��
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
void SatelliteMap::getTile(int x,int y,int z)	//���ڴ棬�ļ����������ȡ��Ƭ
{
	QUrl url(getTileURL(x,y,z));
	qDebug(getTileURL(x,y,z).toLatin1().data());
	QFileInfo file_info(url.path());
	QString file_name = file_info.fileName();
	QPixmap *pTile=tileBuff.getTile(file_name);
	if(pTile)													//��Ƭ�����ڴ���
	{
		tileStick(file_name,*pTile);
	}
	else if(QFile::exists(QString(TILE_TEMP_DIR)+file_name))	//��Ƭ�Ѿ��������ļ�ϵͳ��
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
	{	//�ļ��޷��򿪡�
    //	qDebug("%s open error\n",file_name);
		reply->deleteLater();				
		return;
	}
	file.write(ba);
	file.flush();
	file.close();
	QPixmap *pTile=new QPixmap();
	pTile->loadFromData(ba);
	//��ȡ�ļ��е�x y z��Ϣ ����䵽��Ӧλ��
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
	int origin_x=pMap->width()/2-128-dx*256-centerPixelCor[0],		//������Ϊ��׼��������ͼ
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


