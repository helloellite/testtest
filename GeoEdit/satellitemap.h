#include <QtNetwork>
#include <QPixmap>
#include <QFile>
#include <QFileInfo>
#include <QPainter>
#include <vector>
#include <cmath>
using namespace std;
#define _USE_MATH_DEFINES
#define URL_SCHEMA "http://mt1.google.cn/vt/lyrs=s&hl=zh-CN&gl=cn&x=%1&y=%2&z=%3"		//更新url和filename模式时，同时需要更改对应的解析函数 parseFileName(QString filename,int& x,int& y)
#define FILE_NAME_SCHEMA "lyrs=s&hl=zh-CN&gl=cn&x=%1&y=%2&z=%3"
#define TILE_TEMP_DIR	"CONFIG/tile temp/"
#define TILE_BUFF_SIZE 10000		
class SatelliteMap: public QObject
{
	Q_OBJECT
struct TileBuff 
{
	QPixmap* buff[TILE_BUFF_SIZE];		//存放加载进内存的瓦片，循环队列，实际大小为TILE_BUFF_SIZE-1
	QString names[TILE_BUFF_SIZE];	//存放加载进内存的瓦片的大小
	int tilePointer;	//tileBuff作为循环指针，tilePointer指向下一个插入tile的位置
	void initial()
	{
		for(int i=0;i<TILE_BUFF_SIZE;i++)
			buff[i]=NULL;
		tilePointer=0;
	}
	void insert(QString name,QPixmap *p)
	{
		buff[tilePointer]=p;
		names[tilePointer]=name;
		tilePointer=(tilePointer+1)%TILE_BUFF_SIZE;
	}
	void destroy()
	{
		int i=0;
		while(buff[tilePointer]!=NULL&&i<TILE_BUFF_SIZE)
		{
			delete buff[tilePointer];
			tilePointer=(tilePointer-1+TILE_BUFF_SIZE)%TILE_BUFF_SIZE;
			i++;
		}
	}
	QPixmap* getTile(QString tilename)
	{
		int p=tilePointer;
		while(p=(p-1+TILE_BUFF_SIZE)%TILE_BUFF_SIZE,buff[p]!=NULL && names[p]!=tilename && p!=tilePointer);
		if(names[p]==tilename)
			return buff[p];
		else 
			return NULL;
	}
	
};
signals:
	void update();

private slots:
	void httpFinished(QNetworkReply *reply);

public:
	SatelliteMap(void);				//地图大小默认为1024*1024
	SatelliteMap(int w,int h);		//TIP:w，h最好是偶数，不然会有半个像素的误差
	~SatelliteMap(void);
	//函数接口
	void loadMap(double lon,double lat,int z);//提供需要下载地图的中心经纬度、宽高、缩放等级
	bool save(QString filename);				//保存地图
	bool isMapAvailable(){return requestCount==0 && pMap!=NULL;};				//地图完成所有加载任务
	const QPixmap& getMap(){return *pMap;};		//获得地图
	int getWidth(){return pMap->width();};		//获取地图宽度
	int getHeight(){return pMap->height();};	//获取地图长度
	int getZ(){return Z;};
	const double* getBound(){return bound;}	;	//返回地图范围，以经纬度为单位
	void mapRectify(double dx,double dy);
	void SetRect(int w,int h)					//设置地图大小。设置完后原地图数据将被删除
	{
		width=w;
		height=h;
		if (pMap!=NULL)
			delete pMap;
		pMap=new QPixmap(width,height);
		qDebug("widht=%d",width);
		qDebug("pwidht=%d",pMap->width());

	}

//private:
	//成员变量
	QPixmap* pMap;
	QNetworkAccessManager *pNetMng;
	int requestCount;				//瓦片总数
	int centerTileCor[2];		//中心瓦片的坐标
	int centerPixelCor[2];		//地图中心在图中对应的像素坐标
	int width,height;
	double bound[4];
	double LLOffset[2];
	TileBuff tileBuff;
	//函数功能的实现函数
	int Z;
	void getTile(int x,int y,int z);		//下载z级下的(x,y)瓦片
	QString getTileURL(int x,int y,int z);			//获取第(x,y,z)瓦片的URL地址
	QString getTileFileName(int x,int y,int z);			//获取第(x,y,z)瓦片对应的文件名
	void parseFileName(QString filename,int& x,int& y);		//解析文件名为filename的瓦片，得到其(x,y)
	void tileStick(QString tilename,const QPixmap& tile);		//将名为tilename的瓦片tile贴到map中。tilename用来解析(x,y)
};
