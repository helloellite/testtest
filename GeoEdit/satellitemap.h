#include <QtNetwork>
#include <QPixmap>
#include <QFile>
#include <QFileInfo>
#include <QPainter>
#include <vector>
#include <cmath>
using namespace std;
#define _USE_MATH_DEFINES
#define URL_SCHEMA "http://mt1.google.cn/vt/lyrs=s&hl=zh-CN&gl=cn&x=%1&y=%2&z=%3"		//����url��filenameģʽʱ��ͬʱ��Ҫ���Ķ�Ӧ�Ľ������� parseFileName(QString filename,int& x,int& y)
#define FILE_NAME_SCHEMA "lyrs=s&hl=zh-CN&gl=cn&x=%1&y=%2&z=%3"
#define TILE_TEMP_DIR	"CONFIG/tile temp/"
#define TILE_BUFF_SIZE 10000		
class SatelliteMap: public QObject
{
	Q_OBJECT
struct TileBuff 
{
	QPixmap* buff[TILE_BUFF_SIZE];		//��ż��ؽ��ڴ����Ƭ��ѭ�����У�ʵ�ʴ�СΪTILE_BUFF_SIZE-1
	QString names[TILE_BUFF_SIZE];	//��ż��ؽ��ڴ����Ƭ�Ĵ�С
	int tilePointer;	//tileBuff��Ϊѭ��ָ�룬tilePointerָ����һ������tile��λ��
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
	SatelliteMap(void);				//��ͼ��СĬ��Ϊ1024*1024
	SatelliteMap(int w,int h);		//TIP:w��h�����ż������Ȼ���а�����ص����
	~SatelliteMap(void);
	//�����ӿ�
	void loadMap(double lon,double lat,int z);//�ṩ��Ҫ���ص�ͼ�����ľ�γ�ȡ���ߡ����ŵȼ�
	bool save(QString filename);				//�����ͼ
	bool isMapAvailable(){return requestCount==0 && pMap!=NULL;};				//��ͼ������м�������
	const QPixmap& getMap(){return *pMap;};		//��õ�ͼ
	int getWidth(){return pMap->width();};		//��ȡ��ͼ���
	int getHeight(){return pMap->height();};	//��ȡ��ͼ����
	int getZ(){return Z;};
	const double* getBound(){return bound;}	;	//���ص�ͼ��Χ���Ծ�γ��Ϊ��λ
	void mapRectify(double dx,double dy);
	void SetRect(int w,int h)					//���õ�ͼ��С���������ԭ��ͼ���ݽ���ɾ��
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
	//��Ա����
	QPixmap* pMap;
	QNetworkAccessManager *pNetMng;
	int requestCount;				//��Ƭ����
	int centerTileCor[2];		//������Ƭ������
	int centerPixelCor[2];		//��ͼ������ͼ�ж�Ӧ����������
	int width,height;
	double bound[4];
	double LLOffset[2];
	TileBuff tileBuff;
	//�������ܵ�ʵ�ֺ���
	int Z;
	void getTile(int x,int y,int z);		//����z���µ�(x,y)��Ƭ
	QString getTileURL(int x,int y,int z);			//��ȡ��(x,y,z)��Ƭ��URL��ַ
	QString getTileFileName(int x,int y,int z);			//��ȡ��(x,y,z)��Ƭ��Ӧ���ļ���
	void parseFileName(QString filename,int& x,int& y);		//�����ļ���Ϊfilename����Ƭ���õ���(x,y)
	void tileStick(QString tilename,const QPixmap& tile);		//����Ϊtilename����Ƭtile����map�С�tilename��������(x,y)
};
