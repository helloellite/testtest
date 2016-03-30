#include "geoeditor.h"
#include <QApplication>

int main(int argc, char *argv[])
{

	QApplication a(argc, argv);
	GeoEditor w;
	w.show();
	return a.exec();

}
