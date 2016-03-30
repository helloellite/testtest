#-------------------------------------------------
#
# Project created by QtCreator 2015-08-27T19:20:05
#
#-------------------------------------------------

QT       += core gui
QT += opengl
QT += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
DESTDIR += .
TARGET = GeoEditor
TEMPLATE = app
LIBS +=

SOURCES += \
     *.cpp\
    gp_linux\*.cpp

HEADERS  += \
      *.h\
     gp_linux/*.h \
     gp_linux/robix4/protocols/*.h \
     gp_linux\NJUST_ALV_BYD_H/*.h \

FORMS    += \
     geoeditor.ui
