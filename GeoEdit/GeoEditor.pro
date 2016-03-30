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
     editview.cpp \
     geoeditor.cpp \
     imusvc.cpp \
     main.cpp \
     RawGuideLines.cpp \
     satellitemap.cpp \
     gp_linux/gp_shell.cpp \
     gp_linux/gpmodule.cpp  \
     gp_linux/NJUST_MC_proc.cpp

HEADERS  += \
     editview.h \
     geoeditor.h \
     imusvc.h \
     RawGuideLines.h \
     satellitemap.h \
     gp_linux/gpmodule.h \
	 gp_linux/robix4/protocols/*.h\
	 gl/GLU.h\
	 gl/GL.h
FORMS    += \
     geoeditor.ui
