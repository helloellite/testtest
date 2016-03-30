/********************************************************************************
** Form generated from reading UI file 'geoeditor.ui'
**
** Created by: Qt User Interface Compiler version 5.4.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GEOEDITOR_H
#define UI_GEOEDITOR_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GeoEditorClass
{
public:
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *GeoEditorClass)
    {
        if (GeoEditorClass->objectName().isEmpty())
            GeoEditorClass->setObjectName(QStringLiteral("GeoEditorClass"));
        GeoEditorClass->resize(600, 400);
        centralWidget = new QWidget(GeoEditorClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        GeoEditorClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(GeoEditorClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 23));
        GeoEditorClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(GeoEditorClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        GeoEditorClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(GeoEditorClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        GeoEditorClass->setStatusBar(statusBar);

        retranslateUi(GeoEditorClass);

        QMetaObject::connectSlotsByName(GeoEditorClass);
    } // setupUi

    void retranslateUi(QMainWindow *GeoEditorClass)
    {
        GeoEditorClass->setWindowTitle(QApplication::translate("GeoEditorClass", "GeoEditor", 0));
    } // retranslateUi

};

namespace Ui {
    class GeoEditorClass: public Ui_GeoEditorClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GEOEDITOR_H
