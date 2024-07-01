/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *bt_use_startup;
    QPushButton *bt_nouse_startup;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1118, 869);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        bt_use_startup = new QPushButton(centralWidget);
        bt_use_startup->setObjectName(QStringLiteral("bt_use_startup"));
        bt_use_startup->setGeometry(QRect(60, 40, 491, 361));
        QFont font;
        font.setPointSize(50);
        bt_use_startup->setFont(font);
        bt_nouse_startup = new QPushButton(centralWidget);
        bt_nouse_startup->setObjectName(QStringLiteral("bt_nouse_startup"));
        bt_nouse_startup->setGeometry(QRect(570, 40, 491, 361));
        bt_nouse_startup->setFont(font);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1118, 28));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        bt_use_startup->setText(QApplication::translate("MainWindow", "\343\202\271\343\202\277\343\203\274\343\203\210\343\202\242\343\203\203\343\203\227\n"
"\345\256\237\350\241\214\343\201\231\343\202\213", Q_NULLPTR));
        bt_nouse_startup->setText(QApplication::translate("MainWindow", "\343\202\271\343\202\277\343\203\274\343\203\210\343\202\242\343\203\203\343\203\227\n"
"\345\256\237\350\241\214\343\201\227\343\201\252\343\201\204", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
