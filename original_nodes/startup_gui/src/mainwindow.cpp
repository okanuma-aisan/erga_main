#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->bt_use_startup, SIGNAL(clicked()), this, SLOT(pushUseStartup()));
    connect(ui->bt_nouse_startup, SIGNAL(clicked()), this, SLOT(pushNoUseStartup()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::pushUseStartup()
{
    system("gnome-terminal -- bash -c 'bash /home/sit/autoware_erga/src/all_boot.bash; bash' &");
    close();
}

void MainWindow::pushNoUseStartup()
{
    close();
}

void MainWindow::auto_run()
{
    system("gnome-terminal -- bash -c 'bash /home/sit/autoware_erga/src/all_boot.bash; bash' &");
    close();
}
