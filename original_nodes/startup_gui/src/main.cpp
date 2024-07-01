#include "mainwindow.h"
#include <QApplication>
#include <time.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    time_t first_time;
    time(&first_time);

    //return a.exec();
    while (w.isVisible()){
      a.processEvents();
      time_t sec_time;
      time(&sec_time);
      time_t time_diff = sec_time - first_time;
      if(time_diff > 5) w.auto_run();
    }
}
