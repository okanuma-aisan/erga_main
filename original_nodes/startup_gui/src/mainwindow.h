#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void auto_run();
private:
    Ui::MainWindow *ui;

private slots:
    void pushUseStartup();
    void pushNoUseStartup();
};

#endif // MAINWINDOW_H
