#include "mainwindow.h"

#include <QApplication>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    #include "testdata.h"
    w.mat1 = mat1;
    w.mat2 = mat2;
    w.init();
    w.show();
    return a.exec();
}
