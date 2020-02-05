#include "mainwindow.h"

#include <QApplication>

int mat1[] =
    {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35};
int mat2[] =
    {
        24, 20, 31, 7, 13, 16,
        27, 35, 1, 0, 5, 8,
        6, 12, 2, 14, 15, 11,
        22, 21, 18, 23, 4, 19,
        26, 25, 9, 30, 17, 34,
        33, 10, 3, 32, 28, 29};

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow w;
    w.mat1 = mat1;
    w.mat2 = mat2;
    w.init();
    w.show();
    return a.exec();
}
