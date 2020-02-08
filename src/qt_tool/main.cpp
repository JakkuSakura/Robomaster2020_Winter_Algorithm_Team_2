#include "mainwindow.h"

#include <QApplication>
#include <iostream>
std::vector<int> calculate_path(const int *, const int *);
int main(int argc, char *argv[])
{
    #include "testdata.h"
    std::cout << "Generating global path" << std::endl;
    auto result = calculate_path(mat1, mat2);

    QApplication a(argc, argv);
    MainWindow w;
    w.mat1 = mat1;
    w.mat2 = mat2;
    w.result = result;
    w.show();
    return a.exec();
}
