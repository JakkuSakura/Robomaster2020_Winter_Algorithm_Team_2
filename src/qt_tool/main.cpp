#include "mainwindow.h"

#include <QApplication>
#include <iostream>
#include "testdata.h"
std::vector<int> calculate_path(const int *, const int *);
int main(int argc, char *argv[])
{
    std::cout << "Generating global path" << std::endl;
    auto result = calculate_path(get_mat1(), get_mat2());
    QApplication a(argc, argv);
    MainWindow w;
    w.mat1 = get_mat1();
    w.mat2 = get_mat2();
    w.result = result;
    w.show();
    return a.exec();
}
