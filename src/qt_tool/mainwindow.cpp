#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qpainter.h>
#include <stdio.h>
#include <QKeyEvent>
#include <iostream>
#include "../simple_planner/graph.h"
std::vector<int> calculate_path(const int *, const int *);
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::init()
{
    std::cout << "Generating global path" << std::endl;
    result = calculate_path(mat1, mat2);
}
void MainWindow::keyPressEvent(QKeyEvent *ev)
{
    switch (ev->key())
    {
    case Qt::Key_R:
        result.clear();
        repaint(rect());
        init();
        update();
        break;
    case Qt::Key_Escape:
        exit(0);
        break;
    default:
        break;
    }
}
void MainWindow::paintEvent(QPaintEvent *)
{
    char buf[32];
    QPainter p(this);
    
    p.setPen(Qt::red);
    for (int i = 0; i < 36; ++i)
    {
        float x, y;
        lookup(mat1, mat2, i, x, y);
        QPoint pose(x * 30 + 100, y * 30 + 100);
        sprintf(buf, "%d", i);
        p.drawText(pose, buf);
    }
    p.setPen(Qt::blue);
    for (int i = 0; i < 36; ++i)
    {
        float x, y;
        lookup(mat1, mat2, pair(i), x, y);
        QPoint pose(x * 30 + 100, y * 30 + 100);
        sprintf(buf, "%d", i);
        p.drawText(pose, buf);
    }

    std::vector<QPoint> line;
    for (int i = 0; i < (int)result.size(); ++i)
    {
        {
            float x, y;
            lookup(mat1, mat2, result[i], x, y);
            QPoint pose(x * 30 + 100, y * 30 + 100);
            line.push_back(pose);
        }
        {
            float x, y;
            lookup(mat1, mat2, pair(result[i]), x, y);
            QPoint pose(x * 30 + 100, y * 30 + 100);
            line.push_back(pose);
        }
    }
    
    p.setPen(Qt::black);
    for (int j = 0; j < (int)line.size() - 1; j++)
        p.drawLine(line.at(j), line.at(j + 1));
}