#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qpainter.h>
#include <stdio.h>
#include <QKeyEvent>
#include <iostream>
#include <QThread> 
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
    step = 0;
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
    case Qt::Key_P:
        step = 0;
        break;
    case Qt::Key_Escape:
        exit(0);
        break;
    default:
        break;
    }
}
QColor HSVtoRGB(int H, double S, double V) {
	double C = S * V;
	double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	double m = V - C;
	double Rs, Gs, Bs;

	if(H >= 0 && H < 60) {
		Rs = C;
		Gs = X;
		Bs = 0;	
	}
	else if(H >= 60 && H < 120) {	
		Rs = X;
		Gs = C;
		Bs = 0;	
	}
	else if(H >= 120 && H < 180) {
		Rs = 0;
		Gs = C;
		Bs = X;	
	}
	else if(H >= 180 && H < 240) {
		Rs = 0;
		Gs = X;
		Bs = C;	
	}
	else if(H >= 240 && H < 300) {
		Rs = X;
		Gs = 0;
		Bs = C;	
	}
	else {
		Rs = C;
		Gs = 0;
		Bs = X;	
	}
	
	return QColor((Rs + m) * 255, (Gs + m) * 255, (Bs + m) * 255);
}

void MainWindow::paintEvent(QPaintEvent *)
{
    static char buf[32];
    QPainter p(this);

    std::vector<QPoint> line;
    for (int i = 0; i < (int)result.size(); ++i)
    {
        {
            float x, y;
            lookup(mat1, mat2, result[i], x, y);
            QPoint pose(x * 30 + 105, y * 30 + 95);
            line.push_back(pose);
        }
        {
            float x, y;
            lookup(mat1, mat2, pair(result[i]), x, y);
            QPoint pose(x * 30 + 105, y * 30 + 95);
            line.push_back(pose);
        }
    }
    
    for (int i = 0; i < std::min(step, (int)line.size() - 1); i++)
    {
        p.setPen(QPen(HSVtoRGB((i * 7 + step * 13) % 255, 1.0, 0.5), 3));
        p.drawLine(line.at(i), line.at(i + 1));
    }
    ++step;
    
    
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

    update();
    QThread::msleep(100);

}