#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <vector>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void paintEvent(QPaintEvent *ev);
    void keyPressEvent(QKeyEvent *ev);
    void init();
    std::vector<int> result;
    const int *mat1, *mat2;

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
