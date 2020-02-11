#!/bin/python2

from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np

class Dataset(object):
    def __init__(self, title, x, y):
        self.title = title
        self.x = x
        self.y = y

    def range(self):
        return (self.x[0], self.x[-1])
        
    

class MyWidget(pg.GraphicsWindow):

    def __init__(self, title, datasets, mutex, parent=None):
        super(MyWidget, self).__init__(parent=parent)
        self.setWindowTitle(title)
        self.mutex = mutex
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.onNewData)
        self.timer.start(50)

        self.setBackground('w')
        
        self.datasets = datasets
        for dataset in self.datasets:
            plotItem = self.addPlot(title=dataset.title)
            plotDataItem = plotItem.plot(pen=pg.mkPen('k', width=1), 
            symbolBrush=(0,0,0), symbolSize=2, symbolPen=None)
            self.nextRow()

            dataset.plot = plotItem
            dataset.curve = plotDataItem


    def onNewData(self):
        with self.mutex:
            for data in self.datasets:
                data.curve.setData(data.x, data.y)
                if len(data.x) >= 2:
                    data.plot.setRange(xRange=data.range(), padding=0)



def main(title, datasets, mutex):
    app = QtWidgets.QApplication([])

    win = MyWidget(title, datasets, mutex)
    win.resize(1000, 600)
    win.show()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

if __name__ == "__main__":
    main("Graph", [])