from matplotlib import font_manager as fm, rcParams
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

plt.rcParams['font.sans-serif'] = ['FangSong']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
fm._rebuild()

def draw(xList, yList):
    fig, ax = plt.subplots()
    plt.plot(xList, yList, '-')
    plt.grid()
    plt.show()