import numpy as np

count = 1
num = 1
a = np.array([[1,2,3],[4,5,6],[7,8,9]])
b = np.array([[9,8,7],[6,5,4],[3,2,1]])
while num <= 9:
	index_a = np.argwhere(a == count)
	print(index_a)
	count = count + 1
	num = num + 1
	print("done")
