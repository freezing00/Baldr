# coding=utf-8
import matplotlib.pyplot as plt
import numpy as np

def checkWrong(num) :
	if ( num >= -20000 and num <= 20000 ) :
		return num
	else :
		return 0

t = []
with open("coordinateTime") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		t.append(float(i))
print(t)

L = len(t)
print(L)

y = []
with open("coordinatePredict") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		y.append(checkWrong(float(i)))
print(y)

#模型
dt = (t[1] - t[0]) / 1000.0
F2 = np.mat( [[1, dt], [0, 1]] )	#dt在后面不断更新
H2 = np.mat( [1, 0] )
Q2 = np.mat([[10, 0], [0, 50]])	#过程噪声
R2 = 1000                       	#观测噪声

#设初值
Xplus2 = np.mat(np.zeros((2, L)))
Xplus2[0,1] = y[1]
Xplus2[1,1] = 1
Pplus2 = 0.5

for i in range(2, L):
    dt = (t[i] - t[i-1]) / 1000.0
    F2 = np.mat([[1, dt], [0, 1]])
    #预测步
    Xminus2 = F2 * Xplus2[0:, i-1]
    Pminus2 = F2 * Pplus2 * np.transpose(F2) + Q2
    #更新步
    K2 = (Pminus2 * np.transpose(H2)) * np.linalg.inv(H2 * Pminus2 * np.transpose(H2) + R2)
    Xplus2[0:, i] = Xminus2 + K2 * (y[i] - H2 * Xminus2)
    Pplus2 = (np.eye(2) - K2 * H2) * Pminus2

yF = []
for i in range(0, L):
	yF.append(Xplus2[0, i])

plt.title('origin and filter')
plt.xlabel('t')
plt.ylabel('y')
plt.plot(t, y, label='origin')
plt.plot(t, yF, label='filter')
plt.legend()
plt.show()
