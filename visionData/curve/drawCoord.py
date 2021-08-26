# coding=utf-8
import matplotlib.pyplot as plt
import string

def checkWrong(num) :
	if ( num >= -20000 and num <= 20000 ) :
		return num
	else :
		return 0

xOrigin , yOrigin , zOrigin = [] , [] , []
with open("coordinateOrigin") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		xOrigin.append(checkWrong(float(i)))
	tmp = lines[1].split(', ')
	for i in tmp:
		yOrigin.append(checkWrong(float(i)))
	tmp = lines[2].split(', ')
	for i in tmp:
		zOrigin.append(checkWrong(float(i)))
print(xOrigin , yOrigin , zOrigin )

xPredict, yPredict, zPredict = [], [], []
with open("coordinatePredict") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		xPredict.append(checkWrong(float(i)))
	tmp = lines[1].split(', ')
	for i in tmp:
		yPredict.append(checkWrong(float(i)))
	tmp = lines[2].split(', ')
	for i in tmp:
		zPredict.append(checkWrong(float(i)))
print(xPredict, yPredict, zPredict)

xFilter, yFilter, zFilter = [], [], []
with open("coordinateFilter") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		xFilter.append(checkWrong(float(i)))
	tmp = lines[1].split(', ')
	for i in tmp:
		yFilter.append(checkWrong(float(i)))
	tmp = lines[2].split(', ')
	for i in tmp:
		zFilter.append(checkWrong(float(i)))
print(xFilter, yFilter, zFilter)

t = []
with open("coordinateTime") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		t.append(float(i))
print(t)

#plt.subplot(131)
plt.figure()
plt.title('xCoordinate')
plt.xlabel('t')
plt.ylabel('x')
plt.plot(t, xOrigin, label='xOrigin')
plt.plot(t, xPredict, label='xPredict')
plt.plot(t, xFilter, label='xFilter')
plt.legend()

#plt.subplot(132)
plt.figure()
plt.title('yCoordinate')
plt.xlabel('t')
plt.ylabel('y')
plt.plot(t, yOrigin, label='yOrigin')
plt.plot(t, yPredict, label='yPredict')
plt.plot(t, yFilter, label='yFilter')
plt.legend()

#plt.subplot(133)
plt.figure()
plt.title('zCoordinate')
plt.xlabel('t')
plt.ylabel('z')
plt.plot(t, zOrigin, label='zOrigin')
plt.plot(t, zPredict, label='zPredict')
plt.plot(t, zFilter, label='zFilter')
plt.legend()

plt.show()
