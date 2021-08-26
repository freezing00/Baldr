# coding=utf-8
import matplotlib.pyplot as plt
import string

def checkWrong(num) :
	if ( num >= -20 and num <= 20 ) :
		return num
	else :
		return 0

axOrigin , ayOrigin, azOrigin = [] , [] ,[]
with open("accOrigin") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		axOrigin.append(checkWrong(float(i)))
	tmp = lines[1].split(', ')
	for i in tmp:
		ayOrigin.append(checkWrong(float(i)))
	tmp = lines[2].split(', ')
	for i in tmp:
		azOrigin.append(checkWrong(float(i)))
print(axOrigin , ayOrigin, azOrigin)

t = []
with open("coordinateTime") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		t.append(float(i))
print(t)

#plt.subplot(131)
plt.figure()
plt.title('xAcc')
plt.xlabel('t')
plt.ylabel('x')
plt.plot(t, axOrigin, label='axOrigin')
plt.legend()

#plt.subplot(132)
plt.figure()
plt.title('yAcc')
plt.xlabel('t')
plt.ylabel('y')
plt.plot(t, ayOrigin, label='ayOrigin')
plt.legend()

#plt.subplot(133)
plt.figure()
plt.title('zAcc')
plt.xlabel('t')
plt.ylabel('z')
plt.plot(t, azOrigin, label='azOrigin')
plt.legend()

plt.show()
