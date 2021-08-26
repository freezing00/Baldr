# coding=utf-8
import matplotlib.pyplot as plt
import string

def checkWrong(num) :
	if ( num >= -100 and num <= 100 ) :
		return num
	else :
		return 0

vxOrigin , vyOrigin, vzOrigin = [] , [] ,[]
with open("velocityOrigin") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		vxOrigin.append(checkWrong(float(i)))
	tmp = lines[1].split(', ')
	for i in tmp:
		vyOrigin.append(checkWrong(float(i)))
	tmp = lines[2].split(', ')
	for i in tmp:
		vzOrigin.append(checkWrong(float(i)))
print(vxOrigin , vyOrigin, vzOrigin)

vxFilter , vyFilter , vzFilter = [] , [] , []

with open("velocityFilter") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		vxFilter.append(checkWrong(float(i)))
	tmp = lines[1].split(', ')
	for i in tmp:
		vyFilter.append(checkWrong(float(i)))

	tmp = lines[2].split(', ')
	for i in tmp:
		vzFilter.append(checkWrong(float(i)))
print(vxFilter , vyFilter , vzFilter)

t = []
with open("coordinateTime") as f:
	lines = f.readlines()
	tmp = lines[0].split(', ')
	for i in tmp:
		t.append(float(i))
print(t)

#plt.subplot(131)
plt.figure()
plt.title('xVelocity')
plt.xlabel('t')
plt.ylabel('x')
plt.plot(t, vxOrigin, label='vxOrigin')
plt.plot(t, vxFilter, label='vxFilter')
plt.legend()

#plt.subplot(132)
plt.figure()
plt.title('yVelocity')
plt.xlabel('t')
plt.ylabel('y')
plt.plot(t, vyOrigin, label='vyOrigin')
plt.plot(t, vyFilter, label='vyFilter')
plt.legend()

#plt.subplot(133)
plt.figure()
plt.title('zVelocity')
plt.xlabel('t')
plt.ylabel('z')
plt.plot(t, vzOrigin, label='vzOrigin')
plt.plot(t, vzFilter, label='vzFilter')
plt.legend()

plt.show()
