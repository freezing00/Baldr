# coding=utf-8
import matplotlib.pyplot as plt
import string

def checkWrong(num) :
	if ( num >= -90 and num <= 90 ) :
		return num
	else :
		return 0

yawNoBias, pitchNoBias, yawRef , pitchRef, yawFbd, pitchFbd, angleTime = [], [] , [] , [], [], [], []
with open("angleData") as f:
	lines = f.readlines()

	tmp = lines[0].split(', ')
	for i in tmp:
		yawNoBias.append(float(i))

	tmp = lines[1].split(', ')
	for i in tmp:
		pitchNoBias.append(checkWrong(float(i)))

	tmp = lines[2].split(', ')
	for i in tmp:
		yawRef.append(float(i))

	tmp = lines[3].split(', ')
	for i in tmp:
		pitchRef.append(checkWrong(float(i)))

	tmp = lines[4].split(', ')
	for i in tmp:
		yawFbd.append(float(i))

	tmp = lines[5].split(', ')
	for i in tmp:
		pitchFbd.append(checkWrong(float(i)))

	tmp = lines[6].split(', ')
	for i in tmp:
		angleTime.append(float(i))

print(yawNoBias, pitchNoBias, yawRef , pitchRef, yawFbd, pitchFbd, angleTime)

plt.subplot(121)
plt.title('yaw')
plt.xlabel('t')
plt.ylabel('yaw')
plt.plot(angleTime, yawNoBias, label='yawNoBias')
plt.plot(angleTime, yawRef, label='yawRef')
plt.plot(angleTime, yawFbd, label='yawFbd')
plt.legend()

plt.subplot(122)
plt.title('pitch')
plt.xlabel('t')
plt.ylabel('pitch')
plt.plot(angleTime, pitchNoBias, label='pitchNoBias')
plt.plot(angleTime, pitchRef, label='pitchRef')
plt.plot(angleTime, pitchFbd, label='pitchFbd')
plt.legend()

plt.show()
