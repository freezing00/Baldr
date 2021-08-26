# coding=utf-8
import matplotlib.pyplot as plt
import numpy as np

yR = []
with open("angleData") as f:
	lines = f.readlines()
	tmp = lines[2].split(', ')
	for i in tmp:
		yR.append(float(i))

print("yawRef wrong: ")
cnt = 0
for i in yR:
	if i > 180 or i < -180:
		print(i)
		cnt = cnt + 1

print("yawRef wrong cnt: ")
print(cnt)

pR = []
with open("angleData") as f:
	lines = f.readlines()
	tmp = lines[3].split(', ')
	for i in tmp:
		pR.append(float(i))

print("pitchRef wrong: ")
cnt = 0
for i in pR:
	if i > 180 or i < -180:
		print(i)
		cnt = cnt + 1
print("ptichRef wrong cnt: ")
print(cnt)

yF = []
with open("angleData") as f:
	lines = f.readlines()
	tmp = lines[4].split(', ')
	for i in tmp:
		yF.append(float(i))
cnt = 0
print("yawFbd wrong: ")
for i in yF:
	if i > 180 or i < -180:
		print(i)
		cnt = cnt + 1
print("yawFbd wrong cnt: ")
print(cnt)

pF = []
with open("angleData") as f:
	lines = f.readlines()
	tmp = lines[5].split(', ')
	for i in tmp:
		pF.append(float(i))

print("pitchFbd wrong: ")
cnt = 0
for i in pF:
	if i > 180 or i < -180:
		print(i)
		cnt = cnt + 1
print("pitchFbd wrong cnt: ")
print(cnt)
