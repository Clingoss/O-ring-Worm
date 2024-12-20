import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import statistics as stat

plt.rcParams.update({'font.size': 25})
plt.rcParams["figure.figsize"] = (13,8)
lw = 3

filename = ['balsa','bord','goodfoam','shitfoam']
betterNames = ['Balsa wood','Lab table','Foam PPI30','Foam PPI15']
filenameTOP = ['BalsaTOP','tableTOP','GoodFoamTOP','shitFoamTOP']
num = ['1','2','3']

lineStyles = ['-','--',':']
colors = ['r','b','g','y']

goodkPa = np.array([0,24,30,34,36,41,44])
goodLength = np.array([12.5,13.5,14.,14.5,15.,16.,16.5])

badkPa = np.array([0,30,35,38,39,40,42])
badLength = np.array([12.5,13.5,14.,14.5,14.5,15.,15.3])

actRotRight = np.array([1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1])
deactRotRight = np.array([0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0])
actRotLeft = np.array([0,0,1,0,1,0,1,1,1,1,0,1,1,1,0,0])
deactRotLeft = np.array([0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,1])
rotation = np.array([actRotLeft,actRotRight,deactRotLeft,deactRotRight])

maxVel = np.zeros((4,3))

for i,name in enumerate(filename):
    for j,n in enumerate(num):

        var = pd.read_csv('OringWorm/'+ name + n + '.txt')

        t=np.array(list(var['t']))
        x=np.array(list(var['x']))
        vx=np.array(list(var['vx']))

        maxVel[i,j] = np.average(vx)
        if j>0:
            plt.plot(t,x, color=colors[i], linestyle=lineStyles[j], linewidth=lw, label='_nolegend_')
        else:
            plt.plot(t,x, color=colors[i], linestyle=lineStyles[j], linewidth=lw)

plt.ylabel('Distance [mm]')
plt.xlabel('Time [s]')
plt.legend(betterNames)
plt.grid(True)
ax = plt.gca()

plt.savefig('figures/x_t.png')
plt.show()

avgVel = [(np.sum(maxVel[i]))/3. for i in range(len(filename))]
errorVel = [np.max(maxVel[i])-np.min(maxVel[i]) for i in range(len(filename))]

plt.bar(betterNames, avgVel, color=colors, yerr=errorVel, ecolor="k", capsize=10, zorder=3)
plt.ylabel('Velocity [mm/s]')
plt.grid(True)
ax = plt.gca()

plt.savefig('figures/avg_vx_error.png')
plt.show()

yMax = []
yError = []

for i,name in enumerate(filenameTOP):
    
    var = pd.read_csv('OringWorm/'+ name + '.txt')
    y=np.array(list(var['y']))

    yMax.append(abs(np.min(y)))
    yError.append(stat.stdev(y))

plt.bar(betterNames, yMax, color=colors, yerr=yError, ecolor="k", capsize=10, zorder=3)
plt.ylabel('Displacement [mm]')
plt.grid(True)
ax = plt.gca()

plt.savefig('figures/avg_y_error.png')
plt.show()

plt.plot(goodkPa,goodLength*10.,color='r',marker='*', linewidth=lw)
plt.plot(badkPa,badLength*10.,color='b',marker='*', linewidth=lw)
plt.ylabel('Distance [mm]')
plt.xlabel('Pressure [kPa]')
plt.legend(['Final actuator','Old actuator'])
plt.grid(True)
ax = plt.gca()

plt.savefig('figures/pressureLengthTest.png')
plt.show()

rotNames = ['Left','Right']
pcnt = [(np.sum(rotation[i]))/len(actRotLeft) for i in range(len(rotation))]
pcnt = np.array([[pcnt[0],pcnt[1]],[pcnt[2],pcnt[3]]])*100.

plt.bar(np.arange(len(rotNames))-0.2, pcnt[0], width=0.4, color='g', align='center', label = 'Activate')
plt.bar(np.arange(len(rotNames))+0.2, pcnt[1], width=0.4, color='r', align='center', label = 'Deactivate')
plt.xticks(np.arange(len(rotNames)), rotNames) 
plt.ylabel('Success Rate [%]')
plt.legend() 
plt.grid(True)
ax = plt.gca()

plt.savefig('figures/rotationTest.png')
plt.show()