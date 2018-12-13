from __future__ import division
import numpy as np
import math

import pylab as plt

# lets say that var_i is for discrete,
# var_c is for continuous
resolution = 0.025
numAngles = 16
pi = math.pi
#angle = [0, math.atan(.5), math.atan(1), math.atan(2), \
#         pi/2, math.atan(.5)+pi/2, math.atan(1)+pi/2, math.atan(2)+pi/2, \
#         pi, math.atan(.5)+pi, math.atan(1)+pi, math.atan(2)+pi, \
#         pi*3/2, math.atan(.5)+pi*3/2, math.atan(1)+pi*3/2, math.atan(2)+pi*3/2]
         
velincr = 0.1
vel0 = 0.0 # m/s
vel1 = 0.1
vel2 = 0.2
vel3 = 0.3
timeToTurn22p5degsInPlace = 1.0

numVelocitiesPerAngle = 4

filename = "unicycle_2p5cm.mprim"
primFile = open(filename, 'w+')
primFile.write("resolution_m: %.4f\n" % resolution)
primFile.write("numberofangles: %d\n" % numAngles)
primFile.write("numberofvelocities: 4\n")
primFile.write("timetoturn22.5deginplace: %.4f\n" % timeToTurn22p5degsInPlace)
primFile.write("totalnumberofprimitives: 576\n")
primFile.close()

def generateIntermediatePoses(angleIndex, currentangle, startvel_i, angle, endpose):

    endx = round(endpose[0]*math.cos(angle)-endpose[1]*math.sin(angle))
    endy = round(endpose[0]*math.sin(angle)+endpose[1]*math.cos(angle))
    endth = (angleIndex + endpose[2]) % numAngles

    primExecTime = 0.0
    endpose_i = [endx, endy, endth, endpose[3]]
    endpose_c = [endx*resolution, endy*resolution, \
                 endth*2*pi/numAngles, primExecTime]

    numSamples = 10
    intmposes_c = np.zeros((numSamples,4))

    startvel_c = startvel_i*velincr
    endvel_c = endpose[3]*velincr


#    print("[0 0 %.3f] --> [%.3f %.3f %.3f]" % \
#            (currentangle_c, endpose_c[0], endpose_c[1], endpose_c[2]))


    # calculate primitive execution time (assuming straight line
    dist_c = math.sqrt(endpose_c[0]**2 + endpose_c[1]**2)
    if (endpose[0]==0 and endpose[1]==0): # turn in place
        primExecTime = timeToTurn22p5degsInPlace 
    elif startvel_c==0 and endpose[3]==0: # vel0 -> vel0 transition
        t = 2*dist_c/(vel1/2)
        primExecTime = 2*t
    else:
        primExecTime = 2*dist_c/(startvel_c + endpose[3]*velincr)


    if endpose[2] == 0: # move forward
        #print("move forward")
        if startvel_i==0 and endpose[3]==0: # vel0 -> vel0 transition
            #print("vel0 -> vel0\n")
            for i in range(0,numSamples):
                dt = i/(numSamples-1)
                intmposes_c[i,:] = [endpose_c[0]*dt, endpose_c[1]*dt, \
                                      endpose_c[2], primExecTime*dt] 

        else: # uniform acceleration
            #print("uniform accel\n")
            a = (endvel_c**2 - startvel_c**2)/(2*dist_c)
            dist_c_i = dist_c/(numSamples-1)
            intmposes_c[0,:] = [0, 0, currentangle, 0]
            for i in range(1,numSamples):
                dt = i/(numSamples-1)
                t_i = (2*i*dist_c_i)/ \
                      (math.sqrt(startvel_c**2 + 2*a*i*dist_c_i) + startvel_c)
                      
                intmposes_c[i,:] = [endpose_c[0]*dt, \
                                    endpose_c[1]*dt, \
                                    endpose_c[2], t_i] 

    elif endpose[0]==0 and endpose[1]==0: # turn in place
        #print("turn in place\n")
        for i in range(0,numSamples):
            dt = i/(numSamples-1)
            rotationangle = endpose[2]*2*pi/numAngles
            intmposes_c[i,:] = [0, 0, currentangle+rotationangle*dt, \
                                primExecTime*dt]

    else: # forward and turn
        #print("forward and turn\n")
        R = np.matrix([ \
            [math.cos(currentangle), math.sin(endpose_c[2])-math.sin(currentangle)], \
            [math.sin(currentangle), -(math.cos(endpose_c[2])-math.cos(currentangle))]])
        S = np.matmul(R.getI(), [[endpose_c[0]],[endpose_c[1]]])
        l = S[0]
        tvoverrv = S[1]
        rv = endpose[2]*2*pi/numAngles + l/tvoverrv
        tv = tvoverrv*rv

        if l < 0:
            #print("l<0: bad start/end points")
            l = 0

        # first pass for x, y, theta
        #a = (endvel_c**2 - startvel_c**2)/(2*dist_c)
        #dist_c_i = dist_c/(numSamples-1)
        dist_c = 0
        intmposes_c[0,:] = [0, 0, currentangle, 0]
        for i in range(1,numSamples):
            dt = i/(numSamples-1)
            #t_i = (2*i*dist_c_i)/ \
            #      (math.sqrt(startvel_c**2 + 2*a*i*dist_c_i) + startvel_c)

            if dt*tv < l:
                intmposes_c[i,:] = [dt*tv*math.cos(currentangle), \
                                    dt*tv*math.sin(currentangle), \
                                    currentangle, 0]
            else:
                dtheta = rv*(dt - l/tv) + currentangle
                intmposes_c[i,:] = [l*math.cos(currentangle)+tvoverrv*(math.sin(dtheta)-math.sin(currentangle)), \
                                    l*math.sin(currentangle)-tvoverrv*(math.cos(dtheta)-math.cos(currentangle)), \
                                    dtheta, 0]
            dist_c += math.sqrt((intmposes_c[i,0]-intmposes_c[i-1,0])**2 + \
                                (intmposes_c[i,1]-intmposes_c[i-1,1])**2)
        
        # correct error
        if angleIndex==0 and endpose[2]==-1:
            #endpose_c[2] = -angle[1]
            endpose_c[2] = -2*math.pi/numAngles
        elif angleIndex==15 and endpose[2]==1:
            #startangle_c = -angle[1]
            endpose_c[2] = 2*math.pi
        errorxyth = [endpose_c[0] - intmposes_c[numSamples-1,0], \
                     endpose_c[1] - intmposes_c[numSamples-1,1], \
                     endpose_c[2] - intmposes_c[numSamples-1,2]]
        interp = np.zeros(numSamples)
        for i in range(0,numSamples):
            interp[i] = i/(numSamples-1)
        intmposes_c[:,0] = intmposes_c[:,0] + errorxyth[0]*interp
        intmposes_c[:,1] = intmposes_c[:,1] + errorxyth[1]*interp
        intmposes_c[:,2] = intmposes_c[:,2] + errorxyth[2]*interp

        # second pass for time
        primExecTime = 2*dist_c/(startvel_c + endpose[3]*velincr)
        a = (endvel_c**2 - startvel_c**2)/(2*dist_c)
        #print(dist_c)
        dist_c_i = dist_c/(numSamples-1)
        for i in range(1,numSamples):
            #print("vi^2 2ad = %.4f %.4f" % (startvel_c**2, 2*a*i*dist_c_i))
            t_i = 0
            if startvel_c**2 + 2*a*i*dist_c_i < 0.00001:
                t_i = (2*i*dist_c_i)/startvel_c
            else:
                t_i = (2*i*dist_c_i)/ \
                    (math.sqrt(startvel_c**2 + 2*a*i*dist_c_i) + startvel_c)
            intmposes_c[i,3] = t_i
    for i in range(0,numSamples):
        if intmposes_c[i,2] < 0:
            intmposes_c[i,2] = intmposes_c[i,2] + 2*math.pi
        if intmposes_c[i,2] >= 2*math.pi:
            intmposes_c[i,2] = intmposes_c[i,2] - 2*math.pi

        
    #write to file
    primFile = open(filename, 'a')
    primFile.write("startangle_i: %d\n" % (angleIndex))
    primFile.write("startvel_i: %d\n" % (startvel_i))
    primFile.write("endpose_i: %d %d %d %d\n" % (endpose_i[0],endpose_i[1],endpose_i[2],endpose_i[3]))
    primFile.write("exectime: %.4f\n" % (primExecTime))
    primFile.write("intermediateposes: %d\n" % (numSamples))
    for i in range(0,numSamples):
        primFile.write("%.4f %.4f %.4f %.4f\n" % (intmposes_c[i,0], intmposes_c[i,1], \
                                                  intmposes_c[i,2], intmposes_c[i,3]))
    primFile.close()

    for i in range(1,numSamples):
        plt.plot([intmposes_c[i-1,0],intmposes_c[i,0]],
                 [intmposes_c[i-1,1],intmposes_c[i,1]],
                 color="blue")


#S = np.matmul(np.linalg.pinv(R), np.array([[endpose_c[0]],[endpose_c[1]]]))

# start always assumed to be cell (0,0)
# end pose is (x, y, angle, velocity)
# have 16 angles (0, 26.5, 45, 63.5, ...) corresponding to 0, 1, 2, ...
# and 4 velocities (0, ,0.2, 0.4, 0.6)
# for 2.5cm resolution, max accel will be 0.8 -- within reason,
numPrimsPerVel0 = 7
numPrimsPerVel1 = 12
numPrimsPerVel2 = 10
numPrimsPerVel3 = 7

endpose_angle0_vel0 = np.zeros((numPrimsPerVel0, 4))
endpose_angle0_vel1 = np.zeros((numPrimsPerVel1, 4))
endpose_angle0_vel2 = np.zeros((numPrimsPerVel2, 4))
endpose_angle0_vel3 = np.zeros((numPrimsPerVel3, 4))

endpose_angle22p5_vel0 = np.zeros((numPrimsPerVel0, 4))
endpose_angle22p5_vel1 = np.zeros((numPrimsPerVel1, 4))
endpose_angle22p5_vel2 = np.zeros((numPrimsPerVel2, 4))
endpose_angle22p5_vel3 = np.zeros((numPrimsPerVel3, 4))

endpose_angle45_vel0 = np.zeros((numPrimsPerVel0, 4))
endpose_angle45_vel1 = np.zeros((numPrimsPerVel1, 4))
endpose_angle45_vel2 = np.zeros((numPrimsPerVel2, 4))
endpose_angle45_vel3 = np.zeros((numPrimsPerVel3, 4))


# for 0 degrees (0)
##############################################################################
endpose_angle0_vel0[0,:] = [1, 0, 0, 0]     # forward -> vel0
endpose_angle0_vel0[1,:] = [1, 0, 0, 1]     # forward -> vel1
endpose_angle0_vel0[2,:] = [0, 0, 1, 0]     # rotate ccw in place
endpose_angle0_vel0[3,:] = [0, 0, -1, 0]    # rotate cw in place
endpose_angle0_vel0[4,:] = [4, 0, 0, 1]     # forward long -> vel1
endpose_angle0_vel0[5,:] = [4, 1, 1, 1]     # forward turn ccw long ->vel1
endpose_angle0_vel0[6,:] = [4, -1, -1, 1]   # forward turn cw long -> vel1

endpose_angle0_vel1[0,:] = [1, 0, 0, 0]     # forward -> vel0
endpose_angle0_vel1[1,:] = [1, 0, 0, 1]     # forward -> vel1
endpose_angle0_vel1[2,:] = [4, 1, 1, 0]     # forward turn ccw -> vel0
endpose_angle0_vel1[3,:] = [4, 1, 1, 1]     # forward turn ccw -> vel1
endpose_angle0_vel1[4,:] = [4, -1, -1, 0]   # forward turn cw -> vel0 
endpose_angle0_vel1[5,:] = [4, -1, -1, 1]   # forward turn cw -> vel1 
endpose_angle0_vel1[6,:] = [8, 0, 0, 1]     # forward long -> vel1
endpose_angle0_vel1[7,:] = [8, 0, 0, 2]    # forward long -> vel2
endpose_angle0_vel1[8,:] = [8, 1, 1, 1]    # forward turn ccw long ->vel1
endpose_angle0_vel1[9,:] = [8, 1, 1, 2]    # forward turn ccw long ->vel2
endpose_angle0_vel1[10,:] = [8, -1, -1, 1]  # forward turn cw long -> vel1
endpose_angle0_vel1[11,:] = [8, -1, -1, 2]  # forward turn cw long -> vel2

endpose_angle0_vel2[0,:] = [1, 0, 0, 2]     # forward -> vel2
endpose_angle0_vel2[1,:] = [8, 0, 0, 1]     # forward long -> vel1
endpose_angle0_vel2[2,:] = [8, 0, 0, 2]     # forward long -> vel2
endpose_angle0_vel2[3,:] = [8, 0, 0, 3]    # forward long -> vel3
endpose_angle0_vel2[4,:] = [8, 1, 1, 1]    # forward turn ccw long ->vel1
endpose_angle0_vel2[5,:] = [8, 1, 1, 2]    # forward turn ccw long ->vel2
endpose_angle0_vel2[6,:] = [8, 1, 1, 3]    # forward turn ccw long ->vel3
endpose_angle0_vel2[7,:] = [8, -1, -1, 1]  # forward turn cw long -> vel1
endpose_angle0_vel2[8,:] = [8, -1, -1, 2]  # forward turn cw long -> vel2
endpose_angle0_vel2[9,:] = [8, -1, -1, 3]  # forward turn cw long -> vel3

endpose_angle0_vel3[0,:] = [1, 0, 0, 3]     # forward -> vel3
endpose_angle0_vel3[1,:] = [8, 0, 0, 2]     # forward long -> vel2
endpose_angle0_vel3[2,:] = [8, 0, 0, 3]     # forward long -> vel3
endpose_angle0_vel3[3,:] = [8, 1, 1, 2]     # forward turn ccw long ->vel2
endpose_angle0_vel3[4,:] = [8, 1, 1, 3]     # forward turn ccw long ->vel3
endpose_angle0_vel3[5,:] = [8, -1, -1, 2]   # forward turn cw long -> vel2
endpose_angle0_vel3[6,:] = [8, -1, -1, 3]   # forward turn cw long -> vel3

# for 22.5 degrees (1)
##############################################################################
endpose_angle22p5_vel0[0,:] = [2, 1, 0, 0]     # forward -> vel0
endpose_angle22p5_vel0[1,:] = [2, 1, 0, 1]     # forward -> vel1
endpose_angle22p5_vel0[2,:] = [0, 0, 1, 0]     # rotate ccw in place
endpose_angle22p5_vel0[3,:] = [0, 0, -1, 0]     # rotate cw in place
endpose_angle22p5_vel0[4,:] = [4, 2, 0, 1]     # forward long -> vel1
endpose_angle22p5_vel0[5,:] = [3, 2, 1, 1]     # forward turn ccw long ->vel1
endpose_angle22p5_vel0[6,:] = [4, 1, -1, 1]     # forward turn cw long -> vel1

endpose_angle22p5_vel1[0,:] = [2, 1, 0, 0]     # forward -> vel0
endpose_angle22p5_vel1[1,:] = [2, 1, 0, 1]     # forward -> vel1
endpose_angle22p5_vel1[2,:] = [3, 2, 1, 0]     # forward turn ccw -> vel0
endpose_angle22p5_vel1[3,:] = [3, 2, 1, 1]     # forward turn ccw -> vel1
endpose_angle22p5_vel1[4,:] = [4, 1, -1, 0]     # forward turn cw -> vel0 
endpose_angle22p5_vel1[5,:] = [4, 1, -1, 1]     # forward turn cw -> vel1 
endpose_angle22p5_vel1[6,:] = [8, 4, 0, 1]     # forward long -> vel1
endpose_angle22p5_vel1[7,:] = [8, 4, 0, 2]    # forward long -> vel2
endpose_angle22p5_vel1[8,:] = [7, 5, 1, 1]    # forward turn ccw long ->vel1
endpose_angle22p5_vel1[9,:] = [7, 5, 1, 2]    # forward turn ccw long ->vel2
endpose_angle22p5_vel1[10,:] = [8, 2, -1, 1]    # forward turn cw long -> vel1
endpose_angle22p5_vel1[11,:] = [8, 2, -1, 2]    # forward turn cw long -> vel2

endpose_angle22p5_vel2[0,:] = [2, 1, 0, 2]     # forward -> vel2
endpose_angle22p5_vel2[1,:] = [8, 4, 0, 1]     # forward long -> vel1
endpose_angle22p5_vel2[2,:] = [8, 4, 0, 2]     # forward long -> vel2
endpose_angle22p5_vel2[3,:] = [8, 4, 0, 3]    # forward long -> vel3
endpose_angle22p5_vel2[4,:] = [7, 5, 1, 1]    # forward turn ccw long ->vel1
endpose_angle22p5_vel2[5,:] = [7, 5, 1, 2]    # forward turn ccw long ->vel2
endpose_angle22p5_vel2[6,:] = [7, 5, 1, 3]    # forward turn ccw long ->vel3
endpose_angle22p5_vel2[7,:] = [8, 2, -1, 1]    # forward turn cw long -> vel1
endpose_angle22p5_vel2[8,:] = [8, 2, -1, 2]    # forward turn cw long -> vel2
endpose_angle22p5_vel2[9,:] = [8, 2, -1, 3]    # forward turn cw long -> vel3

endpose_angle22p5_vel3[0,:] = [2, 1, 0, 3]     # forward -> vel3
endpose_angle22p5_vel3[1,:] = [8, 4, 0, 2]     # forward long -> vel2
endpose_angle22p5_vel3[2,:] = [8, 4, 0, 3]     # forward long -> vel3
endpose_angle22p5_vel3[3,:] = [7, 5, 1, 2]     # forward turn ccw long ->vel2
endpose_angle22p5_vel3[4,:] = [7, 5, 1, 3]     # forward turn ccw long ->vel3
endpose_angle22p5_vel3[5,:] = [8, 2, -1, 2]     # forward turn cw long -> vel2
endpose_angle22p5_vel3[6,:] = [8, 2, -1, 3]     # forward turn cw long -> vel3

# for 45 degrees (2)
##############################################################################
endpose_angle45_vel0[0,:] = [1, 1, 0, 0]     # forward -> vel0
endpose_angle45_vel0[1,:] = [1, 1, 0, 1]     # forward -> vel1
endpose_angle45_vel0[2,:] = [0, 0, 1, 0]     # rotate ccw in place
endpose_angle45_vel0[3,:] = [0, 0, -1, 0]     # rotate cw in place
endpose_angle45_vel0[4,:] = [3, 3, 0, 1]     # forward long -> vel1
endpose_angle45_vel0[5,:] = [2, 4, 1, 1]     # forward turn ccw long ->vel1
endpose_angle45_vel0[6,:] = [4, 2, -1, 1]     # forward turn cw long -> vel1

endpose_angle45_vel1[0,:] = [1, 1, 0, 0]     # forward -> vel0
endpose_angle45_vel1[1,:] = [1, 1, 0, 1]     # forward -> vel1
endpose_angle45_vel1[2,:] = [2, 4, 1, 0]     # forward turn ccw -> vel0
endpose_angle45_vel1[3,:] = [2, 4, 1, 1]     # forward turn ccw -> vel1
endpose_angle45_vel1[4,:] = [4, 2, -1, 0]     # forward turn cw -> vel0 
endpose_angle45_vel1[5,:] = [4, 2, -1, 1]     # forward turn cw -> vel1 
endpose_angle45_vel1[6,:] = [6, 6, 0, 1]     # forward long -> vel1
endpose_angle45_vel1[7,:] = [6, 6, 0, 2]    # forward long -> vel2
endpose_angle45_vel1[8,:] = [5, 7, 1, 1]    # forward turn ccw long ->vel1
endpose_angle45_vel1[9,:] = [5, 7, 1, 2]    # forward turn ccw long ->vel2
endpose_angle45_vel1[10,:] = [7, 5, -1, 1]    # forward turn cw long -> vel1
endpose_angle45_vel1[11,:] = [7, 5, -1, 2]    # forward turn cw long -> vel2

endpose_angle45_vel2[0,:] = [1, 1, 0, 2]     # forward -> vel2
endpose_angle45_vel2[1,:] = [6, 6, 0, 1]     # forward long -> vel1
endpose_angle45_vel2[2,:] = [6, 6, 0, 2]     # forward long -> vel2
endpose_angle45_vel2[3,:] = [6, 6, 0, 3]    # forward long -> vel3
endpose_angle45_vel2[4,:] = [5, 7, 1, 1]    # forward turn ccw long ->vel1
endpose_angle45_vel2[5,:] = [5, 7, 1, 2]    # forward turn ccw long ->vel2
endpose_angle45_vel2[6,:] = [5, 7, 1, 3]    # forward turn ccw long ->vel3
endpose_angle45_vel2[7,:] = [7, 5, -1, 1]    # forward turn cw long -> vel1
endpose_angle45_vel2[8,:] = [7, 5, -1, 2]    # forward turn cw long -> vel2
endpose_angle45_vel2[9,:] = [7, 5, -1, 3]    # forward turn cw long -> vel3

endpose_angle45_vel3[0,:] = [1, 1, 0, 3]     # forward -> vel3
endpose_angle45_vel3[1,:] = [6, 6, 0, 2]     # forward long -> vel2
endpose_angle45_vel3[2,:] = [6, 6, 0, 3]     # forward long -> vel3
endpose_angle45_vel3[3,:] = [5, 7, 1, 2]     # forward turn ccw long ->vel2
endpose_angle45_vel3[4,:] = [5, 7, 1, 3]     # forward turn ccw long ->vel3
endpose_angle45_vel3[5,:] = [7, 5, -1, 2]     # forward turn cw long -> vel2
endpose_angle45_vel3[6,:] = [7, 5, -1, 3]     # forward turn cw long -> vel3

for angleIndex in range(0,numAngles):

    # vel 0
    for primIndex in range(0,numPrimsPerVel0):
        endpose = np.zeros(4)
        currentangle = angleIndex*2*pi/numAngles
        currentangle_36000int = round(angleIndex*36000/numAngles)

        if currentangle_36000int % 9000 == 0:
            #print("using angle0")
            endpose = endpose_angle0_vel0[primIndex,:]
            angle = currentangle
        elif currentangle_36000int % 4500 == 0:
            #print("using angle45")
            endpose = endpose_angle45_vel0[primIndex,:]
            angle = currentangle - 45*pi/180
        elif (currentangle_36000int-6750) % 9000 == 0:
            #print("using angle67p5")
            endpose[0] = endpose_angle22p5_vel0[primIndex,1]
            endpose[1] = endpose_angle22p5_vel0[primIndex,0]
            endpose[2] = -endpose_angle22p5_vel0[primIndex,2]
            endpose[3] = endpose_angle22p5_vel0[primIndex,3]
            angle = currentangle - 67.5*pi/180
        elif (currentangle_36000int-2250) % 9000 == 0:
            #print("using angle22p5")
            endpose = endpose_angle22p5_vel0[primIndex,:]
            angle = currentangle - 22.5*pi/180

        generateIntermediatePoses(angleIndex, currentangle, 0, angle, endpose)
        
    # vel 1
    for primIndex in range(0,numPrimsPerVel1):
        endpose = np.zeros(4)
        currentangle = angleIndex*2*pi/numAngles
        currentangle_36000int = round(angleIndex*36000/numAngles)

        if currentangle_36000int % 9000 == 0:
            #print("using angle0")
            endpose = endpose_angle0_vel1[primIndex,:]
            angle = currentangle
        elif currentangle_36000int % 4500 == 0:
            #print("using angle45")
            endpose = endpose_angle45_vel1[primIndex,:]
            angle = currentangle - 45*pi/180
        elif (currentangle_36000int-6750) % 9000 == 0:
            #print("using angle67p5")
            endpose[0] = endpose_angle22p5_vel1[primIndex,1]
            endpose[1] = endpose_angle22p5_vel1[primIndex,0]
            endpose[2] = -endpose_angle22p5_vel1[primIndex,2]
            endpose[3] = endpose_angle22p5_vel1[primIndex,3]
            angle = currentangle - 67.5*pi/180
        elif (currentangle_36000int-2250) % 9000 == 0:
            #print("using angle22p5")
            endpose = endpose_angle22p5_vel1[primIndex,:]
            angle = currentangle - 22.5*pi/180

        generateIntermediatePoses(angleIndex, currentangle, 1, angle, endpose)
        
    # vel 2
    for primIndex in range(0,numPrimsPerVel2):
        endpose = np.zeros(4)
        currentangle = angleIndex*2*pi/numAngles
        currentangle_36000int = round(angleIndex*36000/numAngles)

        if currentangle_36000int % 9000 == 0:
            #print("using angle0")
            endpose = endpose_angle0_vel2[primIndex,:]
            angle = currentangle
        elif currentangle_36000int % 4500 == 0:
            #print("using angle45")
            endpose = endpose_angle45_vel2[primIndex,:]
            angle = currentangle - 45*pi/180
        elif (currentangle_36000int-6750) % 9000 == 0:
            #print("using angle67p5")
            endpose[0] = endpose_angle22p5_vel2[primIndex,1]
            endpose[1] = endpose_angle22p5_vel2[primIndex,0]
            endpose[2] = -endpose_angle22p5_vel2[primIndex,2]
            endpose[3] = endpose_angle22p5_vel2[primIndex,3]
            angle = currentangle - 67.5*pi/180
        elif (currentangle_36000int-2250) % 9000 == 0:
            #print("using angle22p5")
            endpose = endpose_angle22p5_vel2[primIndex,:]
            angle = currentangle - 22.5*pi/180

        generateIntermediatePoses(angleIndex, currentangle, 2, angle, endpose)
        
    # vel 3
    for primIndex in range(0,numPrimsPerVel3):
        endpose = np.zeros(4)
        currentangle = angleIndex*2*pi/numAngles
        currentangle_36000int = round(angleIndex*36000/numAngles)

        if currentangle_36000int % 9000 == 0:
            #print("using angle0")
            endpose = endpose_angle0_vel3[primIndex,:]
            angle = currentangle
        elif currentangle_36000int % 4500 == 0:
            #print("using angle45")
            endpose = endpose_angle45_vel3[primIndex,:]
            angle = currentangle - 45*pi/180
        elif (currentangle_36000int-6750) % 9000 == 0:
            #print("using angle67p5")
            endpose[0] = endpose_angle22p5_vel3[primIndex,1]
            endpose[1] = endpose_angle22p5_vel3[primIndex,0]
            endpose[2] = -endpose_angle22p5_vel3[primIndex,2]
            endpose[3] = endpose_angle22p5_vel3[primIndex,3]
            angle = currentangle - 67.5*pi/180
        elif (currentangle_36000int-2250) % 9000 == 0:
            #print("using angle22p5")
            endpose = endpose_angle22p5_vel3[primIndex,:]
            angle = currentangle - 22.5*pi/180

        generateIntermediatePoses(angleIndex, currentangle, 3, angle, endpose)
        

plt.show()

