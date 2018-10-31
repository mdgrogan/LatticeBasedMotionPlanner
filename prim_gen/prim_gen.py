import numpy as np
import math


resolution = 0.025
numAngles = 16
vel0 = 0.0 # m/s
vel1 = 0.2
vel2 = 0.4
vel3 = 0.6
timeToTurn22p5degsInPlace = 1.0
numPrimsPerVel0 = 10
numPrimsPerVel1 = 18
numPrimsPerVel2 = 15
numPrimsPerVel3 = 9
numVelocitiesPerAngle = 4

filename = "unicycle_2p5cm.mprim"
primFile = open(filename, 'w+')
primFile.write("resolution_m: %d\n" % resolution)
primFile.write("numberofangles: %d\n" % numAngles)
primFile.write("numberofvelocities: 4\n")
primFile.write("totalnumberofprimitives: 832\n")
primFile.close()

# start always assumed to be cell (0,0)
# end pose is (x, y, angle, velocity)
# have 16 angles (0, 22.5, 45, 67.5, ...) corresponding to 0, 1, 2, ...
# and 4 velocities (0, ,0.2, 0.4, 0.6)
# for 2.5cm resolution, max accel will be 0.8 -- within reason,

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

endpose_angle67p5_vel0 = np.zeros((numPrimsPerVel0, 4))
endpose_angle67p5_vel1 = np.zeros((numPrimsPerVel1, 4))
endpose_angle67p5_vel2 = np.zeros((numPrimsPerVel2, 4))
endpose_angle67p5_vel3 = np.zeros((numPrimsPerVel3, 4))

endpose_angle90_vel0 = np.zeros((numPrimsPerVel0, 4))
endpose_angle90_vel1 = np.zeros((numPrimsPerVel1, 4))
endpose_angle90_vel2 = np.zeros((numPrimsPerVel2, 4))
endpose_angle90_vel3 = np.zeros((numPrimsPerVel3, 4))

# for 0 degrees (0)
##############################################################################
endpose_angle0_vel0[0,:] = [1, 0, 0, 0]     # forward -> vel0
endpose_angle0_vel0[1,:] = [1, 0, 0, 1]     # forward -> vel1
endpose_angle0_vel0[2,:] = [0, 0, 1, 0]     # rotate ccw in place
endpose_angle0_vel0[3,:] = [0, 0, 15, 0]    # rotate cw in place
endpose_angle0_vel0[4,:] = [4, 0, 0, 1]     # forward long -> vel1
endpose_angle0_vel0[5,:] = [4, 0, 0, 2]     # forward long -> vel2
endpose_angle0_vel0[6,:] = [4, 1, 1, 1]     # forward turn ccw long ->vel1
endpose_angle0_vel0[7,:] = [4, 1, 1, 2]     # forward turn ccw long ->vel2
endpose_angle0_vel0[8,:] = [4, -1, 15, 1]   # forward turn cw long -> vel1
endpose_angle0_vel0[9,:] = [4, -1, 15, 2]   # forward turn cw long -> vel2

endpose_angle0_vel1[0,:] = [1, 0, 0, 0]     # forward -> vel0
endpose_angle0_vel1[1,:] = [1, 0, 0, 1]     # forward -> vel1
endpose_angle0_vel1[2,:] = [1, 0, 0, 2]     # forward -> vel2
endpose_angle0_vel1[3,:] = [2, 1, 1, 0]     # forward turn ccw -> vel0
endpose_angle0_vel1[4,:] = [2, 1, 1, 1]     # forward turn ccw -> vel1
endpose_angle0_vel1[5,:] = [2, 1, 1, 2]     # forward turn ccw -> vel2
endpose_angle0_vel1[6,:] = [2, -1, 15, 0]   # forward turn cw -> vel0 
endpose_angle0_vel1[7,:] = [2, -1, 15, 1]   # forward turn cw -> vel1 
endpose_angle0_vel1[8,:] = [2, -1, 15, 2]   # forward turn cw -> vel2 
endpose_angle0_vel1[9,:] = [8, 0, 0, 1]     # forward long -> vel1
endpose_angle0_vel1[10,:] = [8, 0, 0, 2]    # forward long -> vel2
endpose_angle0_vel1[11,:] = [8, 0, 0, 3]    # forward long -> vel3
endpose_angle0_vel1[12,:] = [8, 1, 1, 1]    # forward turn ccw long ->vel1
endpose_angle0_vel1[13,:] = [8, 1, 1, 2]    # forward turn ccw long ->vel2
endpose_angle0_vel1[14,:] = [8, 1, 1, 3]    # forward turn ccw long ->vel3
endpose_angle0_vel1[15,:] = [8, -1, 15, 1]  # forward turn cw long -> vel1
endpose_angle0_vel1[16,:] = [8, -1, 15, 2]  # forward turn cw long -> vel2
endpose_angle0_vel1[17,:] = [8, -1, 15, 3]  # forward turn cw long -> vel3

endpose_angle0_vel2[0,:] = [1, 0, 0, 1]     # forward -> vel1
endpose_angle0_vel2[1,:] = [1, 0, 0, 2]     # forward -> vel2
endpose_angle0_vel2[2,:] = [1, 0, 0, 3]     # forward -> vel3
endpose_angle0_vel2[3,:] = [4, 1, 1, 1]     # forward turn ccw -> vel1
endpose_angle0_vel2[4,:] = [4, 1, 1, 2]     # forward turn ccw -> vel2
endpose_angle0_vel2[5,:] = [4, 1, 1, 3]     # forward turn ccw -> vel3
endpose_angle0_vel2[6,:] = [4, -1, 15, 1]   # forward turn cw -> vel1
endpose_angle0_vel2[7,:] = [4, -1, 15, 2]   # forward turn cw -> vel2
endpose_angle0_vel2[8,:] = [4, -1, 15, 3]   # forward turn cw -> vel3
endpose_angle0_vel2[9,:] = [8, 0, 0, 2]     # forward long -> vel2
endpose_angle0_vel2[10,:] = [8, 0, 0, 3]    # forward long -> vel3
endpose_angle0_vel2[11,:] = [8, 1, 1, 2]    # forward turn ccw long ->vel2
endpose_angle0_vel2[12,:] = [8, 1, 1, 3]    # forward turn ccw long ->vel3
endpose_angle0_vel2[13,:] = [8, -1, 15, 2]  # forward turn cw long -> vel2
endpose_angle0_vel2[14,:] = [8, -1, 15, 3]  # forward turn cw long -> vel3

endpose_angle0_vel3[0,:] = [1, 0, 0, 2]     # forward -> vel2
endpose_angle0_vel3[1,:] = [1, 0, 0, 3]     # forward -> vel3
endpose_angle0_vel3[2,:] = [4, 1, 1, 2]     # forward turn ccw -> vel2
endpose_angle0_vel3[3,:] = [4, 1, 1, 3]     # forward turn ccw -> vel3
endpose_angle0_vel3[4,:] = [4, -1, 15, 2]   # forward turn cw -> vel2
endpose_angle0_vel3[5,:] = [4, -1, 15, 3]   # forward turn cw -> vel3
endpose_angle0_vel3[6,:] = [8, 0, 0, 3]     # forward long -> vel3
endpose_angle0_vel3[7,:] = [8, 1, 1, 3]     # forward turn ccw long ->vel3
endpose_angle0_vel3[8,:] = [8, -1, 15, 3]   # forward turn cw long -> vel3

# for 22.5 degrees (1)
##############################################################################
endpose_angle22p5_vel0[0,:] = [2, 1, 1, 0]     # forward -> vel0
endpose_angle22p5_vel0[1,:] = [2, 1, 1, 1]     # forward -> vel1
endpose_angle22p5_vel0[2,:] = [0, 0, 2, 0]     # rotate ccw in place
endpose_angle22p5_vel0[3,:] = [0, 0, 0, 0]     # rotate cw in place
endpose_angle22p5_vel0[4,:] = [4, 2, 1, 1]     # forward long -> vel1
endpose_angle22p5_vel0[5,:] = [4, 2, 1, 2]     # forward long -> vel2
endpose_angle22p5_vel0[6,:] = [3, 2, 2, 1]     # forward turn ccw long ->vel1
endpose_angle22p5_vel0[7,:] = [3, 2, 2, 2]     # forward turn ccw long ->vel2
endpose_angle22p5_vel0[8,:] = [4, 1, 0, 1]     # forward turn cw long -> vel1
endpose_angle22p5_vel0[9,:] = [4, 1, 0, 2]     # forward turn cw long -> vel2

endpose_angle22p5_vel1[0,:] = [2, 1, 1, 0]     # forward -> vel0
endpose_angle22p5_vel1[1,:] = [2, 1, 1, 1]     # forward -> vel1
endpose_angle22p5_vel1[2,:] = [2, 1, 1, 2]     # forward -> vel2
endpose_angle22p5_vel1[3,:] = [3, 2, 2, 0]     # forward turn ccw -> vel0
endpose_angle22p5_vel1[4,:] = [3, 2, 2, 1]     # forward turn ccw -> vel1
endpose_angle22p5_vel1[5,:] = [3, 2, 2, 2]     # forward turn ccw -> vel2
endpose_angle22p5_vel1[6,:] = [4, 1, 0, 0]     # forward turn cw -> vel0 
endpose_angle22p5_vel1[7,:] = [4, 1, 0, 1]     # forward turn cw -> vel1 
endpose_angle22p5_vel1[8,:] = [4, 1, 0, 2]     # forward turn cw -> vel2 
endpose_angle22p5_vel1[9,:] = [8, 4, 1, 1]     # forward long -> vel1
endpose_angle22p5_vel1[10,:] = [8, 4, 1, 2]    # forward long -> vel2
endpose_angle22p5_vel1[11,:] = [8, 4, 1, 3]    # forward long -> vel3
endpose_angle22p5_vel1[12,:] = [7, 5, 2, 1]    # forward turn ccw long ->vel1
endpose_angle22p5_vel1[13,:] = [7, 5, 2, 2]    # forward turn ccw long ->vel2
endpose_angle22p5_vel1[14,:] = [7, 5, 2, 3]    # forward turn ccw long ->vel3
endpose_angle22p5_vel1[15,:] = [9, 3, 0, 1]    # forward turn cw long -> vel1
endpose_angle22p5_vel1[16,:] = [9, 3, 0, 2]    # forward turn cw long -> vel2
endpose_angle22p5_vel1[17,:] = [9, 3, 0, 3]    # forward turn cw long -> vel3

endpose_angle22p5_vel2[0,:] = [2, 1, 1, 1]     # forward -> vel1
endpose_angle22p5_vel2[1,:] = [2, 1, 1, 2]     # forward -> vel2
endpose_angle22p5_vel2[2,:] = [2, 1, 1, 3]     # forward -> vel3
endpose_angle22p5_vel2[3,:] = [3, 2, 2, 1]     # forward turn ccw -> vel1
endpose_angle22p5_vel2[4,:] = [3, 2, 2, 2]     # forward turn ccw -> vel2
endpose_angle22p5_vel2[5,:] = [3, 2, 2, 3]     # forward turn ccw -> vel3
endpose_angle22p5_vel2[6,:] = [4, 1, 0, 1]     # forward turn cw -> vel1 
endpose_angle22p5_vel2[7,:] = [4, 1, 0, 2]     # forward turn cw -> vel2 
endpose_angle22p5_vel2[8,:] = [4, 1, 0, 3]     # forward turn cw -> vel3 
endpose_angle22p5_vel2[9,:] = [8, 4, 1, 2]     # forward long -> vel2
endpose_angle22p5_vel2[10,:] = [8, 4, 1, 3]    # forward long -> vel3
endpose_angle22p5_vel2[11,:] = [7, 5, 2, 2]    # forward turn ccw long ->vel2
endpose_angle22p5_vel2[12,:] = [7, 5, 2, 3]    # forward turn ccw long ->vel3
endpose_angle22p5_vel2[13,:] = [9, 3, 0, 2]    # forward turn cw long -> vel2
endpose_angle22p5_vel2[14,:] = [9, 3, 0, 3]    # forward turn cw long -> vel3

endpose_angle22p5_vel3[0,:] = [2, 1, 1, 2]     # forward -> vel2
endpose_angle22p5_vel3[1,:] = [2, 1, 1, 3]     # forward -> vel3
endpose_angle22p5_vel3[2,:] = [3, 2, 2, 2]     # forward turn ccw -> vel2
endpose_angle22p5_vel3[3,:] = [3, 2, 2, 3]     # forward turn ccw -> vel3
endpose_angle22p5_vel3[4,:] = [4, 1, 0, 2]     # forward turn cw -> vel2 
endpose_angle22p5_vel3[5,:] = [4, 1, 0, 3]     # forward turn cw -> vel3 
endpose_angle22p5_vel3[6,:] = [8, 4, 1, 3]     # forward long -> vel3
endpose_angle22p5_vel3[7,:] = [7, 5, 2, 3]     # forward turn ccw long ->vel3
endpose_angle22p5_vel3[8,:] = [9, 3, 0, 3]     # forward turn cw long -> vel3

# for 45 degrees (2)
##############################################################################
endpose_angle45_vel0[0,:] = [1, 1, 2, 0]     # forward -> vel0
endpose_angle45_vel0[1,:] = [1, 1, 2, 1]     # forward -> vel1
endpose_angle45_vel0[2,:] = [0, 0, 3, 0]     # rotate ccw in place
endpose_angle45_vel0[3,:] = [0, 0, 1, 0]     # rotate cw in place
endpose_angle45_vel0[4,:] = [3, 3, 2, 1]     # forward long -> vel1
endpose_angle45_vel0[5,:] = [3, 3, 2, 2]     # forward long -> vel2
endpose_angle45_vel0[6,:] = [2, 4, 3, 1]     # forward turn ccw long ->vel1
endpose_angle45_vel0[7,:] = [2, 4, 3, 2]     # forward turn ccw long ->vel2
endpose_angle45_vel0[8,:] = [4, 2, 1, 1]     # forward turn cw long -> vel1
endpose_angle45_vel0[9,:] = [4, 2, 1, 2]     # forward turn cw long -> vel2

endpose_angle45_vel1[0,:] = [1, 1, 2, 0]     # forward -> vel0
endpose_angle45_vel1[1,:] = [1, 1, 2, 1]     # forward -> vel1
endpose_angle45_vel1[2,:] = [1, 1, 2, 2]     # forward -> vel2
endpose_angle45_vel1[3,:] = [1, 2, 3, 0]     # forward turn ccw -> vel0
endpose_angle45_vel1[4,:] = [1, 2, 3, 1]     # forward turn ccw -> vel1
endpose_angle45_vel1[5,:] = [1, 2, 3, 2]     # forward turn ccw -> vel2
endpose_angle45_vel1[6,:] = [2, 1, 1, 0]     # forward turn cw -> vel0 
endpose_angle45_vel1[7,:] = [2, 1, 1, 1]     # forward turn cw -> vel1 
endpose_angle45_vel1[8,:] = [2, 1, 1, 2]     # forward turn cw -> vel2 
endpose_angle45_vel1[9,:] = [6, 6, 2, 1]     # forward long -> vel1
endpose_angle45_vel1[10,:] = [6, 6, 2, 2]    # forward long -> vel2
endpose_angle45_vel1[11,:] = [6, 6, 2, 3]    # forward long -> vel3
endpose_angle45_vel1[12,:] = [5, 7, 3, 1]    # forward turn ccw long ->vel1
endpose_angle45_vel1[13,:] = [5, 7, 3, 2]    # forward turn ccw long ->vel2
endpose_angle45_vel1[14,:] = [5, 7, 3, 3]    # forward turn ccw long ->vel3
endpose_angle45_vel1[15,:] = [7, 5, 1, 1]    # forward turn cw long -> vel1
endpose_angle45_vel1[16,:] = [7, 5, 1, 2]    # forward turn cw long -> vel2
endpose_angle45_vel1[17,:] = [7, 5, 1, 3]    # forward turn cw long -> vel3

endpose_angle45_vel2[0,:] = [1, 1, 2, 1]     # forward -> vel1
endpose_angle45_vel2[1,:] = [1, 1, 2, 2]     # forward -> vel2
endpose_angle45_vel2[2,:] = [1, 1, 2, 3]     # forward -> vel3
endpose_angle45_vel2[3,:] = [2, 4, 3, 1]     # forward turn ccw -> vel1
endpose_angle45_vel2[4,:] = [2, 4, 3, 2]     # forward turn ccw -> vel2
endpose_angle45_vel2[5,:] = [2, 4, 3, 3]     # forward turn ccw -> vel3
endpose_angle45_vel2[6,:] = [4, 2, 1, 1]     # forward turn cw -> vel1 
endpose_angle45_vel2[7,:] = [4, 2, 1, 2]     # forward turn cw -> vel2 
endpose_angle45_vel2[8,:] = [4, 2, 1, 3]     # forward turn cw -> vel3 
endpose_angle45_vel2[9,:] = [6, 6, 2, 2]     # forward long -> vel2
endpose_angle45_vel2[10,:] = [6, 6, 2, 3]    # forward long -> vel3
endpose_angle45_vel2[11,:] = [5, 7, 3, 2]    # forward turn ccw long ->vel2
endpose_angle45_vel2[12,:] = [5, 7, 3, 3]    # forward turn ccw long ->vel3
endpose_angle45_vel2[13,:] = [7, 5, 1, 2]    # forward turn cw long -> vel2
endpose_angle45_vel2[14,:] = [7, 5, 1, 3]    # forward turn cw long -> vel3

endpose_angle45_vel3[0,:] = [1, 1, 2, 2]     # forward -> vel2
endpose_angle45_vel3[1,:] = [1, 1, 2, 3]     # forward -> vel3
endpose_angle45_vel3[2,:] = [2, 4, 3, 2]     # forward turn ccw -> vel2
endpose_angle45_vel3[3,:] = [2, 4, 3, 3]     # forward turn ccw -> vel3
endpose_angle45_vel3[4,:] = [4, 2, 1, 2]     # forward turn cw -> vel2 
endpose_angle45_vel3[5,:] = [4, 2, 1, 3]     # forward turn cw -> vel3 
endpose_angle45_vel3[6,:] = [6, 6, 2, 3]     # forward long -> vel3
endpose_angle45_vel3[7,:] = [5, 7, 3, 3]     # forward turn ccw long ->vel3
endpose_angle45_vel3[8,:] = [7, 5, 1, 3]     # forward turn cw long -> vel3

# for 67.5 degrees (3)
##############################################################################
endpose_angle67p5_vel0[0,:] = [1, 2, 3, 0]     # forward -> vel0
endpose_angle67p5_vel0[1,:] = [1, 2, 3, 1]     # forward -> vel1
endpose_angle67p5_vel0[2,:] = [0, 0, 4, 0]     # rotate ccw in place
endpose_angle67p5_vel0[3,:] = [0, 0, 2, 0]     # rotate cw in place
endpose_angle67p5_vel0[4,:] = [2, 4, 3, 1]     # forward long -> vel1
endpose_angle67p5_vel0[5,:] = [2, 4, 3, 2]     # forward long -> vel2
endpose_angle67p5_vel0[6,:] = [2, 3, 4, 1]     # forward turn ccw long ->vel1
endpose_angle67p5_vel0[7,:] = [2, 3, 4, 2]     # forward turn ccw long ->vel2
endpose_angle67p5_vel0[8,:] = [1, 4, 2, 1]     # forward turn cw long -> vel1
endpose_angle67p5_vel0[9,:] = [1, 4, 2, 2]     # forward turn cw long -> vel2

endpose_angle67p5_vel1[0,:] = [1, 2, 3, 0]     # forward -> vel0
endpose_angle67p5_vel1[1,:] = [1, 2, 3, 1]     # forward -> vel1
endpose_angle67p5_vel1[2,:] = [1, 2, 3, 2]     # forward -> vel2
endpose_angle67p5_vel1[3,:] = [2, 3, 4, 0]     # forward turn ccw -> vel0
endpose_angle67p5_vel1[4,:] = [2, 3, 4, 1]     # forward turn ccw -> vel1
endpose_angle67p5_vel1[5,:] = [2, 3, 4, 2]     # forward turn ccw -> vel2
endpose_angle67p5_vel1[6,:] = [1, 4, 2, 0]     # forward turn cw -> vel0 
endpose_angle67p5_vel1[7,:] = [1, 4, 2, 1]     # forward turn cw -> vel1 
endpose_angle67p5_vel1[8,:] = [1, 4, 2, 2]     # forward turn cw -> vel2 
endpose_angle67p5_vel1[9,:] = [4, 8, 3, 1]     # forward long -> vel1
endpose_angle67p5_vel1[10,:] = [4, 8, 3, 2]    # forward long -> vel2
endpose_angle67p5_vel1[11,:] = [4, 8, 3, 3]    # forward long -> vel3
endpose_angle67p5_vel1[12,:] = [5, 7, 4, 1]    # forward turn ccw long ->vel1
endpose_angle67p5_vel1[13,:] = [5, 7, 4, 2]    # forward turn ccw long ->vel2
endpose_angle67p5_vel1[14,:] = [5, 7, 4, 3]    # forward turn ccw long ->vel3
endpose_angle67p5_vel1[15,:] = [3, 9, 2, 1]    # forward turn cw long -> vel1
endpose_angle67p5_vel1[16,:] = [3, 9, 2, 2]    # forward turn cw long -> vel2
endpose_angle67p5_vel1[17,:] = [3, 9, 2, 3]    # forward turn cw long -> vel3

endpose_angle67p5_vel2[0,:] = [1, 2, 3, 1]     # forward -> vel1
endpose_angle67p5_vel2[1,:] = [1, 2, 3, 2]     # forward -> vel2
endpose_angle67p5_vel2[2,:] = [1, 2, 3, 3]     # forward -> vel3
endpose_angle67p5_vel2[3,:] = [2, 3, 4, 1]     # forward turn ccw -> vel1
endpose_angle67p5_vel2[4,:] = [2, 3, 4, 2]     # forward turn ccw -> vel2
endpose_angle67p5_vel2[5,:] = [2, 3, 4, 3]     # forward turn ccw -> vel3
endpose_angle67p5_vel2[6,:] = [1, 4, 2, 1]     # forward turn cw -> vel1 
endpose_angle67p5_vel2[7,:] = [1, 4, 2, 2]     # forward turn cw -> vel2 
endpose_angle67p5_vel2[8,:] = [1, 4, 2, 3]     # forward turn cw -> vel3 
endpose_angle67p5_vel2[9,:] = [4, 8, 3, 2]     # forward long -> vel2
endpose_angle67p5_vel2[10,:] = [4, 8, 3, 3]    # forward long -> vel3
endpose_angle67p5_vel2[11,:] = [5, 7, 4, 2]    # forward turn ccw long ->vel2
endpose_angle67p5_vel2[12,:] = [5, 7, 4, 3]    # forward turn ccw long ->vel3
endpose_angle67p5_vel2[13,:] = [3, 9, 2, 2]    # forward turn cw long -> vel2
endpose_angle67p5_vel2[14,:] = [3, 9, 2, 3]    # forward turn cw long -> vel3

endpose_angle67p5_vel3[0,:] = [1, 2, 3, 2]     # forward -> vel2
endpose_angle67p5_vel3[1,:] = [1, 2, 3, 3]     # forward -> vel3
endpose_angle67p5_vel3[2,:] = [2, 3, 4, 2]     # forward turn ccw -> vel2
endpose_angle67p5_vel3[3,:] = [2, 3, 4, 3]     # forward turn ccw -> vel3
endpose_angle67p5_vel3[4,:] = [1, 4, 2, 2]     # forward turn cw -> vel2 
endpose_angle67p5_vel3[5,:] = [1, 4, 2, 3]     # forward turn cw -> vel3 
endpose_angle67p5_vel3[6,:] = [4, 8, 3, 3]     # forward long -> vel3
endpose_angle67p5_vel3[7,:] = [5, 7, 4, 3]     # forward turn ccw long ->vel3
endpose_angle67p5_vel3[8,:] = [3, 9, 2, 3]     # forward turn cw long -> vel3

# for 90 degrees (4)
##############################################################################
endpose_angle90_vel0[0,:] = [0, 1, 4, 0]     # forward -> vel0
endpose_angle90_vel0[1,:] = [0, 1, 4, 1]     # forward -> vel1
endpose_angle90_vel0[2,:] = [0, 0, 5, 0]     # rotate ccw in place
endpose_angle90_vel0[3,:] = [0, 0, 3, 0]    # rotate cw in place
endpose_angle90_vel0[4,:] = [0, 4, 4, 1]     # forward long -> vel1
endpose_angle90_vel0[5,:] = [0, 4, 4, 2]     # forward long -> vel2
endpose_angle90_vel0[6,:] = [-1, 4, 5, 1]     # forward turn ccw long ->vel1
endpose_angle90_vel0[7,:] = [-1, 4, 5, 2]     # forward turn ccw long ->vel2
endpose_angle90_vel0[8,:] = [1, 4, 3, 1]   # forward turn cw long -> vel1
endpose_angle90_vel0[9,:] = [1, 4, 3, 2]   # forward turn cw long -> vel2

endpose_angle90_vel1[0,:] = [0, 1, 4, 0]     # forward -> vel0
endpose_angle90_vel1[1,:] = [0, 1, 4, 1]     # forward -> vel1
endpose_angle90_vel1[2,:] = [0, 1, 4, 2]     # forward -> vel2
endpose_angle90_vel1[3,:] = [-1, 2, 5, 0]     # forward turn ccw -> vel0
endpose_angle90_vel1[4,:] = [-1, 2, 5, 1]     # forward turn ccw -> vel1
endpose_angle90_vel1[5,:] = [-1, 2, 5, 2]     # forward turn ccw -> vel2
endpose_angle90_vel1[6,:] = [1, 2, 3, 0]   # forward turn cw -> vel0 
endpose_angle90_vel1[7,:] = [1, 2, 3, 1]   # forward turn cw -> vel1 
endpose_angle90_vel1[8,:] = [1, 2, 3, 2]   # forward turn cw -> vel2 
endpose_angle90_vel1[9,:] = [0, 8, 4, 1]     # forward long -> vel1
endpose_angle90_vel1[10,:] = [0, 8, 4, 2]    # forward long -> vel2
endpose_angle90_vel1[11,:] = [0, 8, 4, 3]    # forward long -> vel3
endpose_angle90_vel1[12,:] = [-1, 8, 5, 1]    # forward turn ccw long ->vel1
endpose_angle90_vel1[13,:] = [-1, 8, 5, 2]    # forward turn ccw long ->vel2
endpose_angle90_vel1[14,:] = [-1, 8, 5, 3]    # forward turn ccw long ->vel3
endpose_angle90_vel1[15,:] = [1, 8, 3, 1]  # forward turn cw long -> vel1
endpose_angle90_vel1[16,:] = [1, 8, 3, 2]  # forward turn cw long -> vel2
endpose_angle90_vel1[17,:] = [1, 8, 3, 3]  # forward turn cw long -> vel3

endpose_angle90_vel2[0,:] = [0, 1, 4, 1]     # forward -> vel1
endpose_angle90_vel2[1,:] = [0, 1, 4, 2]     # forward -> vel2
endpose_angle90_vel2[2,:] = [0, 1, 4, 3]     # forward -> vel3
endpose_angle90_vel2[3,:] = [-1, 4, 5, 1]     # forward turn ccw -> vel1
endpose_angle90_vel2[4,:] = [-1, 4, 5, 2]     # forward turn ccw -> vel2
endpose_angle90_vel2[5,:] = [-1, 4, 5, 3]     # forward turn ccw -> vel3
endpose_angle90_vel2[6,:] = [1, 4, 3, 1]   # forward turn cw -> vel1
endpose_angle90_vel2[7,:] = [1, 4, 3, 2]   # forward turn cw -> vel2
endpose_angle90_vel2[8,:] = [1, 4, 3, 3]   # forward turn cw -> vel3
endpose_angle90_vel2[9,:] = [0, 8, 4, 2]     # forward long -> vel2
endpose_angle90_vel2[10,:] = [0, 8, 4, 3]    # forward long -> vel3
endpose_angle90_vel2[11,:] = [-1, 8, 1, 2]    # forward turn ccw long ->vel2
endpose_angle90_vel2[12,:] = [-1, 8, 1, 3]    # forward turn ccw long ->vel3
endpose_angle90_vel2[13,:] = [1, 8, 15, 2]  # forward turn cw long -> vel2
endpose_angle90_vel2[14,:] = [1, 8, 15, 3]  # forward turn cw long -> vel3

endpose_angle90_vel3[0,:] = [0, 1, 4, 2]     # forward -> vel2
endpose_angle90_vel3[1,:] = [0, 1, 4, 3]     # forward -> vel3
endpose_angle90_vel3[2,:] = [-1, 4, 5, 2]     # forward turn ccw -> vel2
endpose_angle90_vel3[3,:] = [-1, 4, 5, 3]     # forward turn ccw -> vel3
endpose_angle90_vel3[4,:] = [1, 4, 3, 2]   # forward turn cw -> vel2
endpose_angle90_vel3[5,:] = [1, 4, 3, 3]   # forward turn cw -> vel3
endpose_angle90_vel3[6,:] = [0, 8, 4, 3]     # forward long -> vel3
endpose_angle90_vel3[7,:] = [-1, 8, 5, 3]     # forward turn ccw long ->vel3
endpose_angle90_vel3[8,:] = [1, 8, 3, 3]   # forward turn cw long -> vel3

for angleIndex in range(0,numAngles):
    endpose = np.zeros(4)
    for primIndex in range(0,numPrimsPerVel0):
        if angleIndex == 0: # 0
            endpose = endpose_angle0_vel0[primIndex,:]
        if angleIndex == 1: # 22.5
            endpose = endpose_angle22p5_vel0[primIndex,:]
        if angleIndex == 2: # 45
            endpose = endpose_angle45_vel0[primIndex,:]
        if angleIndex == 3: # 67.5
            endpose = endpose_angle67p5_vel0[primIndex,:]
        if angleIndex == 4: # 90
            endpose = endpose_angle90_vel0[primIndex,:]
        if angleIndex == 5: # 112.5
            endpose[0] = endpose_angle67p5_vel0[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel0[primIndex,1]
            if endpose_angle67p5_vel0[primIndex,2] == 2:
                endpose[2] = 6
            if endpose_angle67p5_vel0[primIndex,2] == 3:
                endpose[2] = 5
            if endpose_angle67p5_vel0[primIndex,2] == 4:
                endpose[2] = 4
            endpose[3] = endpose_angle67p5_vel0[primIndex,3] 
        if angleIndex == 6: # 135
            endpose[0] = endpose_angle45_vel0[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel0[primIndex,1]
            if endpose_angle45_vel0[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle45_vel0[primIndex,2] == 2:
                endpose[2] = 4
            if endpose_angle45_vel0[primIndex,2] == 3:
                endpose[2] = 5
            endpose[3] = endpose_angle45_vel0[primIndex,3]
        if angleIndex == 7: # 157.5
            endpose[0] = endpose_angle22p5_vel0[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel0[primIndex,1]
            if endpose_angle22p5_vel0[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel0[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle22p5_vel0[primIndex,2] == 2:
                endpose[2] = 6
            endpose[3] = endpose_angle22p5_vel0[primIndex,3]
        if angleIndex == 8: # 180
            endpose[0] = endpose_angle0_vel0[primIndex,0] * -1
            endpose[1] = endpose_angle0_vel0[primIndex,1]
            if endpose_angle0_vel0[primIndex,2] == 15:
                endpose[2] = 9
            if endpose_angle0_vel0[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle0_vel0[primIndex,2] == 1:
                endpose[2] = 7
            endpose[3] = endpose_angle0_vel0[primIndex,3]
        if angleIndex == 9:
            endpose[0] = endpose_angle22p5_vel0[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel0[primIndex,1] * -1
            if endpose_angle22p5_vel0[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel0[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle22p5_vel0[primIndex,2] == 2:
                endpose[2] = 10
            endpose[3] = endpose_angle22p5_vel0[primIndex,3]
        if angleIndex == 10:
            endpose[0] = endpose_angle45_vel0[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel0[primIndex,1] * -1
            if endpose_angle45_vel0[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle45_vel0[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle45_vel0[primIndex,2] == 3:
                endpose[2] = 11
            endpose[3] = endpose_angle45_vel0[primIndex,3]
        if angleIndex == 11:
            endpose[0] = endpose_angle67p5_vel0[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel0[primIndex,1] * -1
            if endpose_angle67p5_vel0[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle67p5_vel0[primIndex,2] == 3:
                endpose[2] = 11
            if endpose_angle67p5_vel0[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel0[primIndex,3] 
        if angleIndex == 12:
            endpose[0] = endpose_angle90_vel0[primIndex,0]
            endpose[1] = endpose_angle90_vel0[primIndex,1] * -1
            if endpose_angle90_vel0[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle90_vel0[primIndex,2] == 4:
                endpose[2] = 12
            if endpose_angle90_vel0[primIndex,2] == 5:
                endpose[2] = 11
            endpose[3] = endpose_angle90_vel0[primIndex,3]
        if angleIndex == 13:
            endpose[0] = endpose_angle67p5_vel0[primIndex,0] 
            endpose[1] = endpose_angle67p5_vel0[primIndex,1] * -1
            if endpose_angle67p5_vel0[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle67p5_vel0[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle67p5_vel0[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel0[primIndex,3] 
        if angleIndex == 14:
            endpose[0] = endpose_angle45_vel0[primIndex,0] 
            endpose[1] = endpose_angle45_vel0[primIndex,1] * -1
            if endpose_angle45_vel0[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle45_vel0[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle45_vel0[primIndex,2] == 3:
                endpose[2] = 13
            endpose[3] = endpose_angle45_vel0[primIndex,3]
        if angleIndex == 15:
            endpose[0] = endpose_angle22p5_vel0[primIndex,0] 
            endpose[1] = endpose_angle22p5_vel0[primIndex,1] * -1
            if endpose_angle22p5_vel0[primIndex,2] == 0:
                endpose[2] = 0
            if endpose_angle22p5_vel0[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle22p5_vel0[primIndex,2] == 2:
                endpose[2] = 14
            endpose[3] = endpose_angle22p5_vel0[primIndex,3]

        # generate intermediate x,y positions w.r.t 0,0 (in meters)
        numSamples = 10
        intermPoses_m = np.zeros((numSamples,2))
        endPose_m = [endpose[0]*resolution, endpose[1]*resolution]
        # turn in place or move straight forward
        if (endpose[0]==0 and endpose[1]==0) or endpose[2]==angleIndex:
            for i in range(0,numSamples):
                intermPoses_m[i,:] = [endPose_m[0]*i/(numSamples-1), endPose_m[1]*i/(numSamples-1)]
        else:
            R = [[math.cos(angleIndex*2*math.pi/numAngles), math.sin(endpose[2]*2*math.pi/numAngles) - math.sin(angleIndex*2*math.pi/numAngles)], \
                 [math.sin(angleIndex*2*math.pi/numAngles), -math.cos(endpose[2]*2*math.pi/numAngles) - math.cos(angleIndex*2*math.pi/numAngles)]]
            S = np.matmul(np.linalg.pinv(R), [[endPose_m[0]],[endPose_m[1]]])
            l = S[0]
            tvoverrv = S[1]
            rv = endpose[2]*2*math.pi/numAngles + 1/tvoverrv
            tv = tvoverrv*rv

            for i in range(0,numSamples):
                dt = i/numSamples
                theta = angleIndex*2*math.pi/numAngles
                if dt*tv < 1:
                    intermPoses_m[i,:] = [dt*tv*math.cos(theta), \
                                          dt*tv*math.sin(theta)]
                else:
                    dtheta = rv*(dt - 1/tv) + theta
                    intermPoses_m[i,:] = [l*math.cos(theta) + tvoverrv*(math.sin(dtheta)-math.sin(theta)), \
                                          l*math.sin(theta) - tvoverrv*(math.cos(dtheta)-math.cos(theta))]
            
            errorxy = [endPose_m[0] - intermPoses_m[numSamples-1,0], \
                       endPose_m[1] - intermPoses_m[numSamples-1,1]]
            interp = np.zeros(10)
            for i in range(0,numSamples):
                interp[i] = i/(numSamples-1)
            intermPoses_m[:,0] = intermPoses_m[:,0] + errorxy[0]*interp
            intermPoses_m[:,1] = intermPoses_m[:,1] + errorxy[1]*interp

        # calculate primitive execution time (assuming straight line
        primExecTime = 0.0
        if (endpose[0]==0 and endpose[1]==0): # turn in place
            primExecTime = timeToTurn22p5degsInPlace 
        elif endpose[3]==0: # vel0 -> vel0 transition
            r = math.sqrt(endPose_m[0]**2 + endPose_m[1]**2)/2
            t = 2*r/(vel1/2)
            primExecTime = 2*t
        else:
            r = math.sqrt(endPose_m[0]**2 + endPose_m[1]**2)
            primExecTime = 2*r/(vel0 + endpose[3]*0.2)

        primFile = open(filename, 'a')
        primFile.write("startangle_i: %d\n" % (angleIndex))
        primFile.write("startvel_i: 0\n")
        primFile.write("endpose_i: %d %d %d %d\n" % (endpose[0],endpose[1],endpose[2],endpose[3]))
        primFile.write("exectime: %.4f\n" % (primExecTime))
        primFile.write("intermediateposes: %d\n" % (numSamples))
        for i in range(0,numSamples):
            primFile.write("%.4f %.4f\n" % (intermPoses_m[i,0], intermPoses_m[i,1]))


    for primIndex in range(0,numPrimsPerVel1):
        if angleIndex == 0: # 0
            endpose = endpose_angle0_vel1[primIndex,:]
        if angleIndex == 1: # 22.5
            endpose = endpose_angle22p5_vel1[primIndex,:]
        if angleIndex == 2: # 45
            endpose = endpose_angle45_vel1[primIndex,:]
        if angleIndex == 3: # 67.5
            endpose = endpose_angle67p5_vel1[primIndex,:]
        if angleIndex == 4: # 90
            endpose = endpose_angle90_vel1[primIndex,:]
        if angleIndex == 5: # 112.5
            endpose[0] = endpose_angle67p5_vel1[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel1[primIndex,1]
            if endpose_angle67p5_vel1[primIndex,2] == 2:
                endpose[2] = 6
            if endpose_angle67p5_vel1[primIndex,2] == 3:
                endpose[2] = 5
            if endpose_angle67p5_vel1[primIndex,2] == 4:
                endpose[2] = 4
            endpose[3] = endpose_angle67p5_vel1[primIndex,3] 
        if angleIndex == 6: # 135
            endpose[0] = endpose_angle45_vel1[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel1[primIndex,1]
            if endpose_angle45_vel1[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle45_vel1[primIndex,2] == 2:
                endpose[2] = 4
            if endpose_angle45_vel1[primIndex,2] == 3:
                endpose[2] = 5
            endpose[3] = endpose_angle45_vel1[primIndex,3]
        if angleIndex == 7: # 157.5
            endpose[0] = endpose_angle22p5_vel1[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel1[primIndex,1]
            if endpose_angle22p5_vel1[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel1[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle22p5_vel1[primIndex,2] == 2:
                endpose[2] = 6
            endpose[3] = endpose_angle22p5_vel1[primIndex,3]
        if angleIndex == 8: # 180
            endpose[0] = endpose_angle0_vel1[primIndex,0] * -1
            endpose[1] = endpose_angle0_vel1[primIndex,1]
            if endpose_angle0_vel1[primIndex,2] == 15:
                endpose[2] = 9
            if endpose_angle0_vel1[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle0_vel1[primIndex,2] == 1:
                endpose[2] = 7
            endpose[3] = endpose_angle0_vel1[primIndex,3]
        if angleIndex == 9:
            endpose[0] = endpose_angle22p5_vel1[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel1[primIndex,1] * -1
            if endpose_angle22p5_vel1[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel1[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle22p5_vel1[primIndex,2] == 2:
                endpose[2] = 10
            endpose[3] = endpose_angle22p5_vel1[primIndex,3]
        if angleIndex == 10:
            endpose[0] = endpose_angle45_vel1[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel1[primIndex,1] * -1
            if endpose_angle45_vel1[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle45_vel1[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle45_vel1[primIndex,2] == 3:
                endpose[2] = 11
            endpose[3] = endpose_angle45_vel1[primIndex,3]
        if angleIndex == 11:
            endpose[0] = endpose_angle67p5_vel1[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel1[primIndex,1] * -1
            if endpose_angle67p5_vel1[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle67p5_vel1[primIndex,2] == 3:
                endpose[2] = 11
            if endpose_angle67p5_vel1[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel1[primIndex,3] 
        if angleIndex == 12:
            endpose[0] = endpose_angle90_vel1[primIndex,0]
            endpose[1] = endpose_angle90_vel1[primIndex,1] * -1
            if endpose_angle90_vel1[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle90_vel1[primIndex,2] == 4:
                endpose[2] = 12
            if endpose_angle90_vel1[primIndex,2] == 5:
                endpose[2] = 11
            endpose[3] = endpose_angle90_vel1[primIndex,3]
        if angleIndex == 13:
            endpose[0] = endpose_angle67p5_vel1[primIndex,0] 
            endpose[1] = endpose_angle67p5_vel1[primIndex,1] * -1
            if endpose_angle67p5_vel1[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle67p5_vel1[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle67p5_vel1[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel1[primIndex,3] 
        if angleIndex == 14:
            endpose[0] = endpose_angle45_vel1[primIndex,0] 
            endpose[1] = endpose_angle45_vel1[primIndex,1] * -1
            if endpose_angle45_vel1[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle45_vel1[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle45_vel1[primIndex,2] == 3:
                endpose[2] = 13
            endpose[3] = endpose_angle45_vel1[primIndex,3]
        if angleIndex == 15:
            endpose[0] = endpose_angle22p5_vel1[primIndex,0] 
            endpose[1] = endpose_angle22p5_vel1[primIndex,1] * -1
            if endpose_angle22p5_vel1[primIndex,2] == 0:
                endpose[2] = 0
            if endpose_angle22p5_vel1[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle22p5_vel1[primIndex,2] == 2:
                endpose[2] = 14
            endpose[3] = endpose_angle22p5_vel1[primIndex,3]

        # generate intermediate x,y positions w.r.t 0,0 (in meters)
        numSamples = 10
        intermPoses_m = np.zeros((numSamples,2))
        endPose_m = [endpose[0]*resolution, endpose[1]*resolution]
        # turn in place or move straight forward
        if (endpose[0]==0 and endpose[1]==0) or endpose[2]==angleIndex:
            for i in range(0,numSamples):
                intermPoses_m[i,:] = [endPose_m[0]*i/(numSamples-1), endPose_m[1]*i/(numSamples-1)]
        else:
            R = [[math.cos(angleIndex*2*math.pi/numAngles), math.sin(endpose[2]*2*math.pi/numAngles) - math.sin(angleIndex*2*math.pi/numAngles)], \
                 [math.sin(angleIndex*2*math.pi/numAngles), -math.cos(endpose[2]*2*math.pi/numAngles) - math.cos(angleIndex*2*math.pi/numAngles)]]
            S = np.matmul(np.linalg.pinv(R), [[endPose_m[0]],[endPose_m[1]]])
            l = S[0]
            tvoverrv = S[1]
            rv = endpose[2]*2*math.pi/numAngles + 1/tvoverrv
            tv = tvoverrv*rv

            for i in range(0,numSamples):
                dt = i/numSamples
                theta = angleIndex*2*math.pi/numAngles
                if dt*tv < 1:
                    intermPoses_m[i,:] = [dt*tv*math.cos(theta), \
                                          dt*tv*math.sin(theta)]
                else:
                    dtheta = rv*(dt - 1/tv) + theta
                    intermPoses_m[i,:] = [l*math.cos(theta) + tvoverrv*(math.sin(dtheta)-math.sin(theta)), \
                                          l*math.sin(theta) - tvoverrv*(math.cos(dtheta)-math.cos(theta))]
            
            errorxy = [endPose_m[0] - intermPoses_m[numSamples-1,0], \
                       endPose_m[1] - intermPoses_m[numSamples-1,1]]
            interp = np.zeros(10)
            for i in range(0,numSamples):
                interp[i] = i/(numSamples-1)
            intermPoses_m[:,0] = intermPoses_m[:,0] + errorxy[0]*interp
            intermPoses_m[:,1] = intermPoses_m[:,1] + errorxy[1]*interp

        # calculate primitive execution time (assuming straight line
        r = math.sqrt(endPose_m[0]**2 + endPose_m[1]**2)
        primExecTime = 2*r/(vel1 + endpose[3]*0.2)

        primFile = open(filename, 'a')
        primFile.write("startangle_i: %d\n" % (angleIndex))
        primFile.write("startvel_i: 1\n")
        primFile.write("endpose_i: %d %d %d %d\n" % (endpose[0],endpose[1],endpose[2],endpose[3]))
        primFile.write("exectime: %.4f\n" % (primExecTime))
        primFile.write("intermediateposes: %d\n" % (numSamples))
        for i in range(0,numSamples):
            primFile.write("%.4f %.4f\n" % (intermPoses_m[i,0], intermPoses_m[i,1]))


    for primIndex in range(0,numPrimsPerVel2):
        if angleIndex == 0: # 0
            endpose = endpose_angle0_vel2[primIndex,:]
        if angleIndex == 1: # 22.5
            endpose = endpose_angle22p5_vel2[primIndex,:]
        if angleIndex == 2: # 45
            endpose = endpose_angle45_vel2[primIndex,:]
        if angleIndex == 3: # 67.5
            endpose = endpose_angle67p5_vel2[primIndex,:]
        if angleIndex == 4: # 90
            endpose = endpose_angle90_vel2[primIndex,:]
        if angleIndex == 5: # 112.5
            endpose[0] = endpose_angle67p5_vel2[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel2[primIndex,1]
            if endpose_angle67p5_vel2[primIndex,2] == 2:
                endpose[2] = 6
            if endpose_angle67p5_vel2[primIndex,2] == 3:
                endpose[2] = 5
            if endpose_angle67p5_vel2[primIndex,2] == 4:
                endpose[2] = 4
            endpose[3] = endpose_angle67p5_vel2[primIndex,3] 
        if angleIndex == 6: # 135
            endpose[0] = endpose_angle45_vel2[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel2[primIndex,1]
            if endpose_angle45_vel2[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle45_vel2[primIndex,2] == 2:
                endpose[2] = 4
            if endpose_angle45_vel2[primIndex,2] == 3:
                endpose[2] = 5
            endpose[3] = endpose_angle45_vel2[primIndex,3]
        if angleIndex == 7: # 157.5
            endpose[0] = endpose_angle22p5_vel2[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel2[primIndex,1]
            if endpose_angle22p5_vel2[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel2[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle22p5_vel2[primIndex,2] == 2:
                endpose[2] = 6
            endpose[3] = endpose_angle22p5_vel2[primIndex,3]
        if angleIndex == 8: # 180
            endpose[0] = endpose_angle0_vel2[primIndex,0] * -1
            endpose[1] = endpose_angle0_vel2[primIndex,1]
            if endpose_angle0_vel2[primIndex,2] == 15:
                endpose[2] = 9
            if endpose_angle0_vel2[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle0_vel2[primIndex,2] == 1:
                endpose[2] = 7
            endpose[3] = endpose_angle0_vel2[primIndex,3]
        if angleIndex == 9:
            endpose[0] = endpose_angle22p5_vel2[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel2[primIndex,1] * -1
            if endpose_angle22p5_vel2[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel2[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle22p5_vel2[primIndex,2] == 2:
                endpose[2] = 10
            endpose[3] = endpose_angle22p5_vel2[primIndex,3]
        if angleIndex == 10:
            endpose[0] = endpose_angle45_vel2[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel2[primIndex,1] * -1
            if endpose_angle45_vel2[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle45_vel2[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle45_vel2[primIndex,2] == 3:
                endpose[2] = 11
            endpose[3] = endpose_angle45_vel2[primIndex,3]
        if angleIndex == 11:
            endpose[0] = endpose_angle67p5_vel2[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel2[primIndex,1] * -1
            if endpose_angle67p5_vel2[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle67p5_vel2[primIndex,2] == 3:
                endpose[2] = 11
            if endpose_angle67p5_vel2[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel2[primIndex,3] 
        if angleIndex == 12:
            endpose[0] = endpose_angle90_vel2[primIndex,0]
            endpose[1] = endpose_angle90_vel2[primIndex,1] * -1
            if endpose_angle90_vel2[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle90_vel2[primIndex,2] == 4:
                endpose[2] = 12
            if endpose_angle90_vel2[primIndex,2] == 5:
                endpose[2] = 11
            endpose[3] = endpose_angle90_vel2[primIndex,3]
        if angleIndex == 13:
            endpose[0] = endpose_angle67p5_vel2[primIndex,0] 
            endpose[1] = endpose_angle67p5_vel2[primIndex,1] * -1
            if endpose_angle67p5_vel2[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle67p5_vel2[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle67p5_vel2[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel2[primIndex,3] 
        if angleIndex == 14:
            endpose[0] = endpose_angle45_vel2[primIndex,0] 
            endpose[1] = endpose_angle45_vel2[primIndex,1] * -1
            if endpose_angle45_vel2[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle45_vel2[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle45_vel2[primIndex,2] == 3:
                endpose[2] = 13
            endpose[3] = endpose_angle45_vel2[primIndex,3]
        if angleIndex == 15:
            endpose[0] = endpose_angle22p5_vel2[primIndex,0] 
            endpose[1] = endpose_angle22p5_vel2[primIndex,1] * -1
            if endpose_angle22p5_vel2[primIndex,2] == 0:
                endpose[2] = 0
            if endpose_angle22p5_vel2[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle22p5_vel2[primIndex,2] == 2:
                endpose[2] = 14
            endpose[3] = endpose_angle22p5_vel2[primIndex,3]

        # generate intermediate x,y positions w.r.t 0,0 (in meters)
        numSamples = 10
        intermPoses_m = np.zeros((numSamples,2))
        endPose_m = [endpose[0]*resolution, endpose[1]*resolution]
        # turn in place or move straight forward
        if (endpose[0]==0 and endpose[1]==0) or endpose[2]==angleIndex:
            for i in range(0,numSamples):
                intermPoses_m[i,:] = [endPose_m[0]*i/(numSamples-1), endPose_m[1]*i/(numSamples-1)]
        else:
            R = [[math.cos(angleIndex*2*math.pi/numAngles), math.sin(endpose[2]*2*math.pi/numAngles) - math.sin(angleIndex*2*math.pi/numAngles)], \
                 [math.sin(angleIndex*2*math.pi/numAngles), -math.cos(endpose[2]*2*math.pi/numAngles) - math.cos(angleIndex*2*math.pi/numAngles)]]
            S = np.matmul(np.linalg.pinv(R), [[endPose_m[0]],[endPose_m[1]]])
            l = S[0]
            tvoverrv = S[1]
            rv = endpose[2]*2*math.pi/numAngles + 1/tvoverrv
            tv = tvoverrv*rv

            for i in range(0,numSamples):
                dt = i/numSamples
                theta = angleIndex*2*math.pi/numAngles
                if dt*tv < 1:
                    intermPoses_m[i,:] = [dt*tv*math.cos(theta), \
                                          dt*tv*math.sin(theta)]
                else:
                    dtheta = rv*(dt - 1/tv) + theta
                    intermPoses_m[i,:] = [l*math.cos(theta) + tvoverrv*(math.sin(dtheta)-math.sin(theta)), \
                                          l*math.sin(theta) - tvoverrv*(math.cos(dtheta)-math.cos(theta))]
            
            errorxy = [endPose_m[0] - intermPoses_m[numSamples-1,0], \
                       endPose_m[1] - intermPoses_m[numSamples-1,1]]
            interp = np.zeros(10)
            for i in range(0,numSamples):
                interp[i] = i/(numSamples-1)
            intermPoses_m[:,0] = intermPoses_m[:,0] + errorxy[0]*interp
            intermPoses_m[:,1] = intermPoses_m[:,1] + errorxy[1]*interp

        # calculate primitive execution time (assuming straight line
        r = math.sqrt(endPose_m[0]**2 + endPose_m[1]**2)
        primExecTime = 2*r/(vel2 + endpose[3]*0.2)

        primFile = open(filename, 'a')
        primFile.write("startangle_i: %d\n" % (angleIndex))
        primFile.write("startvel_i: 2\n")
        primFile.write("endpose_i: %d %d %d %d\n" % (endpose[0],endpose[1],endpose[2],endpose[3]))
        primFile.write("exectime: %.4f\n" % (primExecTime))
        primFile.write("intermediateposes: %d\n" % (numSamples))
        for i in range(0,numSamples):
            primFile.write("%.4f %.4f\n" % (intermPoses_m[i,0], intermPoses_m[i,1]))


    for primIndex in range(0,numPrimsPerVel3):
        if angleIndex == 0: # 0
            endpose = endpose_angle0_vel3[primIndex,:]
        if angleIndex == 1: # 22.5
            endpose = endpose_angle22p5_vel3[primIndex,:]
        if angleIndex == 2: # 45
            endpose = endpose_angle45_vel3[primIndex,:]
        if angleIndex == 3: # 67.5
            endpose = endpose_angle67p5_vel3[primIndex,:]
        if angleIndex == 4: # 90
            endpose = endpose_angle90_vel3[primIndex,:]
        if angleIndex == 5: # 112.5
            endpose[0] = endpose_angle67p5_vel3[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel3[primIndex,1]
            if endpose_angle67p5_vel3[primIndex,2] == 2:
                endpose[2] = 6
            if endpose_angle67p5_vel3[primIndex,2] == 3:
                endpose[2] = 5
            if endpose_angle67p5_vel3[primIndex,2] == 4:
                endpose[2] = 4
            endpose[3] = endpose_angle67p5_vel3[primIndex,3] 
        if angleIndex == 6: # 135
            endpose[0] = endpose_angle45_vel3[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel3[primIndex,1]
            if endpose_angle45_vel3[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle45_vel3[primIndex,2] == 2:
                endpose[2] = 4
            if endpose_angle45_vel3[primIndex,2] == 3:
                endpose[2] = 5
            endpose[3] = endpose_angle45_vel3[primIndex,3]
        if angleIndex == 7: # 157.5
            endpose[0] = endpose_angle22p5_vel3[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel3[primIndex,1]
            if endpose_angle22p5_vel3[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel3[primIndex,2] == 1:
                endpose[2] = 7
            if endpose_angle22p5_vel3[primIndex,2] == 2:
                endpose[2] = 6
            endpose[3] = endpose_angle22p5_vel3[primIndex,3]
        if angleIndex == 8: # 180
            endpose[0] = endpose_angle0_vel3[primIndex,0] * -1
            endpose[1] = endpose_angle0_vel3[primIndex,1]
            if endpose_angle0_vel3[primIndex,2] == 15:
                endpose[2] = 9
            if endpose_angle0_vel3[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle0_vel3[primIndex,2] == 1:
                endpose[2] = 7
            endpose[3] = endpose_angle0_vel3[primIndex,3]
        if angleIndex == 9:
            endpose[0] = endpose_angle22p5_vel3[primIndex,0] * -1
            endpose[1] = endpose_angle22p5_vel3[primIndex,1] * -1
            if endpose_angle22p5_vel3[primIndex,2] == 0:
                endpose[2] = 8
            if endpose_angle22p5_vel3[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle22p5_vel3[primIndex,2] == 2:
                endpose[2] = 10
            endpose[3] = endpose_angle22p5_vel3[primIndex,3]
        if angleIndex == 10:
            endpose[0] = endpose_angle45_vel3[primIndex,0] * -1
            endpose[1] = endpose_angle45_vel3[primIndex,1] * -1
            if endpose_angle45_vel3[primIndex,2] == 1:
                endpose[2] = 9
            if endpose_angle45_vel3[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle45_vel3[primIndex,2] == 3:
                endpose[2] = 11
            endpose[3] = endpose_angle45_vel3[primIndex,3]
        if angleIndex == 11:
            endpose[0] = endpose_angle67p5_vel3[primIndex,0] * -1
            endpose[1] = endpose_angle67p5_vel3[primIndex,1] * -1
            if endpose_angle67p5_vel3[primIndex,2] == 2:
                endpose[2] = 10
            if endpose_angle67p5_vel3[primIndex,2] == 3:
                endpose[2] = 11
            if endpose_angle67p5_vel3[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel3[primIndex,3] 
        if angleIndex == 12:
            endpose[0] = endpose_angle90_vel3[primIndex,0]
            endpose[1] = endpose_angle90_vel3[primIndex,1] * -1
            if endpose_angle90_vel3[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle90_vel3[primIndex,2] == 4:
                endpose[2] = 12
            if endpose_angle90_vel3[primIndex,2] == 5:
                endpose[2] = 11
            endpose[3] = endpose_angle90_vel3[primIndex,3]
        if angleIndex == 13:
            endpose[0] = endpose_angle67p5_vel3[primIndex,0] 
            endpose[1] = endpose_angle67p5_vel3[primIndex,1] * -1
            if endpose_angle67p5_vel3[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle67p5_vel3[primIndex,2] == 3:
                endpose[2] = 13
            if endpose_angle67p5_vel3[primIndex,2] == 4:
                endpose[2] = 12
            endpose[3] = endpose_angle67p5_vel3[primIndex,3] 
        if angleIndex == 14:
            endpose[0] = endpose_angle45_vel3[primIndex,0] 
            endpose[1] = endpose_angle45_vel3[primIndex,1] * -1
            if endpose_angle45_vel3[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle45_vel3[primIndex,2] == 2:
                endpose[2] = 14
            if endpose_angle45_vel3[primIndex,2] == 3:
                endpose[2] = 13
            endpose[3] = endpose_angle45_vel3[primIndex,3]
        if angleIndex == 15:
            endpose[0] = endpose_angle22p5_vel3[primIndex,0] 
            endpose[1] = endpose_angle22p5_vel3[primIndex,1] * -1
            if endpose_angle22p5_vel3[primIndex,2] == 0:
                endpose[2] = 0
            if endpose_angle22p5_vel3[primIndex,2] == 1:
                endpose[2] = 15
            if endpose_angle22p5_vel3[primIndex,2] == 2:
                endpose[2] = 14
            endpose[3] = endpose_angle22p5_vel3[primIndex,3]

        # generate intermediate x,y positions w.r.t 0,0 (in meters)
        numSamples = 10
        intermPoses_m = np.zeros((numSamples,2))
        endPose_m = [endpose[0]*resolution, endpose[1]*resolution]
        # turn in place or move straight forward
        if (endpose[0]==0 and endpose[1]==0) or endpose[2]==angleIndex:
            for i in range(0,numSamples):
                intermPoses_m[i,:] = [endPose_m[0]*i/(numSamples-1), endPose_m[1]*i/(numSamples-1)]
        else:
            R = [[math.cos(angleIndex*2*math.pi/numAngles), math.sin(endpose[2]*2*math.pi/numAngles) - math.sin(angleIndex*2*math.pi/numAngles)], \
                 [math.sin(angleIndex*2*math.pi/numAngles), -math.cos(endpose[2]*2*math.pi/numAngles) - math.cos(angleIndex*2*math.pi/numAngles)]]
            S = np.matmul(np.linalg.pinv(R), [[endPose_m[0]],[endPose_m[1]]])
            l = S[0]
            tvoverrv = S[1]
            rv = endpose[2]*2*math.pi/numAngles + 1/tvoverrv
            tv = tvoverrv*rv

            for i in range(0,numSamples):
                dt = i/numSamples
                theta = angleIndex*2*math.pi/numAngles
                if dt*tv < 1:
                    intermPoses_m[i,:] = [dt*tv*math.cos(theta), \
                                          dt*tv*math.sin(theta)]
                else:
                    dtheta = rv*(dt - 1/tv) + theta
                    intermPoses_m[i,:] = [l*math.cos(theta) + tvoverrv*(math.sin(dtheta)-math.sin(theta)), \
                                          l*math.sin(theta) - tvoverrv*(math.cos(dtheta)-math.cos(theta))]
            
            errorxy = [endPose_m[0] - intermPoses_m[numSamples-1,0], \
                       endPose_m[1] - intermPoses_m[numSamples-1,1]]
            interp = np.zeros(10)
            for i in range(0,numSamples):
                interp[i] = i/(numSamples-1)
            intermPoses_m[:,0] = intermPoses_m[:,0] + errorxy[0]*interp
            intermPoses_m[:,1] = intermPoses_m[:,1] + errorxy[1]*interp

        # calculate primitive execution time (assuming straight line
        r = math.sqrt(endPose_m[0]**2 + endPose_m[1]**2)
        primExecTime = 2*r/(vel3 + endpose[3]*0.2)

        primFile = open(filename, 'a')
        primFile.write("startangle_i: %d\n" % (angleIndex))
        primFile.write("startvel_i: 3\n")
        primFile.write("endpose_i: %d %d %d %d\n" % (endpose[0],endpose[1],endpose[2],endpose[3]))
        primFile.write("exectime: %.4f\n" % (primExecTime))
        primFile.write("intermediateposes: %d\n" % (numSamples))
        for i in range(0,numSamples):
            primFile.write("%.4f %.4f\n" % (intermPoses_m[i,0], intermPoses_m[i,1]))
