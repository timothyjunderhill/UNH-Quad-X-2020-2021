from digi.xbee.devices import XBeeDevice
from digi.xbee.devices import RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
import time
import numpy
import struct

#MAC address of the XBees on the quadcopters (can be found using the Digi XCTU application) used for direct communication
quadOne   = '0013A2004099C764'  #agentNum: 0
quadTwo   = '0013A200409AECE0'  #agentNum: 1 
quadThree = '0000000000000000'  #agentNum: 2 
quadFour  = '0000000000000000'  #agentNum: 3
quadFive  = '0000000000000000'  #agentNum: 4 
quadSix   = '0000000000000000'  #agentNum: 5 
quadSeven = '0000000000000000'  #agentNum: 6
quadEight = '0000000000000000'  #agentNum: 7 

#Combine these addresses into a list with QUAD_ID[0] corresponding to quadOne etc.
QUAD_ID = [quadOne, quadTwo, quadThree, quadFour, quadFive, quadSix, quadSeven, quadEight]


#-------------------------------------------------------
# Setup
# (Taken from the swarm.py code on github)
#-------------------------------------------------------
# X1-X8 hold the XYZ locations of each quadcopter in the swarm ([X,Y,Z])
# Initialized as floating point numbers 
X1 = [0.0, 0.0, 0.0]
X2 = [0.0, 0.0, 0.0]
X3 = [0.0, 0.0, 0.0]
X4 = [0.0, 0.0, 0.0]
X5 = [0.0, 0.0, 0.0]
X6 = [0.0, 0.0, 0.0]
X7 = [0.0, 0.0, 0.0]
X8 = [0.0, 0.0, 0.0]

#Combine these lists into a 8x3 array with AGENT_LOC[0] corresponding to quadOne etc.
AGENT_LOC = numpy.array([X1])
AGENT_LOC = numpy.vstack([AGENT_LOC, X2])
AGENT_LOC = numpy.vstack([AGENT_LOC, X3])
AGENT_LOC = numpy.vstack([AGENT_LOC, X4])
AGENT_LOC = numpy.vstack([AGENT_LOC, X5])
AGENT_LOC = numpy.vstack([AGENT_LOC, X6])
AGENT_LOC = numpy.vstack([AGENT_LOC, X7])
AGENT_LOC = numpy.vstack([AGENT_LOC, X8])

#The next location of each agent in the swarm as determined by the swarm algorithm
next1 = [42.892639,-71.373986,162.000000]   #Random values for testing
next2 = [42.892639,-71.373986,162.000000]
next3 = [0.0,0.0,0.0]
next4 = [0.0,0.0,0.0]
next5 = [0.0,0.0,0.0]
next6 = [0.0,0.0,0.0]
next7 = [0.0,0.0,0.0]
next8 = [0.0,0.0,0.0]

#Combine these lists into a 8x3 array with NEXT_LOC[0] corresponding to quadOne etc.
NEXT_LOC = numpy.array([next1])
NEXT_LOC = numpy.vstack([NEXT_LOC, next2])
NEXT_LOC = numpy.vstack([NEXT_LOC, next3])
NEXT_LOC = numpy.vstack([NEXT_LOC, next4])
NEXT_LOC = numpy.vstack([NEXT_LOC, next5])
NEXT_LOC = numpy.vstack([NEXT_LOC, next6])
NEXT_LOC = numpy.vstack([NEXT_LOC, next7])
NEXT_LOC = numpy.vstack([NEXT_LOC, next8])

#Initializing the XBee device connected to the Groundstation
#NOTE: "COM" value will change with connected computer. Check that this is the correct value for your device before attempting to run code
groundStation = XBeeDevice("COM26", 9600)  #Set up XBee device connected to the computer 
groundStation.open()                       #Open the serial connection
groundStation.set_sync_ops_timeout(10)     # 10 second timeout








#-----------------------------------------------------------
# Send Coordinates Function
#-----------------------------------------------------------
"""
This function is designed to send a single, 3-number coordinate set (pulled from the corresponding row in the NEXT_LOC array) to the specified agent.

The calculations for determining what this location should be using the current locations of all agents in the swarm is assumed to be contained in a separate and as-yet-unwritten function
"""
def SendNewCoords(swarm_control_signal, agentNum):
    #Set up the remote device object for the specified agent
    remote_device = RemoteXBeeDevice(groundStation, XBee64BitAddress.from_hex_string(QUAD_ID[agentNum]))
    
    #Compile the 3-number coordinate set for the specified agent into a single string with each number delimited by a comma
    txPayload = numpy.array2string(swarm_control_signal[agentNum], separator=',')
    
    #For monitoring: 
    print("Sent: ", txPayload)

    #Send the data to the remote_device object defined above
    groundStation.send_data(remote_device, txPayload)
    







#-----------------------------------------------------------
# Data Receive Callback Function
#-----------------------------------------------------------
"""
This function will run automatically when a data packet is available from any agent in the swarm.  
It will pull in the data, parse it into a 3-number coordinate set and update the values in the corresponding row of the AGENT_LOC array.

This XBee library also supports polling methods of data retreival rather than using a callback if we feel this is the better way to go
"""
def update_agent_loc(xbee_message):

    # The data sent from the Teensy-connected XBees is formatted as an array of unsigned chars so the unpack function doesn't work well in this situation.  
    # So, the data is first read and converted into a string...
    rxPayload = xbee_message.data.decode("utf8")
    # Then split into an list of 4 strings with the first containing the agent number sent from the quadcopter and the remaining 3 each containing 1 coordinate value...
    intermediate_array = rxPayload.split()
    # The agent number is translated into an integer for use in array indexing...
    agent_Num = int(intermediate_array[0])

    # And finally the 3 coordinate values are converted into floats and stored in the row of the AGENT_LOC array indicated by the agent number received from the quadcopter
    for i in range(3):
        AGENT_LOC[agent_Num][i] = float(intermediate_array[i+1])
    #For Monitoring: 
    print("Positions: ")
    print(AGENT_LOC)

# This Line sets up the callback functionality of the receive function
groundStation.add_data_received_callback(update_agent_loc)








#---------------------------------------------------------------------
# Code to call the functions for tesing
#---------------------------------------------------------------------
while True:
    
    SendNewCoords(NEXT_LOC,0)
    time.sleep(0.01)
    SendNewCoords(NEXT_LOC,1)
    try:
        time.sleep(1)
    except KeyboardInterrupt:
        break

# Close serial connection with the groundstation XBee
groundStation.close()

