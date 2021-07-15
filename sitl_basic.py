from dronekit_sitl import SITL
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import distance_functions
import time
from common import *
from collections import defaultdict
import sys


vehicle      = {}
BASE_PORT    = 5760
ASSET_TYPE   = 'SITLapm'
CONTROL_MODE = CTRL_MODE_GUIDED

def make_dict():
	return defaultdict(make_dict)

class make_uav():
    def __init__(self, assetType, uavID, swarmID, latInit, lonInit, altInit, headingInit, vehicleIndex, controlMode):
        self.uavID = uavID
        self.swarmID = swarmID
        self.status					= STATUS_PRE_MISSION
        self.vehicleIndex			= vehicleIndex
		
        self.latHome				= latInit
        self.lonHome				= lonInit
        self.altHome				= altInit		# [meters] above MSL

        self.controlMode			= controlMode
        self.steering				= 0
        self.throttle				= 0
		
        self.roiSet					= False
        self.goalTargetID			= None
        self.goalTargetVertex		= None
        self.goalLat				= None
        self.goalLon				= None
        self.goalAlt				= None			# [meters] above home
        self.goalAcptRad			= None			# Acceptance radius [meters] of a sphere around the target.
		
        self.currentSeqID			= -1			# Initialize to dummy value.  Updated when we receive a mission.
        self.setReadyTime			= 0.0			# rospy timestamp.  When did we enter "STATUS_READY" status?  Initialize to dummy value.

class make_mission():
    def __init__(self, target_component, command, frame, current, status, autocontinue, targetID, targetVertex, param1=None, param2=None, param3=None, param4=None, param5=None, param6=None, param7=None):
        # will be using MISSION_ITEM message http://mavlink.org/messages/common#MISSION_ITEM
		self.target_component  	= int(target_component)
		self.command 			= int(command)
		self.frame 				= int(frame)							# IGNORED
		self.autocontinue 		= int(autocontinue)
		self.param1				= param1
		self.param2 			= param2
		self.param3				= param3	
		self.param4				= param4
		self.param5				= param5
		self.param6				= param6
		self.param7				= param7
		self.status				= int(status)			# FIXME:  -1 --> expired, 0 --> in progress, 1 --> future
		self.isCurrent			= int(current)			# NOT REALLY USED HERE...Should be within the asset class?		

		self.targetID			= int(targetID)			# What is the target assoc. with this task?  -1 --> n/a or unknown
		self.targetVertex		= int(targetVertex)		# What is the index of this target (e.g., polygons or lines)? -1 --> n/a or unknown
		
		# NOTE: The targetID and targetVertex are really intended for use in *autonomous* missions.
		#		This will allow our UAV to make adjustments to the route if a target is moved.
		#		For missions provided by ground control, we're just following orders (i.e., use -1 and -1).

		self.availRewards		= []					# We'll update over time.
		self.creditedRewards	= []
		self.verticesHit		= []

class sitlSim():
    def __init__(self):
        self.uav = None
        self.sitls = {}
        self.sitl = None
        self.mission = defaultdict(make_dict)
        self.otherAsset    = {}			# Store info about other UAVs/assets in the network
        self.activeTargets = set()
        self.hitTargets    = set()
        self.target        = {}
        self.hub_connection_strings = {}  
        self.SITL_assetIDs = []			# FIXME -- DO WE NEED THIS?
        self.HITL_assetIDs = []			# FIXME -- DO WE NEED THIS?
        self.children      = {}			# FIXME -- DO WE NEED THIS?
        self.mission       = defaultdict(make_dict)
        # self.swarmGoal = []
        # self.enemyGoal = []
        # self.bogey     = {}

        self.myVehicleIndex = 0
        reqCtrlMode         = CONTROL_MODE
        self.team           = 'red'
        reqAssetID          = 1001

        if ASSET_TYPE == 'actualUAV':
            reqLat              = None 
            reqLon              = None 
            reqAlt              = None 
            reqHeading          = None 
             
        else: 
            reqLat              = 43.001932358569476 
            reqLon              = -78.78695011138917 
            reqAlt              = 0.0 
            reqHeading          = 0.0

        vehicleIndex = self.myVehicleIndex
        
        # Create Asset / Connect drone
        reqLat, reqLon, reqAlt, reqHeading = self.createAsset(reqLat, reqLon, reqAlt, reqHeading)

        # print " Location: %s" % vehicle[vehicleIndex].location.global_frame
        # print " Battery: %s" % vehicle[vehicleIndex].battery
        # print " Last Heartbeat: %s" % vehicle[vehicleIndex].last_heartbeat
        # print " Is Armable?: %s" % vehicle[vehicleIndex].is_armable
        # print " System status: %s" % vehicle[vehicleIndex].system_status.state
        # print " Mode: %s" % vehicle[vehicleIndex].mode.name    # settable

        # Register Asset / Create UAV object
        self.uav = self.register_asset()
        self.uav.status = STATUS_INIT

        # Create Targets
        # .....

        # Get Other Assets Info
        # .....

        # Mission Flags
        self.missionStart = False
        self.hasMission = False
        self.waitToUpdate = False
        self.receivedMission = False
        self.changingMode = False


        # Wait for mission / Load Mission
        # Manual
        while(not self.hasMission):
            msg = {
                'respAssetID': 0,
                'target_system': [1001, 1001],
                'target_component': [0, 0],
                'seq': [1, 2],
                'frame': [0, 0],
                'command': [CMD_TAKEOFF, CMD_LAND],
                'current': [1, 1],
                'autocontinue': [1, 1],
                'param1': [0, 0],
                'param2': [0, 0],
                'param3': [0, 0],
                'param4': [0, 0],
                'param5': [reqLat, reqLat],
                'param6': [reqLon, reqLon],
                'param7': [2, 0.25],
                'status': [1, 1]
            }
            self.mission_info(msg)

            if not self.hasMission:
                time.sleep(2.0)
        
        # self.mission[self.uav.uavID] = {
        #     1: {'command': CMD_TAKEOFF, 'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0, 'param5': latInit, 'param6': lonInit, 'param7': altInit},
        #     2: {'command': CMD_NAV_WP, 'param1': 20, 'param2': 30, 'param3': 0, 'param4': 0, 'param5': 43.00013371535764, 'param6': -78.78793387603166 , 'param7': 145},
        #     3: {'command': CMD_NAV_WP, 'param1': 20, 'param2': 30, 'param3': 0, 'param4': 0, 'param5': 42.99995385103645, 'param6': -78.78793387603166 , 'param7': 145}
        # }

        self.uav.currentSeqID = 1

        while(True):
            
            while self.changingMode:
                pass

            if self.uav.status == STATUS_INIT:
                # Arm and Takeoff
                print ">> Pre-arm checks"
                while not vehicle[vehicleIndex].is_armable:
                    print " Waiting for vehicle to initialise..."
                    time.sleep(1)

                while not vehicle[vehicleIndex].mode.name=='GUIDED':
                    vehicle[vehicleIndex].mode = VehicleMode("GUIDED")
                
                print " Mode: %s" % vehicle[vehicleIndex].mode.name
                
                print ">> Disabling pre-arm checks"
                vehicle[vehicleIndex].parameters["ARMING_CHECK"] = 0
                time.sleep(1)

                print ">> Arming motors"
                vehicle[vehicleIndex].armed = True
                while not vehicle[vehicleIndex].armed:
                    print " Waiting for arming..."
                    time.sleep(0.5)

                print " Vehicle is armed: %s" % vehicle[vehicleIndex].armed

                targetAlt = 5
                print "Taking off from current alt of %f to target alt of %f" % ((vehicle[vehicleIndex].location.global_relative_frame.alt, targetAlt))
                vehicle[vehicleIndex].simple_takeoff(targetAlt)
                
                self.uav.goalAlt = 5 

                self.uav.status = STATUS_TAKEOFF

            if self.uav.status > 0:
                if self.uav.status == STATUS_TAKEOFF:
                    print "Alt: %f" % vehicle[vehicleIndex].location.global_relative_frame.alt
                    if (vehicle[vehicleIndex].location.global_relative_frame.alt + TAKEOFF_ALT_TOL >= self.uav.goalAlt):
                        print " Takeoff Completed"
                        print " Location: %s" % vehicle[vehicleIndex].location.global_frame
                        self.uav.currentSeqID += 1
                        self.uav.status = STATUS_READY
                
                elif self.uav.status == STATUS_READY:
                    if self.uav.currentSeqID in self.mission[self.uav.uavID]:
                        if self.mission[self.uav.uavID][self.uav.currentSeqID]['command'] == CMD_NAV_WP:

                            waitTime = self.mission[self.uav.uavID][self.uav.currentSeqID]['param1']
                            self.uav.goalAcptRad = self.mission[self.uav.uavID][self.uav.currentSeqID]['param2']
                            passRad = self.mission[self.uav.uavID][self.uav.currentSeqID]['param3']
                            yaw = self.mission[self.uav.uavID][self.uav.currentSeqID]['param4']
                            self.uav.goalLat = self.mission[self.uav.uavID][self.uav.currentSeqID]['param5']
                            self.uav.goalLon = self.mission[self.uav.uavID][self.uav.currentSeqID]['param6']
                            self.uav.goalAlt = self.mission[self.uav.uavID][self.uav.currentSeqID]['param7']

                            vehicle[vehicleIndex].simple_goto(LocationGlobalRelative(self.uav.goalLat, self.uav.goalLon, self.uav.goalAlt), groundspeed=10)
                            self.uav.status = STATUS_WP

                            # vehicle[vehicleIndex].groundspeed = 30

                elif self.uav.status == STATUS_WP:

                    lat1deg = self.uav.goalLat
                    lon1deg = self.uav.goalLon
                    alt1m = self.uav.goalAlt

                    lat2deg = vehicle[vehicleIndex].location.global_relative_frame.lat
                    lon2deg = vehicle[vehicleIndex].location.global_relative_frame.lon 
                    alt2m = vehicle[vehicleIndex].location.global_relative_frame.alt

                    # print "Lat: %f, Lon: %f, Alt: %f" % (lat2deg, lon2deg, alt2m)
                    print "SPEED: %s" % vehicle[vehicleIndex].parameters['WPNAV_SPEED']
                    print "GroundSpeed: %s" % vehicle[vehicleIndex].groundspeed
                    print "AirSpeed: %s" % vehicle[vehicleIndex].airspeed
                    print "Velocity: %s" % vehicle[vehicleIndex].velocity
                    
                    dist2goal = distance_functions.getGPSdistance3D(lat1deg, lon1deg, alt1m, lat2deg, lon2deg, alt2m)
                    
                    print "Distance to Goal: %f" % dist2goal
                    print "--------------------------------------------------"
                    if (dist2goal <= self.uav.goalAcptRad):
                        print "UAV %d: Completed WP task (seqID %d)" % (self.uav.uavID, self.uav.currentSeqID)
                        #self.logfile.write("%s, info, Completed WP task (seqID %d).  dist2goal = %f.  goalAcptRad = %f. \n" % (datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S'), self.uav.currentSeqID, dist2goal, self.uav.goalAcptRad))

                        self.uav.currentSeqID += 1	
                        self.uav.status = STATUS_READY

            time.sleep(1)
    
    def createAsset(self, reqLat, reqLon, reqAlt, reqHeading):
        if ASSET_TYPE == 'actualUAV':
            self.connectReal()
        else:
            self.connectSim(reqLat, reqLon, reqAlt, reqHeading)

    def connectReal(self):
        vehicleIndex = self.myVehicleIndex
        print "Connecting to vehicle on %s, baud=%d..." % (DEVICE, BAUD)
        try:
            vehicle[vehicleIndex] = connect(DEVICE, baud=BAUD, wait_ready=True)
            print "Connected"
        except:
            print "Could not connect to vehicle"
            exit()

        # self.clearCmds(True) ADD FUNCTION
        self.getHomeLocation(vehicleIndex)

    def connectSim(self, latInit, lonInit, altInit, headingInit):
        vehicleIndex = self.myVehicleIndex
        try:
            self.sitl = SITL()
            self.sitl.download('copter', '3.3', verbose=True)
            sitl_args = ['-I' + str(vehicleIndex), '--gimbal', '--model', 'quad', '--home='+str(latInit)+','+str(lonInit)+','+str(altInit)+','+str(headingInit)]
            self.sitls[vehicleIndex] = [self.sitl, sitl_args]

            self.sitl.launch(sitl_args, await_ready=True)

            # Each UAV will have a different SITL port:
            port = BASE_PORT + 10*vehicleIndex
            self.hub_connection_strings[vehicleIndex] = 'tcp:127.0.0.1:' + str(port)

            connection_string = self.hub_connection_strings[vehicleIndex]
            print "Connecting to vehicle on: %s" % connection_string
            vehicle[vehicleIndex] = connect(connection_string, wait_ready=True, source_system=1)

            while vehicle[vehicleIndex].parameters["SYSID_THISMAV"] != vehicleIndex:
                print("%d: Resetting its SYID_THISMAV to %d" % (vehicleIndex, vehicleIndex,))
                vehicle[vehicleIndex].parameters["SYSID_THISMAV"] = vehicleIndex
                time.sleep(0.1)
                print("%d: Allowing time for parameter write" % (vehicleIndex,))
                time.sleep(2)

                print "Connected"
        
        except:
            print "Could not connect to vehicle."
            exit()

        # self.clearCmds()
        # get home location
        self.getHomeLocation(vehicleIndex)

    def getHomeLocation(self, vehicleIndex):
        try:
            while not vehicle[vehicleIndex].home_location:
                cmds = vehicle[vehicleIndex].commands
                cmds.download()
                cmds.wait_ready()
                if not vehicle[vehicleIndex].home_location:
                    print " Waiting for home location..."
                
                print "\n Home location: %s" % vehicle[vehicleIndex].home_location

                while not vehicle[vehicleIndex].location.global_frame.alt:
                    print " Waiting for global alt"
                    time.sleep(1)
                
                vehicle[vehicleIndex].home_location = vehicle[vehicleIndex].location.global_frame

                print "\n Home location: %s" % vehicle[vehicleIndex].home_location
        except:
            print "Could not get home location."
            exit()

    def register_asset(self):
        self.isRegistered = True
        self.missionStart = True

        vi = self.myVehicleIndex
        return make_uav(self.myVehicleIndex,
                    vehicle[vi].location.global_relative_frame.lat, 
                    vehicle[vi].location.global_relative_frame.lon, 
                    vehicle[vi].location.global_frame.alt, 
                    vehicle[vi].heading, 
                    vi, 
                    CONTROL_MODE # msg.controlMode
                )

    def mission_info(self, msg):
        # We're only going to listen for mission updates that either
		# a) Come from a push from GCS (respAssetID == 0), or
		# b) Come because we requested a mission (respAssetID == self.uav.uavID)
        if msg['respAssetID'] in [0, self.uav.uavID]:
            self.gotNewMission = True

            while self.waitToUpdate:
                print "mission waiting"
                time.sleep(1)
            
            tmpAssetIDs = set(msg['target_system'])

            for assetID in tmpAssetIDs:
                self.mission[assetID] = {}
                



if __name__ == "__main__":
    sitlSim()
    