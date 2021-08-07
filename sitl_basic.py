from dronekit_sitl import SITL
from dronekit import connect, VehicleMode, Command, LocationGlobalRelative
from pymavlink import mavutil
import distance_functions
import time
from common import *
from collections import defaultdict
import sys
import threading
import rospy

vehicle      = {}
BASE_PORT    = 5760
ASSET_TYPE   = 'actualUAV' #  SITLapm
CONTROL_MODE = CTRL_MODE_GUIDED
MISSION_RATE = 5.0

DEVICE = '/dev/ttyAMA0'
BAUD = 921600

# FIXME -- This should be a response from the GCS.
# These coordinates cover most of the Buffalo region...way too big to be useful!
GEOFENCE = [[43.086441866511805, -78.84681701660156], 
            [42.98003720570486, -78.84475708007812], 
            [42.97953485978903, -78.68133544921876], 
            [43.090453524374716, -78.68545532226564]]
 


def make_dict():
	return defaultdict(make_dict)

def geoIsPointInPoly(loc, poly):
	"""
	Determine if a point is inside a polygon.  Points that are along the perimeter of the polygon (including vertices) are considered to be "inside".

	Parameters
	----------
	loc: list
		The coordinate of the point, in [lat, lon] format
	poly: list of lists
		The polygon to check if the point is inside, in [[lat, lon], [lat, lon], ..., [lat, lon]] format
	Returns
	-------
	boolean
		The point is inside the polygon or not
	"""

	if (loc in poly):
		return True

	x = loc[1]
	y = loc[0]
	inside = False
	j = len(poly) - 1
	for i in range(0,len(poly)):
		# Check if pt is in interior:
		xi = poly[i][1]
		yi = poly[i][0]
		xj = poly[j][1]
		yj = poly[j][0]
		intersect = (yi > y) != (yj > y)
		if (intersect):
			intersect = (x < (xj - xi) * (y - yi) / float(yj - yi) + xi)
		if (intersect):
			inside = not inside
		j = i
		
	return inside

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
        self.createAsset(reqLat, reqLon, reqAlt, reqHeading)

        # print " Location: %s" % vehicle[vehicleIndex].location.global_frame
        # print " Battery: %s" % vehicle[vehicleIndex].battery
        # print " Last Heartbeat: %s" % vehicle[vehicleIndex].last_heartbeat
        # print " Is Armable?: %s" % vehicle[vehicleIndex].is_armable
        # print " System status: %s" % vehicle[vehicleIndex].system_status.state
        # print " Mode: %s" % vehicle[vehicleIndex].mode.name    # settable

        # Register Asset / Create UAV object
        self.uav = self.register_asset()
        print "Uav %d registered." % self.uav.uavID
        self.uav.status = STATUS_INIT

        # Create Targets
        # .....

        # Get Other Assets Info
        # .....

        # Mission Flags
        # self.missionStart = False
        self.hasMission = False
        # self.waitToUpdate = False
        # self.changingMode = False


        # Wait for mission / Load Mission
        # Manual
        while(not self.hasMission):
            msg = {
                'respAssetID': 0,
                'target_system': [0, 0],
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
                time.sleep(1)

        # monitor mission thread
        print('Starting mission thread...')
        missionThread = threading.Thread(target=self.monitorMission, args=(MISSION_RATE,))
        missionThread.start()
        print('DONE')
    
    def monitorMission(self, rate):
        
        # threadRate = rospy.Rate(rate)
        vehicleIndex = self.myVehicleIndex
        while(True):
            
            # while self.changingMode:
            #     pass
            
            # if self.gotNewMission:
            #     self.updatingMission = True
            #     self.waitToUpdate = False
            #     while self.updatingMission:
            #         print "while loop waiting"
            #         time.sleep(0.1)
            # self.waitToUpdate = True

            if self.uav.status == STATUS_INIT: # and self.missionStart
                self.armAndTakeoff(self.mission[self.uav.uavID][self.uav.currentSeqID].param7)

            if self.uav.status > 0:
                if self.uav.status == STATUS_TAKEOFF:
                    print "Alt: %f" % vehicle[vehicleIndex].location.global_relative_frame.alt
                    if (vehicle[vehicleIndex].location.global_relative_frame.alt + TAKEOFF_ALT_TOL >= self.uav.goalAlt):
                        print " Takeoff Completed"
                        print " %s" % vehicle[vehicleIndex].location.global_frame
                        self.uav.currentSeqID += 1
                        self.uav.status = STATUS_READY
                
                elif self.uav.status == STATUS_READY:
                    if self.uav.currentSeqID in self.mission[self.uav.uavID]:

                        if self.mission[self.uav.uavID][self.uav.currentSeqID].command == CMD_NAV_WP:

                            waitTime = self.mission[self.uav.uavID][self.uav.currentSeqID].param1
                            self.uav.goalAcptRad = self.mission[self.uav.uavID][self.uav.currentSeqID].param2
                            passRad = self.mission[self.uav.uavID][self.uav.currentSeqID].param3
                            yaw = self.mission[self.uav.uavID][self.uav.currentSeqID].param4
                            self.uav.goalLat = self.mission[self.uav.uavID][self.uav.currentSeqID].param5
                            self.uav.goalLon = self.mission[self.uav.uavID][self.uav.currentSeqID].param6
                            self.uav.goalAlt = self.mission[self.uav.uavID][self.uav.currentSeqID].param7
                            
                            self.uav.status = STATUS_WP
                            vehicle[vehicleIndex].simple_goto(LocationGlobalRelative(self.uav.goalLat, self.uav.goalLon, self.uav.goalAlt), groundspeed=10)
                            
                            self.mission[self.uav.uavID][self.uav.currentSeqID].status = 0

                            # vehicle[vehicleIndex].groundspeed = 30
                        elif self.mission[self.uav.uavID][self.uav.currentSeqID].command == CMD_LAND:
                            
                            msg = vehicle[vehicleIndex].message_factory.command_long_encode(
                                self.uav.uavID, 0, 
                                mavutil.mavlink.MAV_CMD_NAV_LAND,
                                0,
                                0,0,0,0, # param 1-4 ignored
                                0,
                                0,
                                self.mission[self.uav.uavID][self.uav.currentSeqID].param7,
                            )
                            vehicle[vehicleIndex].send_mavlink(msg)
                            
                            self.uav.status = STATUS_LANDING

                            self.mission[self.uav.uavID][self.uav.currentSeqID].status = 0

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
                    # print "AirSpeed: %s" % vehicle[vehicleIndex].airspeed
                    # print "Velocity: %s" % vehicle[vehicleIndex].velocity
                    
                    dist2goal = distance_functions.getGPSdistance3D(lat1deg, lon1deg, alt1m, lat2deg, lon2deg, alt2m)
                    
                    print "Distance to Goal: %f" % dist2goal
                    print "--------------------------------------------------"
                    if (dist2goal <= self.uav.goalAcptRad):
                        print "UAV %d: Completed WP task (seqID %d)" % (self.uav.uavID, self.uav.currentSeqID)
                        #self.logfile.write("%s, info, Completed WP task (seqID %d).  dist2goal = %f.  goalAcptRad = %f. \n" % (datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H:%M:%S'), self.uav.currentSeqID, dist2goal, self.uav.goalAcptRad))
                        self.mission[self.uav.uavID][self.uav.currentSeqID].status = -1
                        self.uav.currentSeqID += 1	
                        self.uav.status = STATUS_READY
                
                elif self.uav.status == STATUS_LANDING:
                    print "Alt: %f" % vehicle[vehicleIndex].location.global_relative_frame.alt
                    dist2target = vehicle[vehicleIndex].location.global_relative_frame.alt
                    if dist2target < LAND_ALT_TOL:
                        print " Landing Completed."
                        print " %s" % vehicle[vehicleIndex].location.global_frame
                        self.mission[self.uav.uavID][self.uav.currentSeqID].status = -1
                        self.uav.currentSeqID += 1
                        self.uav.status = STATUS_READY

            time.sleep(1)
            # threadRate.sleep()

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
        # Future FIXME -- Add a time limit (e.g., GET_HOME_LIMIT = 30 seconds)
        # Future FIXME -- Publish to a ROS topic to indicate our status.

        try:
            while not vehicle[vehicleIndex].home_location:
                if (vehicle[vehicleIndex].gps_0.fix_type < 3):
                    print " Insufficient GPS fix_type, ", vehicle[vehicleIndex].gps_0.fix_type
                    print " satellites_visible = ", vehicle[vehicleIndex].gps_0.satellites_visible;
                else:
                    print " Got 3D GPS Fix"
                    print " satellites_visible = ", vehicle[vehicleIndex].gps_0.satellites_visible;

                    # We need the following 3 lines to be able to get the home_location:
                    cmds = vehicle[vehicleIndex].commands
                    cmds.download()
                    cmds.wait_ready()
                    if not vehicle[vehicleIndex].home_location:
                        print " Waiting for home location..."
                        if not vehicle[vehicleIndex].location.global_frame.alt:
                            print " Waiting for global alt"
                        else:    
                            vehicle[vehicleIndex].home_location = vehicle[vehicleIndex].location.global_frame
                            print "\n Home location: %s" % vehicle[vehicleIndex].home_location
                            if (not geoIsPointInPoly([vehicle[vehicleIndex].home_location.lat, vehicle[vehicleIndex].home_location.lon], GEOFENCE)):
                                print " ERROR: Home location is outside geofence."
                                vehicle[vehicleIndex].home_location = None

                time.sleep(1)

        except:
            print "Could not get home location."
            e = sys.exc_info()[1]
            print(e)
            exit()

    def register_asset(self):
        self.isRegistered = True
        self.missionStart = True

        vi = self.myVehicleIndex
        return make_uav(
                    ASSET_TYPE,
                    self.myVehicleIndex,
                    -1,
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

            # while self.waitToUpdate:
            #     print "mission waiting"
            #     time.sleep(1)
            
            tmpAssetIDs = set(msg['target_system'])

            for assetID in tmpAssetIDs:
                self.mission[assetID] = {}
                
                if assetID == self.uav.uavID:
                    self.uav.currentSeqID = -1
            
            targetID = -1
            targetVertex = -1

            foundCurrent = False
            for i in range(0, len(msg['target_system'])):
                tmpCmd = msg['command'][i]
                tmpParam2 = msg['param2'][i]
                
                if msg['target_system'][i] == self.uav.uavID:
                    if not foundCurrent and msg['status'] >= 0:
                        foundCurrent = True
                        self.uav.currentSeqID = msg['seq'][i]

                    self.hasMission = True

                self.mission[msg['target_system'][i]][msg['seq'][i]] = make_mission(msg['target_component'][i], tmpCmd, msg['frame'][i], msg['current'][i], msg['status'][i], msg['autocontinue'][i], targetID, targetVertex, msg['param1'][i], tmpParam2, msg['param3'][i], msg['param4'][i], msg['param5'][i], msg['param6'][i], msg['param7'][i])

            self.gotNewMission = False
            self.updatingMission = False

    def armAndTakeoff(self, targetAltMeters):
        # Arm and Takeoff
        vehicleIndex = self.myVehicleIndex
        print ">> Pre-arm checks"
        while not vehicle[vehicleIndex].is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(0.5)

        while not vehicle[vehicleIndex].mode.name=='GUIDED':
            vehicle[vehicleIndex].mode = VehicleMode("GUIDED")
        
        print " Mode: %s" % vehicle[vehicleIndex].mode.name
        
        print ">> Disabling pre-arm checks"
        vehicle[vehicleIndex].parameters["ARMING_CHECK"] = 0
        time.sleep(0.5)

        print ">> Arming motors"
        vehicle[vehicleIndex].armed = True
        while not vehicle[vehicleIndex].armed:
            print " Waiting for arming..."
            time.sleep(0.5)

        print " Vehicle is armed: %s" % vehicle[vehicleIndex].armed

        print "Taking off from current alt of %f to target alt of %f" % ((vehicle[vehicleIndex].location.global_relative_frame.alt, targetAltMeters))
        vehicle[vehicleIndex].simple_takeoff(targetAltMeters)
        
        self.uav.goalAlt = targetAltMeters
        self.uav.goalLat = self.uav.latHome
        self.uav.goalLon = self.uav.lonHome 
        self.uav.status = STATUS_TAKEOFF

if __name__ == "__main__":
    sitlSim()
    
