# =================================================================
# PUT GLOBAL VARIABLES HERE
STATUS_KILLED       = -5
STATUS_PRE_MISSION 	= -4
STATUS_INIT			= -3 	# SITL pre-arm
STATUS_ARMING		= -2
STATUS_OOS			= -1 	# Out of Service.  This uav isn't available to do anything.
STATUS_ARMED		=  0
STATUS_TAKEOFF		=  1 	# Taking off.  Only emergency commands will be followed.
STATUS_RTL			=  2 	# Returning to launch point.  Only emergency commands will be followed.
STATUS_LANDING		=  3 	# Landing (decrease alt).  Only emergency commands will be followed.
STATUS_WP			=  4 	# Traveling to a waypoint.
STATUS_READY		=  5 	# Available to receive any commands.  
STATUS_EMERGENCY	=  6 	# Performing collision avoidance maneuver.
STATUS_TELEOP		=  7	# Controlled via web interface or USB joystick
STATUS_AUTO			=  8	# Operating in autonomous mode?  FIXME -- Should this just be STATUS_WP?  We'd already know CTRL_MODE_AUTO or CTRL_MODE_SEMIAUTO.

CMD_ADD_UAV			= 0		# Create a SITL UAV and arm it
CMD_TAKEOFF			= 22	# Takeoff from where you are
CMD_VEL				= 2		# vx/vy/vz/heading command (velocity)
CMD_NAV_WP			= 16	# NAV_WAYPOINT command
CMD_RTL				= 4		# Return to Launch and land there
CMD_LAND			= 21	# Land (where you are)
CMD_AVOID_VXY 		= 6		# For collision avoidance.  Maneuver using vx/vy/vz/heading command.
CMD_AVOID_WP  		= 7		# For collision avoidance.  Maneuver using a dummy waypoint (not assoc. with a target)
CMD_ROI				= 8
CMD_CLEAR_ALL		= 45	# Deletes all mission commands
CMD_START_MISSION 	= 999	# FIXME -- This isn't a valid MAVlink number.  Tell assets that they can begin.
CMD_SET_CTRL_MODE	= 998	# FIXME -- This isn't a valid MAVlink number.  Change mode to guided, teleop, auto, or semiauto

SENSOR_STATUS_OOS_STOP = -1   # User initiated stopping of sensor
SENSOR_STATUS_OOS_KILL = -9   # Sensor was killed
SENSOR_STATUS_PAUSE    =  0   # Sensor is paused
SENSOR_STATUS_PLAY     =  1   # Sensor is running

SENSOR_CMD_PLAY        = 1
SENSOR_CMD_PAUSE       = 0
SENSOR_CMD_OOS_STOP    = -1
SENSOR_CMD_OOS_KILL    = -9


SWARM_MODE_NONE		= 0		# Swarm Mode: No swarming.  This is the initial behavior.
SWARM_MODE_MIRROR	= 1		# Swarm Mode: SITL UAVs should mimic the HITL parent's motion
SWARM_MODE_ROI		= 2		# Swarm Mode: SITL UAVs should maintain focus on ROI
SWARM_MODE_WATCHME	= 3		# Swarm Mode: SITL UAVs should maintain focus on HITL parent

CTRL_MODE_GUIDED	= 1		# Asset is using mission provided by GCS. 
CTRL_MODE_SEMIAUTO	= 2		# Asset uses GCS mission, but then does its own thing when there are no more mission commands.
CTRL_MODE_AUTO		= 3		# Asset generates its own mission.  It ignores any mission provided by GCS (except RTL/emergency commands).
CTRL_MODE_TELEOP	= 4		# Asset is manually controlled via GCS keypad or USB joystick.
CTRL_MODE_RC		= 5		# FIXME -- Do we need this?  This would be for HITL only?

'''
# How many times do we need to hit a target for it to be considered "done"?
TARGET_WP_HIT_THRESHOLD = 1			# WP -- Just one vertex.
TARGET_FR_HIT_THRESHOLD = 10		# FilledRegion -- Log a hit whenever inside polygon.
TARGET_L_HIT_THRESHOLD 	= 1			# Line -- Multiple vertices.  This is the threshold for each line segment.
TARGET_P_HIT_THRESHOLD 	= 1			# Perimeter -- Multiple vertices.  This is the threshold for each line segment.

# Target types are identified by ID numbers
TARGET_TYPE_WP 	= 1
TARGET_TYPE_FR 	= 2
TARGET_TYPE_P	= 3
TARGET_TYPE_L	= 4
'''

TARGET_ACTIVITY_VISIT    = 1
TARGET_ACTIVITY_STRIKE   = 2
TARGET_ACTIVITY_OBSERVE  = 3
TARGET_ACTIVITY_DEFEND   = 5


PLAY_SOUNDS = True

# =================================================================
# Tolerances
# FIXME -- THESE SHOULD BECOME ROS PARAMS.
# WE WANT TO BE ABLE TO EDIT THESE IN CESIUM.
# THE VALUES BELOW SHOULD JUST BE DEFAULTS.
MONITOR_ENEMY_COLLISIONS         = True
MONITOR_FRIENDLY_COLLISIONS_BLUE = True
MONITOR_FRIENDLY_COLLISIONS_RED  = True
COLLISION_WARN_TOL      = 20	# [meters]
FRIENDLY_COLLISION_TOL  = 5 	# [meters]
ENEMY_COLLISION_TOL     = 5 	# [meters]
TARGET_GPS_TOL 		    = 3		# [meters] 
TARGET_ALT_TOL 		    = -1	# [meters].  A negative value means we don't care about altitude.  FIXME -- What's a better way to do this? 
TAKEOFF_ALT_TOL		    = 1.0	# [meters]
LAND_ALT_TOL		    = 0.25	# [meters]
WAIT_READY_TOL 		    = 10 	# [seconds]. How long are we willing to wait in the "READY" status before issuing RTL command?

# =================================================================
# (Some of) These should be converted to ROS params
#TAKEOFF_SPEED 		= 10.0	# [m/s]
CRUISE_SPEED_UAV	= 30.0  # [m/s] 
CRUISE_SPEED_ROVER	=  0.25	# [m/s]
#LAND_SPEED			= 10.0 	# [m/s] 
#YAW_RATE			= 15.0	# [degrees/sec]
INIT_ALT			= 0.0 	# [m]	
TAKEOFF_ALT			= 10.0	# [m]
LOITER_ALT			= 15.0 	# [m]
FLIGHT_ALT			= 100.0	# [m]
LAND_ALT			= 0.0	# [m]
INIT_HDG			= 0.0	# [degrees]


'''
# =================================================================
THIS IS OLD:
	STATUS_PRE_MISSION	= -2	# FIXME -- Not sure if this is how we want to define this.
	STATUS_PRE_SITL	= -1	# We've defined the asset here, but SITL doesn't know about it.
	STATUS_INIT		= 0	# SITL pre-arm
	STATUS_TAKEOFF	= 1 # Taking off.  Only emergency commands will be followed.
	STATUS_RTL		= 2 # Returning to launch point.  Only emergency commands will be followed.
	STATUS_LANDING	= 3 # Landing (decrease alt).  Only emergency commands will be followed.
	STATUS_OOS		= 4 # Out of Service.  This asset isn't available to do anything.
	STATUS_READY	= 5 # Available to receive any commands (including swarm)
	
	CMD_VEL		= 1		# vs/vy/vz/heading command (velocity)
	CMD_WP		= 2		# NAV_WAYPOINT command
	CMD_RTL		= 3		# Return to Launch and land there
	CMD_LAND	= 4		# Land (where you are)
	CMD_TAKEOFF	= 5		# Takeoff from where you are
	CMD_SET_ROI	= 6		# Set the Region of Interest
	CMD_START_MISSION = 10		# Tell assets that they can begin
'''

