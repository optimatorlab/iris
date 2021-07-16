#!/usr/bin/env python

#from __future__ import division
#import urllib2
#import json
#import numpy
#from geopy.distance import great_circle
import math
import geopy.distance


# =================================================================
# RADIUS_OF_EARTH = 6378100.0	# [meters]
# =================================================================


# This file contains a number of functions for calculating time/distance to target.

# distance_functions.getGPSdistance(43, -79, 43, -78)

def getGPSdistance(lat1deg, lon1deg, lat2deg, lon2deg):
	"""
	Distance between two locations in 2D
	Parameters
	----------
	loc1: list
		First location, in [lat, lon]
	loc2: list
		Second location, in [lat, lon]
	
	Return
	------
	float
		Distance between to locations.
	"""
	
	distMeters = geopy.distance.distance([lat1deg, lon1deg], [lat2deg, lon2deg]).meters

	return distMeters
	

'''
def getGPSdistanceOLD(lat1deg, lon1deg, lat2deg, lon2deg):

	# NOTE:  The lat/lon values in the formula below are in units of ***[radians]***
	lat1rad = lat1deg*(math.pi/180.0)
	lon1rad = lon1deg*(math.pi/180.0)
	lat2rad = lat2deg*(math.pi/180.0)
	lon2rad = lon2deg*(math.pi/180.0)
	
	# Calculate Distance from point 1 to point 2, in [meters]:
	# This is a straight-line distance, ignoring altitude changes and turning
	distance = 2*RADIUS_OF_EARTH*math.asin( math.sqrt( pow(math.sin((lat2rad - lat1rad)/2.0),2) + math.cos(lat1rad)*math.cos(lat2rad)*pow(math.sin((lon2rad-lon1rad)/2.0),2) ))

	return (distance)
'''

def getGPSdistance3D(lat1deg, lon1deg, alt1m, lat2deg, lon2deg, alt2m):
	'''
	CAUTION:  ASSUMES FLAT EARTH
	'''
	
	# First, get the straight-line ground distance:
	straightDistance = getGPSdistance(lat1deg, lon1deg, lat2deg, lon2deg)
	
	# Now, use Pythagorean theorem to get distance including altitude changes:
	distance = math.sqrt ( pow(straightDistance,2) + pow((alt1m - alt2m),2) )
	
	return (distance)


def getHeading(latCurDeg, lonCurDeg, latGoalDeg, lonGoalDeg):
	# NOTE:  The lat/lon values in the formulas below are in units of ***[radians]***
	
	latCurRad = latCurDeg*(math.pi/180.0)
	lonCurRad = lonCurDeg*(math.pi/180.0)
	latGoalRad = latGoalDeg*(math.pi/180.0)
	lonGoalRad = lonGoalDeg*(math.pi/180.0)
	   
	# 1) What angle is required to travel directly from the current location to the goal location?
	#    See http://www.movable-type.co.uk/scripts/latlong.html
	y = math.sin(lonGoalRad - lonCurRad) * math.cos(latGoalRad)
	x = math.cos(latCurRad)*math.sin(latGoalRad) - math.sin(latCurRad)*math.cos(latGoalRad)*math.cos(lonGoalRad-lonCurRad)
	headingRad = (math.atan2(y, x) + 2*math.pi) % (2*math.pi)  # In the range [0,2*pi]
			   
	return (headingRad*(180/math.pi))
	
   
   
   
def pointInPoly(pt, poly):
	# This function returns True if a given point is inside a given polygon (False otherwise).
	
	# Reference:  # https://github.com/substack/point-in-polygon/blob/master/index.js

	# pt = [LAT, LON] 
	# poly is a list of lists.  Each sublist is [LAT, LON].
	
	# poly = [[43.001677, -78.787374], [43.001675, -78.786829], [43.00161, -78.786829], [43.001612, -78.786808], [43.001628, -78.78681], [43.001626, -78.786765], [43.001605, -78.786767], [43.00156, -78.786706], [43.001538, -78.786734], [43.00144, -78.786734], [43.001435, -78.78675], [43.001372, -78.786752], [43.001373, -78.787366], [43.001435, -78.78737], [43.001436, -78.787387], [43.001615, -78.787387], [43.001616, -78.787374]]

	# pt = [43.001591, -78.787042]	# INSIDE
	# pt = [43.001831, -78.787723]	# OUTSIDE

	x = pt[1]
	y = pt[0]

	inside = False

	j = len(poly) - 1
	for i in range(0,len(poly)):
		xi = poly[i][1]
		yi = poly[i][0]
		xj = poly[j][1]
		yj = poly[j][0]
		intersect = (yi > y) != (yj > y)
		#print intersect
		
		if (intersect):
			intersect = (x < (xj - xi)*(y - yi)/float(yj-yi) + xi)
			#print "\t", intersect
			
		if (intersect):
			inside = not inside
			
		j = i	
		
	return (inside)	
	


def pointInPolyLONLAT(pt, polyVertices):
	# This function returns True if a given point is inside a given polygon (False otherwise).
	
	# Reference:  # https://github.com/substack/point-in-polygon/blob/master/index.js

	# pt = [LAT, LON] 
	# polyVertices is a a single list, of the form [LON, LAT, LON, LAT, ...]  <-- NOTE, these are LON, LAT!!!
	
	x = pt[1]
	y = pt[0]

	inside = False

	# Build "poly" data structure, of the form [[lat0, lon0], [lat1, lon1], ...]:
	poly = []
	for i in range(0, len(polyVertices), 2):
		poly.append([polyVertices[i+1], polyVertices[i]])
				
	j = len(poly) - 1
	for i in range(0,len(poly)):
		xi = poly[i][1]
		yi = poly[i][0]
		xj = poly[j][1]
		yj = poly[j][0]
		intersect = (yi > y) != (yj > y)
		#print intersect
		
		if (intersect):
			intersect = (x < (xj - xi)*(y - yi)/float(yj-yi) + xi)
			#print "\t", intersect
			
		if (intersect):
			inside = not inside
			
		j = i	
		
	return (inside)	

	
def pathCrossesPoly(pathCoords, polyCoords, checkInterior):
	'''
	See https://stackoverflow.com/questions/563198/whats-the-most-efficent-way-to-calculate-where-two-line-segments-intersect
	
	This function returns true if the given path *touches* one or more boundaries of the polygon.
		If the path is colinear with a boundary, the function returns true.
		
		FIXME -- Should we modify this so it requires ***crossing*** the boundary?
		If so, I think we require 0 < s < 1 and 0 < t < 1.
	
	pathCoords = [latStart, lonStart, latEnd, lonEnd]
	polyCoords = [lon, lat, lon, lat, lon, lat, ...]
	checkInterior = True or False.  True --> See if either the start or end points of the path are within the poly.
	
	***NOTE***  polyCoords is in LON, LAT, ... order.  This allows us to use the sttr "vertices" data structure.
	
	Note 2:  We will assume a closed polygon.  If the first and last points aren't the same, we'll duplicate the first point.
	'''

	# print polyCoords
		
	if ([polyCoords[0], polyCoords[1]] != [polyCoords[-2], polyCoords[-1]]):
		# print "extend"
		polyCoords.extend([polyCoords[0], polyCoords[1]])	
		
	p0_x = pathCoords[1]
	p0_y = pathCoords[0]
	p1_x = pathCoords[3]
	p1_y = pathCoords[2]
	
	s1_x = p1_x - p0_x
	s1_y = p1_y - p0_y
	
	doesIntersect = False
	for i in range(0, len(polyCoords)-2, 2):
		p2_x = polyCoords[i]
		p2_y = polyCoords[i+1]
		
		s2_x = polyCoords[i+2] - polyCoords[i]
		s2_y = polyCoords[i+3] - polyCoords[i+1]

		denom = float(-s2_x * s1_y + s1_x * s2_y)

		if (denom != 0):	
			s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / denom
			t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / denom
			
			# print "\t\ts = ", s, "t = ", t
			
			if ((0 <= s <= 1) and (0 <= t <= 1)):
				doesIntersect = True

				# We can also find the intersection point (?):
				'''
				(FIXME -- This might fail if the path is coilinear to a boundary.)
				i_x = p0_x + (t * s1_x)
				i_y = p0_y + (t * s1_y)
				'''
								
				return (True)
			
	if (checkInterior):
		if (pointInPolyLONLAT([p0_y, p0_x], polyCoords)):
			# print "START INSIDE"
			return (True)
		elif (pointInPolyLONLAT([p1_y, p1_x], polyCoords)):
			# print "END INSIDE"
			return (True)

	# Otherwise, return False:	
	return (doesIntersect)	
		
	
	
def UAVlocations(lat1deg, lon1deg, d, n):
	'''
	FIXME -- What does this function do?
	'''
	
	UAVs_lon = []
	
	brng = (360/n)*(math.pi/180)
	print brng
	print lat1deg
	# NOTE:  The lat/lon values in the formula below are in units of ***[radians]***
	lat1rad = float(lat1deg)*(math.pi/180)
	lon1rad = float(lon1deg)*(math.pi/180)
	print lat1rad, lon1rad
	
	for i in range(0, n):
		UAVs_lat = []
		print (i+1)*brng
		lat2rad = math.asin((math.sin(lat1rad)*math.cos(d/RADIUS_OF_EARTH)) + (math.cos(lat1rad)*math.sin(d/RADIUS_OF_EARTH)*math.cos((i+1)*brng)))
		lon2rad = lon1rad + math.atan2((math.sin((i+1)*brng)*math.sin(d/RADIUS_OF_EARTH)*math.cos(lat1rad)),(math.cos(d/RADIUS_OF_EARTH)-math.sin(lat1rad)*math.sin(lat2rad)))
		#lon2rad = (lon2rad+540)%360-180
		lat2deg = lat2rad*(180/math.pi)
		lon2deg = lon2rad*(180/math.pi)
		UAVs_lat.append(lat2deg)
		UAVs_lat.append(lon2deg)
		#x,y = map(lon2deg,lat2deg)
		#map.plot(x,y,'bo',ms =10)
		UAVs_lon.append(UAVs_lat)

	return (UAVs_lon)


def minDistPt2Path(lat1, lon1, lat2, lon2, lat3, lon3):
	'''
	Calculate the minimum distance [meters] from a single stationary location (target) to any point along a path.
		lat1: Starting lat of path [degrees]
		lon1: Starting lon of path [degrees]
		
		lat2: Ending lat of path [degrees]
		lon2: Ending lon of path [degrees]
		
		lat3: lat of stationary location [degrees]
		lon3: lon of stationary location [degrees]
	'''
	groundSpeed = 1 # [m/s].  We're using unit speed.  See below for rationale.
	
	# Find the heading [degrees] required to go from start to end:
	hdgDeg = getHeading(lat1, lon1, lat2, lon2)

	# Calculate unit direction vector of the path in x and y directions:
	vx1 = math.sin(hdgDeg*(math.pi/180.0)) * groundSpeed
	vy1 = math.cos(hdgDeg*(math.pi/180.0)) * groundSpeed

	# Find x and y distances between the stationary location and the start point: 
	deltax0 = getGPSdistance(lat1, lon1, lat1, lon3)
	deltay0 = getGPSdistance(lat3, lon3, lat1, lon3)

	# Find the ground distance between the start and end points:
	Spath = getGPSdistance(lat1, lon1, lat2, lon2)			
	
	# Adjust sign:
	if (lat1 < lat3):
		# Target is N of start point
		deltay0 = -deltay0
	if (lon1 < lon3):
		# Target is E of start point
		deltax0 = -deltax0
	

	# Find the time at which the target is closest to the path:
	vx2 = 0.0		# Our target is stationary
	vy2 = 0.0		
	deltavx = vx1 - vx2 
	deltavy = vy1 - vy2

#	if (deltavx + deltavy != 0):
	if ( (abs(deltavx) > 0.001) and (abs(deltavy) > 0.001) ):
		tgo = -(deltax0 * deltavx + deltay0 * deltavy)/float(deltavx**2 + deltavy**2)
		tgo = min(tgo, Spath)			# We are limiting our focus to just points along the path.	
		# Our asset is moving at unit speed.  Thus, Spath is the distance and time (in [seconds]) from start to end point.
	else:
		# We're never closer than we are right now.
		tgo = 0	

	# Find the distance between the target and the path at time tgo:
	if (tgo <= 0):
		# The target is closest to the start point.
		S = getGPSdistance(lat1, lon1, lat3, lon3)		# Distance between target and start point
	else:
		# The target is closest to a point along the path:
		S = math.sqrt((deltax0 + deltavx*tgo)**2 + (deltay0 + deltavy*tgo)**2)

	return (S)		

		
def isPass(lat1, lon1, lat2, lon2, lat3, lon3, tolerance):
	'''
	Determine if any point along a path is within tolerance meters of a stationary point.
	(did our path pass by the target?)
		lat1: Starting lat of path [degrees]
		lon1: Starting lon of path [degrees]
		
		lat2: Ending lat of path [degrees]
		lon2: Ending lon of path [degrees]
		
		lat3: lat of stationary location [degrees]
		lon3: lon of stationary location [degrees]
		
		tolerance:  [meters]
	'''

	d = minDistPt2Path(lat1, lon1, lat2, lon2, lat3, lon3)	
	if (d <= tolerance):
		passed = True
	else:
		passed = False
		
	return (passed)		

def pointInDistance(lat, lon, alt, hdg, distMeters):
	
	[lat, lon, dummyAlt] = list(geopy.distance.distance(meters=distMeters).destination(point=[lat, lon], bearing=hdg))
	
	return ([lat, lon, alt])
	
def pointInTime(lat, lon, alt, hdg, speed, vz, seconds):
	'''
	Determine where a moving entity will be at some time in the future
	
	speed --> groundspeed (horizontal).  In [m/s]
	vz < 0 --> climbing.
	
	Returns [lat, lon, alt]
	'''
	
	distMeters = speed*seconds
	
	# Find location based on ground distance.
	# Ignore altitude changes.
	[lat, lon, dummyAlt] = pointInDistance(lat, lon, alt, hdg, distMeters)
	
	alt = alt - vz*seconds
	
	return ([lat, lon, alt])

	

def interceptTarget(lat1, lon1, alt1, hdg1, speed1, vz1, lat2, lon2, alt2, speed2):

	'''
	Find the location where entity 2 could first collide with entity 1.
	
	Entity 1 is traveling (by assumption) on a fixed trajectory.
	Entity 2 has a fixed speed, but an adjustable heading.
	
	Returns [isFeasible, [lat, lon, alt, timeToCollision]]

	##############################################################################################
	# NOTE:
	# - We have division by zero if either of the entities have zero speed
	#	   - If both entities are stationary, there's nothing to do.
	#      - If entity 1 is stationary, we just need to route entity 2 to entity 1's location.
	#      - If entity 2 is stationary, why would we ever call this function?
	##############################################################################################
	
	THIS COMES FROM http://zulko.github.io/blog/2013/11/11/interception-of-a-linear-trajectory-with-constant-speed/
	
	TESTING:
import distance_functions
distance_functions.interceptTarget(43, -79, 30, 15, 1, 0, 43, -78, 30, 1)
distance_functions.interceptTarget(43, -79, 30, 180, 2, 0, 43, -78, 30, 1)

distance_functions.interceptTarget(43, -79, 30, -45, 1, 0, 43, -79.01, 30, .95)
	'''
	
	
	if (speed2 == 0):
		print('Why did you call this function?  UAV2 is not moving')

		return([False, []])
		
	elif (speed1 == 0):
		# UAV 1 is stationary.  Just route UAV 2 to UAV 1's location.

		# Find the required heading for UAV 2 to get to UAV 1
		colHdg = getHeading(lat2, lon2, lat1, lon1)
		
		# Find the time for UAV 2 to get to UAV 1's location
		tgo = getGPSdistance(lat2, lon2, lat1, lon1) / float(speed2)
		
		return([True, [lat1, lon1, alt1, colHdg, tgo]])
			
	# Find heading from entity 1 to entity 2
	tmpHdg = getHeading(lat1, lon1, lat2, lon2)
		
	# Find angle between entity 1's path and the path from entity 1 to entity 2
	betaDeg = min( max(hdg1, tmpHdg)-min(hdg1, tmpHdg), 
	               360 - (max(hdg1, tmpHdg)-min(hdg1, tmpHdg)))	
	beta = betaDeg*(math.pi/180)
	print('beta = %f' % (betaDeg))

	# Find sine of angle between path from entity 1 to entity 2 and path from entity 2 to collision point
	sine_alpha = (speed1/float(speed2))*math.sin(beta)
	if (sine_alpha > 1):
		print('cannot catch up')
		
		return([False, []])
	else:
		print('alpha = %f' % (math.asin(sine_alpha)*180/math.pi))
		alpha = math.asin(sine_alpha) 
		
		sine_gamma = sine_alpha*math.cos(beta) + math.cos(alpha)*math.sin(beta)
		print('gamma = %f' % (math.asin(sine_gamma)*180/math.pi))
	
		print('sum = %f' % ( (math.asin(sine_alpha)*180/math.pi) + betaDeg + (math.asin(sine_gamma)*180/math.pi) ))
		
		# Find the distance from entity 1 to collision point
		dist1 = (sine_alpha/sine_gamma)*getGPSdistance(lat1, lon1, lat2, lon2)
		print('dist1 = %f' % (dist1))
		
		# Find the time at which the entities will reach the collision point
		tgo = dist1/float(speed1)
		print('tgo = %f' % (tgo))
	
		# Find the location where the collision will occur
		# We'll move entity 1 forward
		[colLat, colLon, colAlt] = pointInTime(lat1, lon1, alt1, hdg1, speed1, vz1, tgo)

		# Find the required heading for UAV2 to get to collision point
		colHdg = getHeading(lat2, lon2, colLat, colLon)

		print('colLat = %f' % (colLat))
		print('colLon = %f' % (colLon))
		print('colAlt = %f' % (colAlt))
		print('colHdg = %f' % (colHdg))

		return([True, [colLat, colLon, colAlt, colHdg, tgo]])
		
		
	
	'''
	print((speed2/float(speed1)) * abs(math.sin(beta)))
	if ( (speed2/float(speed1)) * abs(math.sin(beta)) > 1 ):
		print('2 cannot catch 1')
	else:
		# Find sine of angle between entity 1's path and entity 2's path
		sine_gamma = sine_alpha*math.sqrt(1 - math.sin(beta)**2) + math.sin(beta)*math.sqrt(1-sine_alpha**2)
		print('gamma = %f' % (math.asin(sine_gamma)*180/math.pi))
		
		# Find the distance from entity 1 to collision point
		dist1 = (sine_alpha/sine_gamma)*getGPSdistance(lat1, lon1, lat2, lon2)
		
		# Find the time at which the entities will reach the collision point
		tgo = dist1/float(speed1)
		
		# Find the location where the collision will occur
		# We'll move entity 1 forward
		[colLat, colLon, colAlt] = pointInTime(lat1, lon1, alt1, hdg1, speed1, vz1, tgo)

		print('tgo = %f' % (tgo))
		print('colLat = %f' % (colLat))
		print('colLon = %f' % (colLon))
		print('colAlt = %f' % (colAlt))
		print('dist1 = %f' % (dist1))
		
		print('sum = %f' % ( (math.asin(sine_alpha)*180/math.pi) + betaDeg + (math.asin(sine_gamma)*180/math.pi) ))
	'''
	
	
def nearMiss(lat1, lon1, alt1, hdg1, speed1, vz1, lat2, lon2, alt2, hdg2, speed2, vz2):
	
	'''
	Calculate the anticipated collision point (lat, lon, alt) between two moving entities.
	
	Returns [timeToNearMiss, minDistance, [futLat1, futLon1, futAlt1], [futLat2, futLon2, futAlt2]]
	
	TESTING:
import distance_functions
distance_functions.nearMiss(43, -79, 30, 15, 1, 0, 43, -78, 30, -15, 1, 0)

	'''

	vx1 = math.sin(hdg1*(math.pi/180)) * speed1
	vy1 = math.cos(hdg1*(math.pi/180)) * speed1
	
	deltax0 = getGPSdistance(lat1, lon1, lat1, lon2)
	deltay0 = getGPSdistance(lat2, lon2, lat1, lon2)
	deltaz0 = abs(alt2 - alt1)
	if (lat1 < lat2):
		# UAV 2 is N of UAV 1
		deltay0 = -deltay0
	if (lon1 < lon2):
		# UAV 2 is E of UAV 1
		deltax0 = -deltax0
	
	vx2 = math.sin(hdg2*(math.pi/180)) * speed2
	vy2 = math.cos(hdg2*(math.pi/180)) * speed2		
	deltavx = vx1 - vx2 
	deltavy = vy1 - vy2
	deltavz = vz1 - vz2

	# NO.  NO NO NO NO.  NO.
	# if ( (abs(deltavx) > 0.001) and (abs(deltavy) > 0.001) ):
	
	# Find the time at which the two entities are closest to each other:
	if (abs(deltavx + deltavy + deltavz) > 0.0001):
		tgo = -(deltax0 * deltavx + deltay0 * deltavy + deltaz0 * deltavz)/float(deltavx**2 + deltavy**2 + deltavz**2)
	else:
		# We're never closer than we are right now.
		tgo = -1.0	

	print('tgo = %f' % (tgo))

	if (tgo > 0):
		# Where are the two entities at time tgo?
		[futureLat1, futureLon1, futureAlt1] = pointInTime(lat1, lon1, alt1, hdg1, speed1, vz1, tgo)
		[futureLat2, futureLon2, futureAlt2] = pointInTime(lat2, lon2, alt2, hdg2, speed2, vz2, tgo)
		
		# What is the distance between them at that time?
		nearMissDist = getGPSdistance3D(futureLat1, futureLon1, futureAlt1, futureLat2, futureLon2, futureAlt2)
	else:
		# We're never closer than we are right now.
		tgo = 0
		nearMissDist = getGPSdistance3D(lat1, lon1, alt1, lat2, lon2, alt2)
		[futureLat1, futureLon1, futureAlt1] = [lat1, lon1, alt1]
		[futureLat2, futureLon2, futureAlt2] = [lat2, lon2, alt2]
		
	return (tgo, nearMissDist, [futureLat1, futureLon1, futureAlt1], [futureLat2, futureLon2, futureAlt2])



'''
Modify this code.  It calculates the  
	
	vx1 = sin(uav[uavID].headingCurrent*(pi/180)) * uav[uavID].groundSpeed
	vy1 = cos(uav[uavID].headingCurrent*(pi/180)) * uav[uavID].groundSpeed

	lat1 = uav[uavID].latCurrent
	lon1 = uav[uavID].lonCurrent
	
		
	# Loop over all UAVs controlled by others:
	for otherUAVid in otherUAVs:
		lat2 = otherUAVs[otherUAVid].lat
		lon2 = otherUAVs[otherUAVid].lon
		deltax0 = distance_functions.getGPSdistance(lat1, lon1, lat1, lon2)
		deltay0 = distance_functions.getGPSdistance(lat2, lon2, lat1, lon2)
		if (lat1 < lat2):
			# UAV 2 is N of UAV 1
			deltay0 = -deltay0
		if (lon1 < lon2):
			# UAV 2 is E of UAV 1
			deltax0 = -deltax0
		
		vx2 = sin(otherUAVs[otherUAVid].heading*(pi/180)) * otherUAVs[otherUAVid].groundSpeed
		vy2 = cos(otherUAVs[otherUAVid].heading*(pi/180)) * otherUAVs[otherUAVid].groundSpeed		
		deltavx = vx1 - vx2 
		deltavy = vy1 - vy2

		if (deltavx + deltavy != 0):
			tgo = -(deltax0 * deltavx + deltay0 * deltavy)/float(deltavx**2 + deltavy**2)
		else:
			tgo = -1.0	
		
		if (tgo > 0):
			S = sqrt((deltax0 + deltavx*tgo)**2 + (deltay0 + deltavy*tgo)**2)
			if (S <= NEAR_MISS and tgo < mintgo):
				mintgo = tgo
				myThreat = otherUAVid
				latThreat = lat2
				lonThreat = lon2
	
'''
