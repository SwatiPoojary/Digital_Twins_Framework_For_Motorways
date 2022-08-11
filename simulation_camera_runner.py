#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2022 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

import ast
import inspect
import json
import math
import os
import sys
import optparse
import random

# we need to import python modules from the $SUMO_HOME/tools directory
import time

import sumolib.net

from constants import CAMERA_LOOKUP
from sumo_helper import calculate_camera_values

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

EARTH_EQUATORIAL_RADIUS = 6378137.0
EARTH_EQUATORIAL_METERS_PER_DEGREE = math.pi * EARTH_EQUATORIAL_RADIUS / 180  # 111319.49079327358


def approximate_destination(point, distance, theta):
    # http://stackoverflow.com/questions/2187657/calculate-second-point-knowing-the-starting-point-and-distance
    lon, lat = point
    radians_theta = math.radians(theta)
    dx = distance * math.cos(radians_theta)
    dy = distance * math.sin(radians_theta)

    dlon = dx / (EARTH_EQUATORIAL_METERS_PER_DEGREE * math.cos(math.radians(lat)))
    dlat = dy / EARTH_EQUATORIAL_METERS_PER_DEGREE

    return (lon + dlon, lat + dlat)


def addVehicleToSimulation(data_dic, net, vehicle_dict, i, route_list, partition):
    print('--------------')
    for cam in CAMERA_LOOKUP:
        cam_values = CAMERA_LOOKUP[cam]
        if cam_values['partition'] == partition:
            if data_dic['direction'] == 'N':
                start_x, start_y = cam_values['north_start_edges']
                angle = cam_values['north_angle']
            elif data_dic['direction'] == 'S':
                start_x, start_y = cam_values['south_start_edges']
                angle = cam_values['south_angle']
            break

    if data_dic['direction'] == 'N':
        start_lon, start_lat = net.convertXY2LonLat(start_x, start_y)
        # lon2, lat2 = approximate_destination((start_lon, start_lat), float(data_dic['distance']), 303.4650611496275)
        lon2, lat2 = approximate_destination((start_lon, start_lat), float(data_dic['distance']), angle)
        veh_x, veh_y = net.convertLonLat2XY(lon2, lat2)
    elif data_dic['direction'] == 'S':
        start_lon, start_lat = net.convertXY2LonLat(start_x, start_y)
        # lon2, lat2 = approximate_destination((start_lon, start_lat), float(data_dic['distance']), 123.6058368667546)
        lon2, lat2 = approximate_destination((start_lon, start_lat), float(data_dic['distance']), angle)
        veh_x, veh_y = net.convertLonLat2XY(lon2, lat2)

    lane_index = data_dic['lane_index']
    edgeID = data_dic['lane_id']
    curr_lane = edgeID + '_' + lane_index

    trip_name = 'cam_trip_' + data_dic['direction']+'_'+str(partition)
    if trip_name not in traci.route.getIDList():
        traci.route.add(trip_name, [edgeID])

    current_edge = net.getEdge(edgeID)
    incoming_edge = current_edge.getIncoming()
    incoming_edge = list(incoming_edge.keys())[0]
    incoming_edge_id = incoming_edge.getID()
    veh_in_edge = []
    veh_in_edge.extend(traci.edge.getLastStepVehicleIDs(edgeID))
    veh_in_edge.extend(traci.edge.getLastStepVehicleIDs(incoming_edge_id))
    # veh_in_lane = traci.lane.getLastStepVehicleIDs(curr_lane)

    speed = float(data_dic["speed"])
    meter_per_sec = (speed * 1000 / 3600)

    # veh_name = ''
    # if len(vehicle_dict) > 0:
    #     for key, value in vehicle_dict.items():
    #         #traci.vehicle.getLaneIndex(key) == int(curr_lane) and
    #         #After evaluation check if the lane index check is required
    #         #check if  key in veh_in_edgethis check is required
    #         print(key)
    #         key_direction = key[-1]
    #         if data_dic['direction'] == key_direction and data_dic['distance'] < value[-1]:
    #             veh_name = key
    #             break
    # print('----after dist check vehname: ',veh_name)
    # if veh_name != '' and  veh_name  in traci.vehicle.getIDList():
    #     print('---distance between our veh and the veh_name')
    #     veh_name_lon, veh_name_lat = traci.vehicle.getPosition(veh_name)
    #     print(traci.simulation.getDistance2D(x1=veh_x, y1=veh_y, x2=veh_name_lon, y2=veh_name_lat, isGeo=False))
    # # if veh_name == '':

    veh_name = ''
    veh_name_edge = ''
    min_vehicle_distance = sys.maxsize
    existing_min_distance = sys.maxsize
    # print('veh_in_lane: ',veh_in_lane)
    for edge_veh in veh_in_edge:
        edge_veh_lon, edge_veh_lat = traci.vehicle.getPosition(edge_veh)

        veh_distance = traci.simulation.getDistance2D(x1=veh_x, y1=veh_y, x2=edge_veh_lon, y2=edge_veh_lat, isGeo=False)
        if veh_distance < meter_per_sec and veh_distance < min_vehicle_distance:
            min_vehicle_distance = veh_distance
            veh_name_edge = edge_veh
            if edge_veh in vehicle_dict:
                value = vehicle_dict[edge_veh]
                if float(data_dic['distance']) < float(value[-1]) < existing_min_distance:
                    existing_min_distance = float(value[-1])
                    veh_name = edge_veh

    if veh_name == '':
        veh_name = veh_name_edge


    # check if this vehs position is around the calc ulates pos -> if yes then thts the vehicle

    # veh_name = ''
    # if len(min_veh_list) > 0:
    #     for min_veh in min_veh_list:
    #         value = vehicle_dict[min_veh]
    #         # traci.vehicle.getLaneIndex(key) == int(curr_lane) and
    #         # After evaluation check if the lane index check is required
    #         # check if  key in veh_in_edgethis check is required
    #         if data_dic['distance'] < value[-1]:
    #             veh_name = min_veh
    #             break
    # print('----after dist check vehname: ', veh_name)

    # if veh_name != '' and veh_name in traci.vehicle.getIDList():
    #     print('---distance between our veh and the veh_name')
    #     veh_name_lon, veh_name_lat = traci.vehicle.getPosition(veh_name)
    #     print(traci.simulation.getDistance2D(x1=veh_x, y1=veh_y, x2=veh_name_lon, y2=veh_name_lat, isGeo=False))
    # if veh_name == '':

    if veh_name == '' or veh_name not in traci.vehicle.getIDList():
        veh_name = 'camera_2_' + str(i) + data_dic['direction']
        traci.vehicle.add(veh_name, routeID=trip_name, depart=float(data_dic['distance']))
        traci.vehicle.setSpeed(veh_name, data_dic["speed"])
        traci.vehicle.moveToXY(veh_name, edgeID, int(lane_index), veh_x, veh_y, keepRoute=2)
        # traci.vehicle.changeLane(veh_name,int(lane_index),60)
        vehicle_dict[veh_name] = [data_dic['distance']]
    else:
        # vehicle_position = traci.vehicle.getPosition(veh_name)
        # current_post = (veh_x,veh_y)
        # angle = math.atan2(vehicle_position[1] - current_post[1], vehicle_position[0] - current_post[0])
        # angle = math.degrees(angle)
        # # 90 and 270
        # if angle < 0:
        #     angle = 360 + angle
        # print(angle)
        # if data['direction'] == 'N' and
        if float(data_dic['distance']) < float(50):
            print(edgeID)
            veh_route = []
            for route in route_list[0].route:
                edges = route.edges.split()
                print(edges)
                print('roadid: ',traci.vehicle.getRoadID(veh_name))
                print('laneid :', traci.vehicle.getLaneID(veh_name))
                if traci.vehicle.getRoadID(veh_name) in edges:
                    veh_route.append(route.id)
            if len(veh_route) > 0:
                route_id = random.choice(veh_route)
            else:
                route_id = ''
            print(route_id)
            traci.vehicle.setRouteID(veh_name, route_id)
        traci.vehicle.setSpeed(veh_name, data_dic["speed"])
        traci.vehicle.moveToXY(veh_name, edgeID, int(lane_index), veh_x, veh_y, keepRoute=2)
        # traci.vehicle.changeLane(veh_name,int(lane_index),60)
        vehicle_dict[veh_name].append(data_dic['distance'])


    print('---colloding vehs')
    print(traci.simulation.getCollidingVehiclesNumber())
    print('---end---')


def run():
    """execute the TraCI control loop"""
    data_dic = {}
    i = 1
    t = 0
    vehicle_list = []
    vehicle_dict = {}
    route_list = []
    net = sumolib.net.readNet("ITSC2020_CAV_impact/workspace/M50network.net.xml")
    for vehicle in sumolib.xml.parse("ITSC2020_CAV_impact/workspace/M50_routes.rou.xml", "routes"):
        route_list.append(vehicle)
    # while t < 3600:
    # with open('consumed_topics/probe_vehicles/0.txt') as pv_0:
    #     traci.simulationStep()
    #     t = traci.simulation.getTime()
    #     for pv in pv_0:
    #         addVehicleToSImulation(pv,net,vehicle_list,i, route_list)
    motorway_cameras_list = []
    with open('consumed_topics/motorway_cameras/2.txt') as pv_0:
        # open('consumed_topics/motorway_cameras/1.txt') as pv_1:
        # open('consumed_topics/motorway_cameras/2.txt') as pv_2, \
        # open('consumed_topics/motorway_cameras/3.txt') as pv_3, \
        # open('consumed_topics/motorway_cameras/4.txt') as pv_4, \
        # open('consumed_topics/motorway_cameras/5.txt') as pv_5, \
        # open('consumed_topics/motorway_cameras/6.txt') as pv_6, \
        # open('consumed_topics/motorway_cameras/7.txt') as pv_7, \
        # open('consumed_topics/motorway_cameras/8.txt') as pv_8, \
        # open('consumed_topics/motorway_cameras/9.txt') as pv_9:
        motorway_cameras_list.extend(pv_0.read().splitlines())

    motorway_cameras_data = list(map(ast.literal_eval, motorway_cameras_list))
    motorway_cameras_data = sorted(motorway_cameras_data, key=lambda i: i['timestamp'])
    for camera in motorway_cameras_data:
        if camera['direction'] == 'N':
            addVehicleToSimulation(camera, net, vehicle_dict, i, route_list, 2)
            traci.simulationStep()
            i = i + 1

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    calculate_camera_values()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "ITSC2020_CAV_impact/workspace/M50_simulation.sumo.cfg",
                 "--delay", "1000"])
    run()
