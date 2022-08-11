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
from multiprocessing import Process

# we need to import python modules from the $SUMO_HOME/tools directory
import time

import sumolib.net
from kafka import KafkaConsumer

from constants import CAMERA_LOOKUP, DECODING, BROKER_EP
from sumo_helper import *

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


def addVehicleToSimulation(data_dic, net, vehicle_dict, i, route_list, partition, topic):
    print('---------')
    print(topic)
    if topic == "motorway_cameras":
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
    elif topic == "toll_bridge_cameras":
        cam_values = TOLL_BRIDGE
        if data_dic['direction'] == 'N':
            start_x, start_y = cam_values['north_start_edges']
            angle = cam_values['north_angle']
        elif data_dic['direction'] == 'S':
            start_x, start_y = cam_values['south_start_edges']
            angle = cam_values['south_angle']

    if data_dic['direction'] == 'N':
        start_lon, start_lat = net.convertXY2LonLat(start_x, start_y)
        lon2, lat2 = approximate_destination((start_lon, start_lat), float(data_dic['distance']), angle)
        veh_x, veh_y = net.convertLonLat2XY(lon2, lat2)
    elif data_dic['direction'] == 'S':
        start_lon, start_lat = net.convertXY2LonLat(start_x, start_y)
        lon2, lat2 = approximate_destination((start_lon, start_lat), float(data_dic['distance']), angle)
        veh_x, veh_y = net.convertLonLat2XY(lon2, lat2)

    lane_index = data_dic['lane_index']
    edgeID = data_dic['lane_id']
    curr_lane = edgeID + '_' + lane_index

    trip_name = 'cam_trip_' + data_dic['direction'] + '_' + str(partition)
    if trip_name not in traci.route.getIDList():
        traci.route.add(trip_name, [edgeID])

    veh_in_edge = []
    veh_in_edge.extend(traci.edge.getLastStepVehicleIDs(edgeID))
    if edgeID in net.getEdges():
        current_edge = net.getEdge(edgeID)
        incoming_edge = current_edge.getIncoming()
        incoming_edge = list(incoming_edge.keys())[0]
        incoming_edge_id = incoming_edge.getID()
        veh_in_edge.extend(traci.edge.getLastStepVehicleIDs(incoming_edge_id))

    speed = float(data_dic["speed"])
    meter_per_sec = (speed * 1000 / 3600)
    # since sleep is added of 1 sec
    meter_per_sec = 2 * meter_per_sec
    veh_name = ''
    veh_name_edge = ''
    min_vehicle_distance = sys.maxsize
    existing_min_distance = sys.maxsize

    for edge_veh in veh_in_edge:
        edge_veh_lon, edge_veh_lat = traci.vehicle.getPosition(edge_veh)

        veh_distance = traci.simulation.getDistance2D(x1=veh_x, y1=veh_y, x2=edge_veh_lon, y2=edge_veh_lat, isGeo=False)
        if veh_distance < meter_per_sec and veh_distance < min_vehicle_distance:
            min_vehicle_distance = veh_distance
            veh_name_edge = edge_veh
            if (edge_veh + '_' + str(partition)) in vehicle_dict:
                value = vehicle_dict[edge_veh + '_' + str(partition)]
                if float(data_dic['distance']) < float(value[-1]) < existing_min_distance:
                    existing_min_distance = float(value[-1])
                    veh_name = edge_veh

    if veh_name == '' and veh_name_edge != '':
        veh_name = veh_name_edge
    elif veh_name == '':
        if len(vehicle_dict) > 0:
            for key, value in vehicle_dict.items():
                v_name = key.split('_')[0]
                v_part = key.split('_')[1]
                if int(v_part) == int(partition):
                    key_direction = v_name[-1]
                    if data_dic['direction'] == key_direction and data_dic['distance'] < value[-1]:
                        veh_name = v_name
                        break

    if veh_name == '' or veh_name not in traci.vehicle.getIDList():
        veh_name = 'camera' + str(i) + data_dic['direction']
        traci.vehicle.add(veh_name, routeID=trip_name, depart=float(data_dic['distance']))
        traci.vehicle.setSpeed(veh_name, data_dic["speed"])
        traci.vehicle.moveToXY(veh_name, edgeID, int(lane_index), veh_x, veh_y, keepRoute=2)
        vehicle_dict[veh_name + '_' + str(partition)] = [data_dic['distance']]
        print(veh_name, ' added')

    else:

        if float(data_dic['distance']) < float(100):
            veh_route = []
            for route in route_list[0].route:
                edges = route.edges.split()
                if traci.vehicle.getLaneID(veh_name) is not None and traci.vehicle.getLaneID(veh_name) != '':
                    if traci.vehicle.getRoadID(veh_name) in edges:
                        veh_route.append(route.id)
            if len(veh_route) > 0:
                route_id = random.choice(veh_route)
            else:
                route_id = ''
            if route_id != '':
                traci.vehicle.setRouteID(veh_name, route_id)

        moving_forward = True
        vehicle_position = traci.vehicle.getPosition(veh_name)
        current_post = (veh_x, veh_y)
        if data_dic['direction'] == 'N':
            angle = math.atan2(vehicle_position[1] - current_post[1], vehicle_position[0] - current_post[0])
            angle = math.degrees(angle)
        elif data_dic['direction'] == 'S':
            angle = math.atan2(vehicle_position[0] - current_post[0], vehicle_position[1] - current_post[1])
            angle = math.degrees(angle)
            # 90 and 270
        if angle < 0:
            angle = 360 + angle
        if angle > 90 and angle < 270:
            moving_forward = False

        print('moving_forward: ', moving_forward)

        if moving_forward:
            traci.vehicle.setSpeed(veh_name, data_dic["speed"])
            traci.vehicle.moveToXY(veh_name, edgeID, int(lane_index), veh_x, veh_y, keepRoute=2)
        if veh_name + '_' + str(partition) in vehicle_dict:
            vehicle_dict[veh_name + '_' + str(partition)].append(data_dic['distance'])
        else:
            vehicle_dict[veh_name + '_' + str(partition)] = [data_dic['distance']]
        print(veh_name, ' modified')


def consume_camera_data(EP, net, vehicle_dict, route_list):
    print('inside consume')
    i = 1
    TOPIC = ["motorway_cameras", "toll_bridge_cameras"]
    consumer = KafkaConsumer(bootstrap_servers=EP, value_deserializer=DECODING)
    consumer.subscribe(topics=TOPIC)
    for msg in consumer:
        time.sleep(1)
        # t = traci.simulation.getTime()
        motorway_cameras_data = msg.value
        addVehicleToSimulation(motorway_cameras_data, net, vehicle_dict, i, route_list, int(msg.partition), msg.topic)
        traci.simulationStep()
        i = i + 1


def traci_run(net, route_list):
    i = 1
    vehicle_dict = {}

    print(net)
    print(route_list)
    # Process(target=consume_camera_data, args=(BROKER_EP, net, vehicle_dict, route_list)).start()
    consume_camera_data(BROKER_EP, net, vehicle_dict, route_list)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
    print('----ended-----')
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
    calculate_toll_bridge_values()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "ITSC2020_CAV_impact/workspace/M50_simulation.sumo.cfg", "--delay", "1000"])
    # "--tripinfo-output", "tripinfo.xml", "--fcd-output", "fcd.xml", "--delay", "1000", '--log',
    # 'logfile.txt', '--ignore-route-errors'])

    route_list = []
    net = sumolib.net.readNet("ITSC2020_CAV_impact/workspace/M50network.net.xml")
    for vehicle in sumolib.xml.parse("ITSC2020_CAV_impact/workspace/M50_routes.rou.xml", "routes"):
        route_list.append(vehicle)

    traci_run(net, route_list)
