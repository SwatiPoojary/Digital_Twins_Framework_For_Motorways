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
from pykalman import KalmanFilter
import numpy as np
# we need to import python modules from the $SUMO_HOME/tools directory
import time

import sumolib.net
from kafka import KafkaConsumer

from constants import *
from sumo_helper import *

from sumolib import checkBinary  # noqa
import traci  # noqa


def addProbeVehicleToSimulation(data_dic, net, vehicle_list, probe_vehicle_dict, i, route_list):
    try:
        lat_lon = ast.literal_eval(data_dic["location"])
        lat = lat_lon[0]
        lon = lat_lon[1]
        curr_veh_dict = dict()
        x, y = net.convertLonLat2XY(lon, lat)
        edgeID, lanePosition, laneIndex = traci.simulation.convertRoad(lon, lat, True)
        veh_name = data_dic["probe_id"] + data_dic["vehicle type"]

        if veh_name not in traci.vehicle.getIDList():
            veh_route = []
            for route in route_list[0].route:
                edges = route.edges.split()
                if edgeID in edges:
                    veh_route.append(route.id)
            if len(veh_route) > 0:
                route_id = random.choice(veh_route)
            else:
                route_id = ''
            traci.vehicle.add(veh_name, routeID=route_id)
            vehicle_list.append(veh_name)

            curr_veh_dict['prev_params'] = [0] * 4
            curr_veh_dict['previous_edge'] = [edgeID]
            probe_vehicle_dict[veh_name] = curr_veh_dict

            mps_speed = float(data_dic["speed"])
            # mps_speed = kphToMps(float(data_dic["speed"]))
            traci.vehicle.setSpeed(veh_name, mps_speed)
            traci.vehicle.setType(veh_name, data_dic["vehicle type"])
            traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
            # print(veh_name, ' added to simulation')
        # else:
        #     print(veh_name, ' already present')

        curr_veh_dict = probe_vehicle_dict[veh_name]
        previous_cords = curr_veh_dict['prev_params']
        veh_r = traci.vehicle.getRoute(veh_name)
        if curr_veh_dict['previous_edge'][-1] != edgeID:
            curr_veh_dict['previous_edge'].append(edgeID)
            curr_ed = curr_veh_dict['previous_edge']
        if len(previous_cords) == 4:
            distance_x = x - previous_cords[0]
            distance_y = y - previous_cords[1]
            if abs(distance_x) > abs(previous_cords[2]) and abs(distance_y) > abs(previous_cords[3]):
                veh_cord = traci.vehicle.getPosition(veh_name)
                mps_speed = float(data_dic["speed"])
                # mps_speed = kphToMps(float(data_dic["speed"]))
                traci.vehicle.setSpeed(veh_name, mps_speed)
                traci.vehicle.setType(veh_name, data_dic["vehicle type"])
                traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
                previous_cords[2] = distance_x  # storing total distance covered
                previous_cords[3] = distance_x  # storing total distance covered
            else:
                # mps_speed = kphToMps(float(data_dic["speed"]))
                # print('slow down spped: ', mps_speed)
                curr_Speed = float(data_dic["speed"]) - 10
                traci.vehicle.slowDown(veh_name, curr_Speed, 60)

        if len(previous_cords) < 4:
            previous_cords[0] = x  # storing previous x cordinate
            previous_cords[1] = y  # storing previous y cordinate
            previous_cords[2] = 0
            previous_cords[3] = 0
        curr_veh_dict['prev_params'] = previous_cords

        probe_vehicle_dict[veh_name] = curr_veh_dict

    except Exception as e:
        pass
        # print('Exception in probe: ', e)


def addVehicleToSimulation(data_dic, net, vehicle_dict, i, route_list, partition, topic):
    try:
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

        mps_speed = float(data_dic["speed"])
        # mps_speed = kphToMps(float(data_dic["speed"]))
        speed = mps_speed
        meter_per_sec = (speed * 1000 / 3600)
        # since sleep is added of 1 sec
        # meter_per_sec = 3 * meter_per_sec
        veh_name = ''
        veh_name_edge = ''
        min_vehicle_distance = sys.maxsize
        existing_min_distance = sys.maxsize

        for edge_veh in veh_in_edge:
            edge_veh_lon, edge_veh_lat = traci.vehicle.getPosition(edge_veh)

            veh_distance = traci.simulation.getDistance2D(x1=veh_x, y1=veh_y, x2=edge_veh_lon, y2=edge_veh_lat,
                                                          isGeo=False)
            # if veh_distance < meter_per_sec and veh_distance < min_vehicle_distance:
            if veh_distance < min_vehicle_distance:
                min_vehicle_distance = veh_distance
                veh_name_edge = edge_veh
                # print('veh_name_edge: ', veh_name_edge)
                if (edge_veh + '_' + str(partition)) in vehicle_dict:
                    value = vehicle_dict[edge_veh + '$' + str(partition)]
                    if float(data_dic['distance']) < float(value[-1]) < existing_min_distance:
                        existing_min_distance = float(value[-1])
                        veh_name = edge_veh
                        # print('veh_name: ', veh_name)

        if veh_name == '' and veh_name_edge != '':
            veh_name = veh_name_edge
        elif veh_name == '':
            if len(vehicle_dict) > 0:
                for key, value in vehicle_dict.items():
                    # print(key)
                    # print(value)
                    v_name = key.split('$')[0]
                    v_part = key.split('$')[1]
                    if int(v_part) == int(partition):
                        key_direction = v_name[-1]
                        if data_dic['direction'] == key_direction and data_dic['distance'] < value[-1]:
                            veh_name = v_name
                            # print('veh_name2: ', veh_name)

                            break

        if veh_name == '' or veh_name not in traci.vehicle.getIDList():
            veh_name = 'camera' + str(i) + data_dic['direction']
            traci.vehicle.add(veh_name, routeID=trip_name)
            # traci.vehicle.add(veh_name, routeID=trip_name, depart=float(data_dic['distance']))
            mps_speed = float(data_dic["speed"])
            # mps_speed = kphToMps(float(data_dic["speed"]))

            traci.vehicle.setSpeed(veh_name, mps_speed)
            # print("current_speed: ", float(data_dic["speed"]))
            # print("mps_speed: ", mps_speed)
            traci.vehicle.moveToXY(veh_name, edgeID, int(lane_index), veh_x, veh_y, keepRoute=2)
            vehicle_dict[veh_name + '$' + str(partition)] = [data_dic['distance']]
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

            # print('moving_forward: ', moving_forward)

            if moving_forward:
                mps_speed = float(data_dic["speed"])
                # mps_speed = kphToMps(float(data_dic["speed"]))
                traci.vehicle.setSpeed(veh_name, mps_speed)
                # print("current_speed: ", float(data_dic["speed"]))
                # print("mps_speed: ", mps_speed)
                traci.vehicle.moveToXY(veh_name, edgeID, int(lane_index), veh_x, veh_y, keepRoute=2)
            if veh_name + '_' + str(partition) in vehicle_dict:
                vehicle_dict[veh_name + '$' + str(partition)].append(data_dic['distance'])
            else:
                vehicle_dict[veh_name + '$' + str(partition)] = [data_dic['distance']]
            print(veh_name, ' modified')
    except Exception as e:
        pass
        # print('Exception in camera: ', e)


def addInductiveLoopData(data_dic, partition):
    try:
        print('-------inductive loop data----')
        print(data_dic)
        if int(data_dic['lane 1']) > 0:
            print('lane 1: ', data_dic['lane 1'])
            if int(partition) == 0:
                print(traci.inductionloop.getPosition('M50_Northbound-1'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Northbound-1'))
            elif int(partition) == 1:
                print(traci.inductionloop.getPosition('M50_Southbound-1'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Southbound-1'))
        if int(data_dic['lane 2']) > 0:
            print('lane 2: ', data_dic['lane 2'])
            if int(partition) == 0:
                print(traci.inductionloop.getPosition('M50_Northbound-2'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Northbound-2'))
            elif int(partition) == 1:
                print(traci.inductionloop.getPosition('M50_Southbound-2'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Southbound-2'))
        if int(data_dic['lane 3']) > 0:
            print('lane 3: ', data_dic['lane 3'])
            if int(partition) == 0:
                print(traci.inductionloop.getPosition('M50_Northbound-3'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Northbound-3'))
            elif int(partition) == 1:
                print(traci.inductionloop.getPosition('M50_Southbound-3'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Southbound-3'))
        if int(data_dic['lane 3']) > 0:
            print('lane 4: ', data_dic['lane 4'])
            if int(partition) == 0:
                print(traci.inductionloop.getPosition('M50_Northbound-4'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Northbound-4'))
            elif int(partition) == 1:
                print(traci.inductionloop.getPosition('M50_Southbound-4'))
                print(traci.inductionloop.getLastStepVehicleIDs('M50_Southbound-4'))
    except Exception as e:
        # pass
        print('Exception in inductive: ', e)


def run():
    vehicle_dict = {}
    route_list = []
    vehicle_list = []
    probe_vehicle_dict = {}
    net = sumolib.net.readNet("ITSC2020_CAV_impact/workspace/M50network.net.xml")
    for vehicle in sumolib.xml.parse("ITSC2020_CAV_impact/workspace/M50_routes.rou.xml", "routes"):
        route_list.append(vehicle)

    consumer = KafkaConsumer(bootstrap_servers=BROKER_EP, value_deserializer=DECODING, consumer_timeout_ms=1000)
    consumer.subscribe(topics=TOPIC)
    i = 1
    t = 0
    # simulate = False
    # for msg in consumer:
    #     # time.sleep(0.5)
    #     # if t > 100:
    #     #     break
    #     print('----------------------------------data-----------------------------')
    #     if msg.topic == "probe_vehicles":
    #         addProbeVehicleToSimulation(msg.value, net, vehicle_list, probe_vehicle_dict, i, route_list)
    #         traci.simulationStep()
    #     elif msg.topic == "inductive_loops":
    #         addInductiveLoopData(msg.value, msg.partition)
    #     else:
    #         addVehicleToSimulation(msg.value, net, vehicle_dict, i, route_list, int(msg.partition),
    #                                msg.topic)
    #         simulate = True
    #
    #     # if simulate:
    #     traci.simulationStep()
    #     i = i + 1
    #     t = traci.simulation.getTime()
    record_empty = 0
    t=0
    while True:
        # time.sleep(0.5)
        records = consumer.poll(1000)
        simulate = False
        print('-----*******************--------')
        if len(records) > 0:
            record_empty = 0
            for key, value in records.items():

                message = value[0].value
                # print(key.topic)
                # print(message)
                if key.topic == "probe_vehicles":
                    addProbeVehicleToSimulation(message, net, vehicle_list, probe_vehicle_dict, i, route_list)
                    simulate = True
                    traci.simulationStep()
                elif key.topic == "inductive_loops":
                    addInductiveLoopData(message, key.partition)
                else:
                    addVehicleToSimulation(message, net, vehicle_dict, i, route_list, int(key.partition),
                                           key.topic)
                    simulate = True
                    traci.simulationStep()
                i = i + 1
            # if simulate:
            #     traci.simulationStep()
            t = traci.simulation.getTime()
            # print(t)
        else:
            record_empty += 1
        if record_empty > 5:
            break
    print('outside while')
    print('outside for')
    traci.close()
    sys.stdout.flush()
    print('----ended-----')



# this is the main entry point of this script
if __name__ == "__main__":
    SUMO_HOME_TOOLS()
    calculate_camera_values()
    calculate_toll_bridge_values()

    traci.start(SUMO_CMD)
    run()
