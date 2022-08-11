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
import os
import sys
import optparse
import random

# we need to import python modules from the $SUMO_HOME/tools directory
import sumolib.net

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def addVehicleToSimulation_OLD(data_dic, net, vehicle_list, vehicle_dict, i, route_list):
    # data_dic = ast.literal_eval(pv)
    print('------------')

    lat_lon = ast.literal_eval(data_dic["location"])
    lat = lat_lon[0]
    lon = lat_lon[1]
    x, y = net.convertLonLat2XY(lon, lat)
    edgeID, lanePosition, laneIndex = traci.simulation.convertRoad(lon, lat, True)

    veh_name = data_dic["probe_id"] + data_dic["vehicle type"]
    if veh_name not in vehicle_list:
        veh_route = []
        for route in route_list[0].route:
            edges = route.edges.split()
            if edgeID in edges:
                veh_route.append(route.id)
        if len(veh_route) > 0:
            route_id = random.choice(veh_route)
        else:
            route_id = ''
        # route_id = veh_route[1]
        # route_id = ''
        # traci.route.add("trip", [edgeID])
        # traci.vehicle.add(veh_name,"trip")
        traci.vehicle.add(veh_name, routeID='')
        vehicle_list.append(veh_name)
        vehicle_dict[veh_name] = []
        i += 1
        traci.vehicle.setSpeed(veh_name, data_dic["speed"])
        traci.vehicle.setType(veh_name, data_dic["vehicle type"])
        traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
        # traci.vehicle.setParameter(veh_name,'has.rerouting.device','true')
        traci.simulationStep()
    # try:
    # print('current traci position')
    # print(traci.vehicle.getPosition(veh_name))
    print("data x and y", x, y)
    # print(x, y)
    # print('distance')
    # print(traci.vehicle.getDistance(veh_name))
    previous_cords = vehicle_dict[veh_name]
    print(previous_cords)
    distance_x = 0
    distance_y = 0
    if len(previous_cords) == 4:
        distance_x = x - previous_cords[0]
        distance_y = y - previous_cords[1]  #
        print('@@@@@@@@@@@@@@@@@@@@@@@@')
        print(distance_x)
        print(previous_cords[2])
        print(distance_y)
        print(previous_cords[3])
        print(abs(distance_x) > abs(previous_cords[2]) and abs(distance_y) > abs(previous_cords[3]))
        if abs(distance_x) > abs(previous_cords[2]) and abs(distance_y) > abs(previous_cords[3]):
            # position = traci.vehicle.getPosition(veh_name)
            traci.vehicle.setSpeed(veh_name, data_dic["speed"])
            traci.vehicle.setType(veh_name, data_dic["vehicle type"])
            traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
            # traci.vehicle.setParameter(veh_name, 'has.rerouting.device', 'true')

            # print('current traci position after adjusting')
            # print(traci.vehicle.getPosition(veh_name))
            # previous_cords[2] = abs(previous_cords[2]) + distance_x  # storing total distance covered
            # previous_cords[3] = abs(previous_cords[3]) + distance_x  # storing total distance covered#
            previous_cords[2] = abs(distance_x)  # storing total distance covered
            previous_cords[3] = abs(distance_y)  # storing total distance covered
    elif len(previous_cords) < 4:
        previous_cords = [0] * 4
        previous_cords[0] = x  # storing previous x cordinate
        previous_cords[1] = y  # storing previous y cordinate
        previous_cords[2] = 0
        previous_cords[3] = 0

    # previous_cords[0] = x #storing previous x cordinate
    # previous_cords[1] = y #storing previous y cordinate
    vehicle_dict[veh_name] = previous_cords
    # if len(previous_cords) < 4:
    #     previous_cords[2] = x
    #     previous_cords[3] = y

    traci.simulationStep()

    # except Exception as e:
    #     print(e)
SIMULATION_DURATION = 0 + 1000 #3600 seconds = 1 hour

def addVehicleToSimulation_OLD2(data_dic, net, vehicle_list,vehicle_dict, i, route_list, t):
    # data_dic = ast.literal_eval(pv)
    while t<SIMULATION_DURATION:
        lat_lon = ast.literal_eval(data_dic["location"])
        lat = lat_lon[0]
        lon = lat_lon[1]
        x, y = net.convertLonLat2XY(lon, lat)
        edgeID, lanePosition, laneIndex = traci.simulation.convertRoad(lon, lat, True)
        print(edgeID)
        veh_name = data_dic["probe_id"] + data_dic["vehicle type"]
        if veh_name not in vehicle_list:
            veh_route=[]
            for route in route_list[0].route:
                edges = route.edges.split()
                if edgeID in edges:
                    veh_route.append(route.id)
            if len(veh_route) > 0:
                route_id = random.choice(veh_route)
            else:
                route_id=''
            # route_id = veh_route[0]
            # route_id = ''
            traci.vehicle.add(veh_name,routeID=route_id)
            # traci.route.add("trip", [edgeID])
            # traci.vehicle.add(veh_name, "trip")
            vehicle_list.append(veh_name)
            vehicle_dict[veh_name] = [0]*4
            i += 1
            traci.vehicle.setSpeed(veh_name, data_dic["speed"])
            traci.vehicle.setType(veh_name, data_dic["vehicle type"])
            traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
            # traci.vehicle.setParameter(veh_name,'has.rerouting.device','true')
            # traci.simulationStep()
        try:
            previous_cords = vehicle_dict[veh_name]
            distance_x = 0
            distance_y = 0
            if len(previous_cords) == 4:
                distance_x = x - previous_cords[0]
                distance_y = y - previous_cords[1]
                if abs(distance_x) > abs(previous_cords[2]) and abs(distance_y) > abs(previous_cords[3]):
                    # position = traci.vehicle.getPosition(veh_name)
                    traci.vehicle.setSpeed(veh_name, data_dic["speed"])
                    traci.vehicle.setType(veh_name, data_dic["vehicle type"])
                    traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
                    # traci.vehicle.setParameter(veh_name, 'has.rerouting.device', 'true')

                    # previous_cords[2] = previous_cords[2] + distance_x  # storing total distance covered
                    # previous_cords[3] = previous_cords[3] + distance_x  # storing total distance covered
                    previous_cords[2] = distance_x  # storing total distance covered
                    previous_cords[3] = distance_x  # storing total distance covered

            # previous_cords[0] = x #storing previous x cordinate
            # previous_cords[1] = y #storing previous y cordinate
            if len(previous_cords) < 4:
                previous_cords[0] = x
                previous_cords[1] = y
                previous_cords[2] = 0
                previous_cords[3] = 0
            vehicle_dict[veh_name] = previous_cords
            # traci.simulationStep()

        except Exception as e:
            print(e)
            return
        return

def addVehicleToSimulation(data_dic, net, vehicle_list,vehicle_dict, i, route_list):
    lat_lon = ast.literal_eval(data_dic["location"])
    lat = lat_lon[0]
    lon = lat_lon[1]
    curr_veh_dict = dict()
    x, y = net.convertLonLat2XY(lon, lat)
    print('lon, lat: ',lon, lat)
    print('x, y: ',x, y)
    edgeID, lanePosition, laneIndex = traci.simulation.convertRoad(lon, lat, True)

    veh_name = data_dic["probe_id"] + data_dic["vehicle type"]
    print('--addVehicleToSimulation started--')
    print(traci.vehicle.getIDList())

    if veh_name not in traci.vehicle.getIDList():
        veh_route=[]
        for route in route_list[0].route:
            edges = route.edges.split()
            if edgeID in edges:
                veh_route.append(route.id)
        if len(veh_route) > 0:
            route_id = random.choice(veh_route)
        else:
            route_id=''
        # route_id = veh_route[0]
        # route_id = ''
        # if veh_name == 'NRA_000000001509_Northbound-3.0CAV2':
        #     route_id = veh_route[1]
        trip_name = 'trip'+str(traci.simulation.getTime())
        traci.route.add(trip_name, [edgeID])
        # traci.route.setParameter('trip', 'viaLonLat', [lon,lat])
        traci.vehicle.add(veh_name,routeID=trip_name)
        # traci.route.add("trip", [edgeID])
        # traci.vehicle.add(veh_name, "trip")
        vehicle_list.append(veh_name)

        curr_veh_dict['prev_params'] = [0]*4
        curr_veh_dict['previous_edge'] = edgeID
        vehicle_dict[veh_name] = curr_veh_dict
        i += 1
        traci.vehicle.setSpeed(veh_name, data_dic["speed"])
        traci.vehicle.setType(veh_name, data_dic["vehicle type"])
        traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
        # traci.vehicle.setParameter(veh_name,'has.rerouting.device','true')
        # traci.simulationStep()
        print(veh_name,' added to simulation')
    # try:

    curr_veh_dict =  vehicle_dict[veh_name]
    # curr_veh_dict['current_edge'] = edgeID
    # vehicle_dict[veh_name] = curr_veh_dict
    previous_cords = curr_veh_dict['prev_params']

    print('---prininting route--')
    print(traci.vehicle.getRoute(veh_name))
    veh_r = traci.vehicle.getRoute(veh_name)
    print(type(veh_r))
    if curr_veh_dict['previous_edge'] != edgeID:
        curr_ed = (curr_veh_dict['previous_edge'],edgeID,)
        # new_r = veh_r + curr_ed
        print(curr_ed)
        traci.vehicle.setRoute(veh_name, curr_ed)
        curr_veh_dict['previous_edge'] = edgeID
    # if curr_veh_dict['previous_edge'] != edgeID:
    #     print('-------------------changing route-------------------')
    #     print('previous_edge: ', curr_veh_dict['previous_edge'])
    #     print('current edge: ', edgeID)
    #     veh_route = []
    #     for route in route_list[0].route:
    #         edges = route.edges.split()
    #         if edgeID in edges and curr_veh_dict['previous_edge'] in edges:
    #             veh_route.append(route.id)
    #
    #     current_route = traci.vehicle.getRouteID(veh_name)
    #     # if current_route not in veh_route:
    #     if len(veh_route) > 0:
    #         route_id = random.choice(veh_route)
    #     else:
    #         route_id = ''
    #     traci.vehicle.setRouteID(veh_name, routeID=route_id)
    #     curr_veh_dict['previous_edge'] = edgeID
    #     print('route changed ', route_id)
        # else:
        #     print('edges present in previous route')

    if len(previous_cords) == 4:
        distance_x = x - previous_cords[0]
        distance_y = y - previous_cords[1]
        if abs(distance_x) > abs(previous_cords[2]) and abs(distance_y) > abs(previous_cords[3]):
            veh_cord = traci.vehicle.getPosition(veh_name)
            print('current veh position: ', veh_cord)
            traci.vehicle.setSpeed(veh_name, data_dic["speed"])
            traci.vehicle.setType(veh_name, data_dic["vehicle type"])
            traci.vehicle.moveToXY(veh_name, edgeID, laneIndex, x, y, keepRoute=2)
            previous_cords[2] = distance_x  # storing total distance covered
            previous_cords[3] = distance_x  # storing total distance covered
        else:
            print('else')
            curr_Speed = float(data_dic["speed"])-10
            print(curr_Speed)
            traci.vehicle.slowDown(veh_name,curr_Speed, 60)

    if len(previous_cords) < 4:
        previous_cords[0] = x  # storing previous x cordinate
        previous_cords[1] = y  # storing previous y cordinate
        previous_cords[2] = 0
        previous_cords[3] = 0
    curr_veh_dict['prev_params'] = previous_cords
    vehicle_dict[veh_name] = curr_veh_dict

    print('---addVehicleToSimulation done---')
    # traci.simulationStep()

    # except Exception as e:
    #     print(e)




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
    probe_vehicle_list = []
    with open('consumed_topics/probe_vehicles/0.txt') as pv_0, \
            open('consumed_topics/probe_vehicles/1.txt') as pv_1, \
            open('consumed_topics/probe_vehicles/2.txt') as pv_2, \
            open('consumed_topics/probe_vehicles/3.txt') as pv_3, \
            open('consumed_topics/probe_vehicles/4.txt') as pv_4, \
            open('consumed_topics/probe_vehicles/5.txt') as pv_5, \
            open('consumed_topics/probe_vehicles/6.txt') as pv_6, \
            open('consumed_topics/probe_vehicles/7.txt') as pv_7, \
            open('consumed_topics/probe_vehicles/8.txt') as pv_8, \
            open('consumed_topics/probe_vehicles/9.txt') as pv_9:
        probe_vehicle_list.extend(pv_0.read().splitlines())
        probe_vehicle_list.extend(pv_1.read().splitlines())
        probe_vehicle_list.extend(pv_2.read().splitlines())
        probe_vehicle_list.extend(pv_3.read().splitlines())
        probe_vehicle_list.extend(pv_4.read().splitlines())
        probe_vehicle_list.extend(pv_5.read().splitlines())
        probe_vehicle_list.extend(pv_6.read().splitlines())
        probe_vehicle_list.extend(pv_7.read().splitlines())
        probe_vehicle_list.extend(pv_8.read().splitlines())
        probe_vehicle_list.extend(pv_9.read().splitlines())

    probe_vehicle_data = list(map(ast.literal_eval, probe_vehicle_list))
    probe_vehicle_data = sorted(probe_vehicle_data, key=lambda i: i['timestamp'])
    t=0
    for probe in probe_vehicle_data :
        if probe["probe_id"] == "NRA_000000001509_Northbound-3.0":

            # traci.simulationStep()
            addVehicleToSimulation(probe, net, vehicle_list, vehicle_dict, i, route_list )
        t = traci.simulation.getTime()
        traci.simulationStep()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

    traci.close()
    sys.stdout.flush()

    # step = 0
    # we start with phase 2 where EW has green
    # traci.trafficlight.setPhase("0", 2)
    # while traci.simulation.getMinExpectedNumber() > 0:
    #     traci.simulationStep()
    #     if traci.trafficlight.getPhase("0") == 2:
    #         # we are not already switching
    #         if traci.inductionloop.getLastStepVehicleNumber("0") > 0:
    #             # there is a vehicle from the north, switch
    #             traci.trafficlight.setPhase("0", 3)
    #         else:
    #             # otherwise try to keep green for EW
    #             traci.trafficlight.setPhase("0", 2)
    #     step += 1
    # traci.close()
    # sys.stdout.flush()


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

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "ITSC2020_CAV_impact/workspace/M50_simulation.sumo.cfg",
                 "--tripinfo-output", "tripinfo.xml", "--fcd-output", "fcd.xml", "--delay", "1000"])
    run()
