#################################################################################################################

 
 ##########   ###         ##     ##      #########                  ##         ######
 # ##    ##    ##         ##     ##     ##     ##                  # #           #
   ##   ##     ##         ##     ##      ##                       #  #          #
   ##  ##      ##         ##     ##       ##                     #   #         #
   ####        ##         ##     ##        ##                   ######        #
   #           ##         ##     ##         ##                 #     #       #
   #           ##     ##  ##     ##   ##    ##       ###      #      #      #
  ###          #######      #####    ########        ###    ###     ###  #######


#################################################################################################################
import json
from typing import Any
from dataclasses import dataclass, field
import xml.etree.ElementTree as ET
import numpy as np
from pyeulerspiral.eulerspiral import eulerspiral
import datetime
from xml.dom import minidom
import argparse
from math import *
import math
from copy import deepcopy
from scipy import integrate

def to_string(data):
    return str(data)

def NormalizeAngle(theta: float):
    while theta > pi:
        theta = theta - 2 * pi
    while theta < -pi:
        theta = theta + 2 * pi
    return theta

def Y(tau: float, A: float, fai1: float, fai0: float, deg: int):
    a = 2 *A
    b = fai1 - fai0 - A
    c = fai0
    
    return tau**deg * sin(a * tau**2 / 2 + b*tau + c)

def X(tau: float, A: float, fai1: float, fai0: float, deg: int):
    a = 2*A
    b = fai1 - fai0 - A
    c = fai0

    return tau**deg * cos(a * tau**2 / 2 + b * tau + c)


@dataclass
class JsonFile:
    Json: Any = None
    
    def __post_init__(self):
        self.name = None
        self.Map = self.Json['maps']
        self.Junctions = self.Json['junctions']

class Predecessor:
    def __init__(self, idx):
        self.elementId = idx
        self.elementType = 'road'
        self.contactPoint = 'end'

class Successor:
    def __init__(self, idx):
        self.elementId = idx
        self.elementType = 'road'
        self.contactPoint = 'start'

@dataclass
class Elevation:
    s: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0

class TreeObjects:
    
    def __init__(self):
        self.tag = 'Tree'
        self.repeat = {
            's': None,
            'length': None,
            'distance': 11.0,
            'tStart': None,
            'tEnd': None,
            'widthStart': None,
            'widthEnd': None,
            'heightStart': None,
            'heightEnd': None,
            'zOffsetStart': None,
            'zOffsetEnd': None
        }
        self.object = {
            'type':'tree',
            'name':"VegBushGroupC5m.flt",
            'id': 0,
            's': None,
            't': None,
            'zOffset':0.0,
            'validLength': 0.0,
            'orientation': "none",
            'length': 9.746,
            'width': 9.669,
            'height': 1.5815,
            'hdg': None,
            'pitch': None,
            'roll': None
        }

class OutBarrierObjects:

    def __init__(self):
        self.tag ='OutBarrier'
        self.object = {
            'type':'barrier',
            'name':"RdCrashBarrier8m.flt",
            'id': 0,
            's': None,
            't': None,
            'zOffset':0.0,
            'validLength': 0.0,
            'orientation': "none",
            'length': 17.48,
            'width': 0.68,
            'height': 0.81,
            'hdg': None,
            'pitch': None,
            'roll': None
        }
        self.repeat = {
            's': None,
            'length': None,
            'distance': 5.0,
            'tStart': None,
            'tEnd': None,
            'widthStart': None,
            'widthEnd': None,
            'heightStart': None,
            'heightEnd': None,
            'zOffsetStart': None,
            'zOffsetEnd': None
        }

class InnerBarrierObjects:

    def __init__(self):
        self.tag = 'InnerBarrier'
        self.object = {
            'type':'barrier',
            'name':"RdCrashBarrier8m.flt",
            'id': 0,
            's': None,
            't': None,
            'zOffset':0.0,
            'validLength': 0.0,
            'orientation': "none",
            'length': 8.0,
            'width': 0.949,
            'height': 1.2,
            'hdg': None,
            'pitch': None,
            'roll': None
        }
        self.repeat = {
            's': None,
            'length': None,
            'distance': 9.0,
            'tStart': None,
            'tEnd': None,
            'widthStart': None,
            'widthEnd': None,
            'heightStart': None,
            'heightEnd': None,
            'zOffsetStart': None,
            'zOffsetEnd': None
        }

class Bush:

    def __init__(self):
        self.tag = 'Bush'
        self.object = {
            'type':'vegetation',
            'name':"VegBush06.flt",
            'id': 0,
            's': None,
            't': None,
            'zOffset':0.0,
            'validLength': 0.0,
            'orientation': "none",
            'length': 2.941,
            'width': 2.941,
            'height': 3.16,
            'hdg': None,
            'pitch': None,
            'roll': None
        }
        self.repeat = {
            's': None,
            'length': None,
            'distance': 3.1,
            'tStart': None,
            'tEnd': None,
            'widthStart': None,
            'widthEnd': None,
            'heightStart': None,
            'heightEnd': None,
            'zOffsetStart': None,
            'zOffsetEnd': None
        }

Objects = {'LTree': TreeObjects(), 'LOutBarrier': OutBarrierObjects(),
            'Bush': Bush(), 'LInBarrier': InnerBarrierObjects(), 'RTree': TreeObjects(), 'ROutBarrier': OutBarrierObjects(), 'RInBarrier': InnerBarrierObjects()}

class RoadObject:
    
    def __init__(self, typ):
        self.tag = typ
        self.object = Objects[typ]


class RoadLink:

    def __init__(self, road):
        if road.parent:
            if 'junction' in road.parent:
                self.predecessor = Predecessor(road.junction)
                self.predecessor.contactPoint = None
                self.predecessor.elementType = 'junction'
            else:
                self.predecessor = Predecessor(road.parent)
        else:
            self.predecessor = None

        if road.child:
            if 'junction' in road.child:
                self.successor = Successor(road.junction)
                self.successor.contactPoint = None
                self.successor.elementType = 'junction'
            else:
                self.successor = Successor(road.child)
        else:
            self.successor = None

@dataclass
class LaneWidth:
    sOffset: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0

@dataclass
class RoadMark:
    typ: str = None
    weight = 'standard'
    color ='standard'
    width = 0.17
    height = 1.9999999552965164E-2
    sOffset = 0.0
    def __post_init__(self):
        if self.typ == 'solid':
            self.laneChange = 'none'
            self.space = 0.0
            self.rule = 'no passing'
            self.length = 0.0
        if self.typ == 'broken':
            self.laneChange = 'both'
            self.space = 8.0
            self.rule = 'caution'
            self.length = 4.0
        if self.typ == 'none':
            self.laneChange = 'both'
            self.space = 0.0
            self.rule = 'none'
            self.length = 0.0

class Lane:

    def __init__(self, idx, typ, level='False'):
        self.idx = idx
        self.typ = typ
        self.level = level
        self.link = {
            "predecessor": None,
            "successor": None
        }
        self.width =[]
        self.__roadmark = None
        self.speed = None

    @property
    def roadmark(self):
        return self.__roadmark
    
    @roadmark.setter
    def roadmark(self, *args):
        assert args[0] in ['solid', 'broken', 'none'], f"invalid road mark {args[0]}"
        self.__roadmark = RoadMark(typ=args[0])
    
@dataclass
class Geometry:

    idx: int = None
    typ: str = None
    s: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    hdg: float = 0.0
    length: float = 0.0
    elevation: 'Elevation' = Elevation()
    curStart: float = 0.0
    curEnd: float = 0.0
    av: float = 0.0
    bv: float = 0.0
    cv: float = 0.0
    dv: float = 0.0
    au: float = 0.0
    bu: float = 0.0
    cu: float = 0.0
    du: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    d: float = 0.0



class PlanView:
    def __init__(self):
        self.geometry = []
    
    def add(self, geo: "Geometry"):
        self.geometry.append(geo)

@dataclass
class Lanes:
    laneSection_s: float = 0.0
    left: list = field(default_factory=list)
    right: list = field(default_factory=list)
    center: list = field(default_factory=list)
    length: float = 0.0
    l_width: float = 0.0
    r_width: float = 0.0
    attr: str = None
    leftobjects: list = None
    rightobjects: list = None

    def add(self, lane:'Lane', *args):
        if args[0] == 'left':
            self.left.append(lane)
        elif args[0] == 'right':
            self.right.append(lane)
        elif args[0] == 'center':
            self.center.append(lane)
        else:
            raise ValueError('invalid lane type, lane cannot be added!')
    
    def CreateLeftLaneBorder(self):
        LeftLaneBorder = Lane(len(self.left) + 1, 'none')
        lane_width = LaneWidth()
        lane_width.sOffset = 0.0
        lane_width.a = 1.0
        lane_width.b = 0.0
        lane_width.c = 0.0
        lane_width.d = 0.0
        LeftLaneBorder.width.append(lane_width)
        LeftLaneBorder.roadmark = 'none'
        self.add(LeftLaneBorder, 'left')

        
    def CreateRightLaneBorder(self):
        RightLaneBorder = Lane(-len(self.right) -1, 'none')
        lane_width = LaneWidth()
        lane_width.sOffset = 0.0
        lane_width.a = 1.0
        lane_width.b = 0.0
        lane_width.c = 0.0
        lane_width.d = 0.0
        RightLaneBorder.width.append(lane_width)
        RightLaneBorder.roadmark = 'none'
        self.add(RightLaneBorder, 'right')
    
    def InsertAuxliaryLane(self, idx, position, *width):
        if position == 'left':
            if not width:
                lane = Lane(idx, 'none')
            else:
                lane = Lane(idx, 'driving')
            self.left.insert(idx-1, lane)
            for i in range(idx, len(self.left)):
                self.left[i].idx += 1
        if position == 'right':
            if not width:
                lane = Lane(-idx, 'none')
            else:
                lane = Lane(-idx, 'driving')
            self.right.insert(idx-1, lane)
            for i in range(idx, len(self.right)):
                self.right[i].idx -= 1
        lane_width = LaneWidth()
        lane_width.sOffset = 0.0
        if not width:
            lane_width.a = 1.0
        else:
            lane_width.a = width[0]
        lane_width.b = 0.0
        lane_width.c = 0.0
        lane_width.d = 0.0
        lane.width.append(lane_width)
        if not width:
            lane.roadmark = 'none'
        else:
            lane.roadmark = 'solid'
    
    def PolyInsertAuxliaryLane(self, idx, position, geometry):
        if position == 'left':
            lane = Lane(idx, 'driving')
            self.left.insert(idx-1, lane)
            for i in range(idx, len(self.left)):
                self.left[i].idx += 1
        if position == 'right':
            lane = Lane(-idx, 'driving')
            self.right.insert(idx-1, lane)
            for i in range(idx, len(self.right)):
                self.right[i].idx -= 1
        s0 = 0.0
        s1 = self.length
        if self.attr == 'Shrink':
            w1 = 0.0
            w0 = 3.0
        elif self.attr == 'Extend':
            w1 = 3.0
            w0 = 0.0
            
        in_geometry = None
        for geo in geometry.values():
            if self.laneSection_s >= geo.s:
                in_geometry = geo
                break
        spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(in_geometry.length, in_geometry.curStart, in_geometry.curEnd)
        x0, y0, hdg0 = spiral.calc(s0, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
        x1, y1, hdg1 = spiral.calc(s1, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
        rot = hdg1 - hdg0
        a, b, c, d = HermiteInterplotation(s0, w0, 0.0, s1, w1, rot, 100)

        lane_width = LaneWidth()
        lane_width.sOffset = 0.0
        lane_width.a = a
        lane_width.b = b
        lane_width.c = c
        lane_width.d = d
        lane.width.append(lane_width)
        lane.roadmark = 'none'
            


@dataclass
class Road:

    typ: str = None
    idx: int = None
    links: list = None
    laneinfos: list = None
    objects: list = field(default_factory=dict)
    signals: list = field(default_factory=list)
    junction: str = '-1'
    lanes: dict = field(default_factory=dict)
    name = ''
    starter: str = None
    builders: list = None
    elevations: list = None
    geometry: dict = field(default_factory=dict)
    heading: float = None
    direction: str = None
    reverse: bool = False

    def __post_init__(self):
        self.parent = {}
        self.child = {}
        self.lanelinks = {}
        self.unknown = False
        self.gradualwidth = False
        self.normalstart = True
        self.lanesections = {}
        self.ElevationStart = 0.0
        self.ElevationStartHdg = 0.0
        self.ElevationEnd = 0.0
        self.ElevationEndHdg = 0.0
        self.ElevationResult = {}
        if self.links[0]['parent'] == 'None':
            self.parent = None
        else:
            self.parent = self.links[0]['parent']
        if self.links[0]['child'] == 'None':
            self.child = None
        else:
            self.child = self.links[0]['child']
        if 'lanelinks' in self.links[0]:
            self.lanelinks = self.links[0]['lanelinks']

        if self.starter:
            self.start = {'x': float(self.starter[0].split('=')[-1]),
                        'y': float(self.starter[1].split('=')[-1])}
        assert self.builders, f'Current road {self.idx} has no road builders, Terminating generation'
        self.roadgeometry = {}
        idx = 0
        for builder in self.builders:
            if builder == 'unknown':
                self.unknown = True
                break
            typ = builder.split(':')[0]
            info = builder.split(':')[-1]
            attri = info.split(',')
            attribute = {}
            for att in attri:
                k, v = att.split('=')
                attribute[k] = v
            self.roadgeometry[(idx, typ)] = attribute
            idx += 1   

        if self.elevations:
            idx = 0
            self.elevation = {}
            for ele in self.elevations:
                self.elevation[idx] = {}
                attrs = ele.split(',')
                for attr in attrs:
                    k, v = attr.split('=')
                    self.elevation[idx][k] = v 
                idx += 1   
    
        if self.laneinfos:
            for section in self.laneinfos[0]:
                self.lanesections[section] = {}
                sectioninfos = self.laneinfos[0][section]
                left_idx = 0
                right_idx = 0
                for sectioninfo in sectioninfos:
                    if 'left' in sectioninfo:
                        objects = []
                        ltree, loutb, linb, lbush = TreeObjects(), OutBarrierObjects(), InnerBarrierObjects(), Bush()
                        objects.append(ltree)
                        objects.append(loutb)
                        objects.append(linb)
                        objects.append(lbush)
                        if not 'left' in self.lanesections[section]:
                            self.lanesections[section]['left'] = {}
                        items = sectioninfo.split(',')
                        self.lanesections[section]['left'][left_idx] = {}
                        for item in items:
                            k, v = item.split('=')
                            self.lanesections[section]['left'][left_idx][k] = v
                        left_idx += 1
                        self.lanesections[section]['objectsleft'] = objects
                        
                    else:
                        objects = []
                        rtree, routb, rinb, rbush = TreeObjects(), OutBarrierObjects(), InnerBarrierObjects(), Bush()
                        objects.append(rtree)
                        objects.append(routb)
                        objects.append(rinb)
                        objects.append(rbush)
                        if not 'right' in self.lanesections[section]:
                            self.lanesections[section]['right'] = {}
                        items = sectioninfo.split(',')
                        self.lanesections[section]['right'][right_idx] = {}
                        for item in items:
                            k, v = item.split('=')
                            self.lanesections[section]['right'][right_idx][k] = v
                        right_idx += 1
                        self.lanesections[section]['objectsright'] = objects

    def GenerateXYS(self):
        # Now we have one road geometry, then we recalc the geometry x, y, s, heading
        geo_idxes = sorted(self.geometry.keys())
        for geo_id in geo_idxes:
            if geo_id == 0:
                # the very beginning of current road, ignore it
                continue
            else:
                self.geometry[geo_id].s = self.geometry[geo_id - 1].s + self.geometry[geo_id - 1].length
                spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(self.geometry[geo_id - 1].length, self.geometry[geo_id - 1].curStart, self.geometry[geo_id - 1].curEnd)
                x, y, hdg = spiral.calc(self.geometry[geo_id - 1].length, self.geometry[geo_id - 1].x, self.geometry[geo_id - 1].y, self.geometry[geo_id - 1].curStart, self.geometry[geo_id - 1].hdg)
                self.geometry[geo_id].x = x
                self.geometry[geo_id].y = y
                self.geometry[geo_id].hdg = hdg

    def GenerateEnding(self):
        x = self.geometry[len(self.geometry) - 1].x
        y = self.geometry[len(self.geometry) - 1].y
        curstart = self.geometry[len(self.geometry) - 1].curStart
        curend = self.geometry[len(self.geometry) - 1].curEnd
        length = self.geometry[len(self.geometry) - 1].length
        hdg = self.geometry[len(self.geometry) - 1].hdg
        spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(length, curstart, curend)
        return spiral.calc(length, x, y, curstart, hdg)
    
    def GenerateElevations(self):
        start_s = 0.0
        start_height = self.ElevationStart
        start_hdg = self.ElevationStartHdg
        end_height = None
        end_hdg = None
        for idx in self.elevation:
            end_height = start_height + eval(self.elevation[idx]['dist']) * eval(self.elevation[idx]['slope'])
            end_s = start_s + eval(self.elevation[idx]['dist']) 
            end_hdg = atan(eval(self.elevation[idx]['slope']))

            self.ElevationResult[idx] = []
            ea, eb, ec, ed = HermiteInterplotation(0.0, start_height, start_hdg, eval(self.elevation[idx]['dist']), end_height, end_hdg, 1000)
            self.ElevationResult[idx].extend([start_s, ea, eb, ec, ed])
            start_height, start_s, start_hdg = end_height, end_s, end_hdg
            
        self.ElevationEnd = end_height
        self.ElevationEndHdg = end_hdg
    
    def GenerateElevationsForRamp(self, p, c):
        start_s = 0.0
        start_height = p.ElevationEnd
        start_hdg = p.ElevationEndHdg
        end_s = start_s
        for geo in self.geometry:
            end_s += self.geometry[geo].length
        end_hdg = c.ElevationEndHdg + np.pi
        end_height = c.ElevationEnd
        ea, eb, ec, ed = HermiteInterplotation(start_s, start_height, start_hdg, end_s, end_height, end_hdg, 1000)
        self.ElevationResult[0] = []
        self.ElevationResult[0].extend([0.0, ea, eb, ec, ed]) 
        


class OpenDRIVE:

    def __init__(self, jsonfile: 'JsonFile'):
        self.jsonfile = jsonfile
        self.roads = {}
        self.junctions = {}
        self.main_road_connections = {}
        self.starters = []      # find all road starters for current map
        self.junctions_type = self.jsonfile.Junctions


        for i, road in enumerate(self.jsonfile.Map):
            if road['type'] == 'connector':
                if road['junction'] in self.junctions:
                    # junction exists
                    # append connecting road to junction, specify the connection relationship
                    self.junctions[road['junction']].append(road['id'])
                
                else:
                    # no corresponding junction found, create it !
                    print(f"Sucessfully created a junction {road['junction']}")
                    self.junctions[road['junction']] = [road['id']]
                
                if 'elevations' in road:
                    r = Road(typ=road['type'], 
                            idx=road['id'],
                            junction=road['junction'],
                            links=road['links'],
                            laneinfos=road['lanes'],
                            builders=road['builders'],
                            elevations=road['elevations'])
                else:
                    r = Road(typ=road['type'], 
                            idx=road['id'],
                            junction=road['junction'],
                            links=road['links'],
                            laneinfos=road['lanes'],
                            builders=road['builders'])

                    
                # check if connector only has one parent road and one child road:
                assert isinstance(eval(r.parent), int) and isinstance(eval(r.child), int) , f"Error: Connection error, connector {r.idx} must only has one parent and one child road!, Information {r.parent}, {r.child}, {len(r.parent)}, {len(r.child)}"
                
            else:
                # check if current road is connected to junction:
                if 'Junction' in road['links'][0]['parent']:
                    if road['links'][0]['parent'] in self.junctions:
                        self.junctions[road['links'][0]['parent']].append(road['id'])
                    else:
                        # create it!!!
                        print(f"Sucessfully created a parent junction {road['links'][0]['parent']}")
                        self.junctions[road['links'][0]['parent']] = [road['id']]
                if 'Junction' in road['links'][0]['child']:
                    if road['links'][0]['child'] in self.junctions:
                        self.junctions[road['links'][0]['child']].append(road['id'])
                    else:
                        # create it!!!
                        print(f"Sucessfully created a child junction {road['links'][0]['child']}")
                        self.junctions[road['links'][0]['child']] = [road['id']]
                
                # since the non-connector road might connected to different junctions, here we dont specify which junction current road belongs to.
                # Notice that connectors obviously not the starter of current map, need to dertemine the map starters
                if 'starter' in road:
                    # current road in the starter of current map
                    if 'elevations' in road:
                        r = Road(typ=road['type'],
                                idx=road['id'],
                                links=road['links'],
                                laneinfos=road['lanes'],
                                builders=road['builders'],
                                elevations=road['elevations'],
                                starter=road['starter'],
                                heading=road['heading'],
                                direction=road['direction'])
                    else:
                        r = Road(typ=road['type'],
                                idx=road['id'],
                                links=road['links'],
                                laneinfos=road['lanes'],
                                builders=road['builders'],
                                starter=road['starter'],
                                heading=road['heading'],
                                direction=road['direction'])

                    self.starters.append(road['id'])
                
                else:
                    if 'elevations' in road:
                        r = Road(typ=road['type'],
                                idx=road['id'],
                                links=road['links'],
                                laneinfos=road['lanes'],
                                builders=road['builders'],
                                elevations=road['elevations'],
                                direction=road['direction'])
                    else:
                        r = Road(typ=road['type'],
                                idx=road['id'],
                                links=road['links'],
                                laneinfos=road['lanes'],
                                builders=road['builders'],
                                direction=road['direction'])

                for j_id in self.junctions:
                    if r.idx in self.junctions[j_id]:
                        r.junction = j_id
            
            self.roads[road['id']] = r
    
    def NewRoad(self):
        return len(self.roads) + 1
    
    def SpecifyMainRoadConnections(self):
        """

        To find all coonectors belongs to current main road, to do this simplify the look up process of road connections, and road geometry generation
        """
        for r_id, road in self.roads.items():
            for jc_id in self.junctions:
                if r_id in self.junctions[jc_id] and road.typ != 'connector':
                    # road in junction, find all connector it connected with
                    for jcr_id in self.junctions[jc_id]:
                        if self.roads[jcr_id].typ == 'connector':
                            # successfully find an connector
                            if r_id not in self.main_road_connections:
                                self.main_road_connections[r_id] = [jcr_id]
                            else:
                                # chech if the same connector in different junction
                                assert jcr_id not in self.main_road_connections[r_id], "Error: Same connector in different junction!!!"
                                self.main_road_connections[r_id].append(jcr_id)
                    
    def CreateRoad(self):
        
        # since in init, we have identified the start road of current map, now we just need to find a continous road segment in map
        # we complete it depend on connector geo info

        def DFS(self, S:'Road'):
            if S.child:
                # exist child
                if 'Junction' in S.child:
                    # use in because of 'junction_xxx'
                    rawsuccessorroads = deepcopy(self.main_road_connections[S.idx])  # get connection road id from junction connections
                    # Notice that there is connection road, whose child is current Starter, it's necessary to avoid circular recursion
                    for successor in rawsuccessorroads:
                        if self.roads[successor].child:
                            # exist child
                            if str(S.idx) == self.roads[successor].child:
                                # use '==' because child could only be one road or junction
                                print(f"Warning: found circular recursion, parent {successor}, avoiding it!")
                                rawsuccessorroads.remove(successor)
                    successorroads = rawsuccessorroads
                else:
                    # if one road has child, which is not junction, must has only one successor road
                    assert isinstance(eval(S.child), int), f"Error: current road {S.idx} has more than one child road, which is not on any junction"
                    if self.roads[int(S.child)].geometry:
                        print(f"Warning: found circular recursion, parent {S.idx}, avoiding it!")
                        return
                    else:
                        successorroads = [int(S.child)]
                while successorroads:
                    Starter = successorroads[-1]
                    if self.roads[Starter].unknown:
                        # rule out the roads, which should be euler-generated!
                        successorroads.pop()
                        continue
                    self.CreateRoadGeoFromStarter(self.roads[Starter])
                    DFS(self, self.roads[Starter])
                    successorroads.pop()
            
        for starter in self.starters:

            # 1. build geometry for current road depending on parent
            # 2. give init road info for child road
            Starter = starter
            self.CreateRoadGeoFromStarter(self.roads[Starter])

            # DFS
            DFS(self, self.roads[Starter])
        
        print('*' * 100)
        print('Complete: opendrive Roads have been created!')
        print('*' * 100)
    
    def CreateRoadGeoFromStarter(self, Starter: 'Road'):
        # TODO: Generate road geometry depend on parent road
        if Starter.starter:
            s = 0.0
            x = Starter.start['x']
            y = Starter.start['y']
            for id_g, typ_g in Starter.roadgeometry:
                if typ_g == 'line':
                    Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x, y=y, z=0.0, hdg=Starter.heading, length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']), curStart=0.0, curEnd=0.0)
                elif typ_g == 'arc':
                    Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x, y=y, z=0.0, hdg=Starter.heading, length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']),
                     curStart=float(eval(Starter.roadgeometry[(id_g, typ_g)]['curvature'])), curEnd=float(eval(Starter.roadgeometry[(id_g, typ_g)]['curvature'])))
            for id_g, typ_g in Starter.roadgeometry:
                if typ_g == 'spiral':
                    assert  0 < id_g < len(Starter.roadgeometry) - 1, "Spiral is only used for connection between line and arc, should not appear at the beginning or the ending of a road geometry "
                    Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x, y=y, z=0.0, hdg=Starter.heading, length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']),
                     curStart=Starter.geometry[id_g - 1].curEnd, curEnd=Starter.geometry[id_g + 1].curStart)
            # recalc the s, x, y, 
            Starter.GenerateXYS()
        else:
            # verify the direction and starten offset
            if Starter.lanelinks:
                offset_idx = []
                for link in Starter.lanelinks:
                    roadinfo, laneinfo = link.split(':')
                    roadinfo = eval(roadinfo)
                    laneinfo = eval(laneinfo)
                    if Starter.idx == roadinfo[1]:
                        # children
                        offset_idx.append(abs(laneinfo[0]))
                        if laneinfo[0] * laneinfo[1] < 0:
                            Starter.reverse = True
                            print(f'current road is reverse road {Starter.idx}')
                            REVERSEROAD = self.roads[roadinfo[0]]
                            break
                if min(offset_idx) != 1:
                    Starter.normalstart = False
                        
            if Starter.normalstart:
                # if has parent(main road has parent)
                data = []
                parents = [] # store the parent
                if 'Junction' in Starter.parent:
                    # if current road has more than one parent road, which is defined as junction
                    # roads has geometry, which has been pre-determined, check the end point satisification
                    # need to find all connectors in junction, which are linked to current road.
                    connectors = self.main_road_connections[Starter.idx]
                    for connector_id in connectors:
                        connector = self.roads[connector_id] # connector road
                        if connector.child:
                            if str(Starter.idx) == connector.child and connector.geometry:
                                # means that successfully found a parent road
                                parents.append(connector)
                        
                else:
                    # Now current road has only one parent road
                    parent_id = int(Starter.parent)
                    parents.append(self.roads[parent_id])
                
                for parent in parents:
                    if not parent.unknown:
                        data.append(self.GenerateParentEnding(parent))
                
                
                ParentHasSameEndingPoint = True
                X = np.zeros(len(data))
                Y = np.zeros(len(data))
                H = np.zeros(len(data))
                for i, d in enumerate(data):
                    X[i] = d[0]
                    Y[i] = d[1]
                    H[i] = d[2]
                if not (len(np.unique(X)) == 1 and len(np.unique(Y)) == 1 and len(np.unique(H)) == 1):
                    ParentHasSameEndingPoint = False
                
                if not ParentHasSameEndingPoint:
                    raise RuntimeError("current road has mutiple parent road, which dont have the same ending point")
                
                # pass the checking
                if Starter.reverse:
                    s = 0.0
                    x = REVERSEROAD.geometry[0].x
                    y = REVERSEROAD.geometry[0].y
                    hdg = np.pi + REVERSEROAD.geometry[0].hdg
                    for id_g, typ_g in Starter.roadgeometry:
                        if typ_g == 'line':
                            Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g,  s=s, x=x, y=y, z=0.0, hdg=hdg, length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']), curStart=0.0, curEnd=0.0)
                        elif typ_g == 'arc':
                            Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x, y=y, z=0.0, hdg=hdg, length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']),
                            curStart=float(Starter.roadgeometry[(id_g, typ_g)]['curvature']), curEnd=float(Starter.roadgeometry[(id_g, typ_g)]['curvature']))
                    for id_g, typ_g in Starter.roadgeometry:
                        if typ_g == 'spiral':
                            assert  0 < id_g < len(Starter.roadgeometry) - 1, "Spiral is only used for connection between line and arc, should not appear at the beginning or the ending of a road geometry "
                            Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x, y=y, z=0.0, hdg=hdg, length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']),
                            curStart=Starter.geometry[id_g - 1].curEnd, curEnd=Starter.geometry[id_g + 1].curStart)

                else:
                    s = 0.0
                    x = np.unique(X)
                    y = np.unique(Y)
                    hdg = np.unique(H)
                    for id_g, typ_g in Starter.roadgeometry:
                        if typ_g == 'line':
                            Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x[0], y=y[0], z=0.0, hdg=hdg[0], length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']), curStart=0.0, curEnd=0.0)
                        elif typ_g == 'arc':
                            Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x[0], y=y[0], z=0.0, hdg=hdg[0], length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']),
                            curStart=float(Starter.roadgeometry[(id_g, typ_g)]['curvature']), curEnd=float(Starter.roadgeometry[(id_g, typ_g)]['curvature']))
                    for id_g, typ_g in Starter.roadgeometry:
                        if typ_g == 'spiral':
                            assert  0 < id_g < len(Starter.roadgeometry) - 1, "Spiral is only used for connection between line and arc, should not appear at the beginning or the ending of a road geometry "
                            Starter.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x[0], y=y[0], z=0.0, hdg=hdg[0], length=float(Starter.roadgeometry[(id_g, typ_g)]['dist']),
                            curStart=Starter.geometry[id_g - 1].curEnd, curEnd=Starter.geometry[id_g + 1].curStart)
                print(f'Successfully build road geometry for starter {Starter.idx}')
                # recalc the s, x, y, 
                Starter.GenerateXYS()     
    
    def GenerateParentEnding(self, parentroad: 'Road'):

        # Notice that parent geometry is known
        
        return parentroad.GenerateEnding()
    
    def PostRoadGeneration(self):
        for road in self.roads.values():
            if not road.geometry:
                if not road.normalstart:
                    # merge and split point
                    parent = self.roads[int(road.parent)]
                    child = self.roads[int(road.child)]
                    offset = 0.0
                    s = 0.0
                    x0 = None
                    y0 = None
                    hdg0 = None
                    if road.lanelinks:
                        for link in road.lanelinks:
                            roadinfo, laneinfo = link.split(':')
                            roadinfo = eval(roadinfo)
                            laneinfo = eval(laneinfo)
                            if road.idx == roadinfo[1]:
                                #child
                                offset_lane_nums = laneinfo[0]
                                break

                    if 'Merge' in road.junction:
                        if offset_lane_nums > 0:
                            lanesection = list(parent.lanesections.values())[-1]
                            x0, y0, hdg0 = parent.GenerateEnding()
                            N0 = hdg0 + np.pi / 2
                        else:
                            lanesection = list(parent.lanesections.values())[0]
                            x0, y0, hdg0 = parent.geometry[0].x, parent.geometry[0].y, parent.geometry[0].hdg
                            N0 = hdg0 - np.pi / 2
                            hdg0 += np.pi
                    elif 'Split' in road.junction:
                        if offset_lane_nums > 0:
                            lanesection = list(parent.lanesections.values())[0]
                            x0, y0, hdg0 = parent.geometry[0].x, parent.geometry[0].y, parent.geometry[0].hdg
                            N0 = hdg0 + np.pi / 2
                            hdg0 += np.pi
                        if offset_lane_nums < 0:
                            lanesection = list(parent.lanesections.values())[-1]
                            x0, y0, hdg0 = parent.GenerateEnding()
                            N0 = hdg0 - np.pi / 2                    

                    if offset_lane_nums > 0:
                        # left lane offset
                        for item in lanesection['left']:
                            if abs(offset_lane_nums) - 1 > int(lanesection['left'][item]['num']):
                                offset += int(lanesection['left'][item]['num']) * float(lanesection['left'][item]['width'])
                                offset_lane_nums -= int(lanesection['left'][item]['num'])
                            else:
                                offset += abs(offset_lane_nums) - 1 * float(lanesection['left'][item]['width'])
                                break
                    else:
                        for item in lanesection['right']:
                            if abs(offset_lane_nums) - 1 > int(lanesection['right'][item]['num']):
                                offset += int(lanesection['right'][item]['num']) * float(lanesection['right'][item]['width'])
                                offset_lane_nums += int(lanesection['right'][item]['num'])
                            else:
                                offset += abs(offset_lane_nums) - 1 * float(lanesection['right'][item]['width'])
                                break
                    offset += 3.0
                    x0 += offset * np.cos(N0)
                    y0 += offset * np.sin(N0)
                    
                    for id_g, typ_g in road.roadgeometry:
                        if typ_g == 'line':
                                road.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x0, y=y0, z=0.0, hdg=hdg0, length=float(road.roadgeometry[(id_g, typ_g)]['dist']), curStart=0.0, curEnd=0.0)
                        elif typ_g == 'arc':
                            road.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x0, y=y0, z=0.0, hdg=hdg0, length=float(road.roadgeometry[(id_g, typ_g)]['dist']),
                            curStart=float(road.roadgeometry[(id_g, typ_g)]['curvature']), curEnd=float(road.roadgeometry[(id_g, typ_g)]['curvature']))
                    for id_g, typ_g in road.roadgeometry:
                        if typ_g == 'spiral':
                            assert  0 < id_g < len(road.roadgeometry) - 1, "Spiral is only used for connection between line and arc, should not appear at the beginning or the ending of a road geometry "
                            road.geometry[id_g] = Geometry(idx=id_g, typ=typ_g, s=s, x=x0, y=y0, z=0.0, hdg=hdg0, length=float(road.roadgeometry[(id_g, typ_g)]['dist']),
                            curStart=road.geometry[id_g - 1].curEnd, curEnd=road.geometry[id_g + 1].curStart)
                    
                    road.GenerateXYS()

        for road in self.roads.values():
            if not road.geometry:
                if not road.unknown:
                    self.CreateRoadGeoFromStarter(road)
                else:
                    parent = road.parent
                    child = road.child
                    if 'Junction' in parent and 'Junction' in child:
                        # parent and child are both junction and the unknown length, it must be a ramp
                        connectors = self.main_road_connections[road.idx]
                        parent_and_child = []
                        for connector in connectors:
                            if eval(self.roads[connector].child) == road.idx:
                                parent_and_child.append(connector)
                        assert len(parent_and_child), 'Current ramp has more than two parents or children, not allowed!'
                    road.geometry = EulerRoadGeometry(self.roads[int(parent_and_child[0])], self.roads[int(parent_and_child[1])])
                    for ls in road.lanes:
                        if 'left' in road.lanesections['ls']:
                            road.lanes[ls].InsertAuxliaryLane(1, 'left')
                            break
                        if 'right' in road.lanesections['ls']:
                            road.lanes[ls].InsertAuxliaryLane(1, 'right')
                            break
                    road.GenerateElevationsForRamp(self.roads[int(parent_and_child[0])], self.roads[int(parent_and_child[1])])

                # TODO: check if the reserver it not generated!
                # for road reservers
                

    
    def GenerateLanes(self):
        # generate lanes for main road(non_connector)
        for road in self.roads.values():
            if road.typ == 'non_connector' and not road.unknown:
                # note that for each lane section, we create a new Lanes object
                self.GenerateLanesForRoad(road, 'bio')
    
    def GenerateLanesForRoad(self, road: 'Road', *args):
        s_offset = 0.0
        for lanesection in road.laneinfos[0]:
            attr = None
            next_lane_nums, pre_lane_nums = None, None
            if 'left' in road.lanesections[chr(ord(lanesection))]:
                cur_lane_left_nums = [eval(l['num']) for l in road.lanesections[chr(ord(lanesection))]['left'].values()]
            else:
                cur_lane_left_nums = [0]
            if 'right' in road.lanesections[chr(ord(lanesection))]:
                cur_lane_right_nums = [eval(r['num']) for r in road.lanesections[chr(ord(lanesection))]['right'].values()]
            else:
                cur_lane_right_nums = [0]
            cur_lane_nums = sum(cur_lane_left_nums)+ sum(cur_lane_right_nums)
            try:
                if chr(ord(lanesection)+1) in road.lanesections:
                    if 'left' in road.lanesections[chr(ord(lanesection)+1)]:
                        next_lane_left_nums = [eval(l['num']) for l in road.lanesections[chr(ord(lanesection)+1)]['left'].values()]
                    else:
                        next_lane_left_nums = [0]
                    if 'right' in road.lanesections[chr(ord(lanesection)+1)]:
                        next_lane_right_nums = [eval(r['num']) for r in road.lanesections[chr(ord(lanesection)+1)]['right'].values()]
                    else:
                        next_lane_right_nums = [0]
                    next_lane_nums = sum(next_lane_left_nums)+ sum(next_lane_right_nums)
                if chr(ord(lanesection)-1) in road.lanesections:
                    if 'left' in road.lanesections[chr(ord(lanesection)-1)]:
                        pre_lane_left_nums = [eval(l['num']) for l in road.lanesections[chr(ord(lanesection)-1)]['left'].values()]
                    else:
                        pre_lane_left_nums = [0]
                    if 'right' in road.lanesections[chr(ord(lanesection)-1)]:
                        pre_lane_right_nums = [eval(r['num']) for r in road.lanesections[chr(ord(lanesection)-1)]['right'].values()]
                    else:
                        pre_lane_right_nums = [0]
                    pre_lane_nums = sum(pre_lane_left_nums) + sum(pre_lane_right_nums)
            except KeyError:
                print('Error: Unexpected lane section key appears')
            finally:
                lattr , rattr = 'Constant', 'Constant'
                if next_lane_nums:
                    if next_lane_nums > cur_lane_nums:
                        rattr = 'Extend'
                    elif next_lane_nums < cur_lane_nums:
                        rattr = 'Shrink'
                    else:
                        rattr = 'Constant'
                if pre_lane_nums:
                    if pre_lane_nums < cur_lane_nums:
                        lattr = 'Extend'
                    elif pre_lane_nums > cur_lane_nums:
                        lattr = 'Shrink'
                    else:
                        lattr = 'Constant'
                if lattr == 'Constant' and rattr == 'Constant':
                    attr = 'Constant'
                if lattr != 'Constant':
                    attr = lattr
                if rattr != 'Constant':
                    attr = rattr

                Linfos = road.laneinfos[0][lanesection]
                INFOS = []
                for Linfo in Linfos:
                    Info = Linfo.split(',')
                    INFO = {}
                    for info in Info:
                        k, v = info.split('=')
                        INFO[k] = v
                    INFOS.append(INFO)
                
                if 'objectsleft' in road.lanesections[lanesection] and 'objectsright' in road.lanesections[lanesection]:
                    lanes = Lanes(laneSection_s = s_offset, leftobjects=road.lanesections[lanesection]['objectsleft'], rightobjects=road.lanesections[lanesection]['objectsright']) # initialize lanes object
                else:
                    if 'objectsleft' in road.lanesections[lanesection]:
                        lanes = Lanes(laneSection_s = s_offset, leftobjects=road.lanesections[lanesection]['objectsleft'])
                    elif 'objectsright' in road.lanesections[lanesection]:
                        lanes = Lanes(laneSection_s = s_offset, leftobjects=road.lanesections[lanesection]['objectsright'])
                    else:
                        lanes = Lanes(laneSection_s = s_offset)
                lanes.attr = attr
                if INFOS[0]['dist'] != 'unknown':
                    lanes.length = eval(INFOS[0]['dist'])
                    s_offset += eval(INFOS[0]['dist']) # lanes in the same lane_section share the same extension of geometry
                # in case its a ramp,
                else:
                    lanes.length = road.geometry[0].length
                    s_offset += road.geometry[0].length
                
                s_offset_width_left = 0.0
                left = 0
                s_offset_width_right = 0.0
                right = 0
                for info in INFOS:
                    if 'left' in info:
                        width = 0.0
                        for i in range(eval(info['num'])):
                            lane = Lane(1 + left, 'driving')
                            if info['type'] == 'line':
                                lane_width = LaneWidth(sOffset=s_offset_width_left, a=eval(info['width']))
                                lane.width.append(lane_width)
                                width += eval(info['width'])
                            else:
                                if attr == 'Extend':
                                    diss = eval(info['dist']) / eval(info['num'])
                                    s0 = 0.0
                                    s1 = diss
                                    w0 = 0.0
                                    w1 = eval(info['width'])
                                    width += w1
                                    # identify the start poly angle:
                                    in_geometry = None
                                    for geo in road.geometry.values():
                                        if s_offset > geo.s:
                                            in_geometry = geo
                                            break
                                    spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(in_geometry.length, in_geometry.curStart, in_geometry.curEnd)
                                    x0, y0, hdg0 = spiral.calc(s_offset, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    x1, y1, hdg1 = spiral.calc(s_offset + diss, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    rot = hdg1 - hdg0
                                    a, b, c, d = HermiteInterplotation(s0, w0, 0.0, s1, w1, rot, 100)
                                    if i < eval(info['num']) - 1:
                                        lane_width_base = LaneWidth(sOffset=0.0)
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_left, a=a, b=b, c=c, d=d)
                                        lane_width_1 = LaneWidth(sOffset=eval(info['dist']) - s_offset_width_left - diss, a=eval(info['width']))
                                        lane.width.append(lane_width_base)
                                        lane.width.append(lane_width_0)
                                        lane.width.append(lane_width_1)
                                    else:
                                        lane_width_base = LaneWidth(sOffset=0.0)
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_left, a=a, b=b, c=c, d=d)
                                        lane.width.append(lane_width_base)
                                        lane.width.append(lane_width_0)
                                    s_offset_width_left += diss
                                elif attr == 'Shrink':
                                    diss = eval(info['dist']) / eval(info['num'])
                                    s0 = 0.0
                                    s1 = diss
                                    w0 = eval(info['width'])
                                    w1 = 0.0
                                    width += w0
                                    in_geometry = None
                                    for geo in road.geometry.values():
                                        if s_offset > geo.s:
                                            in_geometry = geo
                                            break
                                    spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(in_geometry.length, in_geometry.curStart, in_geometry.curEnd)
                                    x0, y0, hdg0 = spiral.calc(s_offset, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    x1, y1, hdg1 = spiral.calc(s_offset + diss, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    rot = hdg1 - hdg0
                                    a, b, c, d = HermiteInterplotation(s0, w0, 0.0, s1, w1, rot, 100)
                                    if i < eval(info['num']) - 1:
                                        lane_width_base = LaneWidth(sOffset=0.0)
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_left, a=a, b=b, c=c, d=d)
                                        lane_width_1 = LaneWidth(sOffset=eval(info['dist']) - s_offset_width_left - diss, a=eval(info['width']))
                                        lane.width.append(lane_width_1)
                                        lane.width.append(lane_width_0)
                                        lane.width.append(lane_width_base)
                                    else:
                                        lane_width_base = LaneWidth(sOffset=0.0)
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_left+diss, a=a, b=b, c=c, d=d)
                                        lane.width.append(lane_width_0)
                                        lane.width.append(lane_width_base)
                                    s_offset_width_left += diss
                            left += 1
                            lanes.add(lane, 'left')
                        lanes.l_width = width
                    if 'right' in info:
                        width = 0.0
                        for i in range(eval(info['num'])):
                            lane = Lane(-1 - right, 'driving')
                            if info['type'] == 'line':
                                lane_width = LaneWidth(sOffset=s_offset_width_right, a=eval(info['width']))
                                lane.width.append(lane_width)
                                width += eval(info['width'])
                            else:
                                if attr == 'Extend':
                                    diss = eval(info['dist']) / eval(info['num'])
                                    s0 = 0.0
                                    s1 = diss
                                    w0 = 0.0
                                    w1 = eval(info['width'])
                                    in_geometry = None
                                    width += w1
                                    for geo in road.geometry.values():
                                        if s_offset > geo.s:
                                            in_geometry = geo
                                            break
                                    spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(in_geometry.length, in_geometry.curStart, in_geometry.curEnd)
                                    x0, y0, hdg0 = spiral.calc(s_offset, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    x1, y1, hdg1 = spiral.calc(s_offset + diss, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    rot = hdg1 - hdg0
                                    a, b, c, d = HermiteInterplotation(s0, w0, 0.0, s1, w1, rot, 100)
                                    if i < eval(info['num']) - 1:
                                        lane_width_base = LaneWidth(sOffset=0.0)
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_right, a=a, b=b, c=c, d=d)
                                        lane_width_1 = LaneWidth(sOffset=eval(info['dist']) - s_offset_width_right - diss, a=eval(info['width']))
                                        lane.width.append(lane_width_base)
                                        lane.width.append(lane_width_0)
                                        lane.width.append(lane_width_1)
                                    else:
                                        lane_width_base = LaneWidth(sOffset=0.0)
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_right, a=a, b=b, c=c, d=d)
                                        lane.width.append(lane_width_base)
                                        lane.width.append(lane_width_0)
                                    s_offset_width_right += diss
                                elif attr == 'Shrink':
                                    diss = eval(info['dist']) / eval(info['num'])
                                    s0 = 0.0
                                    s1 = diss
                                    w1 = 0.0
                                    w0 = eval(info['width'])
                                    width += w0
                                    in_geometry = None
                                    for geo in road.geometry.values():
                                        if s_offset > geo.s:
                                            in_geometry = geo
                                            break
                                    spiral = eulerspiral.EulerSpiral.createFromLengthAndCurvature(in_geometry.length, in_geometry.curStart, in_geometry.curEnd)
                                    x0, y0, hdg0 = spiral.calc(s_offset, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    x1, y1, hdg1 = spiral.calc(s_offset + diss, in_geometry.x, in_geometry.y, in_geometry.curStart, in_geometry.hdg)
                                    rot = hdg1 - hdg0
                                    a, b, c, d = HermiteInterplotation(s0, w0, 0.0, s1, w1, rot, 100)
                                    if i < eval(info['num']) - 1:
                                        lane_width_base = LaneWidth(sOffset=0.0, a=eval(info['width']))
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_right, a=a, b=b, c=c, d=d)
                                        lane_width_1 = LaneWidth(sOffset=eval(info['dist']) - s_offset_width_right - diss)
                                        lane.width.append(lane_width_base)
                                        lane.width.append(lane_width_0)
                                        lane.width.append(lane_width_1)
                                    else:
                                        lane_width_base = LaneWidth(sOffset=0.0, a=a, b=b, c=c, d=d)
                                        lane_width_0 = LaneWidth(sOffset=s_offset_width_right + diss)
                                        lane.width.append(lane_width_base)
                                        lane.width.append(lane_width_0)
                                    s_offset_width_right += diss
                            right += 1
                            lanes.add(lane, 'right')
                        lanes.r_width = width

            lane = Lane(0, 'none')
            lane.roadmark = 'none'
            lanes.add(lane, 'center')
            for left in lanes.left:
                if left.idx < len(lanes.left):
                    left.roadmark = 'broken'
                else:
                    left.roadmark = 'solid'
            for right in lanes.right:
                if abs(right.idx) < len(lanes.right):
                    right.roadmark = 'broken'
                else:
                    right.roadmark = 'solid'
            
            # create border for lanes
            if args[0] == 'bio':
                lanes.CreateLeftLaneBorder()
                lanes.CreateRightLaneBorder()
                
            elif args[0] == 'right':
                lanes.CreateRightLaneBorder()
            elif args[0] == 'left':
                lanes.CreateLeftLaneBorder()
            
            if road.typ == 'non_connector':
                lanes.InsertAuxliaryLane(1, 'left')
                lanes.InsertAuxliaryLane(1, 'right')
            else:
                if 'right' in road.lanesections[lanesection]:
                    lanes.InsertAuxliaryLane(1, 'right')
                else:
                    lanes.InsertAuxliaryLane(1, 'left')
            
            # add lanes to road lanes dictionary:
            road.lanes[lanesection] = lanes
    
    def AssertConnectorShareSameStartPoint(self, indexes):
        Starten = {}
        for idx in indexes:
            connector = self.roads[idx]
            x = round(connector.geometry[0].x, 6)
            y = round(connector.geometry[0].y, 6)
            if (x, y) not in Starten:
                Starten[(x, y)] = [idx]
            else:
                Starten[(x, y)].append(idx)
        
        return Starten
        
    
    def PostGenertateLanes(self):
        # for connector lanes generation, typically for connector share the same start point and those not
        for road in self.roads.values():
            # every road should has border
            if road.typ == 'connector':
                connectors = self.main_road_connections[int(road.parent)]
                if road.normalstart:
                    if not self.AssertConnectorInsideJunction(road):
                        self.GenerateLanesForRoad(road, 'bio')
                        print(f'connector {road.idx} not inside junction and normalstart')
                    else:
                        self.GenerateLanesForRoad(road, 'right')
                        print(f'connector {road.idx} inside junction and normalstart')
                else:
                    print(f'connector {road.idx} not inside junction and not share same starten point')
                    for ls in road.lanesections:
                        if 'right' in road.lanesections[ls]:
                            self.GenerateLanesForRoad(road, 'right')
                            break
                        else:
                            self.GenerateLanesForRoad(road, 'left')
                            break
                            
            elif road.typ == 'non_connector' and road.unknown:
                print(f'connector {road.idx} not connector and unknown')
                self.GenerateLanesForRoad(road, 'bio')
            
    
    def CreateConnectorForRamp(self):
        for road in self.roads.values():
            if road.direction:
                if 'Junction' in str(road.parent) and 'Junction' in str(road.child) and road.direction == 'Uni':
                    connectors = self.main_road_connections[road.idx]
                    parent_connector = None
                    for connector_i in connectors:
                        if int(connector_i.child) == road.idx:
                            parent_connector = connector_i
                            ppmainroad = int(parent_connector.parent)
                            target_connector = None
                            for connector_j in connectors:
                                connector_road_i = self.roads[connector_i]
                                connector_road_j = self.roads[connector_j]
                                for lanelinks_i in connector_road_i.lanelinks:
                                    road_info_i, lane_info_i = lanelinks_i.split(',')
                                    for lanelinks_j in connector_road_j.lanelinks:
                                        road_info_j, lane_info_j = lanelinks_j.split(',')
                                        if road_info_i[0] == road_info_j[0] and road_info_i[0] != connector_i and road_info_j[0] != connector_j:
                                            # share the same road parent
                                            # assert whether the connection lane info is different
                                            if eval(lane_info_i[0]) * eval(lane_info_j[0]) > 0:
                                                target_connector = connector_j
                                                break
                                        elif road_info_i[1] == road_info_j[0] and road_info_i[1] != connector_i and road_info_j[0] != connector_j:
                                            if eval(lane_info_i[1]) * eval(lane_info_j[0]) > 0:
                                                target_connector = connector_j
                                                break
                                        elif road_info_i[0] == road_info_j[1] and road_info_i[0] != connector_i and road_info_j[1] != connector_j:
                                            if eval(lane_info_i[0]) * eval(lane_info_j[1]) > 0:
                                                target_connector = connector_j
                                                break
                                        elif road_info_i[1] == road_info_j[1] and road_info_j[1] != connector_i and road_info_j[1] != connector_j:
                                            if eval(lane_info_i[1]) * eval(lane_info_j[1]) > 0:
                                                target_connector = connector_j
                                                break
                            if 'Merge' in parent_connector and target_connector.reverse:
                                target_road = target_connector.parent
                                # start from main road to ramp
                                # insert a left 'none' lane for parentmainroad
                                for lanesection in self.roads[target_road].lanes:
                                    # find how many lanes current mainroad have
                                    nums = len(self.roads[target_road].lanes[lanesection].left)
                                    self.roads[target_road].lanes[lanesection].InsertAuxliaryLane(nums-1, 'left')
                                lane_nums = len(self.roads[target_road].lanes[lanesection].left)
                                ROADID = self.NewRoad()
                                LINKS = {'parent': str(target_road), 'child':str(road.idx), 
                                        'lanelinks':[f'({target_road}, {ROADID}):({lane_nums - 1}, {-1})', f'({ROADID}, {road.idx}):({-1}, {1})']}
                                LANEINFOS = {'A':["type=line,dist=unknown,right=True,num=1,width=1.0"]}
                                BUILDERS = ['unknown']
                                ROAD = Road(typ='connector', idx=ROADID, links=LINKS, laneinfos=LANEINFOS, junction=self.roads[connector_j].junction, builders=BUILDERS)
                                ROAD.normalstart = False
                                self.PostRoadGeneration()
                                self.GenertateLanesForRoad(ROAD, 'right')
                                
                            elif 'Merge' in parent_connector and not target_connector.reverse:
                                target_road = target_connector.parent
                                for lanesection in self.roads[target_road].lanes:
                                    nums = len(self.roads[target_road].lanes[lanesection].right)
                                    self.roads[target_road].lanes[lanesection].InsertAuxliaryLane(1-nums, 'right')
                                lane_nums = len(self.roads[target_road].lanes[lanesection].right)
                                ROADID = self.NewRoad()
                                LINKS = {'parent': str(target_road), 'child':str(road.idx), 
                                        'lanelinks':[f'({target_road}, {ROADID}):({-lane_nums + 1}, {-1})', f'({ROADID}, {road.idx}):({-1}, {1})']}
                                BUILDERS = ['unknown']
                                ROAD = Road(typ='connector', idx=ROADID, links=LINKS, laneinfos=LANEINFOS, junction=self.roads[connector_j].junction, builders=BUILDERS)
                                ROAD.normalstart = False
                                self.PostRoadGeneration()
                                self.GenertateLanesForRoad(ROAD, 'right')
                            
                            elif 'Split' in parent_connector and target_connector.reverse:
                                target_road = target_connector.child
                                for lanesection in self.roads[target_road].lanes:
                                    nums = len(self.roads[target_road].lanes[lanesection].left)
                                    self.roads[target_road].lanes[lanesection].InsertAuxliaryLane(nums - 1, 'left')
                                lane_nums = len(self.roads[target_road].lanes[lanesection].left)
                                ROADID = self.NewRoad()
                                LINKS = {'parent': str(road.idx), 'child':str(target_road), 
                                        'lanelinks':[f'({ROADID}, {target_road}):({-1}, {lane_nums - 1})', f'({road.idx}, {ROADID}):({1}, {-1})']}
                                BUILDERS = ['unknown']
                                ROAD = Road(typ='connector', idx=ROADID, links=LINKS, laneinfos=LANEINFOS, junction=self.roads[connector_j].junction, builders=BUILDERS)
                                ROAD.normalstart = False
                                self.PostRoadGeneration()
                                self.GenertateLanesForRoad(ROAD, 'right')
                            
                            elif 'Split' in parent_connector and not target_connector.reverse:
                                target_road = target_connector.child
                                for lanesection in self.roads[target_road].lanes:
                                    nums = len(self.roads[target_road].lanes[lanesection].right)
                                    self.roads[target_road].lanes[lanesection].InsertAuxliaryLane(-nums + 1, 'right')
                                lane_nums = len(self.roads[target_road].lanes[lanesection].right)
                                ROADID = self.NewRoad()
                                LINKS = {'parent': str(road.idx), 'child':str(target_road), 
                                        'lanelinks':[f'({ROADID}, {target_road}):({-1}, {-lane_nums + 1})', f'({road.idx}, {ROADID}):({1}, {-1})']}
                                BUILDERS = ['unknown']
                                ROAD = Road(typ='connector', idx=ROADID, links=LINKS, laneinfos=LANEINFOS, junction=self.roads[connector_j].junction, builders=BUILDERS)
                                ROAD.normalstart = False
                                self.PostRoadGeneration()
                                self.GenertateLanesForRoad(ROAD, 'right')

                            self.roads[ROADID] = ROAD
    
    def AssertConnectorInsideJunction(self, connector: 'Road'):
        
        # if not connector.reverse:
            junction_id = connector.junction
            junction = self.junctions[junction_id]
            connectors = []
            for jcr_id in junction:
                if self.roads[jcr_id].typ == 'connector' and (self.roads[jcr_id].parent == connector.parent or self.roads[jcr_id].parent == connector.child) and jcr_id != connector.idx:
                    # find an connector share the same parent road
                    connectors.append(self.roads[jcr_id])
            
            for c in connectors:
                
                connector_link_id = []
                c_link_id = []
                for lanelink in connector.lanelinks:
                    roadinfo, laneinfo = lanelink.split(':')
                    roadinfo = eval(roadinfo)
                    laneinfo = eval(laneinfo)
                    if c.parent == connector.parent:
                        if roadinfo[0] == int(connector.parent):
                            connector_link_id.append(abs(laneinfo[0]))
                    else:
                        if roadinfo[1] == int(connector.child):
                            connector_link_id.append(abs(laneinfo[1]))
                for lanelink in c.lanelinks:
                    roadinfo, laneinfo = lanelink.split(':')
                    roadinfo = eval(roadinfo)
                    laneinfo = eval(laneinfo)
                    if roadinfo[0] == int(c.parent):
                        c_link_id.append(abs(laneinfo[0]))
                
                if min(connector_link_id) < min(c_link_id):
                    return True

            return False

    
    def GenerateRelations(self):
        # Generate lane relations and road relations:
        for road in self.roads.values():
            road.roadlinks = RoadLink(road)
            if road.lanelinks:
                for lanelink in road.lanelinks:
                    roadinfo, laneinfo = lanelink.split(':')
                    roadidx1 = eval(roadinfo)[0]
                    roadidx2 = eval(roadinfo)[1]
                    laneidx1 = eval(laneinfo)[0]
                    laneidx2 = eval(laneinfo)[1]
                    if roadidx1 == road.idx:
                        # parent road
                        lanes = list(road.lanes.values())[-1]
                        if laneidx1 < 0:
                            lanes.right[abs(laneidx1) - 1].link['successor'] = laneidx2
                        elif laneidx1 > 0:
                            lanes.left[abs(laneidx1) - 1].link['successor'] = laneidx2

                    if roadidx2 == road.idx:
                        lanes = list(road.lanes.values())[0]
                        if laneidx2 < 0:
                            lanes.right[abs(laneidx2) - 1].link['predecessor'] = laneidx1
                        elif laneidx2 > 0:
                            lanes.left[abs(laneidx2) - 1].link['predecessor'] = laneidx1
        

    
    def GenerateJunctions(self):
        self.JUNCTIONS = {}
        for j_id in self.junctions:
            junction = self.junctions[j_id]
            JUNCTION = {}
            i = 0
            for r_id in junction:
                if self.roads[r_id].typ == 'connector':
                    j = {}
                    j['connection'] = i
                    j['incoming_road'] = self.roads[r_id].parent[0][0]
                    j['connecting_road'] = r_id
                    j['contactPoint'] = 'start'
                    JUNCTION[i] = j
                    i += 1
            self.JUNCTIONS[j_id] = JUNCTION
    
    def InsertEmergencyLanes(self, *args):
        if args[0] == 'Main':
            for road in self.roads.values():
                if road.typ == 'non_connector' and road.direction != 'Uni':
                    for lanesection in road.lanes:
                        
                        nums = len(road.lanes[lanesection].right)
                        road.lanes[lanesection].InsertAuxliaryLane(nums, 'right', 3)
                        nums = len(road.lanes[lanesection].left)
                        road.lanes[lanesection].InsertAuxliaryLane(nums, 'left', 3)
                        
                    lanesection_begin_key = list(road.lanes.keys())[0]
                    lanesection_end_key = list(road.lanes.keys())[-1]
                    lanesection_keys = [lanesection_begin_key, lanesection_end_key]

                    for lsk in lanesection_keys:
                        need_add_left_emergency_lane = False
                        need_add_right_emergency_lane = False
                        leftidx = 0
                        rightidx = 0
                        for i in road.lanesections[lsk]['left']:
                            if road.lanesections[lsk]['left'][i]['type'] == 'line':
                                leftidx += eval(road.lanesections[lsk]['left'][i]['num'])
                            else:
                                need_add_left_emergency_lane = True
                                break
                        for i in road.lanesections[lsk]['right']:
                            if road.lanesections[lsk]['right'][i]['type'] == 'line':
                                rightidx += eval(road.lanesections[lsk]['right'][i]['num'])
                            else:
                                need_add_right_emergency_lane = True
                                break
                        if need_add_right_emergency_lane:
                            road.lanes[lsk].PolyInsertAuxliaryLane(rightidx+2, 'right', road.geometry)
                        if need_add_left_emergency_lane:
                            road.lanes[lsk].PolyInsertAuxliaryLane(leftidx+2, 'left', road.geometry)
        else:
            for road in self.roads.values():
                for lanesection in road.lanes:
                    if road.typ == 'connector' or (road.typ == 'non_connector' and road.direction == 'Uni'):
                        for lsk in road.lanesections:
                            if 'left' in road.lanesections[lsk]:
                                nums = len(road.lanes[lanesection].left)
                                road.lanes[lanesection].InsertAuxliaryLane(nums, 'left', 3)
                                break
                            else:
                                nums = len(road.lanes[lanesection].right)
                                road.lanes[lanesection].InsertAuxliaryLane(nums, 'right', 3)
                                break


    
    def GenerateObjectsLanes(self):
        for road in self.roads.values():
            for lanesection in road.lanes:
                if road.typ == 'non_connector' and road.direction != 'Uni':
                    # for outer barrier
                    nums = len(road.lanes[lanesection].right)
                    road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'right')
                    nums = len(road.lanes[lanesection].left)
                    road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'left')

                    # for trees
                    nums = len(road.lanes[lanesection].right)
                    road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'right')
                    nums = len(road.lanes[lanesection].left)
                    road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'left')
                else:
                    if road.typ == 'connector' and self.AssertConnectorInsideJunction(road):
                        continue
                    for lanesection in road.lanesections:
                        if 'right' in road.lanesections[lanesection]:
                            # for outer barrier
                            nums = len(road.lanes[lanesection].right)
                            road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'right')

                            # for trees
                            nums = len(road.lanes[lanesection].right)
                            road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'right')
                            break
                        if 'left' in road.lanesections[lanesection]:
                            # for outer barrier
                            nums = len(road.lanes[lanesection].left)
                            road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'left')

                            # for trees
                            nums = len(road.lanes[lanesection].left)
                            road.lanes[lanesection].InsertAuxliaryLane(nums + 1, 'left')
                            break
        for road in self.roads.values():
            for lanesection in road.lanes:
                # compute values
                if road.lanes[lanesection].leftobjects:
                    for leftobject in road.lanes[lanesection].leftobjects:
                        if leftobject.tag == 'InnerBarrier':
                            # left
                            leftobject.object['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.object['t'] = 0.5
                            # TODO: need to be specified
                            leftobject.object['hdg'] = 0.0
                            leftobject.object['pitch'] = 0.0
                            leftobject.object['roll'] = 0.0

                            leftobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.repeat['length'] = road.lanes[lanesection].length
                            leftobject.repeat['tStart'] = 0.5
                            leftobject.repeat['tEnd'] = 0.5
                            leftobject.repeat['widthStart'] = leftobject.object['width']
                            leftobject.repeat['widthEnd'] = leftobject.object['width']
                            leftobject.repeat['heightStart'] = leftobject.object['height']
                            leftobject.repeat['heightEnd'] = leftobject.object['height']
                            leftobject.repeat['zOffsetStart'] = 0.0
                            leftobject.repeat['zOffsetEnd'] = 0.0
                        
                        if leftobject.tag == 'OutBarrier':
                            leftobject.object['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.object['t'] = road.lanes[lanesection].l_width- 1.5 + 3.0 + 3.0
                            # TODO: need to be specified
                            leftobject.object['hdg'] = 0.0
                            leftobject.object['pitch'] = 0.0
                            leftobject.object['roll'] = 0.0

                            leftobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.repeat['length'] = road.lanes[lanesection].length
                            leftobject.repeat['tStart'] = road.lanes[lanesection].l_width- 1.5 + 3.0 + 3.0
                            leftobject.repeat['tEnd'] = road.lanes[lanesection].l_width- 1.5 + 3.0 + 3.0
                            leftobject.repeat['widthStart'] = leftobject.object['width']
                            leftobject.repeat['widthEnd'] = leftobject.object['width']
                            leftobject.repeat['heightStart'] = leftobject.object['height']
                            leftobject.repeat['heightEnd'] = leftobject.object['height']
                            leftobject.repeat['zOffsetStart'] = 0.0
                            leftobject.repeat['zOffsetEnd'] = 0.0
                            
                        if leftobject.tag == 'Bush':
                            leftobject.object['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.object['t'] = 0.0
                            # TODO: need to be specified
                            leftobject.object['hdg'] = 0.0
                            leftobject.object['pitch'] = 0.0
                            leftobject.object['roll'] = 0.0

                            leftobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.repeat['length'] = road.lanes[lanesection].length
                            leftobject.repeat['tStart'] = 0.0
                            leftobject.repeat['tEnd'] = 0.0
                            leftobject.repeat['widthStart'] = leftobject.object['width']
                            leftobject.repeat['widthEnd'] = leftobject.object['width']
                            leftobject.repeat['heightStart'] = leftobject.object['height']
                            leftobject.repeat['heightEnd'] = leftobject.object['height']
                            leftobject.repeat['zOffsetStart'] = 0.0
                            leftobject.repeat['zOffsetEnd'] = 0.0
                        
                        if leftobject.tag == 'Tree':
                            leftobject.object['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.object['t'] = road.lanes[lanesection].l_width- 0.5 + 3.0 + 3.0
                            # TODO: need to be specified
                            leftobject.object['hdg'] = 0.0
                            leftobject.object['pitch'] = 0.0
                            leftobject.object['roll'] = 0.0

                            leftobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            leftobject.repeat['length'] = road.lanes[lanesection].length
                            leftobject.repeat['tStart'] = road.lanes[lanesection].l_width- 0.5 + 3.0 + 3.0
                            leftobject.repeat['tEnd'] = road.lanes[lanesection].l_width- 0.5 + 3.0 + 3.0
                            leftobject.repeat['widthStart'] = leftobject.object['width']
                            leftobject.repeat['widthEnd'] = leftobject.object['width']
                            leftobject.repeat['heightStart'] = leftobject.object['height']
                            leftobject.repeat['heightEnd'] = leftobject.object['height']
                            leftobject.repeat['zOffsetStart'] = 0.0
                            leftobject.repeat['zOffsetEnd'] = 0.0
                if road.lanes[lanesection].rightobjects:
                    for rightobject in road.lanes[lanesection].rightobjects:
                        if rightobject.tag == 'InnerBarrier':
                            # left
                            rightobject.object['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.object['t'] = -0.5
                            # TODO: need to be specified
                            rightobject.object['hdg'] = 0.0
                            rightobject.object['pitch'] = 0.0
                            rightobject.object['roll'] = 0.0

                            rightobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.repeat['length'] = road.lanes[lanesection].length
                            rightobject.repeat['tStart'] = -0.5
                            rightobject.repeat['tEnd'] = -0.5
                            rightobject.repeat['widthStart'] = rightobject.object['width']
                            rightobject.repeat['widthEnd'] = rightobject.object['width']
                            rightobject.repeat['heightStart'] = rightobject.object['height']
                            rightobject.repeat['heightEnd'] = rightobject.object['height']
                            rightobject.repeat['zOffsetStart'] = 0.0
                            rightobject.repeat['zOffsetEnd'] = 0.0
                        
                        if rightobject.tag == 'OutBarrier':
                            rightobject.object['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.object['t'] =  -(road.lanes[lanesection].r_width- 1.5 + 3.0 + 3.0)
                            # TODO: need to be specified
                            rightobject.object['hdg'] = 0.0
                            rightobject.object['pitch'] = 0.0
                            rightobject.object['roll'] = 0.0

                            rightobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.repeat['length'] = road.lanes[lanesection].length
                            rightobject.repeat['tStart'] = -(road.lanes[lanesection].r_width- 1.5 + 3.0 + 3.0)
                            rightobject.repeat['tEnd'] = -(road.lanes[lanesection].r_width- 1.5 + 3.0 + 3.0)
                            rightobject.repeat['widthStart'] = rightobject.object['width']
                            rightobject.repeat['widthEnd'] = rightobject.object['width']
                            rightobject.repeat['heightStart'] = rightobject.object['height']
                            rightobject.repeat['heightEnd'] = rightobject.object['height']
                            rightobject.repeat['zOffsetStart'] = 0.0
                            rightobject.repeat['zOffsetEnd'] = 0.0
                            
                        if rightobject.tag == 'Bush':
                            rightobject.object['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.object['t'] = 0.0
                            # TODO: need to be specified
                            rightobject.object['hdg'] = 0.0
                            rightobject.object['pitch'] = 0.0
                            rightobject.object['roll'] = 0.0

                            rightobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.repeat['length'] = road.lanes[lanesection].length
                            rightobject.repeat['tStart'] = 0.0
                            rightobject.repeat['tEnd'] = 0.0
                            rightobject.repeat['widthStart'] = rightobject.object['width']
                            rightobject.repeat['widthEnd'] = rightobject.object['width']
                            rightobject.repeat['heightStart'] = rightobject.object['height']
                            rightobject.repeat['heightEnd'] = rightobject.object['height']
                            rightobject.repeat['zOffsetStart'] = 0.0
                            rightobject.repeat['zOffsetEnd'] = 0.0
                        
                        if rightobject.tag == 'Tree':
                            rightobject.object['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.object['t'] = -(road.lanes[lanesection].r_width- 0.5 + 3.0 + 3.0)
                            # TODO: need to be specified
                            rightobject.object['hdg'] = 0.0
                            rightobject.object['pitch'] = 0.0
                            rightobject.object['roll'] = 0.0

                            rightobject.repeat['s'] = road.lanes[lanesection].laneSection_s
                            rightobject.repeat['length'] = road.lanes[lanesection].length
                            rightobject.repeat['tStart'] = -(road.lanes[lanesection].r_width- 0.5 + 3.0 + 3.0)
                            rightobject.repeat['tEnd'] = -(road.lanes[lanesection].r_width- 0.5 + 3.0 + 3.0)
                            rightobject.repeat['widthStart'] = rightobject.object['width']
                            rightobject.repeat['widthEnd'] = rightobject.object['width']
                            rightobject.repeat['heightStart'] = rightobject.object['height']
                            rightobject.repeat['heightEnd'] = rightobject.object['height']
                            rightobject.repeat['zOffsetStart'] = 0.0
                            rightobject.repeat['zOffsetEnd'] = 0.0

    def GenerateRoadElevation(self):
        for road in self.roads.values():
            if road.parent:
                if ('Junction' not in str(road.parent)) and ('Junction' not in str(road.child)):
                    road.ElevationStart = self.roads[int(road.parent)].ElevationEnd
                    road.ElevationStartHdg = self.roads[int(road.parent)].ElevationEndHdg
                    road.GenerateElevations()
                else:
                    if not (('Junction' in str(road.parent)) and ('Junction' in str(road.child)) and road.direction == 'Uni'):
                        for r in self.main_road_connections[road.idx]:
                            if self.roads[r].child == road.idx:
                                road.ElevationStart = self.roads[r].ElevationEnd
                                break
                        road.GenerateElevations()
            else:
                road.GenerateElevations()

    def GenerateXODRFile(self):
        top = ET.Element('OpenDRIVE')
        top.attrib['xmlns'] = "http://www.opendrive.org"
        now = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        # generate header
        east, west, south, north = '-1000', '1000', '-1000', '1000'
        header = ET.SubElement(top, 'header')
        header.attrib['revMajor'] = '1'
        header.attrib['revMinor'] = '0'
        header.attrib['name'] = 'plusai_demo'
        header.attrib['version'] = '1.00'
        header.attrib['date'] = now
        header.attrib['north'] = north
        header.attrib['south'] = south
        header.attrib['east'] = east
        header.attrib['west'] = west
        header.attrib['vendor'] = 'PlusAI'
        
        # generate road
        for r in self.roads:
            road = ET.SubElement(top, 'road')
            road.attrib['name'] = self.roads[r].name
            road.attrib['length'] = to_string(list(self.roads[r].geometry.values())[-1].s + list(self.roads[r].geometry.values())[-1].length)
            road.attrib['id'] = to_string(self.roads[r].idx)
            road.attrib['junction'] = self.roads[r].junction
            road.attrib['rule'] = 'RHT'
            link = ET.SubElement(road, 'link')
            if self.roads[r].roadlinks.predecessor:
                predecessor = ET.SubElement(link, 'predecessor')
                if self.roads[r].roadlinks.predecessor.elementType == 'road':
                    predecessor.attrib['elementType'] = to_string(self.roads[r].roadlinks.predecessor.elementType)
                    predecessor.attrib['elementId'] = to_string(self.roads[r].roadlinks.predecessor.elementId)
                    predecessor.attrib['contactPoint'] = to_string(self.roads[r].roadlinks.predecessor.contactPoint)
                else:
                    predecessor.attrib['elementType'] = to_string(self.roads[r].roadlinks.predecessor.elementType)
                    predecessor.attrib['elementId'] = to_string(self.roads[r].roadlinks.predecessor.elementId)
            if self.roads[r].roadlinks.successor:
                successor = ET.SubElement(link, 'successor')
                if self.roads[r].roadlinks.successor.elementType == 'road':
                    successor.attrib['elementType'] = to_string(self.roads[r].roadlinks.successor.elementType)
                    successor.attrib['elementId'] = to_string(self.roads[r].roadlinks.successor.elementId)
                    successor.attrib['contactPoint'] = to_string(self.roads[r].roadlinks.successor.contactPoint)
                else:
                    successor.attrib['elementType'] = to_string(self.roads[r].roadlinks.successor.elementType)
                    successor.attrib['elementId'] = to_string(self.roads[r].roadlinks.successor.elementId)
            planview = ET.SubElement(road, 'planView')
            idxes = sorted(self.roads[r].geometry, key=lambda v: self.roads[r].geometry[v].s)
            for idx in idxes:
                geo = self.roads[r].geometry[idx]
                geometry = ET.SubElement(planview, 'geometry')
                geometry.attrib['s'] = to_string(geo.s)
                geometry.attrib['x'] = to_string(geo.x)
                geometry.attrib['y'] = to_string(geo.y)
                geometry.attrib['hdg'] = to_string(geo.hdg)
                geometry.attrib['length'] = to_string(geo.length)    
                if geo.typ == 'line':
                    line = ET.SubElement(geometry, 'line')
                elif geo.typ == 'spiral':
                    spiral = ET.SubElement(geometry, 'spiral')
                    spiral.attrib['curvStart'] = to_string(geo.curStart)
                    spiral.attrib['curvEnd'] = to_string(geo.curEnd)
                elif geo.typ == 'arc':
                    arc = ET.SubElement(geometry, 'arc')
                    arc.attrib['curvature'] = to_string(geo.curStart)
                elif geo.typ == 'paramPoly3':
                    poly = ET.SubElement(geometry, 'paramPoly3')
                    poly.attrib['aU'] = to_string(geo.au)
                    poly.attrib['bU'] = to_string(geo.bu)
                    poly.attrib['cU'] = to_string(geo.cu)
                    poly.attrib['dU'] = to_string(geo.du)
                    poly.attrib['aV'] = to_string(geo.av)
                    poly.attrib['bV'] = to_string(geo.bv)
                    poly.attrib['cV'] = to_string(geo.cv)
                    poly.attrib['dV'] = to_string(geo.dv)
                    poly.attrib['pRange'] = 'normalized'
                elif geo.typ == 'Poly3':
                    poly = ET.SubElement(geometry, 'poly3')
                    poly.attrib['a'] = to_string(geo.a)
                    poly.attrib['b'] = to_string(geo.b)
                    poly.attrib['c'] = to_string(geo.c)
                    poly.attrib['d'] = to_string(geo.d)
            # END FOR THE PLANVIEW
            # START FOR elevation PROFILE, BY DEFAULT
            elevationprofile = ET.SubElement(road, 'elevationProfile')
            if self.roads[r].ElevationResult:
                for idx in self.roads[r].ElevationResult:
                    elevation = ET.SubElement(elevationprofile, 'elevation')
                    elevation.attrib['s'] = to_string(self.roads[r].ElevationResult[idx][0])
                    elevation.attrib['a'] = to_string(self.roads[r].ElevationResult[idx][1])
                    elevation.attrib['b'] = to_string(self.roads[r].ElevationResult[idx][2])
                    elevation.attrib['c'] = to_string(self.roads[r].ElevationResult[idx][3])
                    elevation.attrib['d'] = to_string(self.roads[r].ElevationResult[idx][4])
            # END FOR elevation PROFILE
            # START FOR LATERAL PROFILE
            lateralprofile = ET.SubElement(road, 'lateralProfile')
            # END FOR LATERAL PROFILE
            # START FOR LANES
            Lanes = ET.SubElement(road, 'lanes')
            for laneSection in self.roads[r].lanes:
                lanesection = ET.SubElement(Lanes, 'laneSection')
                lanesection.attrib['s'] = to_string(self.roads[r].lanes[laneSection].laneSection_s)
                lanes = self.roads[r].lanes[laneSection]
                left = ET.SubElement(lanesection, 'left')
                for leftlane in lanes.left:
                    lane = ET.SubElement(left, 'lane')
                    lane.attrib['id'] = to_string(leftlane.idx)
                    lane.attrib['type'] = leftlane.typ
                    lane.attrib['level'] = str(leftlane.level).lower()
                    link = ET.SubElement(lane, 'link')
                    if leftlane.link['predecessor']:
                        predecessor = ET.SubElement(link, 'predecessor')
                        predecessor.attrib['id'] = to_string(leftlane.link['predecessor'])
                    if leftlane.link['successor']:
                        successor = ET.SubElement(link, 'successor')
                        successor.attrib['id'] = to_string(leftlane.link['successor'])
                    for w in leftlane.width:
                        width = ET.SubElement(lane, 'width')
                        width.attrib['sOffset'] = to_string(w.sOffset)
                        width.attrib['a'] = to_string(w.a)
                        width.attrib['b'] = to_string(w.b)
                        width.attrib['c'] = to_string(w.c)
                        width.attrib['d'] = to_string(w.d)
                    roadmark = ET.SubElement(lane, 'roadMark')
                    roadmark.attrib['sOffset'] = to_string(leftlane.roadmark.sOffset)
                    roadmark.attrib['type'] = to_string(leftlane.roadmark.typ)
                    roadmark.attrib['weight'] = to_string(leftlane.roadmark.weight)
                    roadmark.attrib['color'] = to_string(leftlane.roadmark.color)
                    roadmark.attrib['width'] = to_string(leftlane.roadmark.width)
                    roadmark.attrib['laneChange'] = to_string(leftlane.roadmark.laneChange)
                    roadmark.attrib['height'] = to_string(leftlane.roadmark.height)
                    material = ET.SubElement(lane, 'material')
                    material.attrib['sOffset'] = '0.0'
                    material.attrib['surface'] = '0'
                    material.attrib['friction'] = '0.9'
                    material.attrib['roughness'] = '0.0'
                    speed = ET.SubElement(lane, 'speed')
                    speed.attrib['sOffset'] = '0.0'
                    speed.attrib['max'] = to_string(leftlane.speed)
                center = ET.SubElement(lanesection, 'center')
                for centerlane in lanes.center:
                    lane = ET.SubElement(center, 'lane')
                    lane.attrib['id'] = to_string(centerlane.idx)
                    lane.attrib['type'] = centerlane.typ
                    lane.attrib['level'] = str(centerlane.level).lower()
                    link = ET.SubElement(lane, 'link')
                    roadmark = ET.SubElement(lane, 'roadMark')
                    roadmark.attrib['sOffset'] = to_string(centerlane.roadmark.sOffset)
                    roadmark.attrib['type'] = to_string(centerlane.roadmark.typ)
                    roadmark.attrib['weight'] = to_string(centerlane.roadmark.weight)
                    roadmark.attrib['color'] = to_string(centerlane.roadmark.color)
                    roadmark.attrib['width'] = to_string(centerlane.roadmark.width)
                    roadmark.attrib['laneChange'] = to_string(centerlane.roadmark.laneChange)
                    roadmark.attrib['height'] = to_string(centerlane.roadmark.height)
                right = ET.SubElement(lanesection, 'right')
                for rightlane in lanes.right:
                    lane = ET.SubElement(right, 'lane')
                    lane.attrib['id'] = to_string(rightlane.idx)
                    lane.attrib['type'] = rightlane.typ
                    lane.attrib['level'] = str(rightlane.level).lower()
                    link = ET.SubElement(lane, 'link')
                    if rightlane.link['predecessor']:
                        predecessor = ET.SubElement(link, 'predecessor')
                        predecessor.attrib['id'] = to_string(rightlane.link['predecessor'])
                    if rightlane.link['successor']:
                        successor = ET.SubElement(link, 'successor')
                        successor.attrib['id'] = to_string(rightlane.link['successor'])
                    for w in rightlane.width:
                        width = ET.SubElement(lane, 'width')
                        width.attrib['sOffset'] = to_string(w.sOffset)
                        width.attrib['a'] = to_string(w.a)
                        width.attrib['b'] = to_string(w.b)
                        width.attrib['c'] = to_string(w.c)
                        width.attrib['d'] = to_string(w.d)
                    roadmark = ET.SubElement(lane, 'roadMark')
                    roadmark.attrib['sOffset'] = to_string(rightlane.roadmark.sOffset)
                    roadmark.attrib['type'] = to_string(rightlane.roadmark.typ)
                    roadmark.attrib['weight'] = to_string(rightlane.roadmark.weight)
                    roadmark.attrib['color'] = to_string(rightlane.roadmark.color)
                    roadmark.attrib['width'] = to_string(rightlane.roadmark.width)
                    roadmark.attrib['laneChange'] = to_string(rightlane.roadmark.laneChange)
                    roadmark.attrib['height'] = to_string(rightlane.roadmark.height)
                    material = ET.SubElement(lane, 'material')
                    material.attrib['sOffset'] = '0.0'
                    material.attrib['surface'] = '0'
                    material.attrib['friction'] = '0.9'
                    material.attrib['roughness'] = '0.0' 
                    speed = ET.SubElement(lane, 'speed')
                    speed.attrib['sOffset'] = '0.0'
                    speed.attrib['max'] = to_string(rightlane.speed)
            objects = ET.SubElement(road, 'objects')
            for ls in self.roads[r].lanes:
                if not (self.roads[r].lanes[ls].leftobjects and self.roads[r].lanes[ls].rightobjects):
                    break
                objs = self.roads[r].lanes[ls].leftobjects + self.roads[r].lanes[ls].rightobjects
                for o in objs:
                    obj = ET.SubElement(objects, 'object')
                    obj.attrib['type'] = to_string(o.object['type'])
                    obj.attrib['name'] = to_string(o.object['name'])
                    obj.attrib['s'] = to_string(o.object['s'])
                    obj.attrib['t'] = to_string(o.object['t'])
                    obj.attrib['zOffset'] = to_string(o.object['zOffset'])
                    obj.attrib['validLength'] = to_string(o.object['validLength'])
                    obj.attrib['orientation'] = to_string(o.object['orientation'])
                    obj.attrib['length'] = to_string(o.object['length'])
                    obj.attrib['width'] = to_string(o.object['width'])
                    obj.attrib['height'] = to_string(o.object['height'])
                    obj.attrib['hdg'] = to_string(o.object['hdg'])
                    obj.attrib['pitch'] = to_string(o.object['pitch'])
                    obj.attrib['roll'] = to_string(o.object['roll'])

                    rep = ET.SubElement(obj, 'repeat')
                    rep.attrib['s'] = to_string(o.repeat['s'])
                    rep.attrib['length'] = to_string(o.repeat['length'])
                    rep.attrib['distance'] = to_string(o.repeat['distance'])
                    rep.attrib['tStart'] = to_string(o.repeat['tStart'])
                    rep.attrib['tEnd'] = to_string(o.repeat['tEnd'])
                    rep.attrib['widthStart'] = to_string(o.repeat['widthStart'])
                    rep.attrib['widthEnd'] = to_string(o.repeat['widthEnd'])
                    rep.attrib['heightStart'] = to_string(o.repeat['heightStart'])
                    rep.attrib['heightEnd'] = to_string(o.repeat['heightEnd'])
                    rep.attrib['zOffsetStart'] = to_string(o.repeat['zOffsetStart'])
                    rep.attrib['zOffsetEnd'] = to_string(o.repeat['zOffsetEnd'])

            signals = ET.SubElement(road, 'signals')
            surface = ET.SubElement(road, 'surface')
        for junction_id in self.JUNCTIONS:
            junction = ET.SubElement(top, 'junction')
            for c_id in self.JUNCTIONS[junction_id]:
                connection = ET.SubElement(junction, 'connection')
                connection.attrib['id'] = to_string(c_id)
                connection.attrib['incoming_road'] = to_string(self.JUNCTIONS[junction_id][c_id]['incoming_road'])
                connection.attrib['connecting_road'] = to_string(self.JUNCTIONS[junction_id][c_id]['connecting_road'])
                connection.attrib['contactPoint'] = to_string(self.JUNCTIONS[junction_id][c_id]['contactPoint'])
                connector = self.roads[int(connection.attrib['connecting_road'])]
                connector_start_lanesection = list(connector.lanes.values())[0]
                connector_end_lanesection = list(connector.lanes.values())[-1]
                for leftlane in connector_start_lanesection.left:
                    if leftlane.link['predecessor']:
                        lanelink = ET.SubElement(connection, 'lanelink')
                        lanelink.attrib['from'] = to_string(leftlane.link['predecessor'])
                        lanelink.attrib['to'] = to_string(leftlane.idx)
                for rightlane in connector_start_lanesection.right:
                    if rightlane.link['predecessor']:
                        lanelink = ET.SubElement(connection, 'lanelink')
                        lanelink.attrib['from'] = to_string(rightlane.link['predecessor'])
                        lanelink.attrib['to'] = to_string(rightlane.idx)

        s = minidom.parseString(ET.tostring(top, encoding='UTF-8'))
        s = s.toprettyxml(indent='    ')
        with open(f'/home/simulation/zltplusai/MapConverter/Maps/{self.jsonfile.name}.xodr', 'w') as o:
            o.write(s)

def EulerRoadGeometry(parent: 'Road', child: 'Road'):
    # geometry: Line: default_length: 10, EulerSpiral, Line: default_length: 10
    Error = 0.001
    s = 0.0
    x0, y0, hdg0 = parent.GenerateEnding()
    x1, y1, hdg1 = child.GenerateEnding()
    hdg1 += np.pi

    #G1 continous euler spiral:
    deltax = x1- x0
    deltay = y1- y0
    fai = atan2(deltay, deltax)
    r = sqrt(deltax**2 + deltay**2)
    fai0, fai1 = NormalizeAngle(hdg0-fai), NormalizeAngle(hdg1-fai)
    A = 3*(fai1+fai0)
    while abs(integrate.quad(Y, 0, 1, args=(A, fai1, fai0, 0))[0]) > Error:
        A -= integrate.quad(Y, 0, 1, args=(A, fai1, fai0, 0))[0] / (integrate.quad(X, 0, 1, args=(A, fai1, fai0, 2))[0] - integrate.quad(X, 0, 1, args=(A, fai1, fai0, 1))[0]) 
    L = r / integrate.quad(X, 0, 1, args=(A, fai1, fai0, 0))[0]
    kappa = (fai1 - fai0 - A) / L
    kappa_dot = 2 *A / L**2 
    
    geometry = {0:None}
    # rectify the s of geometry 2
    geometry[0] = Geometry(idx=0, typ='spiral', s=s, x=x0, y=y0, z=0.0, hdg=hdg0, length=L, curStart=kappa, curEnd=kappa + L * kappa_dot)
        
    return geometry


def HermiteInterplotation(x0, y0, theta0, x1, y1, theta1, point_nums):

    P = x0 / (x1 - x0)
    Q = 1 / (x1 - x0)

    X = []
    Y = []
    for i in range(point_nums + 1):
        x = x0 + (x1 - x0) * i / point_nums
        s1 = y0 * (1 + 2 * (Q * x - P)) * (P - x * Q + 1) ** 2
        s2 = y1 * (3 + 2 * (P - Q * x)) * (Q * x - P) ** 2
        s3 = math.tan(theta0) * (x - x0) * (P - Q * x + 1) ** 2
        s4 = math.tan(theta1) * (x - x1) * (Q * x - P) ** 2
        
        y = sum([s1, s2, s3, s4])
        X.append(x)
        Y.append(y)
    if not all(np.polyfit(X, Y, 3) == 0):
        d, c, b, a = np.poly1d(np.polyfit(X,Y,3)).c
    else:
        return 0.0, 0.0, 0.0, 0.0
    
    
    return a, b, c, d

if __name__ == '__main__':

    parser = argparse.ArgumentParser('Generator for building OpenDRIVE map file from predefined Json file.')
    parser.add_argument('input_json')
    args = parser.parse_args() 

    with open("./" + args.input_json, 'r') as f:

        jsonfile = JsonFile(Json=json.load(f))

        jsonfile.name = args.input_json.split('.')[0]

        opendrive = OpenDRIVE(jsonfile)

        opendrive.SpecifyMainRoadConnections()

        opendrive.CreateRoad()

        opendrive.GenerateLanes()

        opendrive.InsertEmergencyLanes('Main')

        opendrive.PostRoadGeneration()

        opendrive.PostGenertateLanes()
        
        opendrive.GenerateRelations()

        opendrive.GenerateJunctions()

        opendrive.InsertEmergencyLanes('Other')

        opendrive.GenerateObjectsLanes()

        opendrive.GenerateRoadElevation()

        opendrive.GenerateXODRFile()
        
        






    
    

            



                







    
    
