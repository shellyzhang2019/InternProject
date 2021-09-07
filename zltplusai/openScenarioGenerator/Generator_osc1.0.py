import json
from dataclasses import dataclass
from typing import Any
from copy import deepcopy
from xml.dom import minidom
import xml.etree.ElementTree as ET
import datetime
import random
import argparse

def to_string(data):
    return str(data)

class RoadPosition:
    
    def __init__(self) -> None:
        self.roadId = None
        self.s = None
        self.t = None
        self.type = None # Absolute or Relative
        self.orientation = {
            'h': None,
            'p': None,
            'r': None
        }

class LanePosition:
    
    def __init__(self) -> None:
        self.roadId = None
        self.laneId = None
        self.offset = None
        self.s = None
        self.type = None # Absolute or Relative
        self.orientation = {
            'h': None,
            'p': None,
            'r': None
        }

class RelativeRoadPosition: 

    def __init__(self) -> None:
        self.entityRef = None
        self.ds = None
        self.dt = None
        self.orientation = {
            'h': None,
            'p': None,
            'r': None
        }

class RelativeLanePosition:
    
    def __init__(self) -> None:
        self.entityRef = None
        self.dLane = None
        self.ds = None
        self.offset = None
        self.orientation = {
            'h': None,
            'p': None,
            'r': None
        }

class Trigger:

    def __init__(self) -> None:
        self.conditiontypes = {'E':'ByEntityCondition', 'V':'ByValueCondition'}
        self.relativedistancetype = {'Lon':'longitudinal', 'Lat':'lateral'}
        self.entityref = None
        self.rules = {'lt':'lessThan', 'gt': 'greaterThan', 'goe': 'greaterOrEqual', 'loe': 'lessOrEqual'}
        ########################################## Params ####################################################
        #                                                                                                    #
        # * freespace: distance between a given points and a entity                                          #
        #           false -> return distance between given point and the origin of entity                    #
        #           true  -> return minimal distance between given point and the bounding box of entity.     #
        #
        # * rule: 'greaterThan' : >
        #       'lessthan': <
        #       'greaterOrEqual: >=
        #       'lessOrEqual': <=
        #
        # * entityRef: entity object                                                                         #
        ######################################################################################################
        
        #################################### Entity Condition ##############################
        self.RelativeDistanceCondition = {
            'freespace': 'false', 
            'rule': None,
            'entityRef': None,
            'value':None,
            'relativeDistanceType':None
        }

        # Amount of time to reach the end of the road
        self.EndOfRoadCondition = {
            'duration': None
        }

        self.CollisionCondition = {
            'entityRef': None,
            'byType': None
        }

        self.OffRoadCondition = {
            'duration': None
        }

        self.TimeHeadawayCondition = {
            'entityRef': None,
            'value': None,
            'freespace': None,
            'alongRoute': None,
            'rule': None
        }

        self.TimeToCollisionCondition = {
            'value': None,
            'freespace': None,
            'alongRoute': None,
            'rule': None,
            'timeToCollisionConditionTarget': {
                'Position': None,
                'entityRef': None
            }
        }

        self.AccelerationCondition = {
            'value': None,
            'rule': None
        }

        self.StandStillCondition = {
            'duration': None
        }

        self.SpeedCondition = {
            'value': None,
            'rule': None
        }

        self.TraveledDistanceCondition = {
            'value': None
        }

        self.ReachPositionCondition = {
            'tolerance': None,
            'position': None
        }

        # Value Condition
        self.SimulationTimeCondition = {
            'rule':None,
            'value':None
        }



class LaneChangeProfile:

    def __init__(self) -> None:
        self.targetLaneOffset = 0.0     # 0.023880048828
        self.dynamics = {
            'Dimension':'distance',
            'shape':'cubic',
            'value':None
        }
        self.targetLane = None
        self.trigger = None

@dataclass
class JsonFile:
    Json: Any = None

    def __post_init__(self):
        self.name = None
        self.MapFilePath = None
        self.SceneFilePath = None
        self.EgoEntities = self.Json['ego']
        self.AgentEntities = self.Json['agents']
        self.entities = {}
        self.entities['ego'] = self.Json['ego']
        count = 0
        for agent in self.AgentEntities:
            agent_name = 'agent'+ str(count)
            self.entities[agent_name] = agent
            count += 1

class openScenario:

    def __init__(self, jsonfile: 'JsonFile') -> None:
        self.jsonfile = jsonfile
        self.Entities = {}
        # for each dynamic traffic participants
    
    def init_process(self):
        for entity in self.jsonfile.entities:
            self.Entities[entity] = self.jsonfile.entities[entity]

            init_configs = None
            nodes = list(self.Entities[entity].keys())
            nvals = set()
            if not isinstance(list(self.Entities[entity].values())[0], str):
                nvals.add(tuple(list(self.Entities[entity].values())[0]))
            else:
                nvals.add(tuple())
                nodes.remove(list(self.Entities[entity].keys())[0])
            # dummy node
            def find_combination(val, n_vals, i, nodes):
                s = set()
                delete = False
                for nv in n_vals:
                    for v in val:
                        if (not isinstance(nv, dict)) and (not isinstance(nv, str)):
                            v = list(v)
                            v.append(nv)
                            v = tuple(v)

                            if v not in s:
                                s.add(v)
                            
                        else:
                            nodes.remove(list(self.Entities[entity].keys())[i])
                            delete = True
                            break
                        
                    if delete:
                        break
                if not delete:
                    i += 1
                    return s, i, nodes
                else:
                    return val, i, nodes
            i = 0
            while i < len(nodes):
                nvals, i, nodes = find_combination(nvals, self.Entities[entity][nodes[i]], i, nodes)
            init_configs = nvals
            self.Entities[entity] = {'keys': nodes, 'vals': init_configs}
    

    def GenerateXOSCFile(self):
        top = ET.Element('OpenSCENARIO')
        now = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        header = ET.SubElement(top, 'FileHeader')
        header.attrib['author'] = 'Plusai'
        header.attrib['date'] = now
        header.attrib['description'] = self.jsonfile.name
        header.attrib['revMajor'] = "1"
        header.attrib['revMinor'] = "0"
        params = ET.SubElement(top, 'ParameterDeclarations')
        cataloglocations = ET.SubElement(top, 'CatalogLocations')
        VehicleCatalog = ET.SubElement(cataloglocations, 'VehicleCatalog')
        directory = ET.SubElement(VehicleCatalog, 'Directory')
        directory.attrib['path'] = "Distros/Current/Config/Players/Vehicles"
        ControllerCatalog = ET.SubElement(cataloglocations, 'ControllerCatalog')
        directory = ET.SubElement(ControllerCatalog, 'Directory')
        directory.attrib['path'] = "Distros/Current/Config/Players/driverCfg.xml"
        MiscObjectCatalog = ET.SubElement(cataloglocations, 'MiscObjectCatalog')
        directory = ET.SubElement(MiscObjectCatalog, 'Directory')
        directory.attrib['path'] = "Distros/Current/Config/Players/Objects"
        RoadNetwork = ET.SubElement(top, 'RoadNetwork')
        Logics = ET.SubElement(RoadNetwork, 'LogicFile')
        Logics.attrib['filepath'] = self.jsonfile.MapFilePath
        SceneGraph = ET.SubElement(RoadNetwork, 'SceneGraphFile')
        SceneGraph.attrib['filepath'] = self.jsonfile.SceneFilePath

        Entities = ET.SubElement(top, 'Entities')
        
        for entity in self.Entities:
            Object = ET.SubElement(Entities, 'ScenarioObject')
            Object.attrib['name'] = entity
            vehicle = ET.SubElement(Object, 'Vehicle')
            if 'ego' in entity:
                vehicle.attrib['name'] = 'MANTGS_11_DarkBlue'
                vehicle.attrib['vehicleCategory'] = 'truck'
                ParameterDeclaration = ET.SubElement(vehicle, 'ParameterDeclarations')

                BoundingBox = ET.SubElement(vehicle, 'BoundingBox')
                Center = ET.SubElement(BoundingBox, 'Center')
                Dimensions = ET.SubElement(BoundingBox, 'Dimensions')
                print(self.Entities[entity]['keys'])
                HIdx = self.Entities[entity]['keys'].index('h')
                LIdx = self.Entities[entity]['keys'].index('l')
                WIdx = self.Entities[entity]['keys'].index('w')
                config = random.choice(list(self.Entities[entity]['vals']))
                print(config)
                Center.attrib['x'] = to_string(config[LIdx] / 2)
                Center.attrib['y'] = to_string(0.0)
                Center.attrib['z'] = to_string(config[HIdx] / 2)
                Dimensions.attrib['height'] = to_string(config[HIdx])
                Dimensions.attrib['width'] = to_string(config[WIdx])
                Dimensions.attrib['length'] = to_string(config[LIdx])

                Performance = ET.SubElement(vehicle, 'Performance')
                Performance.attrib['maxSpeed'] = to_string(26.4 * 3.6)
                Performance.attrib['maxAcceleration'] = to_string(20.0)
                Performance.attrib['maxDeceleration'] = to_string(20.0)
                
                Axles = ET.SubElement(vehicle, 'Axles')
                Front = ET.SubElement(Axles, 'FrontAxle')
                Rear = ET.SubElement(Axles, 'RearAxle')
                Front.attrib['maxSteering'] = to_string(0.48)
                Rear.attrib['maxSteering'] = to_string(0.48)
                Front.attrib['positionX'] = to_string(6.4)
                Front.attrib['positionZ'] = to_string(config[HIdx] / 2)
                Front.attrib['trackWidth'] = to_string(2.320)
                Front.attrib['wheelDiameter'] = to_string(1)
                Rear.attrib['positionX'] = to_string(2.344)
                Rear.attrib['positionZ'] = to_string(config[HIdx] / 2)
                Rear.attrib['trackWidth'] = to_string(2.321)
                Rear.attrib['wheelDiameter'] = to_string(1)

                Properties = ET.SubElement(vehicle, 'Properties')
                Property = ET.SubElement(Properties, 'Property')
                Property.attrib['name'] = 'control'
                Property.attrib['value'] = 'external'

                ObjectController = ET.SubElement(Object, 'ObjectController')
                controller = ET.SubElement(ObjectController, 'Controller')
                controller.attrib['name'] = "DefaultDriver"
                Properties = ET.SubElement(controller, 'Properties')

            else:
                vehicle.attrib['name'] = 'Audi_A3_2009_red'
                vehicle.attrib['vehicleCategory'] = 'car'
                ParameterDeclarations = ET.SubElement(vehicle, 'ParameterDeclarations')
                HIdx = self.Entities[entity]['keys'].index('h')
                LIdx = self.Entities[entity]['keys'].index('l')
                WIdx = self.Entities[entity]['keys'].index('w')
                VMaxIdx = self.Entities[entity]['keys'].index('vmx')
                config = random.choice(list(self.Entities[entity]['vals']))

                BoundingBox = ET.SubElement(vehicle, 'BoundingBox')
                Center = ET.SubElement(BoundingBox, 'Center')
                Dimensions = ET.SubElement(BoundingBox, 'Dimensions')

                Performance = ET.SubElement(vehicle, 'Performance')
                Performance.attrib['maxSpeed'] = to_string(eval(str(config[VMaxIdx])) * 3.6)
                Performance.attrib['maxAcceleration'] = to_string(20.0)
                Performance.attrib['maxDeceleration'] = to_string(20.0)
                
                Center.attrib['x'] = to_string(config[LIdx] / 2)
                Center.attrib['y'] = to_string(0.0)
                Center.attrib['z'] = to_string(config[HIdx] / 2)
                Dimensions.attrib['height'] = to_string(config[HIdx])
                Dimensions.attrib['width'] = to_string(config[WIdx])
                Dimensions.attrib['length'] = to_string(config[LIdx])
                
                Axles = ET.SubElement(vehicle, 'Axles')
                Front = ET.SubElement(Axles, 'FrontAxle')
                Rear = ET.SubElement(Axles, 'RearAxle')
                Front.attrib['maxSteering'] = to_string(0.48)
                Rear.attrib['maxSteering'] = to_string(0.48)
                Front.attrib['positionX'] = to_string(3.658)
                Front.attrib['positionZ'] = to_string(config[HIdx] / 2)
                Front.attrib['trackWidth'] = to_string(1.439)
                Front.attrib['wheelDiameter'] = to_string(0.677)
                Rear.attrib['positionX'] = to_string(1.016)
                Rear.attrib['positionZ'] = to_string(config[HIdx] / 2)
                Rear.attrib['trackWidth'] = to_string(1.439)
                Rear.attrib['wheelDiameter'] = to_string(0.677)

                Properties = ET.SubElement(vehicle, 'Properties')
                Property = ET.SubElement(Properties, 'Property')
                Property.attrib['name'] = 'control'
                Property.attrib['value'] = 'internal'

                ObjectController = ET.SubElement(Object, 'ObjectController')
                controller = ET.SubElement(ObjectController, 'Controller')
                controller.attrib['name'] = "DefaultDriver"
                Properties = ET.SubElement(controller, 'Properties')
        
        Storyboard = ET.SubElement(top, 'Storyboard')
        Init = ET.SubElement(Storyboard, 'Init')
        Actions = ET.SubElement(Init, 'Actions')
        EntitiesV = []
        for entity in self.Entities:
            Private  = ET.SubElement(Actions, 'Private')
            Private.attrib['entityRef'] = to_string(entity)

            # choose config randomly
            config = random.choice(list(self.Entities[entity]['vals']))

            # init pose and pos
            if entity == 'ego':
                PrivateAction = ET.SubElement(Private, 'PrivateAction')
                TeleportAction = ET.SubElement(PrivateAction, 'TeleportAction')
                Position = ET.SubElement(TeleportAction, 'Position')
                RoadPosition = ET.SubElement(Position, 'RoadPosition')
                RoadPosition.attrib['roadId'] = to_string(0)
                RoadPosition.attrib['s'] = to_string(50)
                RoadPosition.attrib['t'] = to_string(-6.625)
                Orientation = ET.SubElement(RoadPosition, 'Orientation')
                Orientation.attrib['type'] = 'relative'
                YawIdx = self.Entities[entity]['keys'].index('yaw')
                Yaw = config[YawIdx]
                Orientation.attrib['h'] = to_string(float(Yaw))
                Orientation.attrib['p'] = to_string(0.0)
                Orientation.attrib['r'] = to_string(0.0)
            else:
                RefEntity = self.jsonfile.entities[entity]['ref_car']
                DsIdx = self.Entities[entity]['keys'].index('ds')
                LonDisDelta = config[DsIdx]
                DlIdx = self.Entities[entity]['keys'].index('dl')
                LatDisDelta = config[DlIdx]
                PrivateAction = ET.SubElement(Private, 'PrivateAction')
                TeleportAction = ET.SubElement(PrivateAction, 'TeleportAction')
                Position = ET.SubElement(TeleportAction, 'Position')
                RoadPosition = ET.SubElement(Position, 'RoadPosition')
                RoadPosition.attrib['roadId'] = to_string(0)
                RoadPosition.attrib['s'] = to_string(float(LonDisDelta) + 50)
                RoadPosition.attrib['t'] = to_string(float(LatDisDelta) - 6.625)
                Orientation = ET.SubElement(RoadPosition, 'Orientation')
                Orientation.attrib['type'] = 'relative'
                YawIdx = self.Entities[entity]['keys'].index('yaw')
                Yaw = config[YawIdx]
                Orientation.attrib['h'] = to_string(float(Yaw))
                Orientation.attrib['p'] = to_string(0.0)
                Orientation.attrib['r'] = to_string(0.0)

            # lon action
            PrivateAction = ET.SubElement(Private, 'PrivateAction')
            Longitudinal = ET.SubElement(PrivateAction, 'LongitudinalAction')
            EntityVIdx = self.Entities[entity]['keys'].index('v')
            EntityV = config[EntityVIdx]
            SpeedAction = ET.SubElement(Longitudinal, 'SpeedAction')
            SpeedActionDynamics = ET.SubElement(SpeedAction, 'SpeedActionDynamics')
            SpeedActionDynamics.attrib['dynamicsShape'] = 'step'
            SpeedActionDynamics.attrib['dynamicsDimension'] = 'time'
            SpeedActionDynamics.attrib['value'] = to_string(0.0)
            SpeedActionTarget = ET.SubElement(SpeedAction, 'SpeedActionTarget')
            AbsoluteTargetSpeed = ET.SubElement(SpeedActionTarget, 'AbsoluteTargetSpeed')
            AbsoluteTargetSpeed.attrib['value'] = to_string(EntityV)
            EntitiesV.append(EntityV)
            # if 'ds' in self.Entities[entity]['keys']:
            #     DsIdx = self.Entities[entity]['keys'].index(self.Entities[entity]['keys'] == 'ds')
            #     LDisDelta = config[DsIdx]
            #     LongitudinalDistanceAction = ET.SubElement(Longitudinal, 'LongitudinalDistanceAction')
            #     EntityRef = ET.SubElement(LongitudinalDistanceAction, 'EntityRef')
            #     EntityRef.attrib['object'] = 'ego'
            #     Distance = ET.SubElement(LongitudinalDistanceAction, 'Distance')
            #     Distance.attrib['distance'] = to_string(LDisDelta)
            #     Freespace = ET.SubElement(LongitudinalDistanceAction, 'Freespace')
            #     Freespace.attrib['freespace'] = 'false'
            #     Continuous = ET.SubElement(LongitudinalDistanceAction, 'Continuous')
            #     Continuous.attrib['continuous'] = 'true'
            
        Story = ET.SubElement(Storyboard, 'Story')
        Story.attrib['name'] = 'PlusaiStory'
        for e, entity in enumerate(self.Entities):
            SimulationTime = 0.0
            EntityV = EntitiesV[e]
            if 'agent' in entity:
                Act = ET.SubElement(Story, 'Act')
                Act.attrib['name'] = to_string(entity + 'act')
                ManeuverGroup = ET.SubElement(Act, 'ManeuverGroup')
                ManeuverGroup.attrib['name'] = to_string(entity + 'maneuver')
                ManeuverGroup.attrib['maximumExecutionCount'] = "1"
                Actors = ET.SubElement(ManeuverGroup, 'Actors')
                Actors.attrib['selectTriggeringEntities'] = 'false'
                EntityRef = ET.SubElement(Actors, 'EntityRef')
                EntityRef.attrib['entityRef'] = to_string(entity)
                Maneuver = ET.SubElement(ManeuverGroup, 'Maneuver')
                Maneuver.attrib['name'] = to_string(entity + 'maneuver')
                Actions = self.jsonfile.entities[entity]['actions']
                for i, act in enumerate(Actions):
                    Event = ET.SubElement(Maneuver, 'Event')
                    Event.attrib['priority'] = 'overwrite'
                    Event.attrib['maximumExecutionCount'] = '1'
                    Event.attrib['name'] = to_string(act['action'])
                    Action = ET.SubElement(Event, 'Action')
                    Action.attrib['name'] = to_string(act['action'])
                    PrivateAction = ET.SubElement(Action, 'PrivateAction')
                    if act['action'] == 'lf':
                        LongitudinalAction = ET.SubElement(PrivateAction, 'LongitudinalAction')
                        SpeedAction = ET.SubElement(LongitudinalAction, 'SpeedAction')
                        SpeedActionDynamics = ET.SubElement(SpeedAction, 'SpeedActionDynamics')
                        SpeedActionDynamics.attrib['dynamicsShape'] = 'linear'
                        SpeedActionDynamics.attrib['dynamicsDimension'] = 'rate'
                        a_choice = random.choice(act['params']['a'])
                        EntityV += a_choice * act['params']['t'][0]
                        EntityV = max(0, EntityV)
                        SpeedActionDynamics.attrib['value'] = to_string(a_choice)
                        SpeedActionTarget = ET.SubElement(SpeedAction, 'SpeedActionTarget')
                        AbsoluteTargetSpeed = ET.SubElement(SpeedActionTarget, 'AbsoluteTargetSpeed')
                        AbsoluteTargetSpeed.attrib['value'] = to_string(EntityV)
                    else:
                        pass
                    trigger = Trigger()
                    StartTrigger = ET.SubElement(Event, 'StartTrigger')
                    ConditionGroup = ET.SubElement(StartTrigger, 'ConditionGroup')
                    Condition = ET.SubElement(ConditionGroup, 'Condition')
                    Condition.attrib['delay'] = to_string(0)
                    Condition.attrib['conditionEdge'] = 'rising'
                    Condition.attrib['name'] = 'Simulation Time'
                    ConditionType = trigger.conditiontypes['V']
                    ConditionType = ET.SubElement(Condition, ConditionType)
                    SimulationTimeCondition = ET.SubElement(ConditionType, 'SimulationTimeCondition')
                    SimulationTimeCondition.attrib['value'] = to_string(SimulationTime)
                    SimulationTimeCondition.attrib['rule'] = to_string(trigger.rules['gt'])
                    SimulationTime += act['params']['t'][0]
                StartTrigger = ET.SubElement(Act, 'StartTrigger')
                StopTrigger = ET.SubElement(Act, 'StopTrigger')
        StopTrigger = ET.SubElement(Storyboard, 'StopTrigger')
        # ConditionGroup = ET.SubElement(StartTrigger, 'ConditionGroup')
        # Condition = ET.SubElement(ConditionGroup, 'Condition')
        # Condition.attrib['delay'] = to_string(0.0)
        # Condition.attrib['conditionEdge'] = 'rising'
        # Condition.attrib['name'] = 'Simulation Time'
        # ConditionType = trigger.conditiontypes['V']
        # ConditionType = ET.SubElement(Condition, ConditionType)
        # SimulationTimeCondition = ET.SubElement(ConditionType, 'SimulationTimeCondition')
        # SimulationTimeCondition.attrib['value'] = to_string(0.0)
        # SimulationTimeCondition.attrib['rule'] = to_string(trigger.rules['gt'])
        s = minidom.parseString(ET.tostring(top, encoding='UTF-8'))
        s = s.toprettyxml(indent='    ')
        with open(f'/home/simulation/VIRES/VTD/Data/Projects/Plusai/Scenarios/{self.jsonfile.name}.xosc', 'w') as o:
            o.write(s)

if __name__ == '__main__':

    parser = argparse.ArgumentParser('Scenario Generator for building OpenSCENARIO file from predefined Json File',
                                    'Before Generation make sure the xodr map file and osgb file are already existed')
    parser.add_argument('input_json')
    args = parser.parse_args()

    with open("./" + args.input_json, 'r') as f:
        jsonfile = JsonFile(Json=json.load(f))

        jsonfile.name = args.input_json.split('.')[0]
        jsonfile.MapFilePath = '/home/simulation/VIRES/VTD/Runtime/Tools/ROD/Plusai/Odr/acc_aeb_reducebeforesteady_0628.xodr'
        jsonfile.SceneFilePath = '/home/simulation/VIRES/VTD/Runtime/Tools/ROD/Plusai/Database/acc_aeb_reducebeforesteady_0628.opt.osgb'

        openscenario = openScenario(jsonfile)

        openscenario.init_process()

        openscenario.GenerateXOSCFile()