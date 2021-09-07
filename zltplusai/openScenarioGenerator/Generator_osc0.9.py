import json
from dataclasses import dataclass
from typing import Any
from copy import deepcopy
from xml.dom import EMPTY_NAMESPACE, minidom
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
        self.conditiontypes = {'E':'ByEntity', 'V':'ByValue'}
        self.relativedistancetype = {'Lon':'longitudinal', 'Lat':'lateral'}
        self.entityref = None
        self.rules = {'lt':'less_than', 'gt': 'greater_than', 'goe': 'greaterOrEqual', 'loe': 'lessOrEqual'}
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

        self.DistanceCondition = {
            'freespace':'false',
            'rule':None,
            'alongRoute':None,
            'value':None
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
        params = ET.SubElement(top, 'ParameterDeclaration')
        catalogs = ET.SubElement(top, 'Catalogs')
        VehicleCatalog = ET.SubElement(catalogs, 'VehicleCatalog')
        directory = ET.SubElement(VehicleCatalog, 'Directory')
        directory.attrib['path'] = "Distros/Current/Config/Players/Vehicles"
        DriverCatalog = ET.SubElement(catalogs, 'DriverCatalog')
        directory = ET.SubElement(DriverCatalog, 'Directory')
        directory.attrib['path'] = "Distros/Current/Config/Players/driverCfg.xml"
        MiscObjectCatalog = ET.SubElement(catalogs, 'MiscObjectCatalog')
        directory = ET.SubElement(MiscObjectCatalog, 'Directory')
        directory.attrib['path'] = "Distros/Current/Config/Players/Objects"
        EnvironmentCatalog = ET.SubElement(catalogs, 'EnvironmentCatalog')
        directory = ET.SubElement(EnvironmentCatalog, 'Directory')
        directory.attrib['path'] = ""
        ManeuverCatalog = ET.SubElement(catalogs, 'ManeuverCatalog')
        directory = ET.SubElement(ManeuverCatalog, 'Directory')
        directory.attrib['path'] = ""
        TrajectoryCatalog = ET.SubElement(catalogs, 'TrajectoryCatalog')
        directory = ET.SubElement(TrajectoryCatalog, 'Directory')
        directory.attrib['path'] = ""
        RouteCatalog = ET.SubElement(catalogs, 'RouteCatalog')
        directory = ET.SubElement(RouteCatalog, 'Directory')
        directory.attrib['path'] = ""

        RoadNetwork = ET.SubElement(top, 'RoadNetwork')
        Logics = ET.SubElement(RoadNetwork, 'Logics')
        Logics.attrib['filepath'] = self.jsonfile.MapFilePath
        SceneGraph = ET.SubElement(RoadNetwork, 'SceneGraph')
        SceneGraph.attrib['filepath'] = self.jsonfile.SceneFilePath

        Entities = ET.SubElement(top, 'Entities')
        EntityKeys = list(self.Entities.keys())
        EntityKeys.reverse()
        
        for entity in EntityKeys:
            Object = ET.SubElement(Entities, 'Object')
            Object.attrib['name'] = entity
            vehicle = ET.SubElement(Object, 'Vehicle')
            if 'agent' in entity:
                vehicle.attrib['name'] = 'Audi_A3_2009_red'
                vehicle.attrib['category'] = 'car'
                ParameterDeclaration = ET.SubElement(vehicle, 'ParameterDeclaration')
                HIdx = self.Entities[entity]['keys'].index('h')
                LIdx = self.Entities[entity]['keys'].index('l')
                WIdx = self.Entities[entity]['keys'].index('w')
                VMaxIdx = self.Entities[entity]['keys'].index('vmx')
                config = random.choice(list(self.Entities[entity]['vals']))

                BoundingBox = ET.SubElement(vehicle, 'BoundingBox')
                Center = ET.SubElement(BoundingBox, 'Center')
                Dimension = ET.SubElement(BoundingBox, 'Dimension')

                Performance = ET.SubElement(vehicle, 'Performance')
                Performance.attrib['mass'] = to_string(1600)
                Performance.attrib['maxSpeed'] = to_string(eval(str(config[VMaxIdx])) * 3.6)
                Performance.attrib['maxAcceleration'] = to_string(20.0)
                Performance.attrib['maxDeceleration'] = to_string(20.0)
                
                Center.attrib['x'] = to_string(config[LIdx] / 2)
                Center.attrib['y'] = to_string(0.0)
                Center.attrib['z'] = to_string(config[HIdx] / 2)
                Dimension.attrib['height'] = to_string(config[HIdx])
                Dimension.attrib['width'] = to_string(config[WIdx])
                Dimension.attrib['length'] = to_string(config[LIdx])
                
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

                Controller = ET.SubElement(Object, 'Controller')
                driver = ET.SubElement(Controller, 'Driver')
                driver.attrib['name'] = "DefaultDriver"
                description = ET.SubElement(driver, 'Description')
                description.attrib['age'] = to_string(28)
                description.attrib['eyeDistance'] = to_string(0.065)
                description.attrib['height'] = to_string(1.8)
                description.attrib['sex'] = 'male'
                description.attrib['weight'] = '60'
                Properties = ET.SubElement(description, 'Properties')
            
            else:
                vehicle.attrib['name'] = 'MANTGS_11_DarkBlue'
                vehicle.attrib['category'] = 'truck'
                ParameterDeclaration = ET.SubElement(vehicle, 'ParameterDeclaration')

                BoundingBox = ET.SubElement(vehicle, 'BoundingBox')
                Center = ET.SubElement(BoundingBox, 'Center')
                Dimension = ET.SubElement(BoundingBox, 'Dimension')
                print(self.Entities[entity]['keys'])
                HIdx = self.Entities[entity]['keys'].index('h')
                LIdx = self.Entities[entity]['keys'].index('l')
                WIdx = self.Entities[entity]['keys'].index('w')
                config = random.choice(list(self.Entities[entity]['vals']))
                print(config)
                Center.attrib['x'] = to_string(config[LIdx] / 2)
                Center.attrib['y'] = to_string(0.0)
                Center.attrib['z'] = to_string(config[HIdx] / 2)
                Dimension.attrib['height'] = to_string(config[HIdx])
                Dimension.attrib['width'] = to_string(config[WIdx])
                Dimension.attrib['length'] = to_string(config[LIdx])

                Performance = ET.SubElement(vehicle, 'Performance')
                Performance.attrib['mass'] = to_string(10000)
                Performance.attrib['maxSpeed'] = to_string(26.4 * 3.6)
                Performance.attrib['maxAcceleration'] = to_string(20.0)
                Performance.attrib['maxDeceleration'] = to_string(20.0)
                
                Axles = ET.SubElement(vehicle, 'Axles')
                Front = ET.SubElement(Axles, 'Front')
                Rear = ET.SubElement(Axles, 'Rear')
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

                Controller = ET.SubElement(Object, 'Controller')
                driver = ET.SubElement(Controller, 'Driver')
                driver.attrib['name'] = "DefaultDriver"
                description = ET.SubElement(driver, 'Description')
                description.attrib['age'] = to_string(28)
                description.attrib['eyeDistance'] = to_string(0.065)
                description.attrib['height'] = to_string(1.8)
                description.attrib['sex'] = 'male'
                description.attrib['weight'] = '60'
                Properties = ET.SubElement(description, 'Properties')
        
        Storyboard = ET.SubElement(top, 'Storyboard')
        Init = ET.SubElement(Storyboard, 'Init')
        Actions = ET.SubElement(Init, 'Actions')
        EntitiesV = []
        for entity in self.Entities:
            Private  = ET.SubElement(Actions, 'Private')
            Private.attrib['object'] = to_string(entity)

            # choose config randomly
            config = random.choice(list(self.Entities[entity]['vals']))            
            # lon action
            PrivateAction = ET.SubElement(Private, 'Action')
            Longitudinal = ET.SubElement(PrivateAction, 'Longitudinal')
            EntityVIdx = self.Entities[entity]['keys'].index('v')
            EntityV = config[EntityVIdx]
            SpeedAction = ET.SubElement(Longitudinal, 'Speed')
            SpeedActionDynamics = ET.SubElement(SpeedAction, 'Dynamics')
            SpeedActionDynamics.attrib['shape'] = 'step'
            # SpeedActionDynamics.attrib['dynamicsDimension'] = 'time'
            # SpeedActionDynamics.attrib['value'] = to_string(0.0)
            SpeedActionTarget = ET.SubElement(SpeedAction, 'Target')
            AbsoluteTargetSpeed = ET.SubElement(SpeedActionTarget, 'Absolute')
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

            # init pose and pos
            if entity == 'ego':
                
                # TeleportAction = ET.SubElement(PrivateAction, 'Action')
                Position = ET.SubElement(PrivateAction, 'Position')
                RoadPosition = ET.SubElement(Position, 'World')
                # RoadPosition.attrib['roadId'] = to_string(0)
                RoadPosition.attrib['x'] = to_string(50)
                RoadPosition.attrib['y'] = to_string(-6.625)
                RoadPosition.attrib['z'] = to_string(0)
                # Orientation = ET.SubElement(RoadPosition, 'Orientation')
                # Orientation.attrib['type'] = 'relative'
                YawIdx = self.Entities[entity]['keys'].index('yaw')
                Yaw = config[YawIdx]
                RoadPosition.attrib['h'] = to_string(Yaw)
                RoadPosition.attrib['p'] = to_string(0.0)
                RoadPosition.attrib['r'] = to_string(0.0)
            else:
                RefEntity = self.jsonfile.entities[entity]['ref_car']
                DsIdx = self.Entities[entity]['keys'].index('ds')
                LonDisDelta = config[DsIdx]
                DlIdx = self.Entities[entity]['keys'].index('dl')
                LatDisDelta = config[DlIdx]
                # TeleportAction = ET.SubElement(PrivateAction, 'TeleportAction')
                Position = ET.SubElement(PrivateAction, 'Position')
                RoadPosition = ET.SubElement(Position, 'World')
                RoadPosition.attrib['x'] = to_string(LonDisDelta + 50 + 100)
                RoadPosition.attrib['y'] = to_string(LatDisDelta -6.625)
                RoadPosition.attrib['z'] = to_string(0)
                # Orientation = ET.SubElement(RelativeRoadPosition, 'Orientation')
                # Orientation.attrib['type'] = 'relative'
                YawIdx = self.Entities[entity]['keys'].index('yaw')
                Yaw = config[YawIdx]
                RoadPosition.attrib['h'] = to_string(Yaw)
                RoadPosition.attrib['p'] = to_string(0.0)
                RoadPosition.attrib['r'] = to_string(0.0)

            
            
        Story = ET.SubElement(Storyboard, 'Story')
        Story.attrib['name'] = 'PlusaiStory'
        for e, entity in enumerate(self.Entities):
            SimulationTime = 0.0
            EntityV = EntitiesV[e]
            if 'agent' in entity:
                Act = ET.SubElement(Story, 'Act')
                Act.attrib['name'] = to_string(entity + 'act')
                ManeuverGroup = ET.SubElement(Act, 'Sequence')
                ManeuverGroup.attrib['name'] = to_string(entity + 'sequence')
                ManeuverGroup.attrib['numberOfExecutions'] = "1"
                Actors = ET.SubElement(ManeuverGroup, 'Actors')
                # Actors.attrib['selectTriggeringEntities'] = 'false'
                EntityRef = ET.SubElement(Actors, 'Entity')
                EntityRef.attrib['name'] = to_string(entity)
                Maneuver = ET.SubElement(ManeuverGroup, 'Maneuver')
                Maneuver.attrib['name'] = to_string(entity + 'maneuver')
                Actions = self.jsonfile.entities[entity]['actions']
                
                
                Event = ET.SubElement(Maneuver, 'Event')
                Event.attrib['priority'] = 'overwrite'
                # Event.attrib['maximumExecutionCount'] = '1'
                Event.attrib['name'] = to_string(entity + 'waiting')
                Action = ET.SubElement(Event, 'Action')
                Action.attrib['name'] = to_string('waiting')
                PrivateAction = ET.SubElement(Action, 'Private')
                trigger = Trigger()
                StartTrigger = ET.SubElement(Event, 'StartConditions')
                ConditionGroup = ET.SubElement(StartTrigger, 'ConditionGroup')
                Condition = ET.SubElement(ConditionGroup, 'Condition')
                Condition.attrib['delay'] = to_string(0.0)
                Condition.attrib['edge'] = 'rising'
                Condition.attrib['name'] = 'Simulation Time'
                ConditionType = trigger.conditiontypes['V']
                ConditionType = ET.SubElement(Condition, ConditionType)
                SimulationTimeCondition = ET.SubElement(ConditionType, 'SimulationTime')
                SimulationTimeCondition.attrib['value'] = to_string(0.0)
                SimulationTimeCondition.attrib['rule'] = to_string(trigger.rules['gt'])

                Event = ET.SubElement(Maneuver, 'Event')
                Event.attrib['priority'] = 'overwrite'
                # Event.attrib['maximumExecutionCount'] = '1'
                Event.attrib['name'] = to_string(entity + 'preaction')
                Action = ET.SubElement(Event, 'Action')
                Action.attrib['name'] = to_string(entity + 'preaction')
                PrivateAction = ET.SubElement(Action, 'Private')

                LongitudinalAction = ET.SubElement(PrivateAction, 'Longitudinal')
                SpeedAction = ET.SubElement(LongitudinalAction, 'Speed')
                SpeedActionDynamics = ET.SubElement(SpeedAction, 'Dynamics')
                SpeedActionDynamics.attrib['shape'] = 'step'
                # SpeedActionDynamics.attrib['time'] = to_string(act['params']['t'][0])
                # a_choice = random.choice(act['params']['a'])
                # SpeedActionDynamics.attrib['value'] = to_string(a_choice)
                
                # SpeedActionDynamics.attrib['value'] = to_string(a_choice)
                
                # EntityV += a_choice * act['params']['t'][0]
                # EntityV = max(EntityV, 0.0)
                SpeedActionTarget = ET.SubElement(SpeedAction, 'Target')
                AbsoluteTargetSpeed = ET.SubElement(SpeedActionTarget, 'Absolute')
                AbsoluteTargetSpeed.attrib['value'] = to_string(EntityV)

                trigger = Trigger()
                StartTrigger = ET.SubElement(Event, 'StartConditions')
                ConditionGroup = ET.SubElement(StartTrigger, 'ConditionGroup')
                Condition = ET.SubElement(ConditionGroup, 'Condition')
                Condition.attrib['delay'] = to_string(0.0)
                Condition.attrib['edge'] = 'falling'
                Condition.attrib['name'] = 'ConditionWaiting'
                ConditionType = trigger.conditiontypes['E']
                ConditionType = ET.SubElement(Condition, ConditionType)
                TriggeringEntities = ET.SubElement(ConditionType, 'TriggeringEntities')
                TriggeringEntities.attrib['rule'] = 'all'
                Entity = ET.SubElement(TriggeringEntities, 'Entity')
                Entity.attrib['name'] = entity
                EntityCondition = ET.SubElement(ConditionType, 'EntityCondition')
                Distance = ET.SubElement(EntityCondition, 'Distance')
                trigger.DistanceCondition['alongRoute'] = 'false'
                trigger.DistanceCondition['freespace'] = 'false'
                trigger.DistanceCondition['rule'] = 'less_than'
                trigger.DistanceCondition['value'] = to_string(self.jsonfile.entities[entity]['ds'][0])
                Distance.attrib['rule'] = trigger.DistanceCondition['rule']
                Distance.attrib['alongRoute'] = trigger.DistanceCondition['alongRoute']
                Distance.attrib['value'] = trigger.DistanceCondition['value']
                Distance.attrib['freespace'] = trigger.DistanceCondition['freespace']
                Position = ET.SubElement(Distance, 'Position')
                RelativeObject = ET.SubElement(Position, 'RelativeObject')
                RelativeObject.attrib['object'] = self.jsonfile.entities[entity]['ref_car']
                RelativeObject.attrib['dx'] = to_string(0.0)
                RelativeObject.attrib['dy'] = to_string(0.0)
                
                for i, act in enumerate(Actions):
                    Event = ET.SubElement(Maneuver, 'Event')
                    Event.attrib['priority'] = 'overwrite'
                    # Event.attrib['maximumExecutionCount'] = '1'
                    Event.attrib['name'] = to_string(act['action'])
                    Action = ET.SubElement(Event, 'Action')
                    Action.attrib['name'] = to_string(act['action'])
                    PrivateAction = ET.SubElement(Action, 'Private')
                    if act['action'] == 'lf':

                        LongitudinalAction = ET.SubElement(PrivateAction, 'Longitudinal')
                        SpeedAction = ET.SubElement(LongitudinalAction, 'Speed')
                        SpeedActionDynamics = ET.SubElement(SpeedAction, 'Dynamics')
                        SpeedActionDynamics.attrib['shape'] = 'linear'
                        SpeedActionDynamics.attrib['time'] = to_string(act['params']['t'][0])
                        a_choice = random.choice(act['params']['a'])
                        SpeedActionDynamics.attrib['value'] = to_string(a_choice)
                        
                        SpeedActionDynamics.attrib['value'] = to_string(a_choice)
                        
                        EntityV += a_choice * act['params']['t'][0]
                        EntityV = max(EntityV, 0.0)
                        SpeedActionTarget = ET.SubElement(SpeedAction, 'Target')
                        AbsoluteTargetSpeed = ET.SubElement(SpeedActionTarget, 'Absolute')
                        AbsoluteTargetSpeed.attrib['value'] = to_string(EntityV)
                        
                        trigger = Trigger()
                        StartTrigger = ET.SubElement(Event, 'StartConditions')
                        ConditionGroup = ET.SubElement(StartTrigger, 'ConditionGroup')
                        Condition = ET.SubElement(ConditionGroup, 'Condition')
                        Condition.attrib['delay'] = to_string(0.0)
                        Condition.attrib['edge'] = 'rising'
                        Condition.attrib['name'] = 'Simulation Time'
                        if abs(a_choice) == 0:
                            ConditionType = trigger.conditiontypes['E']
                            ConditionType = ET.SubElement(Condition, ConditionType)
                            TriggeringEntities = ET.SubElement(ConditionType, 'TriggeringEntities')
                            TriggeringEntities.attrib['rule'] = 'all'
                            Entity = ET.SubElement(TriggeringEntities, 'Entity')
                            Entity.attrib['name'] = entity
                            EntityCondition = ET.SubElement(ConditionType, 'EntityCondition')
                            TraveledDistanceCondition = ET.SubElement(EntityCondition, 'TraveledDistance')
                            trigger.TraveledDistanceCondition['value'] = to_string(act['params']['t'][0] * EntityV)
                            TraveledDistanceCondition.attrib['value'] = to_string(trigger.TraveledDistanceCondition['value'])
                        else:
                            ConditionType = trigger.conditiontypes['E']
                            ConditionType = ET.SubElement(Condition, ConditionType)
                            TriggeringEntities = ET.SubElement(ConditionType, 'TriggeringEntities')
                            TriggeringEntities.attrib['rule'] = 'all'
                            Entity = ET.SubElement(TriggeringEntities, 'Entity')
                            Entity.attrib['name'] = entity
                            EntityCondition = ET.SubElement(ConditionType, 'EntityCondition')
                            Speed = ET.SubElement(EntityCondition, 'Speed')
                            trigger.SpeedCondition['rule'] = 'equal_to'
                            trigger.SpeedCondition['value'] = EntityV
                            Speed.attrib['rule'] = to_string(trigger.SpeedCondition['rule'])
                            Speed.attrib['value'] = to_string(trigger.SpeedCondition['value'])
                # StartTrigger = ET.SubElement(Act, 'StartTrigger')
                # StopTrigger = ET.SubElement(Act, 'StopTrigger')
                Conditions = ET.SubElement(Act, 'Conditions')
                Start = ET.SubElement(Conditions, 'Start')
                ConditionGroup = ET.SubElement(Start, 'ConditionGroup')
                Condition = ET.SubElement(ConditionGroup, 'Condition')
                Condition.attrib['delay'] = to_string(0)
                Condition.attrib['edge'] = 'rising'
                Condition.attrib['name'] = 'ConditionWaiting'
                ConditionType = trigger.conditiontypes['V']
                ConditionType = ET.SubElement(Condition, ConditionType)
                SimulationTimeCondition = ET.SubElement(ConditionType, 'SimulationTime')
                SimulationTimeCondition.attrib['value'] = to_string(0.0)
                SimulationTimeCondition.attrib['rule'] = to_string(trigger.rules['gt'])
        # StopTrigger = ET.SubElement(Storyboard, 'StopTrigger')
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



            


            





            



                

                
                    


                    
                    


