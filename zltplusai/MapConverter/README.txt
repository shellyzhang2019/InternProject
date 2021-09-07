/******************************** PLUSAI ****************************************/
This Generator is designed for automatically generate opendrive map for simulation

Current generator can only be used in motorway road generation


# Notes:

Map Params:

* type: params for road attribute, there are only two kinds of road segment:
        1. Connector -- connecting two road segment in junction
        2. Non-connector -- main road
    
    * Connector: a. Always belongs to a junction;
                 b. Always has outcoming road as parent, and incoming road as child,
                 c. In case of b, connectors always has one way lanes

    * Non-connector:  main road

* direction: 
    * Bio: Dual driving direction road
    * Uni: Single driving direction road


* id: id for current road

* links: defines the road link and lane link both for main road and connectors

    * parent: current road parent (can be mutiple)
    * child: current road child (can be mutiple)
    * lanelinks: all infos for lane links
        Example:
            (0, 1): (-1, -1):
                parent: 0
                current: 1
                connection: start from parent -1 lane end at current -1 lane

* startxy:

    Only road with no parent has startxy

* heading(rad)

* builders:

    Road Geometry

* elevations:

    no details

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Usage: python3 Generator2.py StraightRoad.json


>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Maps Folder Contains:

	1. Stright Road Model
	2. Curve Road Model
	3. Complicated Road Model --> HIghway

>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

ScenarioMaps Contains:
	
	corresponding odr map for example scenarios






