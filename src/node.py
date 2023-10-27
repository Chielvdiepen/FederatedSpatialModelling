from edge import Edge
from triangle import Triangle
from adjacent import Adjacent
import numpy as np
import math
import copy
from mymath import yaw, roll, pitch
import time
from config import DIM2D, RSSI, MAXTRIANGLES

# Node class, which represents a individual Crownstone
class Node:

    def __init__(self, HWuuid: str):
        self.uuid = HWuuid
        self.coord = (0,0,0)
        self.SurNodes = [] # list of (node reference, rssi)
        self.adjEdgeList = []
        self.oppEdgeList = []
        self.triangleList = []
        self.adjTrianglesDict = {}
        
        self.mapCoordsDict = {}
        self.mapEdgeSetDict = {}
        self.mappingDict = {}
        self.transCoordsDict = {}
   
    def __repr__(self):
        return str(self)
    
    def __str__(self) -> str:
        edges_str = ', '.join(map(str, self.adjEdgeList))
        triangles_str = '\n'.join(map(str, self.triangleList))
        nodes_str = ', '.join(map(str, [node.uuid for node, rssi in self.SurNodes])) 
        oppos_str = ', '.join(map(str, self.oppEdgeList)) 
        line = "------------------------------------------------------------------------------------------------\n"
        return f"{line}Node uuid={self.uuid}:{self.coord} [:] Surrounding Nodes= {nodes_str}, Edgelist= {edges_str}\n\nTriangleList:\n{triangles_str}\n\nOpposite edges={oppos_str} [:]\n"
    
    # Print all node information
    def printNodeInfo(self):
        print(self)
        if len(self.adjTrianglesDict) == 0:
            print("No adj triangles dict")
            return
        
        print(f"Adjacent triangles of {self.uuid}:") 
        for edge, adj_list in self.adjTrianglesDict.items():
            print(f"Edge {edge.id}:")
            for adj in adj_list:
                print(adj)

        print('\nPlotting the coordinate map of each base edge:')
        for edge, mapCoords in self.mapCoordsDict.items():
            print(f'Base_edge {edge}:')
            for coord in sorted(mapCoords.items()):
                print(f"Node {coord[0]}: ({coord[1][0]:.3f},{coord[1][1]:.3f},{coord[1][2]:.3f})")
            print()

        print('\nFor each map, the translations:')
        for edge, mapping in self.mappingDict.items():
            print(f'Base Edge {edge}:')
            for uuid, translation in mapping.items():
                print(f"Node {uuid}: Factor= {translation[0]:.3f} Angle= {translation[1]:.3f}, MapAngle= {translation[2]:.3f}")
            print()

        print('\nAfter translation:')
        for edge, mapCoords in self.transCoordsDict.items():
            print(f'Base_edge {edge}:')
            for coord in sorted(mapCoords.items()):
                print(f"Node {coord[0]}: ({coord[1][0]:.3f},{coord[1][1]:.3f},{coord[1][2]:.3f})")
            print()
        print()
        print("------------------------------------------------------------------------------------------------\n")
      
    def compare(self, other: object):
        return self.uuid == other.uuid
    
    # Initial scan for surrounding nodes in the environment 
    def scanSurroundings(self, network_edges):
        surNodeSet = set()
        for edge in network_edges[self.uuid]:
            surNodeSet.add((edge.dst,edge.rssi))
        self.SurNodes = sorted(surNodeSet, key=lambda x: x[1], reverse=RSSI)

    # Node search request for searched_Node at target_node
    def requestNodeSearch(self, target_node, searched_Node):
        # send BLE message to target and wait for response
        found = target_node.nodeSearch(searched_Node)
        if (found == False):
            print(f"{self.uuid}: Node NOT visible")
        else:
            return found # (Node, RSSI)

    # NodeSearch response, checks if requested node is visible in surroundings
    def nodeSearch(self, checkFor: object) -> tuple:
        for surNode in self.SurNodes:
            if surNode[0] == checkFor:
                return surNode
        return False
    
    # Checks if edge is in current list of edges
    def checkEdge(self, check_edgeDst):
        for edge in self.adjEdgeList:
            if edge.dst == check_edgeDst:
                return edge
        return False
    
    # Make a new edge from the supplied nodes and add it to the adjEdgeList if it does not exist already
    def addEdge(self, node1, node2, rssi):
        # check if Edge not already exist
        if self.checkEdge(node2) == False:
            newEdge = Edge(node1, node2, rssi)
            self.adjEdgeList.append(newEdge)
            return True
        else:
            # print("Duplicate!")
            return False

    # Creates an edge with other_sur by making a requestNodeSearch() for itself at other_sur.
    # The edge will be stored in the adjacent edge list, it uses the rssi of other_sur. 
    # List will be sorted due to sorted sur_nodes
    def createEdgeWith(self, other_Sur: tuple):
        check = self.requestNodeSearch(other_Sur[0],self)
        if check == False:
            print(f"{self.uuid}: Can not form edge, Node not visible!")
            return False
        else:
            return self.addEdge(self,other_Sur[0],other_Sur[1])
    
    # Check if triangle is already added to internal trianglelist
    def checkTriangle(self, checkTriangle):
        for triangle in self.triangleList:
            if triangle == checkTriangle:
                return True
        return False
    
    # Internal triangle/opposite_edge search in all triangles of the trianglelist
    def triangleSearch(self, opposite_edge):
        for triangle in self.triangleList:
            if triangle.hasEdge(opposite_edge):
                return triangle
        return False  
    
    # Internal triangle/opposite_edge search in all triangles of the trianglelist
    # def adjMapSearch(self, baseEdge, thirdNodeUUID):
    #     for adjTriangle in self.adjTrianglesDict[baseEdge]:
    #         if adjTriangle.triangle.getLastNode(baseEdge) == thirdNodeUUID:
    #             if adjTriangle.mapAngle == 0 
    #             return 
    #     return False
    
    # Triangle search request for target, should actually return the needed values instead of whole Triangle object
    def requestTriangleSearch(self, target_Node, searched_Edge):
        # send BLE message to target and wait for response
        return target_Node.triangleSearch(searched_Edge)

    # Adjacent Triangle search request for target, should actually return the needed values instead of whole Triangle object
    def requestAdjTriangleSearch(self, target_Node, searched_Edge):
        # send BLE message to target and wait for response
        return target_Node.adjMapSearch(searched_Edge)

    # Make a new triangle from the supplied edges and add it to the trianglist if it does not exist already, 
    def addTriangle(self, edge1,edge2,common_edge):
        temp = Triangle(edge1,edge2,common_edge)
        # check if triangle not already exist, false is not in list
        if self.checkTriangle(temp) == False and temp.area > 0.5:
            self.oppEdgeList.append(common_edge)
            self.oppEdgeList.sort(key=lambda x: x.rssi,reverse=RSSI)
            self.triangleList.append(temp)
            # self.triangleList.sort(key=lambda x: x.contribution, reverse=True)
            self.triangleList.sort(key=lambda x: x.area, reverse=False)
            return True
        else:
            # print("Duplicate!")
            return False

    # create temporary triangle to make altitude and angle calculations    
    def createTempTriangle(self, defaultOtherNode, otherNode):
        checkForOther = self.requestNodeSearch(defaultOtherNode,otherNode)
        checkForDef = self.requestNodeSearch(otherNode,defaultOtherNode)
        if checkForOther == False or checkForDef == False:
            print(f"{self.uuid}: Check Error {defaultOtherNode.uuid} {otherNode.uuid}")
            return False
        if checkForOther[0].compare(otherNode) and checkForDef[0].compare(defaultOtherNode):
            common_edge = Edge(defaultOtherNode,otherNode,checkForDef[1])       
            temp = Triangle(Edge(self,defaultOtherNode,self.nodeSearch(defaultOtherNode)[1]),Edge(self,otherNode,self.nodeSearch(otherNode)[1]),common_edge)
            # print(f'{self.uuid}: Temp triangle = {temp}')
            return temp
        else:
            print(f"{self.uuid}: Compare Error {defaultOtherNode.uuid} {otherNode.uuid}")
            return False
        
    # Triangle search request for target
    def requestTempTriangle(self, target_Node, defaultNode, otherNode):
        # send BLE message to target and wait for response
        return target_Node.createTempTriangle(defaultNode,otherNode)

    # Creates a Triangle with self, dst1 and dst2, by making a requestNodeSearch() at dst1 for dst2, and at dst2 for dst1.
    # If there is mutual agreement, an edge between dst1 and dst2 will be made and stored in opposite edge list. 
    # The three edges will be added togheter to form a new triangle
    def createTriangleWith(self, edge1, edge2):
        dst1, dst2 = edge1.dst, edge2.dst
        if self.compare(dst1) or self.compare(dst2):
            print(f"{self.uuid}: Can not make Triangle with itself")
            return False
        
        checkForDst2 = self.requestNodeSearch(dst1,dst2)
        checkForDst1 = self.requestNodeSearch(dst2,dst1)
        if checkForDst2 == False or checkForDst1 == False:
            print(f"{self.uuid}: Triangle not possible, no common edge")
            return False
        if checkForDst2[0].compare(dst2) and checkForDst1[0].compare(dst1):
            common_edge = Edge(dst1,dst2,checkForDst1[1])
            return self.addTriangle(edge1,edge2,common_edge)


    # This procedure will walk all the nodes and will make N triangles. routine: iterate surnodes, make edge from first, add them to edgelist, 
    # check if edge sees other edge, if True, form edges and make triangle, add triangle to trianglelist
    def triangleProcedure(self):
        surCount = len(self.SurNodes)
        # self.triangleList = array(MAXTRIANGLES) in C++ allocate array of 
        for i in range(surCount):
            for j in range(i+1, surCount):
                self.createEdgeWith(self.SurNodes[i]) # tuple(node,rssi)
                self.createEdgeWith(self.SurNodes[j])
                edgeP1 = self.checkEdge(self.SurNodes[i][0]) # search on dst node
                edgeP2 = self.checkEdge(self.SurNodes[j][0]) 
                self.createTriangleWith(edgeP1,edgeP2)
                if len(self.triangleList) == MAXTRIANGLES:
                    # TODO: replace when lower weight
                    return
    
    # Links all triangles from trianglelist to individual edges from the edgelist, so edge(AB) has adj
    def createAdjTriangles(self):
        if len(self.adjEdgeList) == 0 or len(self.triangleList) == 0:
            print(f"{self.uuid}: NO edges or triangles")
            return False 
        
        triangle_dict = {}
        for base_edge in self.adjEdgeList:
            for triangle in self.triangleList:
                if triangle.hasEdge(base_edge):
                    adjTriangle = Adjacent(triangle)
                    if base_edge not in triangle_dict:
                        adjTriangle.setCoord((base_edge.dst.uuid,base_edge.dist))
                        triangle_dict[base_edge] = [adjTriangle]
                    else:
                        triangle_dict[base_edge].append(adjTriangle)          
        self.adjTrianglesDict = triangle_dict
    

    # ABC and ABD: From the two adjacent nodes C and D, get the height and xpos of the altitude, calculate dXpos
    # Get the distance between C and D, together with dXpos calculate the translation with pythagoras.
    # Dihedral angle calculation with Triangle()
    def getMapAngle(self, defaultOtherNode, otherNode,base_altiX,altiX,base_altiH,altiH):
        reqTri = self.triangleSearch(Edge(defaultOtherNode,otherNode,0)) # first check internal
        if reqTri == False:
            # call make temp triangle, for calculation of altitude -> angle
            reqTri = self.createTempTriangle(defaultOtherNode,otherNode)
            if reqTri == False:
                return None
            
        nodeDist = reqTri.edges[2].dist
        deltaX = abs(base_altiX - altiX)
        pyth = max(0, nodeDist**2 - deltaX**2)

        translation = math.sqrt(pyth)
        a, b, c = translation, base_altiH, altiH
        p = max(-1, min(1, (b**2 + c**2 - a**2) / (2 * b * c)))
        mapAngle = np.rad2deg(np.arccos(p)) 
        return mapAngle
    

    # Requests altitude position and height from the askNode, for example: you are A in triangle ABC, base_edge AB, askNode C, check if it has triangle with the values for altitude information 
    def mapAltiRequest(self, askNode, base_edge, self_angle):
        reqTri = self.requestTriangleSearch(askNode, base_edge)
        if reqTri == False:
            # call make temp triangle at target_node, for calculation of altitude -> angle
            reqTri = self.requestTempTriangle(askNode,base_edge.src,base_edge.dst)
            if reqTri == False:
                return None, None

        if reqTri.altiX[0][0] == self.uuid: # return the x pos from perspective self, to the asked node
            if self_angle > 90:
                return (reqTri.altiX[0][0],-reqTri.altiX[0][1]), reqTri.altiH # list(tuple(node,-alti),tuple(node,alti))
            else:
                return reqTri.altiX[0], reqTri.altiH # list(tuple(node,alti),tuple(node,alti))
        else:
            if self_angle > 90:
                return (reqTri.altiX[1][0],-reqTri.altiX[1][1]), reqTri.altiH # list(tuple(node,-alti),tuple(node,alti))
            else:
                return reqTri.altiX[1], reqTri.altiH # list(tuple(node,alti),tuple(node,alti))
    

    # Iterates on the adjacent triangles, set the first on XY/default and calculate for the following triangles the angle between default, altiX, altiH 
    def mapAdjacents(self):
        for base_edge, adj_list in self.adjTrianglesDict.items():
            defaultOtherNode = adj_list[0].triangle.getLastNode(base_edge)
            selfAngle = adj_list[0].triangle.angle[1]  # tuple(node,angle)
            base_altiX, base_altiH = self.mapAltiRequest(defaultOtherNode,base_edge, selfAngle)

            if base_altiX == None or base_altiH == None:
                print(f"{self.uuid}: mapAltiRequest failed!")
            else:
                adj_list[0].setAltiH(base_altiH)
                adj_list[0].setAltiX(base_altiX)        
            for i in range(1,len(adj_list)):
                otherNode = adj_list[i].triangle.getLastNode(base_edge)
                selfAngle = adj_list[i].triangle.angle[1] # tuple(node,angle)
                altiX, altiH = self.mapAltiRequest(otherNode,base_edge,selfAngle)
            
                if altiX == None or altiH == None:
                    print(f"{self.uuid}: mapAltiRequest failed!")
                else:
                    adj_list[i].setAltiH(altiH)
                    adj_list[i].setAltiX(altiX)
                    adj_list[i].setCoord(('X',0))

                    angle = self.getMapAngle(defaultOtherNode,otherNode,base_altiX[1],altiX[1],base_altiH[1],altiH[1])
                    if angle == None:
                        print(f"{self.uuid}: mapAngle failed!")
                    else:
                        adj_list[i].setAngle(angle)
                        newCoord = np.dot(roll(angle),np.array([altiX[1], altiH[1], 0.00]))
                        adj_list[i].setMapHeight(newCoord[2])

    # For each base_edge in the adjTrianglesDict, make a map of the coords and edges
    def makeMap(self):
        for base_edge, adjList in self.adjTrianglesDict.items():

            mapCoords = {self.uuid: self.coord,}
            mapEdgeSet = [] 

            baseLength = base_edge.dist
            mapping = {base_edge.src.uuid:(1,0,0), base_edge.dst.uuid:(1,0,0),}
            
            # go over all triangles and place the coords on the x-axis and rotate, also make list of all edges from all adj triangles
            for i, adj in enumerate(adjList):
                if i == 0:
                    mapCoords.update({adj.otherCoord[0]:(adj.otherCoord[1],0.000,0.000)})
                
                if adj.triangle.edges[0].compare(base_edge):
                    compareEdge = adj.triangle.edges[1]
                else: 
                    compareEdge = adj.triangle.edges[0]

                point = np.array([adj.basealtiX[1], adj.basealtiH[1], 0.000])    
                remapped = np.dot(roll(adj.mapAngle), point)
                
                mapCoords.update({adj.basealtiH[0]:(remapped[0],remapped[1],remapped[2])})
                mapEdgeSet.extend(edge for edge in adj.triangle.edges if not any(edge.compare(existing_edge) for existing_edge in mapEdgeSet))
                mapping.update({compareEdge.dst.uuid:((baseLength/compareEdge.dist),adj.triangle.angle[1],adj.mapAngle)}) # np.rad2deg(np.tan(remapped[2]/remapped[0]))
            
            self.mapCoordsDict.update({base_edge.id: mapCoords})
            self.mapEdgeSetDict.update({base_edge.id: mapEdgeSet})
            self.mappingDict.update({base_edge.id: mapping})
    
    # function should use the first translation map, multiply the first coord map with the base_edge.dst
    # coordinates multiple with translation (factor, angle, map angle)
    def translateCoord(self):
        translations = self.mappingDict[self.adjEdgeList[0]]
        self.transCoordsDict = copy.deepcopy(self.mapCoordsDict)

        # looprange = range(0,360,1)
        # looprange = range(90,271,90)
        looprange = range(180,360,180)

        for base_edge, mapCoord in self.transCoordsDict.items():
            r, angle, map = translations[base_edge[1]]
            for uuid, coord in mapCoord.items():
                remapped = np.array([coord[0], coord[1],coord[2]]) 

                remapped = np.dot(yaw(angle), remapped)
                remapped = np.dot(roll(map), remapped)

                # mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                
                # match the coord with the first map, working for 2D, tryout for 3D
                base_coords = self.transCoordsDict[self.adjEdgeList[0]]
                if DIM2D == True:
                    if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                        mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                    else:
                        remapped = np.dot(yaw(-angle), remapped)
                        remapped = np.dot(roll(180), remapped)
                        remapped = np.dot(yaw(angle), remapped)
                        if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                            mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                        else:
                            remapped = np.dot(yaw(-angle), remapped)
                            remapped = np.dot(roll(-180), remapped)
                            remapped = np.dot(yaw(angle), remapped)
                            remapped = np.dot(roll(180), remapped)
                            if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                                mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                            else:
                                remapped = np.dot(roll(-180), remapped)
                                # back to original
                                mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                                # mapCoord.update({uuid:(0,0,0)})
                else:
                    if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                        mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                    else:
                        for i in looprange:
                            remapped = np.dot(yaw(-angle), remapped)
                            remapped = np.dot(roll(i), remapped)
                            remapped = np.dot(yaw(angle), remapped)
                            if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                                mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                                break
                            else:
                                remapped = np.dot(yaw(-angle), remapped)
                                remapped = np.dot(roll(-i), remapped)
                                remapped = np.dot(yaw(angle), remapped)

                        for i in looprange:
                            remapped = np.dot(roll(i), remapped)
                            if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                                mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                                break
                            else:
                                remapped = np.dot(roll(-i), remapped)

                        # change coord map angle to opposite side
                        remapped = np.dot(yaw(-angle), remapped)
                        remapped = np.dot(roll(-map), remapped)
                        remapped = np.dot(roll(-map), remapped)
                        remapped = np.dot(yaw(angle), remapped)

                        for i in looprange:
                            remapped = np.dot(yaw(-angle), remapped)
                            remapped = np.dot(roll(i), remapped)
                            remapped = np.dot(yaw(angle), remapped)
                            if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                                mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                                break
                            else:
                                remapped = np.dot(yaw(-angle), remapped)
                                remapped = np.dot(roll(-i), remapped)
                                remapped = np.dot(yaw(angle), remapped)

                        for i in looprange:
                            remapped = np.dot(roll(i), remapped)
                            if round(remapped[0],3) == round(base_coords[uuid][0],3) and round(remapped[1],3) == round(base_coords[uuid][1],3) and round(remapped[2],3) == round(base_coords[uuid][2],3):
                                mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})
                                break
                            else:
                                remapped = np.dot(roll(-i), remapped)
                        
                        mapCoord.update({uuid:(remapped[0],remapped[1],remapped[2])})

    # init function, scan surroundings, make triangles
    def init(self, network_edges):
        self.scanSurroundings(network_edges)
        time.sleep(5)
        self.triangleProcedure()
        time.sleep(5)

    # main function, calls init first, for getting edges amd triangles, after: create adj triangles, map adjacents, make map, translate coords, print node info
    def main(self, network_edges):
        self.init(network_edges)
        self.createAdjTriangles()
        self.mapAdjacents()
        time.sleep(5)
        self.makeMap()
        time.sleep(7)
        self.translateCoord()
        time.sleep(7)
        self.printNodeInfo()