# -*- coding: utf-8 -*-
"""
Created on Fri Apr  2 23:26:27 2021

@author: Yiran
"""

#--------------------------------
# project scripts
from services.vrp_src.modelComponents import Tour, Instance, ParamSet
#--------------------------------

#--------------------------------
# packages
import random
import warnings
#--------------------------------


class HeurMethodBase:
    # Base class for a schedule of all tours
    def __init__(self, ins: Instance, path_plan=None):
        ins.isValid()
        self.tour_dict = {} #all tours do NOT contain depots
        self.ins = ins
        
        # node status is None if no tour assigned else tour_name
        self.depot_start = self.ins.depot_start
        self.depot_end = self.ins.depot_end
        self.ignore_tw = False

        if path_plan is not None:
            self.setPathPlan(path_plan)
        
        self.params = ParamSet()
        self.params.addParameter('frozen_vehicles', 0)
        self.updateCustomerPool()
        
        # use Tour class built-in objective function
        # adjust below for customised objective function
        self._tour_obj = None
        self._tour_obj_type = 'simple'

    
    @property
    def score(self):
        score = sum(tour.score for tour in self.tour_dict.values())
        return score
    
    def setTourObjective(self, tour_obj, obj_type):
        assert(callable(tour_obj) and obj_type in ['simple', 'expected', 'worst'])
        
        self._tour_obj = tour_obj
        self._tour_obj_type = obj_type
    

    def setTourObjType(self, obj_type):
        assert(obj_type in ['simple', 'expected', 'worst'])
        self._tour_obj_type = obj_type


    def clearAllTours(self):
        self._node_to_tour = {node.index: None for node in self.customer_pool} 

        del self.tour_dict

        self.tour_dict = {}


    def freezeVehicles(self):
        self.params.frozen_vehicles.setValue(1)
    
    
    def defreezeVehicles(self):
        self.params.frozen_vehicles.setValue(0)
    
    
    def updateCustomerPool(self):
        self.customer_pool = [node for node in self.ins.node_dict.values()
                                  if node.is_active and not node.is_depot] # all active customers
        
        self._node_to_tour = {node.index: None for node in self.customer_pool}         
        
    
    def __str__(self):
        # format print output
        wp_str = "Current WorkPlan:"
        for tour in self.getListOfTours():
            wp_str += "\n" + str(tour)
        
        return wp_str
    
    
    def _workloadDiffAfterAddingNew(self, node_left, new_node, node_right):
        #arc_dict = self.ins.arc_dict
        left_ind = node_left.index
        right_ind = node_right.index
        new_ind = new_node.index
        
        return new_node.service_time + \
                self.ins.arc_dict[left_ind, new_ind].travel_time + \
                self.ins.arc_dict[new_ind, right_ind].travel_time - \
                self.ins.arc_dict[left_ind, right_ind].travel_time
                
    
    def _greedyInsertNode(self, node):
        # Find the best tour to insert the given node
        DRIVER_MAX_WORKLOAD = self.params['L']
        tour_score_list = []
        for tour in self.getListOfTours():
            position_score_list = []
            for p in range(len(tour)+1):
                if p < len(tour):
                    p_node = tour[p] # suppose insert ahead of p_node
                    position_score_list.append( \
                        (p, \
                         self._workloadDiffAfterAddingNew(tour._prevNode(p_node), node, p_node)\
                         ) \
                        )
                else:
                    p_node = tour[-1] # suppose insert at the end
                    position_score_list.append( \
                        (p, \
                         self._workloadDiffAfterAddingNew(p_node, node, tour._nextNode(p_node))\
                         ) \
                        )
            if len(position_score_list):
                best_position, best_score = min(position_score_list, key=lambda x: x[1])
                if best_score + tour.worst_workload <= DRIVER_MAX_WORKLOAD:
                    # consider tour valid only if the total workload satisfies the workload limit
                    tour_score_list.append((tour, best_position, best_score))
        
        # consider creating a new tour
        new_tour_score = self.ins.params.cost_hire + \
                         self._workloadDiffAfterAddingNew(self.depot_start, node, self.depot_end)
        
        if len(tour_score_list):
            best_tour, best_position, best_score = min(tour_score_list, key=lambda x:x[2])
            if best_score < new_tour_score:
                self.insertIntoTourByName(best_tour.name, node)
                return # terminate
            
        # if not terminate i.e. no existed tour is chosen    
        self.createNewTourByPath([node]) # create a new tour
    
    
    def solve(self):
        # use bin-packing greedy stretegy to obtain initial solution
        candidate_list = list(self.customer_pool)
        while len(candidate_list):
            node = random.sample(candidate_list, 1)[0]
            candidate_list.remove(node)
            self._greedyInsertNode(node)
    
    
    def getListOfTours(self):
        return list(self.tour_dict.values())
        
    
    def verify(self):
        # check if the invoking plan is valid
        # 1. all customer nodes should have exactly one tour assigned
        # 2. (more to be added)
        for node in self.customer_pool:
            if self._getTourHavingNode(node) is None:
                return False
        
        return True
    
    
    def removeNodeFromPool(self, node):
        if node in self.customer_pool:
            self.removeNodeFromTour(node)
            self.customer_pool.remove(node)
    
    
    def removeNodeFromTour(self, node):
        if node.is_depot:
            raise ValueError('Cannot remove depot')
        tour = self._getTourHavingNode(node)
        if not tour is None:
            self._node_to_tour[node.index] = None
            tour.remove(node)
            if self.params.frozen_vehicles:
                pass
                #warnings.warn('Vehicles are frozen: cannot delete vehicle')
            else:
                if tour.isEmpty():
                    self.tour_dict.pop(tour.name)
                    del tour
    
    
    def _getTourHavingNode(self, node):
        tour_name = self._node_to_tour[node.index]
        if tour_name is None:
            return None
        else:
            return self.tour_dict[tour_name]
    
    def _createTour(self, name=None):
        tour = Tour('{}'.format(name), 
            self.ins, 
            self._tour_obj, 
            self._tour_obj_type,
            self.ignore_tw)

        return tour

    def createTourIntoSol(self, name=None):
        name = 0 if name is None else name
        while '{}'.format(name) in self.tour_dict:
            name += 1
        tour = self._createTour('{}'.format(name))
        self.tour_dict[tour.name] = tour

        return tour

    def _createNewTourByPathResult(self, path_result, name=None):
        # enfoce create
        new_tour = self.createTourIntoSol(name)
        
        new_tour._setPathFromResultList(path_result)
        for node in path_result[0]:
            self._node_to_tour[node.index] = new_tour.name
        
        return new_tour
        
    
    def _createNewTourByPath(self, path, name=None):
        # enfoce create
        name = 0 if name is None else name
        while '{}'.format(name) in self.tour_dict:
            name += 1
        new_tour = Tour('{}'.format(name), 
            self.ins, 
            self._tour_obj, 
            self._tour_obj_type, 
            self.ignore_tw)
        
       
        new_tour.setPath(path)
        
        self.tour_dict[new_tour.name] = new_tour
        for node in path:
            self._node_to_tour[node.index] = new_tour.name
        
        return new_tour
    
    def createNewTourByPath(self, path, name=None):
        if self.params.frozen_vehicles:
            raise Warning('Vehicles are frozen: cannot create new vehicle')
            return None
        
        return self._createNewTourByPath(path, name)
        
    
    def _deleteTourByName(self, tour_name):
        # enforce delete
        tour = self.tour_dict[tour_name]
        for node in tour:
            self._node_to_tour[node.index] = None
        tour.clear()
        self.tour_dict.pop(tour_name)
        
                
    def deleteTourByName(self, tour_name):
        if self.params.frozen_vehicles:
            raise Warning('Vehicles are frozen: cannot delete vehicle')
            return None
        
        self._deleteTourByName(tour_name)
        
    
    def insertIntoTourByName(self, tour_name, node, position = -1):
        # add $node into the tour 
        # new $node position: $position
        # default: add node at the end
        tour = self._node_to_tour[node.index]
        if tour != None:
            raise ValueError('the node is currently in the tour {}'.format(tour.name))
            
        # add node into the given tour
        tour = self.tour_dict[tour_name] if tour_name in self.tour_dict \
                                    else self.createTourIntoSol(tour_name)
                                
        tour.insert(position, node)
        self._node_to_tour[node.index] = tour.name
   
    
    def exportPathPlan(self):
        # each path is a list
        # export all paths for external usage or make copy
        path_plan = {}
        for name in self.tour_dict:
            path_plan[name] = self.tour_dict[name].path.copy()
        
        return path_plan
    
    
    def _exportPathResultPlan(self):
        path_result_plan = [tour._exportPath() for tour in self.tour_dict.values()]
        
        return path_result_plan
    
    
    def _setPathResultPlan(self, path_result_plan):
        
        if self.params.frozen_vehicles:
            assert(len(self.tour_dict) == len(path_result_plan))
        
        self.tour_dict = {}
        
        for plan in path_result_plan:
            self._createNewTourByPathResult(plan)
            
    
    def _getNodeListFromPath(self, path: list):
        self._pathIsValid(path)
        return [node for node in path if not node.is_depot and node.is_active]
        
    
    def _pathIsValid(self, path):
        index_set = set()
        for node in path:
            if node.index in index_set:
                raise IndexError("repeated node found")
            index_set.add(node.index)
            if node.index not in self.ins.node_dict:
                raise IndexError(f"Unable to find nodes {node.index} in the invoking instance")
            
    
    def setPathPlan(self, path_plan):
        # set the invoking path plan to be the input one
        self.clearAllTours()
        if self.params.frozen_vehicles and len(path_plan) != len(self.tour_dict):
            raise Warning('vehicles are frozen')
        
        path_plan = {p: self._getNodeListFromPath(path_plan[p]) for p in path_plan}
            
        self.tour_dict = {} # reset tour dict
        for name in path_plan:
            self._createNewTourByPath(path_plan[name], name)


    def setIdxPathPlan(self, idx_path_plan):
        path_plan = {}
        for i in idx_path_plan:
            path_plan[i] = [self.ins.node_dict[j] for j in idx_path_plan[i]]
        
        self.setPathPlan(path_plan)


    def exportSelectedEdges(self):
        # export a dictionary of edges
        # in which 1 denotes that edge is selected
        edge_dict = {i:0 for i in self.ins.arc_dict}
        for tour in self.tour_dict.values():
            edge_dict[self.ins.depot_start.index, tour[0].index] = 1
            for i in range(1, len(tour)):
                edge_dict[tour[i-1].index, tour[i].index] = 1
                
            edge_dict[tour[-1].index, self.ins.depot_end.index] = 1
        
        
        return edge_dict
    
    
    def tourIsValid(self, tour: Tour) -> bool:
        # check if tour is valid
        # return False if tour is INvalid
        # otherwise, return True
        
        if tour.worst_workload > self.ins.params.strict_max_workload:
            return False
        
        return True
    
    
    def _allToursAreValid(self)->bool:
        for i in self.tour_dict:
            t = self.tour_dict[i]
            if not self.tourIsValid(t):
                return False
        
        return True


    def solIsValid(self) -> bool:
        # check if the current solution is valid
        # make sure each tour is valid
        # make sure all customers are assigned to exactly 1 driver

        all_customer = {c.index : False for c in self.customer_pool}

        for i in self.tour_dict:
            t = self.tour_dict[i]
            if not self.tourIsValid(t):
                print(f"Tour {i} is invalid")
                return False
            for c in t:
                if all_customer[c.index]:
                    print(f"Duplicated customer {c.name}")
                    return False 
                
                all_customer[c.index] = True
        
        for c in all_customer:
            if not all_customer[c]:
                return False
        
        return True
    

    def getNodeVisitTime(self, mean=False) -> dict:
        for t in self.tour_dict.values():
            if not mean:
                t.getNodeVisitTime()
            else:
                t.getNodeVisitTime(self.ins._scenario, skip_tw=True)
        
        node_visit_dict = {idx: \
            self.tour_dict[self._node_to_tour[idx]].node_visit_time[idx]
            for idx in self._node_to_tour
        }

        return node_visit_dict
    

    def assignTimeWindow(self):
        mean_node_visit_time = self.getNodeVisitTime(True)
        for idx in mean_node_visit_time:
            arrival_time = mean_node_visit_time[idx]
            self.ins.node_dict[idx].setTimeWindow(
                max(0, arrival_time-1), arrival_time +1
            )

            
if __name__ == "__main__":
    ins = Instance(10)
    hm = HeurMethodBase(ins)
    hm.solve()
    print(hm)