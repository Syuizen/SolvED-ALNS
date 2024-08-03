# -*- coding: utf-8 -*-
"""
Created on Fri Mar 26 13:23:04 2021

@author: Yiran
"""
#---------------------
# project scripts
from services.vrp_src.heuristic.serialCompute import makeMyUpdateTW, makeAllSceUpdate
#---------------------

#---------------------
# packages
import copy
import numpy as np
import math
import random
import ast
import json
import networkx as nx
#---------------------

class Parameter:
    def __init__(self, name, value):
        assert(type(name)==str and name.isidentifier())
        # name must be a valid identifier
        
        self.__name = name
        self.__value = value
    
    
    def __str__(self):
        return "Parameter [{0}]: {1}".format(self.name, self.value)
        
    
    def __add__(self, v):
        return self.value + v
    
    
    def __radd__(self, v):
        return v + self.value
    
    
    def __sub__(self, v):
        return self.value - v
    
    
    def __rsub__(self, v):
        return v - self.value
    
    
    def __mul__(self, v):
        return self.value * v
    
    
    def __rmul__(self, v):
        return v * self.value
    
    
    def __truediv__(self, v):
        return self.value / v
    
    
    def __rtruediv__(self, v):
        return v / self.value
    
    
    def __neg__(self):
        return - self.value
    
    
    def __abs__(self):
        return abs(self.value)
    
    
    def __pos__(self):
        return + self.value
    
    
    def __pow__(self, n):
        return pow(self.value, n)
    
    
    def __bool__(self):
        return True if self.value else False
    
    
    def __lt__(self, v):
        return self.value < v
    
    
    def __le__(self, v):
        return self.value <= v
    
    
    def __eq__(self, v):
        return self.value == v
    
    
    def __ne__(self, v):
        return self.value != v
    
    
    def __ge__(self, v):
        return self.value >= v
    
    
    def __gt__(self, v):
        return self.value > v
    
    
    def __mod__(self, v):
        return self.value % v
    
    
    def __rmod__(self, v):
        return v % self.value
    
    
    @property
    def value(self):
        return self.__value
    
    
    @property
    def name(self):
        return self.__name
    
    
    def setValue(self, value):
        self.__value = value
        


class ParamSet:
    
    def __init__(self):
        self.__nameSet = set()
    
    
    def __str__(self):
        out_str = ''
        for name in self.__nameSet:
            out_str += str(getattr(self, name))
            out_str += '\n'
        
        return out_str
    
    
    def setOrAddParameter(self, name, value):
        if hasattr(self, name):
            param = getattr(self, name)
            param.setValue(value)
        else:
            self.addParameter(name, value)
    
    
    def addParameter(self, name, value):
        self.add(Parameter(name, value))
        self.__nameSet.add(name)
    
    
    def add(self, param: Parameter):
        assert(not hasattr(self, param.name))
        
        setattr(self, param.name, param)
        self.__nameSet.add(param.name)
    
    
    def remove(self, param: Parameter):
        if hasattr(self, param.name):
            delattr(self, param.name)
            


class Node:
    def __init__(self, x_loc, y_loc, name, index, is_depot=False):
        self.__name = name
        self.__x_loc = x_loc
        self.__y_loc = y_loc
        self.__index = index
        self.__is_active = True # active, switch to 0 if the order is cancelled
        self.__cancel_rate = 0
        self.__service_time = 0 # need to be initialized
        self.__time_window = []
        self.__is_depot = is_depot
        self.appointment_time = -1
    
    
    @property
    def __name__(self):
        return "Node {}".format(self.index)
    
    
    @property
    def time_window(self):
        return self.__time_window
    
    
    @property    
    def start_time(self):
        if self.__time_window:
            return self.__time_window[0]
        else:
            return 0.0
    
    
    @property
    def end_time(self):
        if self.__time_window:
            return self.__time_window[1]
        else:
            return 24.0
    
    
    @property
    def loc(self):
        return (self.__x_loc, self.__y_loc)
    
    
    @property
    def name(self):
        return self.__name

    
    @property
    def x_loc(self):
        return self.__x_loc
    
    
    @property
    def y_loc(self):
        return self.__y_loc
    
    
    @property
    def index(self):
        return self.__index
    
    
    @property
    def is_depot(self):
        return self.__is_depot
    
    
    @property
    def service_time(self):
        return self.__service_time
    
    
    @property
    def is_active(self):
        return self.__is_active
    
    
    @property
    def cancel_rate(self):
        return self.__cancel_rate
    
    
    def _clearTimeWindow(self):
        self.__time_window = ()
    
    
    def setTimeWindow(self, start_t, end_t):
        if not self.is_depot:
            self.__time_window = (start_t, end_t)
        
    
    def refineTimeWindow(self, visit_time, duration, alpha=0, beta=0):
        # get the best time window based on visit_time
        if self.time_window:
            # if has pre-assigned time window
            # then new time window must be a subset of the previous one
            #print("tw: index: ", self.index, self.time_window)
            if alpha+beta > 0:
                s_tw = visit_time - duration * (alpha/(alpha+beta))
            else:
                s_tw = visit_time - duration/2
            if s_tw < self.end_time:
                if s_tw < self.start_time:
                    s_tw = self.start_time
                    e_tw = min(s_tw+duration, self.end_time)
               
                elif s_tw + duration > self.end_time:
                    e_tw = self.end_time    
                    s_tw = max(e_tw-duration, self.start_time)
                else:
                    e_tw = s_tw + duration
            else:
                e_tw = self.end_time
                s_tw = max(e_tw-duration, self.start_time)
            self.setTimeWindow(s_tw, e_tw)
            #print('ea: ', visit_time, " tw: index: ", self.index, "[", s_tw, ", ", e_tw, " ]")
        else:
            # if no pre-assigned time window
            if alpha+beta > 0:
                s_tw = visit_time - duration * (alpha/(alpha+beta))
            else:
                s_tw = visit_time - duration/2
            self.setTimeWindow(s_tw, s_tw + duration)
    
    
    def getDelayTime(self, visit_time):
        # if arrive early, then return negative
        # if in time window, then return 0
        # if arrive late, then return positive
        if not self.time_window:
            return 0
        
        if visit_time < self.start_time:
            return visit_time - self.start_time
        
        if visit_time > self.end_time:
            return visit_time - self.end_time
        
        return 0
    
    
    def setServiceTime(self, t):
        if self.is_depot:
            self.__service_time = 0
        else:
            self.__service_time = t
    
    
    def distanceFrom(self, c):
        return math.sqrt(math.pow(self.x_loc-c.x_loc, 2)+ math.pow(self.y_loc-c.y_loc, 2))


    def setCancellationRate(self, r):
        self.__cancel_rate = r
    
    
    def resetOrderStatus(self):
        # Probability of keeping order is 1-r
        if self.is_depot: # depot node cannot be de-active
            self.__is_active = True
            return True
        if random.random() <= self.cancel_rate:
            self.__is_active = False
            return False
        else:
            self.__is_active = True
            return True
    
    
    def deActive(self):
        self.__is_active = False
    
    
    def active(self):
        self.__is_active = True
    
    
    def setActiveStatus(self, status):
        self.__is_active = status



class Arc:
    def __init__(self, from_node: Node, to_node: Node):
        self.__name = "{0}_arc_{1}".format(from_node.name, to_node.name)
        self.__length = from_node.distanceFrom(to_node)
        self.__velocity = None # need to be initialized
        self.__travel_time = None # need to be initialized
        self.__index = (from_node.index, to_node.index)
        self.__from_loc = (from_node.x_loc, from_node.y_loc)
        self.__to_loc = (to_node.x_loc, to_node.y_loc)
        self.__from_node = from_node
        self.__to_node = to_node
    
    
    @property
    def __name__(self):
        return "Arc {}".format(self.index)
    
    
    
    def setLength(self, l):
        if l < 0:
            raise ValueError('arc length must be non-negative')
        self.__length = l
    
    
    def setVelocity(self, v):
        if v <= 0:
            raise ValueError('velocity must be positive')
        self.__velocity = v
        self.__travel_time = self.__length/self.__velocity
    
    
    def setTravelTime(self, t):
        if t < 0:
            raise ValueError('travel time must be non-negative')
        self.__travel_time = t
        self.__velocity = self.__length/t if t > 0 else 1e6
    
    
    def orient(self, loc):
        # find the orientation of arc and input node
        # 0 : colinear
        # 1 : clockwise
        # 2 : counterclockwise
        val = (float(self.__from_loc[1] - self.__to_loc[1]) * (loc[0] - self.__from_loc[0])) -\
              (float(self.__from_loc[0] - self.__to_loc[0]) * (loc[1] - self.__from_loc[1]))
        if (val > 0):
            # Clockwise orientation
            return 1
        elif (val < 0):
            # Counterclockwise orientation
            return 2
        else:
            # Colinear orientation
            return 0
    
    
    def passThrough(self, loc):
        # if the arc passes through give point
        # note that arc is a segment not line
        # so, not passing != not colinear
        if self.orient(loc) != 0:
            return False

        if ( (loc[0] <= max(self.__from_loc[0], self.__to_loc[0])) and 
            (loc[0] >= min(self.__from_loc[0], self.__to_loc[0])) and 
           (loc[1] <= max(self.__from_loc[1], self.__to_loc[1])) and 
           (loc[1] >= min(self.__from_loc[1], self.__to_loc[1]))):
            
            return True
        
        return False
        
    
    def intersect(self, arc):
        # if the arc intersects with the input arc
        
        f_node, t_node = arc.arrow
        o1 = self.orient(f_node)
        o2 = self.orient(t_node)
        o3 = arc.orient(self.__from_loc)
        o4 = arc.orient(self.__to_loc)
        
        # General case
        if ((o1 != o2) and (o3 != o4)):
            return True
      
        # Special Cases
      
        # General case
        if ((o1 != o2) and (o3 != o4)):
            return True
      
        # Special Cases
      
        if ((o1 == 0) and self.passThrough(f_node)):
            return True
      
        if ((o2 == 0) and self.passThrough(t_node)):
            return True
      
        if ((o3 == 0) and arc.passThrough(self.__from_loc)):
            return True
      
        if ((o4 == 0) and arc.passThrough(self.__to_loc)):
            return True
      
        # If none of the cases
        return False
        
    
    @property
    def arrow(self):
        return (self.__from_loc, self.__to_loc)
    
    
    @property
    def length(self):
        return self.__length
    
    
    @property
    def velocity(self):
        return self.__velocity
    
    
    @property
    def travel_time(self):
        return self.__travel_time
    
    
    @property
    def index(self):
        return self.__index
    
    
    @property
    def name(self):
        return self.__name
    
    
    @property
    def from_node(self):
        return self.__from_node
    
    
    @property
    def to_node(self):
        return self.__to_node


class Instance:
    # a single instance of the model which contains the following parts
    # 1. customer list
    # 2. depot location
    # 3. parameters
    def __init__(self, nCustomer=0, default_setting=True, name='init'):
        self.__name = name
        self.has_depot = False
        self.__n_customer = nCustomer # number of customers
        self.customer_dict = {}
        self.setDepotLocation(0, 0)
        self._scenarioSet = set()
        self._scenario = None
        self.params = ParamSet()
        
        if nCustomer:
            self.initCustomerList(nCustomer)
        
        self.params = ParamSet()   
        
        # default initialization
        if default_setting:
            self.setting()
            self.build()
    
    
    @property
    def name(self):
        return self.__name
    
    
    @property
    def n_customer(self):
        return self.__n_customer
    
    
    @property
    def n_node(self):
        return self.__n_customer + (1 if self.has_depot else 0)


    def arc(self, i, j):
        try:
            return self.arc_dict[i,j]
        except KeyError:
            return None
    
    
    def _defaultStocParam(self):
        mean_service_time = random.randint(30, 60)
        standard_dev = 0.5*mean_service_time
        shape, scale = (mean_service_time/standard_dev)**2, standard_dev**2/mean_service_time
        
        #normalized
        unit_travel_time_generator = lambda: np.random.lognormal(0, 0.5, 1)[0]/60
        service_time_generator = lambda: np.random.gamma(shape, scale, 1)[0]/60
        
        self.setting(
            unit_travel_time_generator = unit_travel_time_generator,
            service_time_generator = service_time_generator
            )
        
    
    def formalizeData(self):
        # make data ready for solving
        # call this method only when all data have been finalized
            
        N = len(self.customer_dict) + 1
        full_arc_length = [0.0] * N * N
        for i in range(N):
            node_ind = self.depot_start.index if i==0 else i
            #full_node_start_t[i] = float(self.node_dict[node_ind].start_time)
            #full_node_end_t[i] = float(self.node_dict[node_ind].end_time)
            for j in range(N):
                ind = self.depot_end.index if j == 0 else j
                full_arc_length[i*N + j] = self.arc_dict[node_ind, ind].length if (node_ind, ind) in self.arc_dict else 0.0

        if self._scenario is None:
            self._scenario = self.getScenario()
        
        self._scenario.formalizeData(full_arc_length)
        for sce in self._scenarioSet:
            sce.formalizeData(full_arc_length)
        
        if self._scenarioSet:
            all_sce_arc_time = []
            all_sce_node_time = []
            all_sce_prob = []
            for sce in self._scenarioSet:
                sce_arc_time, sce_node_time = sce.exportFullArcNodeTime()
                all_sce_arc_time.append(sce_arc_time)
                all_sce_node_time.append(sce_node_time)
                all_sce_prob.append(sce.probability)

            all_sce_arc_time = np.array(all_sce_arc_time)
            all_sce_node_time = np.array(all_sce_node_time)
            all_sce_prob = np.array(all_sce_prob)
            full_arc_length = np.array(full_arc_length)

            self._allSceUpdate = makeAllSceUpdate(all_sce_node_time, 
                all_sce_arc_time, 
                full_arc_length, 
                all_sce_prob,
                self.params.max_workload.value,
                self.params.strict_max_workload.value
            )

            # enforce Complie
            self._allSceUpdate(np.array([0, 1, 2, 0]), 
                np.array([0.0, 0,0, 0.0, 0.0]), 
                np.array([0.0, 0,0, 0.0, 0.0]))
            

    

    def getNodesTimeWindow(self):
        full_node_start = np.array([self.node_dict[self.depot_start.index].start_time]+ \
                [self.node_dict[i].start_time for i in range(1, len(self.customer_dict)+1)]).astype(float)
        full_node_end = np.array([self.node_dict[self.depot_start.index].end_time]+ \
                [self.node_dict[i].end_time for i in range(1, len(self.customer_dict)+1)]).astype(float)

        return full_node_start, full_node_end

        
    def addScenario(self, sce):
        # add scenario into instance
        # this action will NOT change instance built-in scenario
        # instead, it makes algorithm able to solve over multi-scenarios
        self._scenarioSet.add(sce)

    
    def clearAllScenario(self):
        self._scenarioSet = set()
        
    
    def getScenario(self):
        # return scenario of current instance
        sce = Scenario(self)
        return sce
    
    
    def setScenario(self, sce):
        # change instance built-in scenario to be sce
        for arc in self.arc_dict.values():
            arc.setTravelTime(sce.travelTime(arc.index))
        
        for node in self.node_dict.values():
            node.setServiceTime(sce.serviceTime(node.index))
        
        self._scenario = self.getScenario()


    def rename(self, name):
        self.__name = name
        

    def initCustomerList(self, nCustomer):
        x_min = -25
        x_max = 25
        y_min = -25
        y_max = 25
        c_ind = 1
        for c in range(1, self.n_customer+1):
            name = 'c_{}'.format(c)
            new_c = Node(random.randint(x_min, x_max), random.randint(y_min, y_max), name, c_ind)
            self.customer_dict[c_ind] = new_c
            c_ind = c_ind + 1
    

    def build(self):
        self._initNodeDict()
        self._initArcDict()
        self._initParams()
        

    def getInactiveCustomerList(self):
        c_list = []
        for c in self.customer_dict.values():
            if not c.is_active:
                c_list.append(c)
        
        return c_list

    
    def setDepotLocation(self, x_loc, y_loc):
        self.depot_start = Node(x_loc, y_loc,'depot_start', '_s_', is_depot=True)
        self.depot_end = Node(x_loc, y_loc, 'depot_end', '_e_', is_depot=True)
        self.has_depot = True
        
    
    def addCustomer(self, x_loc, y_loc, name=None, index=None):
        c_ind = (1+self.__n_customer) if index is None else index
        name = 'c_{}'.format(c_ind) if name is None else name
        customer_node = Node(x_loc, y_loc, name, c_ind)
        self.customer_dict[c_ind] = customer_node
        self.__n_customer += 1
    
        return customer_node
    
    
    def isValid(self):
        if not self.has_depot:
            raise Exception('Please set depot location for this instance')
        
        index_set = set(self.node_dict.keys())
        if len(index_set) != len(self.node_dict):
            for ind in self.node_dict:
                print("node: {0}, index {1}".format(self.node_dict[ind].name, ind))
            raise IndexError('Error: Node indices are not distinct')
        
        return True
    
    
    def _initNodeDict(self):
        self.node_dict = {c: self.customer_dict[c] for c in self.customer_dict}
        self.node_dict[self.depot_start.index] = self.depot_start
        self.node_dict[self.depot_end.index] = self.depot_end
            
     
    def _returnConstFunc(self, const):
        return lambda : const    
    
    
    def _initArcDict(self):
        DEPOT_START_IND = self.depot_start.index
        DEPOT_END_IND = self.depot_end.index
        # for customer nodes, all possible pair of two different customers
        # should be a valid arc. For depots, however, DEPOT_START_IND is 
        # for outbound arcs only while DEPOT_END_IND is for inbound arcs
        # in particular, there is no arc connecting DEPOT_START_IND and 
        # DEPOT_END_IND directly
        self.arc_dict = {(i,j): Arc(self.node_dict[i], self.node_dict[j]) 
                         for i in self.node_dict.keys() for j in self.node_dict.keys()
                         if ((i != j and i != DEPOT_END_IND) and j != DEPOT_START_IND) and \
                             not (i == DEPOT_START_IND and j == DEPOT_END_IND)}
    
    def setting(self, 
                     cancel_rate = 0.05,
                     unit_travel_time_generator = lambda: np.exp(0.125)/60,
                     service_time_generator = lambda: 45/60,
                     max_workload = 8,
                     strict_max_workload = 10,
                     cost_wait_time = 5, # per hour
                     cost_idle_time = 2.5, # per hour
                     cost_work_overtime = 4, # per hour
                     cost_travel_per_km = 0,
                     cost_travel_time = 1, # per hour
                     cost_hire_per_team = 100,
                     service_time_mean = 45, # minute
                     ):
        
        strict_max_workload = max(max_workload, strict_max_workload)
        
        self.params.setOrAddParameter('max_workload', max_workload)
        self.params.setOrAddParameter('cancel_rate', cancel_rate)
        self.params.setOrAddParameter('strict_max_workload', strict_max_workload)
        self.params.setOrAddParameter('cost_wait', cost_wait_time)
        self.params.setOrAddParameter('cost_idle', cost_idle_time)
        self.params.setOrAddParameter('cost_overtime', cost_work_overtime)
        self.params.setOrAddParameter('cost_travel', cost_travel_per_km)
        self.params.setOrAddParameter('cost_hire', cost_hire_per_team)
        self.params.setOrAddParameter('cost_travel_time', cost_travel_time)
        self.params.setOrAddParameter('service_time_mean', service_time_mean)
        
        self.stoc_params_setting = {'r': cancel_rate,
            'v': unit_travel_time_generator,
            's': service_time_generator,
        }
        
        
    def _initParams(self):
        
        ### all values are fixed at their mean
                
        self.service_time_mean = self.params.service_time_mean.value 
        self.service_time_std = 0.5* self.service_time_mean
                
        lognormal_mean = 0 # this is the mean of underlying normal dist.
        lognormal_std = 0.5 # this is the std of underlying normal dist.
        
        self.unit_time_mean = np.exp(lognormal_mean + lognormal_std**2/2)
        self.unit_time_std = 1
        
        ### the below is designed to be kept
        
        cancel_rate = self.params.cancel_rate.value
        v_gen = self.stoc_params_setting['v']
        s_gen = self.stoc_params_setting['s']
        
        for c in self.node_dict.values():
            if not c.is_depot:
                c.setCancellationRate(cancel_rate)
        
        if not callable(s_gen):
            s_gen = self._returnConstFunc(s_gen)
        if not callable(v_gen):
            v_gen = self._returnConstFunc(v_gen)
        
        # create service time for each customer node
        for c_1 in self.node_dict.values():
            c_1.setServiceTime(s_gen())
        
        # create travel time for each arc (directed)
        for c_1, c_2 in self.arc_dict.keys():
            arc = self.arc_dict[c_1, c_2]
            self.arc_dict[c_1, c_2].setTravelTime(arc.length * v_gen())
    
        self._scenario = self.getScenario()        

            
    def resetStocParams(self, order_status=True, service_time=True, arc_velocity=True):
        # re-generate stochastic parameters randomly
        v_gen = self.stoc_params_setting['v']
        s_gen = self.stoc_params_setting['s']
        
        if order_status:
            for c in self.node_dict.values():
                _ = c.resetOrderStatus()
        if not callable(s_gen):
            s_gen = self._returnConstFunc(s_gen)
        if not callable(v_gen):
            v_gen = self._returnConstFunc(v_gen)
        
        if service_time:
            for c_1 in self.node_dict.values():
                c_1.setServiceTime(s_gen())
        
        if arc_velocity:
            for c_1, c_2 in self.arc_dict.keys():
                self.arc_dict[c_1, c_2].setVelocity(v_gen())
        
        self._scenario = self.getScenario()
         
    
    def getServiceTeamBound(self, delta = 0.1):
        rank_dict = {}
        for i in self.customer_dict:
            rank_dict[i] = [j for j in self.customer_dict if (i,j) in self.arc_dict]
            rank_dict[i].sort(key=lambda x: self.arc_dict[i, x].length)
        min_travel_time = sum([self.arc_dict[i, rank_dict[i][0]].travel_time for i in self.customer_dict])
        max_travel_time = sum([self.arc_dict[i, rank_dict[i][-1]].travel_time for i in self.customer_dict])
        total_service_time = sum([self.customer_dict[i].service_time for i in self.customer_dict])
        
        lb = (total_service_time+min_travel_time)/(self.params.max_workload * (1+delta))
        ub = (total_service_time+max_travel_time)/(self.params.max_workload * (1-delta))
        lb = math.ceil(lb)
        ub = math.ceil(ub)
        
        return lb, ub
    
    
    def getActiveNodeIndexList(self) -> list:
        node_list = [i.index for i in self.node_dict.values() if i.is_active]
        return node_list
    
    
    def getNode(self, ind) -> Node:
        return self.node_dict[ind]
        
    
    def getActiveArcPairIndexList(self) -> tuple:
        arc_pair = [(i, j) for i,j in self.arc_dict.keys() if self.getNode(i).is_active and self.getNode(j).is_active]
        if (self.depot_start.index, self.depot_end.index) in arc_pair:
            arc_pair.remove((self.depot_start.index, self.depot_end.index))
        return arc_pair
    
    
    def getProbability(self):
        # return the probability of base scenario
        return self._scenario.probability
    
    
    def exportParams(self):
        # export all parameters and order status as a dict-type record
        return copy.deepcopy(self.params)
    
    
    def exportJSON(self, file_name):
        # export instance to JSON file
        data = {}
        data['depot'] = [self.depot_start.loc]
        data['customer'] = {}
        for c in self.customer_dict.values():
            data['customer'][c.index]= {'service_time': c.service_time, 
                                                     'status': int(c.is_active),
                                                     'loc': c.loc}
        
        data['arc'] = {}
        for arc in self.arc_dict.values():
            data['arc'][str(arc.index)] = {'length': arc.length, 
                                                   'travel_time': arc.travel_time}
        
        with open(file_name, 'w') as outfile:
            json.dump(data, outfile)
        
    
    def readJSON(self, file_name):
        # read instance from JSON file
        with open(file_name) as infile:
            data = json.load(infile)
            depot_loc = data['depot'][0]
            self.setDepotLocation(depot_loc[0], depot_loc[1])
            for c in data['customer']:
                c_data = data['customer'][c]
                c_loc = c_data['loc']
                c_node = self.addCustomer(c_loc[0], c_loc[1], index=int(c))
                c_node.setActiveStatus(c_data['status'])
                c_node.setServiceTime(c_data['service_time'])
            
            self.build()
            for arc in self.arc_dict.values():
                arc.setTravelTime(data['arc']["{}".format(arc.index)]['travel_time'])
                arc.setLength(data['arc']["{}".format(arc.index)]['length'])
        
    
    def _benchmarkBuild(self, n_scenario=20, n_sample=2000, tour_dict=None, seed=None):
        # this is a method for benckmarking only 
        self.clearAllScenario()
        mean_service_time = 45 #fixed
        standard_dev = 0.5*mean_service_time
        shape, scale = (mean_service_time/standard_dev)**2, standard_dev**2/mean_service_time
        n_sample = int(n_sample)
        if tour_dict is None:
            arc_index = [i for i in self.arc_dict]
        else:
            arc_index = []
            for tour in tour_dict.values():
                arc_index += [arc.index for arc in tour.genArc()]
        cust_index = [i.index for i in self.customer_dict.values()]
        
        #normalized
        rng = np.random.default_rng(seed)
        X_1 = rng.lognormal(0, 0.5, (n_sample, len(arc_index)))
        X_2 = rng.gamma(shape, scale, (n_sample, len(cust_index)))
        
        X = np.concatenate((X_1, X_2), axis=1)
        
        if n_scenario < n_sample:
            from sklearn.cluster import KMeans
            kmeans = KMeans(n_clusters=n_scenario, random_state=seed)
            kmeans.fit(X)
        
            sample_count = {i: 0 for i in range(n_scenario)}
            
            for i in range(n_sample):
                sample_count[kmeans.labels_[i]] += 1
            
            sample_iter = 0
            for center in kmeans.cluster_centers_:
                sce = Scenario(self) # create scenario
                sce.setProbability(sample_count[sample_iter]/n_sample)
                for n_iter in range(len(center)):
                    val = float(center[n_iter])
                    if n_iter < len(arc_index):
                        sce.setTravelTime(arc_index[n_iter], val/60*self.arc_dict[arc_index[n_iter]].length)
                        
                    else:
                        r_ind = n_iter - len(arc_index)
                        sce.setServiceTime(cust_index[r_ind], val/60)

                sce.enforceTriOnTravelTime()
                sce.update()
                self.addScenario(sce)
                sample_iter += 1
        
        else:
            for row in X:
                sce = Scenario(self) # create scenario
                sce.setProbability(1/n_sample)
                for n_iter in range(n_scenario):
                    val = float(row[n_iter])
                    if n_iter < len(arc_index):
                        sce.setTravelTime(arc_index[n_iter], val/60*self.arc_dict[arc_index[n_iter]].length)
                        
                    else:
                        r_ind = n_iter - len(arc_index)
                        sce.setServiceTime(cust_index[r_ind], val/60)
            
                self.addScenario(sce)


    def _getRoutePlanFromEdges(self, selected_edges):
        '''
        Retrieve the route plan from selected_edges
        
        selected_edges is a set of edges that are chosen
        Please be aware that selected_edges must be a valid service plan
        i.e. each customer should be visited exactly once
        With infeasible service plan, the function behavior may be erroneous 
        '''
        
        route_start_customers = [j for (i,j) in selected_edges if i == self.depot_start.index]
        route_plan = {}
        route_id = 0
        for s_c in route_start_customers:
            single_route = [self.depot_start.index, s_c]
            # iteratively search for next
            current_node = s_c
            while current_node != self.depot_end.index:
                next_node = [j for (i,j) in selected_edges if i == current_node]
                current_node = next_node[0] # next_node should be length 1
                single_route.append(current_node)
            
            route_plan[route_id] = single_route
            route_id += 1
        
        return route_plan        

    
    def _getMeanArrivalTime(self, selected_edges):
        '''
        Return a dictionary contains mean arrival time of service team at each customer
        Output dictionary has customer index as key and arrival time as value
        
        selected_edges is a set of edges that are chosen
        Please be aware that selected_edges must be a valid service plan
        i.e. each customer should be visited exactly once
        With infeasible service plan, the function behavior may be erroneous 
        '''

        route_plan = self._getRoutePlanFromEdges(selected_edges)
        arrival_time_dict = {i: None for i in self.customer_dict}
        for route in route_plan.values():
            current_node = route[1] # first customer node
            arrival_time = self.arc_dict[self.depot_start.index, current_node].length * self.unit_time_mean
            arrival_time_dict[current_node] = arrival_time/60
            for c in route[2:]:
                if c != self.depot_end.index:
                    # new customer node
                    arrival_time += self.service_time_mean +\
                                    self.arc_dict[current_node, c].length * self.unit_time_mean
                    arrival_time_dict[c] = arrival_time/60
                    current_node = c
        
        return arrival_time_dict


    def exportToNX(self):
        '''
        export instance customers as networkx graph
        the nodes are customers and depots
        the weights are travel times
        '''
        
        G = nx.DiGraph()
        for arc in self.arc_dict.values():
            start_ind = 'depot' if arc.from_node.is_depot else arc.from_node.index
            end_ind = 'depot' if arc.to_node.is_depot else arc.to_node.index
            G.add_edge(start_ind, end_ind, weight=arc.travel_time)
            
        return G
    

    def enforceTriOnTravelTime(self):
        '''
        enforce travel times of arcs satisfy triangular inequality
        the instance should be a fully-connected graph
        this algorithm replace the travel time between two nodes A and B by the shortest travel time in the graph
        '''
        
        nx_G = self.exportToNX()
        shortest_path_dict = dict(nx.all_pairs_dijkstra_path_length(nx_G))
        for start_ind in shortest_path_dict:
            for end_ind in shortest_path_dict[start_ind]:
                if end_ind == start_ind:
                    continue
                s_idx = '_s_' if start_ind == 'depot' else start_ind
                e_idx = '_e_' if end_ind == 'depot' else end_ind
                self.arc_dict[(s_idx, e_idx)].setTravelTime(shortest_path_dict[start_ind][end_ind]) 


class Scenario:
    
    def __init__(self, ins: Instance=None):
        '''
        all indices are "index"
        there are two reserved node indices '_s_' and '_e_'
        both indices are representing depot node, however
        '_s_' means the depot_start
        '_e_' means the depot_end
        '''
        
        if ins:
            self.__arc_travel_time_dict = {arc.index: arc.travel_time for arc in ins.arc_dict.values()}
            self.__node_service_time_dict = {node.index: node.service_time for node in ins.node_dict.values()}
        else:
            self.__arc_travel_time_dict = {}
            self.__node_service_time_dict = {'_s_': 0, '_e_': 0}
            
        self.__probability = 1
        self.update()
    
    def travelTime(self, arc_index):
        return self.__arc_travel_time_dict[arc_index]
    
    
    def serviceTime(self, node_index):
        return self.__node_service_time_dict[node_index]
    
    
    @property
    def probability(self):
        return self.__probability
    
    @property
    def travelTimeLB(self):
        return self.__travelTimeLB


    def setProbability(self, prob):
        assert(prob >= 0)
        self.__probability = prob
    
    
    def setTravelTime(self, arc_index, travel_time):
        assert(travel_time >= 0)
        self.__arc_travel_time_dict[arc_index] = travel_time
    
    
    def setServiceTime(self, node_index, service_time):
        assert(service_time >= 0)
        self.__node_service_time_dict[node_index] = service_time
        
    
    def setTravelTimeFromList(self, arc_index_list, travel_time_list):
        assert(len(arc_index_list) == len(travel_time_list))
        for i in range(arc_index_list):
            arc_index = arc_index_list[i]
            travel_time = travel_time_list[i]
            self.setTravelTime(arc_index, float(travel_time))
        
    
    def setServiceTimeFromList(self, node_index_list, service_time_list):
        assert(len(node_index_list) == len(service_time_list))
        for i in range(len(node_index_list)):
            node_index = node_index_list[i]
            service_time = service_time_list[i]
            self.setServiceTime(node_index, float(service_time))
    

    def exportFullArcNodeTime(self):
        N = len(self.__node_service_time_dict) - 1
        full_arc_time = [0.0] * N * N
        full_node_time = [0.0] * N
        for i in range(N):
            node_ind = "_s_" if i==0 else i
            full_node_time[i] = float(self.serviceTime(node_ind))
            for j in range(N):
                ind = "_e_" if j == 0 else j
                full_arc_time[i*N + j] = self.travelTime((node_ind, ind)) if (node_ind, ind) in self.__arc_travel_time_dict else 0.0
            
        return full_arc_time, full_node_time
    

    def formalizeData(self, full_arc_length):
        # formalize data into list format
        # and make a C-function for updating tour stat
                    
        #for i in range(N):
        #    print("index: ", i, ' start_t: ', full_node_start_t[i], ' end_t: ', full_node_end_t[i])
        
        #  full_arc_length = [0.0] * N * N
        full_arc_time, full_node_time = self.exportFullArcNodeTime()
        
        self._numba_update = makeMyUpdateTW( 
                    np.array(full_node_time), 
                    np.array(full_arc_time), 
                    np.array(full_arc_length))
       
        # enforce compile
        self._numba_update(np.array([0,1,0]), np.array([0.0,2.0,3.0]), np.array([1.0,3.0,4.0]))
    
    
    def readJSON(self, file_name):
        with open(file_name) as infile:
            data = json.load(infile)
            for c in data['customer']:
                c_data = data['customer'][c]
                self.setServiceTime(c, c_data['service_time'])
            
            for arc in data['arc']:
                self.setTravelTime(ast.literal_eval(arc), data['arc'][arc]['travel_time'])


    def update(self):
        '''
        Return min possible travel cost (Lower bound)
        '''
        sorted_time = sorted([t for t in self.__arc_travel_time_dict.values()])
        self.__travelTimeLB = sum(sorted_time[:len(self.__node_service_time_dict)])


    def getMaxWorkload(self, route, appointed, time_window_width):
        '''
        Return the maximum workload
        '''
        
        current_node = route[1]
        workload = self.travelTime(('_s_', current_node))
        for next_node in route[2:]:
            if current_node != '_e_':
                appointed_start = appointed[current_node] - time_window_width/2
                workload = workload if workload > appointed_start else appointed_start
                workload += self.serviceTime(current_node)
            workload += self.travelTime((current_node, next_node))
            current_node = next_node
        
        return workload


    def exportToNX(self):
        # export instance customers as networkx graph
        # the nodes are customers and depots
        # the weights are travel times
        G = nx.DiGraph()
        for arc_index in self.__arc_travel_time_dict:
            travel_time = self.__arc_travel_time_dict[arc_index]
            start_ind = 'depot' if arc_index[0] == '_s_' else arc_index[0]
            end_ind = 'depot' if arc_index[1] == '_e_' else arc_index[1]
            G.add_edge(start_ind, end_ind, weight=travel_time)
            
        return G
    

    def enforceTriOnTravelTime(self):
        # enforce travel times of arcs satisfy triangular inequality
        # the instance should be a fully-connected graph
        # this algorithm replace the travel time between two nodes A and B by the shortest travel time in the graph
        nx_G = self.exportToNX()
        shortest_path_dict = dict(nx.all_pairs_dijkstra_path_length(nx_G))
        for start_ind in shortest_path_dict:
            for end_ind in shortest_path_dict[start_ind]:
                s_idx = '_s_' if start_ind == 'depot' else start_ind
                e_idx = '_e_' if start_ind == 'depot' else end_ind
                if s_idx != e_idx:
                    self.setTravelTime((s_idx, e_idx), shortest_path_dict[start_ind][end_ind]) 


class Tour:

    ### System Methods
    
    """
    Methods Usage: 
        tour = Tour(name, ins)     -> create a Tour object 
        for c_node in tour         -> iterate over all customer nodes
        len(tour)                  -> get the number of customer nodes
        tour[i]                    -> access customer node at position i
        tour.append(i)             -> append customer i at the end
        tour.insert(p, i)          -> insert customer i at position $p
        tour.remove(i)             -> remove customer i
        tour.swap(i, j)            -> swap customer i and j
        tour.length                -> total distance [different from len(tour)]
        tour.workload              -> total workload (time)
        print(tour)                -> string representation of tour
        tour.optimize('two_opt')   -> use two-opt to optimize tour
        tour.clear()               -> clear all customer nodes
        if i in tour               -> check if tour contains i
    """
    
    def __init__(self, name, ins: Instance, obj=None, obj_type='simple', ignore_tw=False):
        self.name = name
        self.path = []
        self.node_position = {} # map: node.index -> tour_position 
        self.arc_dict = ins.arc_dict
        self.depot_start = ins.depot_start
        self.depot_end = ins.depot_end
        self.length = 0 # total trasversal distance
        self.workload = 0 # total time needed to complete work
        self.service_time = 0 # total service time needed
        self.travel_time = 0 # total trasversal time needed
        self.start_time = 0 # time to start working 
        self.worst_workload = 0 # work load under the worst scenario
        #-- start-time is auto-detected, we expect vehicle arrive at its first customer at exactly appointment time
        #-- with time-window available, the appointment time will be the mid of time window
        self.node_visit_time = {self.depot_start.index: self.start_time} # visit time of each node in tour
        self.node_idle_time = {self.depot_start.index: 0}
        self.node_wait_time = {self.depot_start.index: 0}
        self.worker_idle_time = 0
        self.customer_wait_time = 0
        self.ins = ins
        self.ignore_tw = ignore_tw
        
        # reserved parameters
        self._OBJ_TYPE_SIMPLE = 1
        self._OBJ_TYPE_EXPECTED = 2
        self._OBJ_TYPE_WORST = 3 # NOT USED

        # define default obj
        if obj is None:
            self.setObjective(self.defaultObjective)
            self.setObjType(obj_type)
        else:
            self.setObjective(obj, obj_type)


    def __len__(self):
        return len(self.path)


    def __getitem__(self, ind):
        # get node at position ind
        return self.path[ind]


    def __contains__(self, node):
        return node in self.path


    def __iter__(self):
        return self.path.__iter__()


    def __str__(self):
        tour_str = "Tour {0}: ({1})->".format(self.name, self.depot_start.name)
        for node in self.path:
            tour_str += "({})->".format(node.name)
        tour_str += "({})".format(self.depot_end.name)

        return tour_str


    ### Non-system Methods        

    @property
    def end_time(self):
        last_node = self.path[-1]
        return self.node_visit_time[last_node.index]+\
                last_node.service_time +\
                self.arc_dict[last_node.index, self.depot_end.index].travel_time


    @property
    def score(self):
        return self.__base_objective(self)


    def distance(self, node: Node):
        return min(i.distanceFrom(node) for i in self.path)


    def setObjType(self, obj_type):
        assert(obj_type in ['simple', 'expected', 'worst'])
        
        if obj_type == 'simple':
            self.obj_type = self._OBJ_TYPE_SIMPLE
        
        if obj_type == 'expected':
            assert(len(self.ins._scenarioSet) > 0)
            self.obj_type = self._OBJ_TYPE_EXPECTED
            
    
    def setObjective(self, obj, obj_type='simple'):
        # input obj should ba a callable function with a single argument "tour"
        # refer to default obj as an example
        # obj should not modify any attribute of tour
        assert(callable(obj))
        
        self.__base_objective = obj
        self.setObjType(obj_type)
        
    
    def defaultObjective(self, tour):
        
        return  tour.ins.params.cost_travel.value * tour.length + \
                tour.ins.params.cost_travel_time.value * tour.travel_time +\
                tour.ins.params.cost_idle.value * tour.worker_idle_time + \
                tour.ins.params.cost_overtime.value * tour.overtime + \
                tour.ins.params.cost_wait.value * tour.customer_wait_time + \
                tour.ins.params.cost_hire.value
    
    
    def genArc(self, start_pos=-1, skip_end=False):
        #arc generator
        if start_pos < len(self.path) and len(self.path) > 0:
            current = self.depot_start if start_pos < 0 else self.path[start_pos]
            
            for node in self.path[start_pos+1:]:
                next_node = node
                yield self.arc_dict[current.index, next_node.index]
                current = next_node
            
            if not skip_end:
                yield self.arc_dict[current.index, self.depot_end.index]        
        
        
    def _arcLength(self, left_ind, right_ind):
        return self.arc_dict[left_ind, right_ind].length
    
    
    def _arcTravelTime(self, left_ind, right_ind):
        return self.arc_dict[left_ind, right_ind].travel_time
        
    
    def _prevNode(self, customer):
        position = self.node_position[customer.index]
        if position:
            prev_node = self.path[position-1]
        else:
            prev_node = self.depot_start
        
        return prev_node
    
    
    def _nextNode(self, customer):
        position = self.node_position[customer.index]
        if position < len(self.path)-1:
            next_node = self.path[position+1]
        else:
            next_node = self.depot_end
        
        return next_node
    
    
    def isEmpty(self):
        return False if self.path else True
    
    
    def reverse(self, node_1, node_2, ignore_update=False):
        # reverse segment between node_1 and node_2        
        is_adjecent, is_1_before_2, pos_1, pos_2 = self._is_adjecent_pair(node_1, node_2)
        
        if not is_1_before_2:
            pos_1, pos_2 = pos_2, pos_1

        self.path[pos_1:pos_2+1] = self.path[pos_1:pos_2+1][::-1]
        
        for i in range(pos_1, pos_2+1):
            node =  self.path[i]
            self.node_position[node.index] = i
        
        if not ignore_update:
            self.update()
    
    
    def replace(self, node, new_node, ignore_update=False):
        # replace node by new_node
                
        pos = self.node_position.pop(node.index)
        self.path[pos] = new_node
        self.node_position[new_node.index] = pos
        if not ignore_update:
            self.update()

    
    def _getNodeTimeWindow(self):
        if self.ignore_tw:
            self.getNodeVisitTime(self.ins._scenario, skip_tw=True)
            full_node_start = np.zeros(len(self.ins.customer_dict)+1, dtype=np.float64)
            full_node_end = np.zeros(len(self.ins.customer_dict)+1, dtype=np.float64)
            for i in self.path:
                idx = i.index
                full_node_start[idx] = self.node_visit_time[idx] - 1
                full_node_end[idx] = self.node_visit_time[idx] + 1
        else:
            full_node_start, full_node_end = self.ins.getNodesTimeWindow()
        
        return full_node_start, full_node_end

    def _detailUpdate(self, sce=None):
        # slow
        if self.isEmpty():
            return
        
        if sce is None:
            sce = self.ins._scenario
            
        # recompute tour parameters: length & workload & time
        length = 0
        worker_idle_time = 0
        total_travel_time = 0
        total_service_time = 0
        self.start_time = sum(self.path[0].time_window)/2 - self._arcTravelTime(self.depot_start.index, self.path[0].index)
        arrival_time = self.start_time
        customer_wait_time = 0
        arrival_time_table = {self.depot_start.index: self.start_time}
        customer_wait_table = {self.depot_start.index: 0}
        worker_idle_table = {self.depot_start.index: 0}
        for arc in self.genArc():
            # leave to next node
            length += arc.length
            arrival_time += sce.travelTime(arc.index)
            total_travel_time += sce.travelTime(arc.index)
            time_gap = arc.to_node.getDelayTime(arrival_time)
            if time_gap > 0:
                customer_wait_time += time_gap
                customer_wait_table[arc.to_node.index] = time_gap 
                worker_idle_table[arc.to_node.index] = 0
            else:
                worker_idle_time -= time_gap
                customer_wait_table[arc.to_node.index] = 0 
                worker_idle_table[arc.to_node.index] = -time_gap
            
            arrival_time_table[arc.to_node.index] = arrival_time
            
            # wait till start service time
            arrival_time += abs(time_gap) if time_gap < 0 else 0
            # serve at node
            arrival_time += sce.serviceTime(arc.to_node.index)
            total_service_time += sce.serviceTime(arc.to_node.index)
            # ready to move to next node
        
        self.length = length
        self.worker_idle_time = worker_idle_time
        self.workload = arrival_time - self.start_time
        self.customer_wait_time = customer_wait_time
        self.node_visit_time = arrival_time_table
        self.node_wait_time = customer_wait_table
        self.node_idle_time = worker_idle_table
        self.travel_time = total_travel_time
        self.service_time = total_service_time

    def _evalScenario_ui(self, sce: Scenario):
        # evaluate scenario by __base_objective
        on_time_service = {}
     
        self._detailUpdate(sce)
        self.getNodeVisitTime(sce)
        
        for c in self.path:
            time_gap = c.getDelayTime(self.node_visit_time[c.index])
            on_time_service[c.index] = 1 if time_gap == 0 else 0
        
        result = {
            'cost': self.__base_objective(self),
            'travel_distance': self.length,
            'workload': self.workload,
            'idle_time': self.worker_idle_time,
            'wait_time': self.customer_wait_time,
            'end_time': self.end_time,
            'arrival_time': self.node_visit_time.copy(),
            'service_status': on_time_service,
            }
    
        return result

    def _evalScenario(self, sce):
        path = [i.index for i in self.path]
        
        full_node_start, full_node_end = self._getNodeTimeWindow()

        (length, 
        travel_time,
        service_time,
        workload,
        worker_idle_time,
        customer_wait_time,
        start_time) = sce._numba_update(np.array(path), np.array(full_node_start), np.array(full_node_end))
        
        overtime = max(0, workload - self.ins.params.max_workload)

        return (length, 
            travel_time, 
            service_time, 
            workload, 
            worker_idle_time, 
            customer_wait_time,
            start_time,
            overtime)

    
    def update(self, sce=None):
        self.clearStat()
        if (len(self.path) > 0):
            if self.obj_type == self._OBJ_TYPE_SIMPLE or sce is not None:
                
                if sce is None:
                    sce = self.ins._scenario

                (self.length, 
                self.travel_time,
                self.service_time,
                self.workload,
                self.worker_idle_time,
                self.customer_wait_time,
                self.start_time,
                self.overtime) = self._evalScenario(sce)

                self.worst_workload = self.workload

            elif self.obj_type == self._OBJ_TYPE_EXPECTED:
                
                assert(len(self.ins._scenarioSet) > 0)

                path = np.array([i.index for i in self.path])
                full_node_start, full_node_end = self._getNodeTimeWindow()

                (self.length, 
                self.travel_time,
                self.service_time,
                self.workload,
                self.worker_idle_time,
                self.customer_wait_time,
                self.start_time,
                self.overtime,
                self.worst_workload,
                self.infeas) = self.ins._allSceUpdate(path, 
                    np.array(full_node_start), 
                    np.array(full_node_end))  
    
    
    def append(self, node: Node):
        
        self.insert(len(self), node)
        

    def insert(self, position, node: Node, ignore_update=False):
        # add $node into the tour 
        # new $node position: $position
        # if position is larger than total number of nodes in the tour, then add new node at the end
                
        if self.isEmpty():
            self.start_time = sum(node.time_window)/2 - self._arcTravelTime(self.depot_start.index, node.index)
            self.node_visit_time[self.depot_start.index] = self.start_time
        
        position = len(self) if position < 0 or position > len(self) else position
        self.node_position[node.index] = position
        self.path.insert(position, node)

        for t_node in self.path[position+1:]:
            self.node_position[t_node.index] += 1
        
        if not ignore_update:
            self.update()
    
    
    def swap(self, node_1: Node, node_2: Node, ignore_update=False):
        # swap positions of node_1 and node_2
        
        self.node_position[node_1.index], self.node_position[node_2.index] = self.node_position[node_2.index], self.node_position[node_1.index]
        
        self.path[self.node_position[node_1.index]] = node_1
        self.path[self.node_position[node_2.index]] = node_2
        
        if not ignore_update:
            self.update() # update statistical properties of tour
        
        
    def remove(self, node: Node, ignore_update=False):
        # remove node from the tour
        if not node.index in self.node_position:
            raise Warning('This node is not in the tour, no need to remove')
            return
                
        position = self.node_position[node.index]
        self.path.pop(position)
        self.node_position.pop(node.index)
        for node in self.path[position:]:
            self.node_position[node.index] -= 1
        
        if self.isEmpty():
            self.start_time = None
        
        if not ignore_update:
            self.update()
        
        return position
    
    
    def removalGain(self, node: Node):
        
        score_before = self.score
        pos = self.remove(node)
        score_after = self.score
        gain = score_before - score_after
        self.insert(pos, node)
        
        return gain

    
    def addGain(self, position, node: Node):
        
        score_before = self.score
        self.insert(position, node)
        score_after = self.score
        new_workload = self.workload
        gain = score_before - score_after
        self.remove(node)
        
        return gain, new_workload
    

    def exportStat(self):
        
        return (self.length,
                self.workload,
                self.start_time,
                self.travel_time,
                self.service_time,
                self.worker_idle_time,
                self.customer_wait_time,
                self.worst_workload,
                self.overtime)
    
    
    def setStat(self, stat):
                
        (self.length,
        self.workload,
        self.start_time,
        self.travel_time,
        self.service_time,
        self.worker_idle_time,
        self.customer_wait_time,
        self.worst_workload,
        self.overtime) = stat


    def clearStat(self):
        self.length = 0 # total trasversal distance
        self.workload = 0 # total time needed to complete work
        self.service_time = 0 # total service time needed
        self.travel_time = 0 # total trasversal time needed
        self.start_time = 0 # time to start working 
        self.worker_idle_time = 0
        self.customer_wait_time = 0
        self.worst_workload = 0
        self.overtime = 0


    def clear(self):
        # clear all nodes and reset tour to empty
        #while not self.isEmpty():
        #    self.remove(self[0]) # remove the first node
        
        self.path = []
        self.node_position = {}
        self.clearStat()
    
    
    def getNodeVisitTime(self, sce=None, skip_tw=False):
        self.node_visit_time = {self.depot_start.index: self.start_time,
            self.depot_end.index: 0,
        }
        
        for i in self.path:
            self.node_visit_time[i.index] = 0
        
        if sce is None and self.obj_type == self._OBJ_TYPE_EXPECTED:
        
            for sce in self.ins._scenarioSet:
                arrival_time = self.start_time
            
                for arc in self.genArc():
                    arrival_time += sce.travelTime(arc.index)
                    self.node_visit_time[arc.to_node.index] += arrival_time * sce.probability

                    if not skip_tw:
                        time_gap = arc.to_node.getDelayTime(arrival_time)
                    else:
                        time_gap = 0

                    if time_gap < 0:
                        arrival_time += abs(time_gap)
                    
                    arrival_time += sce.serviceTime(arc.to_node.index)
            
        elif sce is not None or self.obj_type == self._OBJ_TYPE_SIMPLE:
            if sce is None:
                sce = self.ins._scenario
            
            arrival_time = self.start_time
            
            for arc in self.genArc():
                arrival_time += sce.travelTime(arc.index)
                self.node_visit_time[arc.to_node.index] = arrival_time
            
                if not skip_tw:
                    time_gap = arc.to_node.getDelayTime(arrival_time)
                else:
                    time_gap = 0
            
                if time_gap < 0:
                    arrival_time += abs(time_gap)
                
                arrival_time += sce.serviceTime(arc.to_node.index)
    

    def setIdxPath(self, idx_path: list):
        path = [self.ins.node_dict[idx] for idx in idx_path]

        self.setPath(path)
        
    
    def setPath(self, path: list):
        # set the given path as tour path
        self.clear()
                
        self.path = path.copy()
        while self.depot_start in self.path:
            self.path.remove(self.depot_start)
        while self.depot_end in self.path:
            self.path.remove(self.depot_end)
        
        for ind, node in enumerate(self.path):
            self.node_position[node.index] = ind
            
        self.update()
        
    
    def _exportPath(self):
        # export path & statistics
        
        result = [self.path.copy(),
                  self.length,
                  self.workload,
                  self.start_time,
                  self.travel_time,
                  self.service_time,
                  self.worker_idle_time,
                  self.customer_wait_time,
                  self.worst_workload,
                  self.overtime]
        
        return result
        
    
    def _setPathFromResultList(self, result):
        self._setPath(path              =result[0], 
                      length            =result[1], 
                      workload          =result[2],
                      start_time        =result[3],
                      travel_time       =result[4],
                      service_time      =result[5],
                      worker_idle_time  =result[6],
                      customer_wait_time=result[7],
                      worst_workload    =result[8],
                      overtime          =result[9])
    
    
    def _setPath(self, path,
                 length,
                 workload,
                 start_time,
                 travel_time,
                 service_time,
                 worker_idle_time,
                 customer_wait_time,
                 worst_workload,
                 overtime):
        
        # this is to set path without updating
        # all stat data should be provided
                
        self.path = path
        self.length = length
        self.workload = workload
        self.start_time = start_time
        self.travel_time = travel_time
        self.service_time = service_time
        self.worker_idle_time = worker_idle_time
        self.customer_wait_time = customer_wait_time
        self.worst_workload = worst_workload
        self.overtime = overtime
        self.node_position = {node.index: i for i, node in enumerate(path)}
        
    
    def overlap(self, tour):
        # check if the tour overlaps with the given tour
        for arc in self.genArc(0):
            if arc.to_node.index == self.depot_end.index:
                continue
            for input_arc in tour.genArc(0):
                if input_arc.to_node.index == self.depot_end.index:
                    continue
                if arc.intersect(input_arc):
                    return True
        
        return False
    
    
    def splitPathIntoHalf(self, by=0):
        # split the path into two paths
        # by: 0 - number of nodes
        # by: 1 - length (distance)
        # by: 2 - workload
        
        assert(by in [0, 1, 2])
        
        if self.isEmpty():
            return [], []    
            
        if by == 0:
            tour_length = len(self)
            sub_path_1 = self.path[:round(tour_length/2)]
            sub_path_2 = self.path[round(tour_length/2):]
        
        if by == 1:
            # obtain two sub-paths with almost-same lengths
            half_tour_length = self.length/2
            cut_ind = 0
            sub_path_length = 0
            while sub_path_length < half_tour_length and cut_ind < len(self)-1:
                sub_path_length += self._arcLength(self.path[cut_ind].index, self.path[cut_ind+1].index)
                cut_ind += 1
            
            sub_path_1 = self.path[:cut_ind]
            sub_path_2 = self.path[cut_ind:]
        
        if by == 2:
            # split by workload
            half_tour_workload = self.workload/2
            cut_ind = 1
            sub_path_workload = 0
            arrival_time = self.start_time
            for arc in self.genArc():
                arrival_time += arc.travel_time
                time_gap = arc.to_node.getDelayTime(arrival_time)
                # leaving time
                arrival_time +=  arc.to_node.service_time -\
                                    (time_gap if time_gap < 0 else 0)
                
                sub_path_workload = arrival_time + self._arcTravelTime(arc.to_node.index, self.depot_end.index)
                                    
                if sub_path_workload > half_tour_workload:
                    break
                else:
                    cut_ind += 1
            
            cut_ind = min(cut_ind, len(self.path) - 1)
            
            sub_path_1 = self.path[:cut_ind]
            sub_path_2 = self.path[cut_ind:]
        
        return sub_path_1, sub_path_2
        
    
    def _is_adjecent_pair(self, node_1, node_2):
        pos_1 = self.node_position[node_1.index]
        pos_2 = self.node_position[node_2.index]
        position_diff =  pos_1 - pos_2 
        is_adjecent = True if abs(position_diff) == 1 else False
        is_1_before_2 = True if position_diff < 0 else False
        return is_adjecent, is_1_before_2, pos_1, pos_2
    
    
    ### Heuristic Tour Optimizer Definition
    
    def optimize(self, tool, max_iter=1e4, log=False):
        if tool == 'two_opt':
            if log:
                print('run two-opt method')
            self._twoOpt(max_iter, log)


    def _moveHeur(self, max_iter =1e3):
        # optimized ordering by moving nodes
        is_improved = True
        iter_count = 0
        n_nodes = len(self.path)
        while is_improved and iter_count < max_iter:
            is_improved = False
            iter_count += 1
            temp_path = self.path.copy()
            for i in temp_path:
                pos = self.node_position[i.index]
                result_stat = self.exportStat()
                for j in range(pos+1, n_nodes):
                    # move node i to position j
                    accept = False
                    score_before = self.score
                    self.remove(i, True)
                    self.insert(j, i)
                    score_after = self.score
                    
                    if score_before > score_after:
                        accept = True
                
                    if not accept:                
                        self.remove(i, True)
                        self.insert(pos, i, True)
                        self.setStat(result_stat)


    def _twoSwap(self, max_iter = 1e3):
        #optimized ordering by swaping nodes
        is_improved = True
        iter_count = 0
        node_pair_pool = [(i, j) for i in self.path for j in self.path if i.index < j.index]
        while is_improved and iter_count < max_iter:
            is_improved = False
            iter_count += 1
            for pair in node_pair_pool:
                result_stat = self.exportStat()
                score_before = self.score
                self.swap(pair[0], pair[1])
                score_after = self.score
                accept = False
            
                if score_before > score_after:
                    accept = True
                
                if not accept:                
                    self.swap(pair[0], pair[1], True)
                    self.setStat(result_stat)

    
    def _twoOpt(self, max_iter = 1e3, log=False, distance_only=False):
        # optimize ordering by reversing segments
        is_improved = True
        iter_count = 0
        node_pair_pool = [(i, j) for i in self.path for j in self.path if i.index < j.index]
        while is_improved and iter_count < max_iter:
            is_improved = False
            iter_count += 1
            for pair in node_pair_pool:
                result_stat = self.exportStat()
                score_before = self.score
                distance_before = self.length

                self.reverse(pair[0], pair[1])
                
                score_after = self.score
                distance_after = self.length
                score = score_before - score_after
                accept = False
                if distance_only:
                    if distance_after < distance_before:
                        accept=True
                else:
                    if score_before > score_after:
                        accept = True
                
                if accept:                
                    if log:
                        print("Optimizer-Two Opt: swap nodes {0} and {1} with score: {2}".format(
                            pair[0].name, pair[1].name, score))
                    is_improved = True
                else:
                    self.reverse(pair[0], pair[1], True)
                    self.setStat(result_stat)
        
    
    def setTimeWindowToPathNode(self, duration, refine=False):
        self.getNodeVisitTime()
        for c in self.path:
           # print('idle: ', self.ins.params.cost_idle.value, 'wait: ', self.ins.params.cost_wait.value)
            e_arrival_time = self.node_visit_time[c.index]
            if refine:
                c.refineTimeWindow(e_arrival_time, duration, self.ins.params.cost_idle, self.ins.params.cost_wait)
            else:
                diff = duration/2 if (self.ins.params.cost_idle+self.ins.params.cost_wait == 0) else\
                                    duration * (self.ins.params.cost_idle/(self.ins.params.cost_idle + self.ins.params.cost_wait))
                st = max(self.start_time, e_arrival_time-diff)
                c.setTimeWindow(st, st + duration)
                #print("check: ", e_arrival_time, self.start_time, c.time_window)
    
    def _benckmark(self):
        assert(len(self.ins._scenarioSet)>0)
        
        result = {
                'workload': 0,
                'end_time': 0,
                'arrival_time': {i.index: 0 for i in self.path},
                'service_status': {i.index: 0 for i in self.path},
            }
        
        for sce in self.ins._scenarioSet:
            sce_result = self._evalScenario_ui(sce)
            result['workload'] += sce_result['workload'] * sce.probability
            result['end_time'] += sce_result['end_time'] * sce.probability
            for c in self.path:
                result['arrival_time'][c.index] += sce_result['arrival_time'][c.index] * sce.probability
                result['service_status'][c.index] += sce_result['service_status'][c.index] * sce.probability
        
        return result
    ### Tour end
            
    
if __name__ == '__main__':
    # for testing methods defined in this script
    ins = Instance(10)
    