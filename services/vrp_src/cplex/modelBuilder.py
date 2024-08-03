# -*- coding: utf-8 -*-
"""
Created on Fri Feb 19 16:22:15 2021

@author: Yiran

Code standard:
    Class object name: ThisIsTheCodeStandard
    Class (public) method name: thisIsTheCodeStandard
    Class (public) variable name: this_is_the_code_standard
    Class (private) method name: _thisIsTheCodeStandard
    class (private) variable name: _this_is_the_code_standard
"""

"""
This script is used to build Cplex VRP Model
"""


#---------------------
# project scripts
from services.vrp_src.modelComponents import Instance, Tour, Scenario
from services.vrp_src.heuristic.alns import ALNS
#---------------------


#--------------------------------------
# packages
from docplex.mp import model as cpx
import matplotlib.pyplot as plt
import numpy as np
import random
import time
#---------------------------------------

class ModelBuilder:
    
    def __init__(self, instance: Instance=None):
        instance.isValid()
        self.resetParams()

        if instance is not None:
            self.loadInstance(instance)
    
            
    def buildVRP(self, n_team=None, time_window_width=0, time_window_given=False, uniform_start_time=None, ignore_fleet_size_cost=False):
        self.clearModel()
        if time_window_given:
            assert(uniform_start_time is not None)
        self.uniform_start_time = uniform_start_time
        self.time_window_given = time_window_given
        self.time_window_width = time_window_width
        self.ignore_fleet_size_cost = ignore_fleet_size_cost
        self._initModel(n_team)
        self.addScenarioIntoModel(self.ins.getScenario())
    
    
    def buildStocVRP(self, n_team=None, 
            time_window_width=0, 
            time_window_given=False, 
            uniform_start_time=None, 
            ignore_fleet_size_cost=False,
            using_lifted_formulation=False,
            target_sce = None,
            ):
        assert(len(self.ins._scenarioSet) > 0)
        self.clearModel()

        if time_window_given:
            assert(uniform_start_time is not None)
        self.uniform_start_time = uniform_start_time
        self.time_window_given = time_window_given
        self.time_window_width = time_window_width
        self.ignore_fleet_size_cost = ignore_fleet_size_cost
        self._initModel(n_team)
        
        self.using_lifted_formulation = using_lifted_formulation

        if target_sce:
            self.addScenarioIntoModel(target_sce)
        else:
            for sce in self.ins._scenarioSet:
                self.addScenarioIntoModel(sce)
    

    def buildMyBenderVRP(self,
                        n_team=None,
                        ignore_fleet_size_cost=False,
                        using_customer_order_var=False,
                        binary_subproblem_cut=True):
        assert(len(self.ins._scenarioSet) > 0)
        self.binary_subproblem_cut = binary_subproblem_cut
        self.clearModel() #clear model if exists
        self.ignore_fleet_size_cost = ignore_fleet_size_cost
        self._initModel(n_team)
        if not self.binary_subproblem_cut:
            # no binary subproblem cut
            # add travel cost to the master problem
            for sce in self.ins._scenarioSet:
                self.addScenarioIntoModel(sce, travel_cost_only=True)

        self._addMyBenderVars()
        self._appointedAtMeanArrival()
        if using_customer_order_var:
            self._addCustomerOrderVar()

    def resetParams(self):
        self.using_lifted_formulation = False
        self.ignore_fleet_size_cost = False
        self._n_scenario = 0
        self._usingAppointedAtMeanArrival = False
        self.use_multi_opt_cut = False
        self.use_my_bender = False
        self._sce_dict = {}
    

    def clearModel(self):
        if hasattr(self, 'model'):
            del self.model_vars
            del self.obj_expr
            del self.model
        
        if hasattr(self, 'solution'):
            del self.solution
        
        self.resetParams()

    
    def loadInstance(self, instance):
        self.ins = instance
        self._DEPOT_START_IND = self.ins.depot_start.index # first node
        self._DEPOT_END_IND = self.ins.depot_end.index # last node
        self.customer_dict = instance.customer_dict
        self.arc_dict = instance.arc_dict
        
    
    def _allocateVarToExistedNode(self, instance: Instance):
        active_node_list = instance.getActiveNodeIndexList()
        active_arc_pair_list = instance.getActiveArcPairIndexList()
        var_dict = {}
        var_dict['x'] = {
                z: self.model_vars['x'][z] for z in active_arc_pair_list 
             }
        
        cust_list = active_node_list.copy()
        cust_list.remove(instance.depot_start.index)
        cust_list.remove(instance.depot_end.index) 
        var_dict['t'] = {
                z: self.model_vars['t'][z] for z in cust_list
            }
        
        return var_dict
 

    def _addCustomerOrderVar(self):
        assert(hasattr(self, "model"))
        
        arc_pair =  self.ins.getActiveArcPairIndexList()
        node_list = self.ins.getActiveNodeIndexList()
        cust_list = [i for i in node_list if i != self._DEPOT_START_IND and i != self._DEPOT_END_IND]
        cust_arc_pair = [(i, j) for (i, j) in arc_pair if i in cust_list and j in cust_list]

        x = self.model_vars['x']
        y = self.model.binary_var_dict(cust_arc_pair, name="y")
        self.model_vars["y"] = y

        # add constraints
        self.model.add_constraints_(
            (y[i, j] + y[j, k] - 1 <= y[i, k]
                for i in cust_list for j in cust_list for k in cust_list if i != j and j != k and i != k),
            names=[ f"order_y_{i}_{j}_{k}_lb" 
                for i in cust_list for j in cust_list for k in cust_list if i != j and j != k and i != k]
        )

        self.model.add_constraints_(
            (x[i, j] <= y[i, j]
                for i in cust_list for j in cust_list if i!=j),
            names=[f"consecutive_y_{i}_{j}"
                for i in cust_list for j in cust_list if i!=j]
        )

        self.model.add_constraints_(
            (y[i, k] <= (1 - x[j, k] + y[i, j]) + x[i, k]
                for i in cust_list for j in cust_list for k in cust_list if i != j and j != k and i != k),
            names = [ f"order_y_{i}_{j}_{k}_ub"
                for i in cust_list for j in cust_list for k in cust_list if i != j and j != k and i != k]
        )

        self.model.add_constraints_(
            (y[i, j] + y[j, i] <= 1
                for i in cust_list for j in cust_list if i > j),
            names = [f"asym_y_{i}_{j}"
                for i in cust_list for j in cust_list if i > j]
        )

        self.model.add_constraints_(
            (y[i, j] <= 1 - x[i, self._DEPOT_END_IND]
                for i in cust_list for j in cust_list if i != j),
            names = [f"last_y_{i}_{j}"
                for i in cust_list for j in cust_list if i != j]
        )

        self.model.add_constraints_(
            (y[j, i] <= 1 - x[self._DEPOT_START_IND, i]
                for i in cust_list for j in cust_list if i != j),
            names = [f"first_y_{j}_{i}"
                for i in cust_list for j in cust_list if i != j]
        )

        if self._n_scenario > 0:
            # add scenario cuts
            for sce_id in range(1, self._n_scenario+1):
                a = self.model_vars[f"a_{sce_id}"]
                h = self.model_vars[f"h_{sce_id}"]
                sce = self._sce_dict[sce_id]
                self.model.add_constraints_(
                    (   a[j] +  h[j] + sce.serviceTime(j) + sce.travelTime((j, i)) - self.ins.params.strict_max_workload.value * (1 - y[j, i])
                    <=
                        a[i] for i in cust_list
                        for j in cust_list
                        if i != j
                    ),
                    names = ['define_a_lb({})'.format(sce_id) + str(i) + ',' + str(j) for i in cust_list
                        for j in cust_list
                        if i != j]
                    )                

    

    def _initModel(self, n_team = None, ):
        assert(not hasattr(self, 'model'))
        self.model = cpx.Model('vrp')
        
        if n_team is None:
            # create contraints
            #team_lb = min(1, len(self.customer_dict))
            #team_ub = len(self.customer_dict)
            team_lb, team_ub = self.ins.getServiceTeamBound()
        else:
            team_lb, team_ub = n_team, n_team
                
        #callback
        self.model._cb_tour_lb, self.model._cb_tour_ub = team_lb, team_ub
        self.model._cb_heur_sol = None
        
        instance = self.ins
        node_list = self.ins.getActiveNodeIndexList()
        cust_list = [i for i in node_list if i != self._DEPOT_START_IND and i != self._DEPOT_END_IND]
        #node_list = list(self.ins.node_dict.keys())
        
        # define variables
        arc_pair =  self.ins.getActiveArcPairIndexList()
                
        x = self.model.binary_var_dict(arc_pair, name="x")        # arc
        t = self.model.continuous_var_dict(cust_list, name="t", lb=0, ub=24)   # appointment_time
        self.model_vars = {'x': x, 't': t}
    

        self._nteam_ub = self.model.add_constraint(
            self.model.sum(x[i,j] for i, j in x if i==self._DEPOT_START_IND) <= team_ub,
            ctname = 'max_num_of_routes_({})'.format(instance.name)
        )
        
        self._nteam_lb = self.model.add_constraint(
            self.model.sum(x[i,j] for i, j in x if i==self._DEPOT_START_IND) >= team_lb, 
            ctname = 'min_num_of_routes_({})'.format(instance.name)
            )
        
        self.model.add_constraints_(
            (self.model.sum(x[k,j] for k, j in x if k==i) == 1 for i in node_list 
                             if not instance.node_dict[i].is_depot), 
            names = ['1-out_({})'.format(instance.name) + str(i) for i in node_list if not instance.node_dict[i].is_depot]
        )
                
        self.model.add_constraints_(
            (self.model.sum(x[k,j] for k, j in x if k==i) == self.model.sum(x[k,j] for k, j in x if j==i) for i in node_list
                             if not instance.node_dict[i].is_depot),
            names = ['1-in-1-out_({})'.format(instance.name)+ str(i) for i in node_list if not instance.node_dict[i].is_depot]
        )
         
        
        self.model.add_constraint_(
            self.model.sum(x[i,j] for i, j in x if i==self._DEPOT_START_IND)
            == 
            self.model.sum(x[i,j] for i, j in x if j==self._DEPOT_END_IND), 
            ctname ='out_equal_in_({})'.format(instance.name)
            )
             
        self.obj_expr = 0
        
        if not self.ignore_fleet_size_cost:
            self.obj_expr += \
                            self.ins.params.cost_hire.value * self.model.sum( x[i, j] for i, j in x if j==self._DEPOT_END_IND)
       
        self.model.minimize(self.obj_expr)
    

    def _addMyBenderVars(self, multi_opt_cut=True):
        assert(hasattr(self, "model"))
        assert('theta' not in self.model_vars)
        self.use_my_bender = True
        self.use_multi_opt_cut = multi_opt_cut
        if not multi_opt_cut:
            theta = self.model.continuous_var_dict(['theta'], lb=0)
            self.model_vars['theta'] = theta
            self.obj_expr += theta['theta']
        else:
            sce_theta = ['theta_' + str(i) for i in range(len(self.ins._scenarioSet))]
            theta = self.model.continuous_var_dict(sce_theta, lb=0)
            self.sce_theta_pair = []
            theta_idx = 0
            for sce in self.ins._scenarioSet:
                self.sce_theta_pair.append((theta['theta_' + str(theta_idx)], sce))
                theta_idx += 1
            self.model_vars['theta'] = theta
            self.obj_expr += self.model.sum(theta)

        self.model.minimize(self.obj_expr)


    def _appointedAtMeanArrival(self):
        # make appointed time to be the mean arrival time
        assert(hasattr(self, "model"))
        self._usingAppointedAtMeanArrival = True
        node_list = self.ins.getActiveNodeIndexList()
        cust_list = [i for i in node_list if i != self._DEPOT_START_IND and i != self._DEPOT_END_IND]
        var_dict = self._allocateVarToExistedNode(self.ins)

        x = var_dict['x']   # arc
        t = var_dict['t']   # appointment_time
        sce = self.ins.getScenario()

        self.model.add_constraints_(
            (   t[j] + sce.serviceTime(j) + sce.travelTime((j, i)) - self.ins.params.strict_max_workload.value * (1 - x[j, i])
             <=
                 t[i] for i in cust_list
                 for j in cust_list
                 if i != j
             ),
            names = ['define_t_lb(mean)' + str(i) + ',' + str(j) for i in cust_list
                 for j in cust_list
                 if i != j]
            )

        self.model.add_constraints_(
            (   t[j] + sce.serviceTime(j) + sce.travelTime((j, i)) + self.ins.params.strict_max_workload.value * (1 - x[j, i])
             >=
                 t[i] for i in cust_list
                 for j in cust_list
                 if i != j
             ),
            names = ['define_t_ub(mean)' + str(i) + ',' + str(j) for i in cust_list
                 for j in cust_list
                 if i != j]
            )
        
        self.model.add_constraints_(
          (    t[i] 
            >= 
                sce.travelTime((self._DEPOT_START_IND, i)) - self.ins.params.strict_max_workload.value * (1 - x[self._DEPOT_START_IND, i])
                for i in cust_list
          ),
          names = ['first_cust_t_lb(mean)' +str(i) for i in cust_list]
        )
        
        self.model.add_constraints_(
          (    t[i] 
            <= 
                sce.travelTime((self._DEPOT_START_IND, i)) + self.ins.params.strict_max_workload.value * (1 - x[self._DEPOT_START_IND, i])
                for i in cust_list
          ),
          names = ['first_cust_a_ub(mean)'+str(i) for i in cust_list]
        )

    
    def searchBestNTeams(self, relaxFleetSizeBounding=False):
        team_lb, team_ub = self.ins.getServiceTeamBound()
        self.buildVRP()
        log = []
        if relaxFleetSizeBounding:
            team_lb = max(1, team_lb - 2)
        for n in range(team_lb, team_ub+1):
            self._nteam_lb.set_right_expr(n)
            self._nteam_ub.set_right_expr(n)
            val = self.getRootNodeObjVal()
            if val > 0:
                log.append((n, val))
        
        return log
        
    
    def getRootNodeObjVal(self):
        default_n_nodes = self.model.parameters.mip.limits.nodes.value
        self.model.parameters.mip.limits.nodes = 0
        sol = self.model.solve(log_output=True)
        if sol is not None:
            val = self.model.objective_value
        else:
            val = -1
        self.model.parameters.mip.limits.nodes = default_n_nodes
        return val
        
    
    def addScenarioIntoModel(self, sce: Scenario, travel_cost_only=False):
        self._n_scenario += 1
        sce_id = self._n_scenario
        self._sce_dict[sce_id] = sce
        node_list = self.ins.getActiveNodeIndexList()
        cust_list = node_list.copy()
        cust_list.remove(self._DEPOT_START_IND)
        cust_list.remove(self._DEPOT_END_IND)
        var_dict = self._allocateVarToExistedNode(self.ins)

        x = var_dict['x']   # arc
        t = var_dict['t']   # appointment_time

        workload_gap = self.ins.params.strict_max_workload.value - self.ins.params.max_workload.value

        if not travel_cost_only:
            a = self.model.continuous_var_dict(node_list, name="a_{}".format(sce_id), lb=0)   # arrival_time
            g = self.model.continuous_var_dict(cust_list, name="g_{}".format(sce_id), lb=0, ub=workload_gap)   # overwork_time
            h = self.model.continuous_var_dict(cust_list, name="h_{}".format(sce_id), lb=0)   # idle_time
            w = self.model.continuous_var_dict(cust_list, name="w_{}".format(sce_id), lb=0)   # wait_time
            
            self.model_vars["h_{}".format(sce_id)] = h
            self.model_vars["w_{}".format(sce_id)] = w
            self.model_vars["a_{}".format(sce_id)] = a
            self.model_vars["g_{}".format(sce_id)] = g
            
            # create contraints    


            if self.using_lifted_formulation:

                ax = self.model.continuous_var_dict(list(x.keys()), name="ax_{}".format(sce_id), lb=0) # a*x
                hx = self.model.continuous_var_dict(list(x.keys()), name="hx_{}".format(sce_id), lb=0) # h*x
                # mcc
                self.model.add_constraints_(
                    ( ax[i, j] >= a[i] - self.ins.params.strict_max_workload.value * (1 - x[i, j])
                        for i, j in ax if i in a
                    ),
                    names=[f"mcc_ax_lb_{i,j}_{sce_id}" for i, j in ax]
                )

                self.model.add_constraints_(
                    ( ax[i, j] <= self.ins.params.strict_max_workload.value * x[i, j]
                        for i, j in ax
                    ),
                    names=[f"mcc_ax_ub_{i,j}_{sce_id}" for i, j in ax]
                )

                self.model.add_constraints_(
                    ( ax[i, j] <= a[i] + self.ins.params.strict_max_workload.value * (1- x[i, j])
                        for i, j in ax if i in a
                    ),
                    names=[f"mcc_ax_ub2_{i,j}_{sce_id}" for i, j in ax]
                )

                self.model.add_constraints_(
                    ( hx[i, j] <= self.ins.params.strict_max_workload.value * x[i, j]
                        for i, j in hx
                    ),
                    names=[f"mcc_hx_ub_{i,j}_{sce_id}" for i, j in hx]
                )

                self.model.add_constraints_(
                    ( hx[i, j] >= h[i] - self.ins.params.strict_max_workload.value * (1 - x[i, j])
                        for i, j in hx if i in h
                    ),
                    names=[f"mcc_hx_lb_{i,j}_{sce_id}" for i, j in ax]
                )

                self.model.add_constraints_(
                    ( hx[i, j] <= h[i] + self.ins.params.strict_max_workload.value * (1 - x[i, j])
                        for i, j in hx if i in h
                    ),
                    names=[f"mcc_hx_ub2_{i,j}_{sce_id}" for i, j in ax]
                )


                self.model.add_constraints_(
                    (a[i] >=
                        self.model.sum(
                            ax[j, i] + hx[j, i] + sce.serviceTime(j)*x[j, i] + sce.travelTime((j, i))*x[j, i]
                            for j in cust_list
                            if i != j
                        )
                    for i in cust_list),
                    names = [f"lifted_{sce_id}_{i}" for  i in cust_list]
                )


            
            self.model.add_constraints_(
                (   a[j] + h[j] + sce.serviceTime(j) + sce.travelTime((j, i)) + self.ins.params.strict_max_workload.value * (1 - x[j, i])
                >=
                    a[i] for i in cust_list
                    for j in cust_list
                    if i != j
                ),
                names = ['define_a_ub({})'.format(sce_id) + str(i) + ',' + str(j) for i in cust_list
                    for j in cust_list
                    if i != j]
                )
            
            self.model.add_constraints_(
                (   a[j] +  h[j] + sce.serviceTime(j) + sce.travelTime((j, i)) - self.ins.params.strict_max_workload.value * (1 - x[j, i])
                <=
                    a[i] for i in cust_list
                    for j in cust_list
                    if i != j
                ),
                names = ['define_a_lb({})'.format(sce_id) + str(i) + ',' + str(j) for i in cust_list
                    for j in cust_list
                    if i != j]
                )
            
            self.model.add_constraint_(
                a[self._DEPOT_START_IND] == 0, 
                ctname = 'start_time_is_0_({})'.format(sce_id)
                )
            
            self.model.add_constraints_(
            (    a[i] 
                >= 
                    sce.travelTime((self._DEPOT_START_IND, i)) - self.ins.params.strict_max_workload.value * (1 - x[self._DEPOT_START_IND, i])
                    for i in cust_list
            ),
            names = ['first_cust_a_lb({})'.format(sce_id)+str(i) for i in cust_list]
            )
            
            self.model.add_constraints_(
            (    a[i] 
                <= 
                    sce.travelTime((self._DEPOT_START_IND, i)) + self.ins.params.strict_max_workload.value * (1 - x[self._DEPOT_START_IND, i])
                    for i in cust_list
            ),
            names = ['first_cust_a_ub({})'.format(sce_id)+str(i) for i in cust_list]
            )
            
            self.model.add_constraints_(
                (   g[i] 
                >= 
                    ((a[i] + h[i] + sce.serviceTime(i) + sce.travelTime((i, self._DEPOT_END_IND))
                        - self.ins.params.max_workload.value) - 2*workload_gap*(1-x[i, self._DEPOT_END_IND]))
                for i in cust_list), 
                names = ['define_g_({})'.format(sce_id) + str(i) for i in cust_list]
                ) 
            
            if self.time_window_given:
                self.model.add_constraints_(
                    ((h[i] >= (self.ins.node_dict[i].start_time -self.uniform_start_time) - a[i]) for i in cust_list
                    ), 
                    names = ['twg_define_h_({})'.format(sce_id) + str(i) for i in cust_list]
                )
                
                self.model.add_constraints_(
                    ((w[i] >= a[i] - (self.ins.node_dict[i].end_time-self.uniform_start_time)) for i in cust_list
                    ), 
                    names = ['twg_define_w ({})'.format(sce_id) + str(i) for i in cust_list] 
                )
            
            # if given time_window width, then
            if self.time_window_width:
                self.model.add_constraints_(
                        ((h[i] >= t[i] - (self.time_window_width/2) - a[i]) for i in cust_list
                        ), 
                        names = ['tww_define_h_({})'.format(sce_id) + str(i) for i in cust_list]
                    )
                    
                self.model.add_constraints_(
                    ((w[i] >= a[i] - t[i] - (self.time_window_width/2)) for i in cust_list
                    ), 
                    names = ['tww_define_w ({})'.format(sce_id) + str(i) for i in cust_list] 
                )
            
            if not self.time_window_given and not self.time_window_width:
                self.model.add_constraints_(
                    (h[i] == 0 for i in cust_list
                    ), 
                    names = ['define_h_({})'.format(sce_id) + str(i) for i in cust_list]
                )
                
                self.model.add_constraints_(
                    (w[i] == 0 for i in cust_list
                    ), 
                    names = ['define_w ({})'.format(sce_id) + str(i) for i in cust_list] 
                )
        
        
        distance_time_dict = {ind: sce.travelTime(ind) for ind in x}


        if not travel_cost_only:
            self.obj_expr += sce.probability * ( 
                            self.ins.params.cost_wait.value * self.model.sum(w) +\
                            self.ins.params.cost_idle.value * self.model.sum(h) + \
                            self.ins.params.cost_overtime.value * self.model.sum(g) +\
                            self.ins.params.cost_travel_time.value * self.model.sum(x[i, j]* distance_time_dict[i, j] for i, j in x)
                            )
        else:
            self.obj_expr += sce.probability * ( 
                            self.ins.params.cost_travel_time.value * self.model.sum(x[i, j]* distance_time_dict[i, j] for i, j in x)
                            )
            
            
        self.model.minimize(self.obj_expr)


    def getScenarioLP(self, sce: Scenario, x_sol, t_sol, time_window_width=2):
        # this method returns a scenario subproblem by using given x_sol and t_sol
        model = cpx.Model('sce_sublp')
        node_list = self.ins.getActiveNodeIndexList()
        cust_list = node_list.copy()
        cust_list.remove(self._DEPOT_START_IND)
        cust_list.remove(self._DEPOT_END_IND)

        x = x_sol   # arc
        t = t_sol   # appointment_time

        workload_gap = self.ins.params.strict_max_workload.value - self.ins.params.max_workload.value

        a = model.continuous_var_dict(node_list, name="a", lb=0)   # arrival_time
        g = model.continuous_var_dict(cust_list, name="g", lb=0, ub=workload_gap)   # overwork_time
        h = model.continuous_var_dict(cust_list, name="h", lb=0)   # idle_time
        w = model.continuous_var_dict(cust_list, name="w", lb=0)   # wait_time
        
        model_vars = {}
        model_vars["h"] = h
        model_vars["w"] = w
        model_vars["a"] = a
        model_vars["g"] = g
        
        # create contraints    
        model_cstrs = {}
        model_cts_const = {}
        model_cts_coeff = {}
        for idx in x_sol:
            model_cts_coeff[f"x_{idx}"] = {}
        for idx in t_sol:
            model_cts_coeff[f"t_{idx}"] = {}
        
        # constraint set 1
        
        model_cstrs['define_a_ub'] = \
            model.add_constraints(
                (   a[j] + h[j] + sce.serviceTime(j) + sce.travelTime((j, i)) + self.ins.params.strict_max_workload.value * (1 - x[j, i])
                 >=
                     a[i] for i in cust_list
                     for j in cust_list
                     if i != j
                 ),
                names = ['define_a_ub' + str(i) + ',' + str(j) for i in cust_list
                     for j in cust_list
                     if i != j]
                )
            
        model_cts_const['define_a_ub'] = \
            [-(sce.serviceTime(j) + sce.travelTime((j, i)) + self.ins.params.strict_max_workload.value) for i in cust_list
                     for j in cust_list
                     if i != j]
        
        full_coef = [0 for i in cust_list
                     for j in cust_list
                     if i != j]
        
        for idx in x_sol:
            model_cts_coeff[f"x_{idx}"]['define_a_ub'] = full_coef.copy()
            coef_count = 0
            for i in cust_list:
                for j in cust_list:
                    if i!=j:
                        if (i, j) == idx:
                            model_cts_coeff[f"x_{idx}"]['define_a_ub'][coef_count] = - self.ins.params.strict_max_workload.value 
                        coef_count += 1
                        
        for idx in t_sol:
            model_cts_coeff[f"t_{idx}"]['define_a_ub'] = full_coef.copy()
        
        
        # constraint set 2
        
        model_cstrs['define_a_lb'] = \
            model.add_constraints(
                (   a[j] +  h[j] + sce.serviceTime(j) + sce.travelTime((j, i)) - self.ins.params.strict_max_workload.value * (1 - x[j, i])
                 <=
                     a[i] for i in cust_list
                     for j in cust_list
                     if i != j
                 ),
                names = ['define_a_lb' + str(i) + ',' + str(j) for i in cust_list
                     for j in cust_list
                     if i != j]
                )
        
        model_cts_const['define_a_lb'] = \
            [-(sce.serviceTime(j) + sce.travelTime((j, i)) - self.ins.params.strict_max_workload.value) for i in cust_list
                     for j in cust_list
                     if i != j]
        
        full_coef = [0 for i in cust_list
                     for j in cust_list
                     if i != j]
        
        for idx in x_sol:
            model_cts_coeff[f"x_{idx}"]['define_a_lb'] = full_coef.copy()
            coef_count = 0
            for i in cust_list:
                for j in cust_list:
                    if i!=j:
                        if (i, j) == idx:
                            model_cts_coeff[f"x_{idx}"]['define_a_lb'][coef_count] = self.ins.params.strict_max_workload.value 
                        coef_count += 1
                        
        for idx in t_sol:
            model_cts_coeff[f"t_{idx}"]['define_a_lb'] = full_coef.copy()
        
        
        # constraint set 3
        
        model_cstrs['start_time_is_0'] = \
            [model.add_constraint(
                a[self._DEPOT_START_IND] == 0, 
                ctname = 'start_time_is_0'
                )]
        
        model_cts_const['start_time_is_0'] = [0]
        
        for idx in x_sol:
            model_cts_coeff[f"x_{idx}"]['start_time_is_0'] = [0]
        for idx in t_sol:
            model_cts_coeff[f"t_{idx}"]['start_time_is_0'] = [0]
        
        # constraint set 4
        
        model_cstrs['first_cust_a_lb'] = \
            model.add_constraints(
              (    a[i] 
                >= 
                    sce.travelTime((self._DEPOT_START_IND, i)) - self.ins.params.strict_max_workload.value * (1 - x[self._DEPOT_START_IND, i])
                    for i in cust_list
              ),
              names = ['first_cust_a_lb'+str(i) for i in cust_list]
            )
        
        model_cts_const['first_cust_a_lb'] = \
            [sce.travelTime((self._DEPOT_START_IND, i)) - self.ins.params.strict_max_workload.value for i in cust_list]
        
        
        full_coef = [0 for i in cust_list]
        
        for idx in x_sol:
            model_cts_coeff[f"x_{idx}"]['first_cust_a_lb'] = full_coef.copy()
            coef_count = 0
            for i in cust_list:
                if (self._DEPOT_START_IND, i) == idx:
                    model_cts_coeff[f"x_{idx}"]['first_cust_a_lb'][coef_count] = - self.ins.params.strict_max_workload.value 
                coef_count += 1
                        
        for idx in t_sol:
            model_cts_coeff[f"t_{idx}"]['first_cust_a_lb'] = full_coef.copy()
        
        
        # constraint set 5
        
        model_cstrs['first_cust_a_ub'] = \
            model.add_constraints(
              (    a[i] 
                <= 
                    sce.travelTime((self._DEPOT_START_IND, i)) + self.ins.params.strict_max_workload.value * (1 - x[self._DEPOT_START_IND, i])
                    for i in cust_list
              ),
              names = ['first_cust_a_ub'+str(i) for i in cust_list]
            )
        
        model_cts_const['first_cust_a_ub'] = \
            [sce.travelTime((self._DEPOT_START_IND, i)) + self.ins.params.strict_max_workload.value for i in cust_list]
        
        full_coef = [0 for i in cust_list]
        
        for idx in x_sol:
            model_cts_coeff[f"x_{idx}"]['first_cust_a_ub'] = full_coef.copy()
            coef_count = 0
            for i in cust_list:
                if (self._DEPOT_START_IND, i) == idx:
                    model_cts_coeff[f"x_{idx}"]['first_cust_a_ub'][coef_count] = self.ins.params.strict_max_workload.value 
                coef_count += 1
                        
        for idx in t_sol:
            model_cts_coeff[f"t_{idx}"]['first_cust_a_ub'] = full_coef.copy()
        
        
        # constraint set 6
        
        model_cstrs['define_g'] = \
            model.add_constraints(
                (   g[i] 
                 >= 
                    ((a[i] + h[i] + sce.serviceTime(i) + sce.travelTime((i, self._DEPOT_END_IND))
                           - self.ins.params.max_workload.value) - 2*workload_gap*(1-x[i, self._DEPOT_END_IND]))
                 for i in cust_list), 
                names = ['define_g' + str(i) for i in cust_list]
                )
        
        model_cts_const['define_g'] = \
            [sce.serviceTime(i) + sce.travelTime((i, self._DEPOT_END_IND))
                           - self.ins.params.max_workload.value - 2*workload_gap for i in cust_list]
        
        full_coef = [0 for i in cust_list]
        
        for idx in x_sol:
            model_cts_coeff[f"x_{idx}"]['define_g'] = full_coef.copy()
            coef_count = 0
            for i in cust_list:
                if (i, self._DEPOT_END_IND) == idx:
                    model_cts_coeff[f"x_{idx}"]['define_g'][coef_count] = -2*workload_gap 
                coef_count += 1
                        
        for idx in t_sol:
            model_cts_coeff[f"t_{idx}"]['define_g'] = full_coef.copy()
        
        
        # if given time_window width, then
        if time_window_width:
            
            # constraint set 7
            
            model_cstrs['tww_define_h'] = \
                model.add_constraints(
                        ((h[i] >= t[i] - (time_window_width/2) - a[i]) for i in cust_list
                        ), 
                        names = ['tww_define_h' + str(i) for i in cust_list]
                    )
            
            model_cts_const['tww_define_h'] = \
                [- (time_window_width/2) for i in cust_list]
            
            full_coef = [0 for i in cust_list]
        
            for idx in x_sol:
                model_cts_coeff[f"x_{idx}"]['tww_define_h'] = full_coef.copy()
                
                            
            for idx in t_sol:
                model_cts_coeff[f"t_{idx}"]['tww_define_h'] = full_coef.copy()
                coef_count = 0
                for i in cust_list:
                    if i == idx:
                        model_cts_coeff[f"t_{idx}"]['tww_define_h'][coef_count] = -1
                    coef_count += 1
            
            # constraint set 8
            
            model_cstrs['tww_define_w'] = \
                model.add_constraints(
                    ((w[i] >= a[i] - t[i] - (time_window_width/2)) for i in cust_list
                    ), 
                    names = ['tww_define_w'+ str(i) for i in cust_list] 
                )
        
            model_cts_const['tww_define_w'] = \
                [- (time_window_width/2) for i in cust_list]
                
            for idx in x_sol:
                model_cts_coeff[f"x_{idx}"]['tww_define_w'] = full_coef.copy()
                
                            
            for idx in t_sol:
                model_cts_coeff[f"t_{idx}"]['tww_define_w'] = full_coef.copy()
                coef_count = 0
                for i in cust_list:
                    if i == idx:
                        model_cts_coeff[f"t_{idx}"]['tww_define_w'][coef_count] = 1
                    coef_count += 1
        
        distance_time_dict = {ind: sce.travelTime(ind) for ind in x}
        
        if self.binary_subproblem_cut:
            obj_expr = self.ins.params.cost_wait.value * model.sum(w) +\
                        self.ins.params.cost_idle.value * model.sum(h) + \
                        self.ins.params.cost_overtime.value * model.sum(g) +\
                        self.ins.params.cost_travel_time.value * model.sum(x[i, j]* distance_time_dict[i, j] for i, j in x)
        else:
            obj_expr = self.ins.params.cost_wait.value * model.sum(w) +\
                        self.ins.params.cost_idle.value * model.sum(h) + \
                        self.ins.params.cost_overtime.value * model.sum(g)
            
        model.minimize(obj_expr)
        
        return model, model_cstrs, model_cts_const, model_cts_coeff
        
    def solve(self, 
        time_limit=1200, 
        bender_decomposition=False, 
        mipgap=0.005,
        node_limit = 1e19,
        turnoff_all_cuts = False,
        ):
        
        self.model.parameters.mip.tolerances.mipgap.set(mipgap)
        self.model.parameters.timelimit.set(time_limit)
        self.model.parameters.threads.set(1)
        self.model.parameters.mip.limits.nodes.set(node_limit)
        if turnoff_all_cuts:
            self.model.parameters.mip.limits.cutsfactor.set(0)

        if bender_decomposition and self._n_scenario > 1:
            self.model.parameters.benders.strategy = 1
            for v_name in ['x', 't']:
                for v in self.model_vars[v_name].values():
                    v.benders_annotation = 0
            
            name_to_anno = {}
            anno = 1
            for v_name in self.model_vars:
                if 'x' not in v_name and 't' not in v_name:
                    name = v_name[2:]
                    if name not in name_to_anno:
                        name_to_anno[name] = anno
                        anno += 1
                    this_anno = name_to_anno[name]
                    for v in self.model_vars[v_name].values():
                        v.benders_annotation = this_anno
        
        
        sol = self.model.solve(log_output=True)
        
        return sol
    

    def getToursFromX(self, x_sol):
        # obtain routes
        node_dict = self.ins.node_dict
        node_list = self.ins.getActiveNodeIndexList()
        customer_list = [i for i in node_list if not self.ins.node_dict[i].is_depot]
        route_count = 0
        route_dict = {}
        tour_dict = {}
        depot_start = node_dict[self._DEPOT_START_IND]
        for i in customer_list:
            if round(x_sol[self._DEPOT_START_IND, i]) == 1:
                route_count += 1
                next_node = node_dict[i]
                route_dict[route_count] = [depot_start, node_dict[i]]
                next_node_ind = i
                while next_node.index != self._DEPOT_END_IND:
                    for j in customer_list + [self._DEPOT_END_IND]:
                        if next_node_ind != j and round(x_sol[next_node_ind, j]) == 1:
                            next_node = node_dict[j]
                            next_node_ind = j
                            break
                    route_dict[route_count].append(next_node)
        
        for i in route_dict:
            tour_dict[i] = Tour('_', self.ins)
            tour_dict[i].setPath(route_dict[i])

        return tour_dict

    def getToursFromSolution(self):
        sol = self.model.solution
        assert(sol is not None)
        self.solution = {}
        for var in self.model_vars.keys():
            self.solution[var] = {}
            if var == 'x':
                for arc in self.ins.getActiveArcPairIndexList():
                    self.solution['x'][arc] = sol.get_value(self.model_vars['x'][arc])
        
        return self.getToursFromX(self.solution['x'])

    def getSolution(self):
        sol = self.model.solution
        assert(sol is not None)
        self.solution = {}
        node_list = self.ins.getActiveNodeIndexList()
        customer_list = [i for i in node_list if not self.ins.node_dict[i].is_depot]
        for var in self.model_vars.keys():
            self.solution[var] = {}
            if var == 'theta':
            #    self.solution[var] = sol.get_value(self.model_vars[var]['theta'])
                continue
            
            if var == 'x':
                for arc in self.ins.getActiveArcPairIndexList():
                    self.solution['x'][arc] = sol.get_value(self.model_vars['x'][arc])
            else:
                iter_list = node_list if var == 'a' else customer_list
                for c in iter_list:
                    self.solution[var][c] = sol.get_value(self.model_vars[var][c])
        
        self.tour_dict = self.getToursFromX(self.solution['x'])
        
        return self.solution
    

    def convertPathPlanToX(self, path_plan: dict):
        x = {idx:0 for idx in self.model_vars['x']}
        
        for path in path_plan.values():
            # add start
            if len(path) == 0:
                continue

            x[(self.ins.depot_start.index, path[0].index)] = 1
            x[(path[-1].index, self.ins.depot_end.index)] = 1
            for i in range(1, len(path)):
                x[(path[i-1].index, path[i].index)] = 1
        
        return x


    def addInitSolution(self, path_plan: dict, appointed_time:dict):
        t_warm_start = appointed_time
        x_warm_start = self.convertPathPlanToX(path_plan)
        
        sol = {self.model_vars['x'][idx]: x_warm_start[idx] for idx in x_warm_start}
        for idx in self.model_vars['t']:
            sol[self.model_vars['t'][idx]] = t_warm_start[idx]

        warm_start = self.model.new_solution(sol)
        self.model.add_mip_start(warm_start)


    def addInitPathPlan(self, path_plan:dict):
        # add input path_plan as solution to have a warm start
        x_warm_start = self.convertPathPlanToX(path_plan)
        
        warm_start = self.model.new_solution({self.model_vars['x'][idx]: x_warm_start[idx] for idx in x_warm_start})
        self.model.add_mip_start(warm_start)
    
    
    def dropFleetSizeCost(self,):
        # drop fleet_sizing cost
        x = self.var_dict['x']
        self.obj_expr -= self.ins.params.cost_hire.value * self.model.sum( x[i, j] for i, j in x if j==self._DEPOT_END_IND)
    
        self.model.minimize(self.obj_expr)
    
    
    def plot(self, route_dict):
        # Depot: Blue Sqaure
        # Customer with order cancelled: Green Circle
        # Customer with order NOT cancelled: Red Circle
        
        ax = plt.axes()
        node_dict = self.ins.node_dict
        for p in node_dict.values():
            if 'depot' in p.name:
                ax.plot(p.x_loc, p.y_loc, 'bs', markersize=10, zorder=3)
            elif p.is_active:
                ax.plot(p.x_loc, p.y_loc, 'ro', zorder=3)
            else:
                ax.plot(p.x_loc, p.y_loc, 'go', zorder=3)
                
        route_num = len(route_dict)
        cmap = plt.get_cmap('jet', route_num*2+1)
        cind = 0
        for route in route_dict.values():
            for i in range(len(route)-1):
                from_node = route[i]
                to_node = route[i+1]
                diff_x = to_node.x_loc - from_node.x_loc
                diff_y = to_node.y_loc - from_node.y_loc
                ax.arrow(from_node.x_loc+diff_x*0.1, from_node.y_loc+diff_y*0.1, diff_x*0.75, diff_y*0.75, 
                         head_width=1, head_length=2, color=cmap(2*cind), zorder=2)
            cind += 1
        plt.show()
        
        
    
if __name__ == '__main__':
    
    pass
    