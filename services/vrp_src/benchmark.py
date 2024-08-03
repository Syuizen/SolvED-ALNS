# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 21:49:50 2021

@author: Yiran
"""

"""
A benchmark test framework based on Monte Carlo simulation
"""

# project scripts ----------------
from services.vrp_src.modelComponents import Instance, Scenario
from services.vrp_src.cplex.districtModel import DistModel
from services.vrp_src.cplex.modelBuilder import ModelBuilder
from services.vrp_src.heuristic.alns import ALNS
# --------------------------------

# packages -----------------------
import numpy as np
import random
import time
import os
import ast
import ujson
# --------------------------------

class BenchmarkTest:
    
    def __init__(self, ins:Instance):
        self.ins = ins
        self.solver_dict ={}
        
        
    def _benchmarkBuild(self, n_scenario, n_sample, tour_dict=None):
        # cluster n_sample into n_scenario
        pass


    def quickEvaluate(self, n_sample, tour_dict=None):
        # evaluate each sample iteratively
        pass
    
    
    def benchmarkFromDir(self):
        pass
    
    
    def addSolver(self, name, solver):
        assert(callable(solver))
        
        self.solver_dict[name] = solver
    
    

def readJSONtoScenario(file_name, ins: Instance):
    sce = Scenario(ins)
    print('load json file: ', file_name)
    with open(file_name) as infile:
        data = ujson.load(infile)
        sce.setProbability(data['probability'])
        service_time_total = 0
        for c in data['customer']:
            c_data = data['customer'][c]
            input_service_time = c_data['service_time'] * ins.params.service_time_mean.value / 45
            sce.setServiceTime(int(c), input_service_time)
            service_time_total += input_service_time
        
        print('mean_service_time: ', service_time_total/len(ins.customer_dict))
            
        for arc in data['arc']:
            arc_ind = list(ast.literal_eval(arc))
            
            # BELOW is for n_node <= 50
            #if ins.n_customer <= 50:
            #    if type(arc_ind[0]) is int:
            #        arc_ind[0] = arc_ind[0] + 1 
            #    if type(arc_ind[1]) is int:
            #        arc_ind[1] += 1
            if arc_ind[0] in ['_s_', '_e_'] and arc_ind[1] in ['_s_', '_e_']:
                arc_length = 0
            else:
                arc_length = ins.arc_dict[arc_ind[0], arc_ind[1]].length
            sce.setTravelTime(ast.literal_eval(arc), 
                              arc_length/60 * data['arc'][arc]['unit_travel_time'])
        
        infile.close()
        
    return sce


def heur_opt_only(ins, path_plan, acceptor="Simulated_Annealing"):
    
    #mb = ModelBuilder(ins)
    #mb.buildVRP()
    #team_log = mb.searchBestNTeams()
    #n_team = min(team_log, key=lambda x: x[1])[0]
    #dM = DistModel(ins)
    #dist_dict = dM.quickDist(n_team)
    #path_plan = dM.assignTimeSlots(dist_dict, working_hours=8, start_working_time=8, 
    #                               n_team=n_team, duration=2,
    #                               assign_time=False)
    
    alns = ALNS(ins)
    if path_plan is None:
        alns.initSolution()
    else:
        alns.setPathPlan(path_plan)
    
    
    #alns.freezeVehicles()
    #alns._localSearchMoveHeur.disable()
    #alns._localSearchTwoSwapBetweenTours.disable()
    #alns._localSearchTwoSwapIntra.disable()
    alns._localSearchTourOverlapBreaker.disable()
    alns.solve(num_to_destroy=max(1, int(0.1*ins.n_customer)), max_iter=10*ins.n_customer, acceptor=acceptor)
    
    for tour in alns.tour_dict.values():
        tour.getNodeVisitTime()
        tour.setTimeWindowToPathNode(0.5, True)

    return alns.tour_dict, 0, 0
    

def heur_multi(ins, path_plan, acceptor="Simulated_Annealing"):
    
    #mb = ModelBuilder(ins)
    #mb.buildVRP()
    #team_log = mb.searchBestNTeams()
    #n_team = min(team_log, key=lambda x: x[1])[0]
    #dM = DistModel(ins)
    #dist_dict = dM.quickDist(n_team)
    #path_plan = dM.assignTimeSlots(dist_dict, working_hours=8, start_working_time=8, 
    #                               n_team=n_team, duration=2,
    #                               assign_time=False)
    
    alns = ALNS(ins)
    
    #alns.setALNSObj(lambda x: 0)
    
    if path_plan is None:
        alns.initSolution()
    else:
        alns.setPathPlan(path_plan)
    
    #for i in ins.customer_dict.values():
    #    print(i.time_window)
    
    #print(alns)
    
    
    #alns.freezeVehicles()
    alns.setTourObjType('expected')
    alns.solve(num_to_destroy=max(1, int(0.1*ins.n_customer)),  max_iter=10*ins.n_customer, acceptor=acceptor)
    for tour in alns.tour_dict.values():
        tour.getNodeVisitTime()
        tour.setTimeWindowToPathNode(0.5, True)
        
    return alns.tour_dict, 0, 0
    

def ALNS_solver(ins, path_plan, _acceptor="Simulated_Annealing"):
    for i in ins.customer_dict.values():
        i._clearTimeWindow()
        
    mb = ModelBuilder(ins)
    mb.buildVRP()
    team_log = mb.searchBestNTeams()
    n_team = min(team_log, key=lambda x: x[1])[0]
    path_plan = None
    while path_plan is None:
        dM = DistModel(ins)
        dist_dict = dM.quickDist(n_team)
        path_plan = dM.assignTimeSlots(dist_dict, working_hours=8, start_working_time=8, 
                                       n_team=n_team, duration=2, assign_time=False)
        n_team += 1
     
    alns = ALNS(ins)
    
    
    #alns.setALNSObj(lambda x: 0)
    
    
    if path_plan is None:
        alns.initSolution()
    else:
        alns.setPathPlan(path_plan)
    
    #for i in ins.customer_dict.values():
    #    print(i.time_window)
    
    #print(alns)
    
    #alns.freezeVehicles()
    alns.solve(num_to_destroy=max(1, int(0.1*ins.n_customer)), max_iter=10*ins.n_customer, acceptor=_acceptor)
    #print(alns)
    for tour in alns.tour_dict.values():
        tour.getNodeVisitTime()
        tour.setTimeWindowToPathNode(0.5, True)
    
    return alns.tour_dict, 0, 0


def heur_solver(ins, path_plan, acceptor="Simulated_Annealing"):
    
    """
    Stage 1: assign time window by districting model    
    """
   
    #print("---- ", dM.getServiceTeamBound(), " ------")
    #mb = ModelBuilder(ins)
    #mb.buildVRP()
    #team_log = mb.searchBestNTeams()
    #n_team = min(team_log, key=lambda x: x[1])[0]
    #dM = DistModel(ins)
    #dist_dict = dM.quickDist(n_team)
    #path_plan = dM.assignTimeSlots(dist_dict, working_hours=8, start_working_time=8, n_team=n_team, duration=2)
     
    """
    Stage 1: create simple routes
    """
    alns = ALNS(ins)
    
    #alns.setALNSObj(lambda x: 0)
    
    if path_plan is None:
        alns.initSolution()
    else:
        alns.setPathPlan(path_plan)
    
    #alns.freezeVehicles() # cannot delete or add vehicles
    #alns.plot()
    #print(alns)
    #alns.solve(num_to_destroy=5, acceptor="Hill_Climbing")
    #print(alns)
    
    """
    Stage 2.1: reset order status (customers may cancel order)
    """
    #order_cancelled_list = []
    #ins.resetStocParams(True, False, False)
    #order_cancelled_list = ins.getInactiveCustomerList()
    
    
    """
    Stage 2.2: use ALNS to build routes
    """
    #for c in order_cancelled_list:
    #    alns.removeNodeFromPool(c)
    
    #print(alns)
    
    alns.solve(num_to_destroy=max(1, int(0.1*ins.n_customer)), max_iter=10*ins.n_customer, acceptor=acceptor)
    print(alns)
    for tour in alns.tour_dict.values():
        tour.getNodeVisitTime()
        tour.setTimeWindowToPathNode(0.5, True)
    
    return alns.tour_dict, 0, 0


def heur_solver_no_balance(ins, path_plan, acceptor="Simulated_Annealing"):
    
    alns = ALNS(ins)
    
    alns.setALNSObj(lambda x: 0)
    
    if path_plan is None:
        alns.initSolution()
    else:
        alns.setPathPlan(path_plan)
    
    alns.solve(num_to_destroy=max(1, int(0.1*ins.n_customer)), max_iter=10*ins.n_customer, acceptor=acceptor)
    print(alns)
    for tour in alns.tour_dict.values():
        tour.getNodeVisitTime()
        tour.setTimeWindowToPathNode(0.5, True)
    
    return alns.tour_dict, 0, 0


def vrp_solver(ins, suggested_fleet=None, time_limit=1800):
    
    mb = ModelBuilder(ins)
    mb.buildVRP(time_window_given=True, time_window_width=0.5, uniform_start_time=8)
    sol = mb.getSolution(assign_time_window=True, mipgap=0, timelimit=time_limit)
    if sol is None:
       return {}, 0, 0
    info = {'t': sol.solve_details.time, 'gap': sol.solve_details.gap}
    return mb.tour_dict, sol.solve_details.best_bound, info


def vrp_solver_best_n_team(ins, suggested_fleet=None, time_limit=1800):
    
    mb = ModelBuilder(ins)
    team_log = mb.searchBestNTeams()
    n_team = min(team_log, key=lambda x: x[1])[0]
    if not suggested_fleet is None and suggested_fleet > 0:
        n_team = min(n_team, suggested_fleet)
    mb.buildVRP(n_team=n_team, time_window_given=True, time_window_width=0.5, uniform_start_time=8, ignore_fleet_size_cost=True)
    sol = mb.getSolution(assign_time_window=True, mipgap=0, duration=0.5, timelimit=time_limit)
    
    if sol is None:
       return {}, 0, 0
   
    info = {'t': sol.solve_details.time, 'gap': sol.solve_details.gap}
    return mb.tour_dict, sol.solve_details.best_bound, info


def vrp_solver_best_n_team_gap(ins, suggested_fleet=None, time_limit=1800):
    
    mb = ModelBuilder(ins)
    team_log = mb.searchBestNTeams()
    n_team = min(team_log, key=lambda x: x[1])[0]
    if not suggested_fleet is None and suggested_fleet > 0:
        n_team = min(n_team, suggested_fleet)
        
    mb.buildVRP(n_team=n_team, time_window_given=True,  time_window_width=0.5, uniform_start_time=8, ignore_fleet_size_cost=True)
    sol = mb.getSolution(assign_time_window=True, mipgap=0.02, duration=0.5, timelimit=time_limit)
    
    if sol is None:
       return {}, 0, 0
    info = {'t': sol.solve_details.time, 'gap': sol.solve_details.gap}
    return mb.tour_dict, sol.solve_details.best_bound, info
    

def vrp_solver_multi(ins, suggested_fleet=None):
    
    mb = ModelBuilder(ins)
    team_log = mb.searchBestNTeams()
    n_team = min(team_log, key=lambda x: x[1])[0]
    if not suggested_fleet is None and suggested_fleet > 0:
        n_team = min(n_team, suggested_fleet)
    
    mb.buildStocVRP(n_team=n_team,  time_window_given=True, time_window_width=0.5, uniform_start_time=8)
    sol = mb.getSolution(assign_time_window=True, mipgap=0.02, duration=0.5, timelimit=1800)
    
    if sol is None:
       return {}, 0, 0
    
    info = sol.solve_details.time
    return mb.tour_dict, sol.solve_details.gap, info



def vrp_solver_multi_bender(ins, suggested_fleet=None):
    mb = ModelBuilder(ins)
    team_log = mb.searchBestNTeams()
    n_team = min(team_log, key=lambda x: x[1])[0]
    if not suggested_fleet is None and suggested_fleet > 0:
        n_team = min(n_team, suggested_fleet)
    mb.buildStocVRP(n_team=n_team,  time_window_given=True, time_window_width=0.5, uniform_start_time=8)
    sol = mb.getSolution(assign_time_window=True, bender_decomposition=True, mipgap=0.02, timelimit=1800)
    if sol is None:
       return {}, 0, 0
   
    info = sol.solve_details.time
    return mb.tour_dict, sol.solve_details.gap, info


def bender_solver_no_TW(ins):
    mb = ModelBuilder(ins)
    mb.buildStocVRP(time_window_given=False, time_window_width=0.5, uniform_start_time=8)
    sol = mb.getSolution(assign_time_window=True, bender_decomposition=True, mipgap=0.02, timelimit=1800)
    if sol is None:
       return {}, 0, 0
   
    info = sol.solve_details.time
    return mb.tour_dict, sol.solve_details.gap, info
    

def my_bender_solver_no_TW(ins):
    mb = ModelBuilder(ins)
    #mb.buildStocVRP(time_window_given=False, time_window_width=0.5, uniform_start_time=8)
    mb._initModel(myBender=True)
    mb.time_window_width = 0.5
    sol = mb.getSolution(assign_time_window=True, own_bender=True, mipgap=0.02, timelimit=1800)
    if sol is None:
       return {}, 0, 0
   
    info = sol.solve_details.time
    return mb.tour_dict, sol.solve_details.gap, info


# chance constraints


def mulscore(ins: Instance, mul_tour_dict: dict, num_scenario: int) -> list:
    
    score_dict = {i: [] for i in mul_tour_dict}
    
    # change velocity and service_time generator
    mean_service_time = random.randint(30, 60)/60
    standard_dev = 0.5*mean_service_time
    shape, scale = (mean_service_time/standard_dev)**2, standard_dev**2/mean_service_time
    
    ins.setting(
            velocity_generator = lambda: np.random.lognormal(60, 10, 1)[0],
            service_time_generator = lambda:  np.random.gamma(shape, scale, 1)[0],
            )
    
    for i in range(num_scenario):
        ins.resetStocParams(False, True, True) # only reset velocity and service_time
        for j in mul_tour_dict:
            tour_dict = mul_tour_dict[j]
            score_list = score_dict[j]
            for tour in tour_dict.values():
                tour.update()
            
            score = sum(tour.score for tour in tour_dict.values())
            score_list.append(score)
    
    return score_dict


def benchamrk_load(ins: Instance):
    assert (ins.n_customer in [10,15,20,25,30,35,40,45,50,100,150])
    # based on the dataset avaliable, currently only support finitely problem size
    
    for subdir, dirs, files in os.walk('data/benchmark/{}'.format(ins.n_customer)):
        for file in files:
            print(file)
            if file.endswith(".json"):
                sce = readJSONtoScenario(subdir + os.sep + file, ins)
                ins.addScenario(sce)
                

def getTotalScore(ins: Instance, tour_dict: dict, sce_set=None):
    
    score = 0
    detail = {'fleet': len(tour_dict), 'wait': 0, 'idle': 0, 'overtime': 0, 'workload': 0}
    for tour in tour_dict.values():
        result = tour.costMultiScenario('expected', ins._scenarioSet if sce_set is None else sce_set)
        #tour.ins = ins
        #tour.setObjType('expected')
        #tour.update()
        score += result['cost']
        detail['wait'] += result['wait_time']
        detail['idle'] += result['idle_time']
        detail['overtime'] += max(0, result['workload'] - ins.params.max_workload.value) 
        detail['workload'] += result['workload']

    return score, detail

if __name__ == "__main__":
    pass
        
    #vrp_tour = vrp_solver(ins)
    
    