# -*- coding: utf-8 -*-
"""
Created on Fri Mar 26 13:23:45 2021

@author: Yiran
"""

#--------------------------------
# project scripts
from services.vrp_src.modelComponents import Instance, Tour
from services.vrp_src.heuristic.heurMethod import HeurMethodBase
#--------------------------------

#--------------------------------
# packages
import random
import math
import time
import sys
import numpy as np

from matplotlib import pyplot as plt
#--------------------------------

class Operator:
    # a single operator for using in ALNS
    
    def __init__(self, operator, name, _preserve_n_tour):
        assert(type(name) is str and name.isidentifier())
        self.__name = name
        
        # operator should take ALNS instance as the first argument
        assert(callable(operator))
        self.__operator = operator
        
        self.__preserve_n_tour = _preserve_n_tour
        
        #initial status
        self.__active = True
        self.__n_attempts = 0
        self.__score = 0
        self.__accu_score = 0
        self.__weight = 1 # affect probability of choosing this operator
        self.__prev_weight = 0
        # set the frequency of calling, if frequency = 5, the operator will run every 5 calls
        # when set to be larger than 1, you may need to define silentCall maunally to avoid unexcepted behavior 
        self.__call_frequency = 1
        
        
    @property
    def __name__(self):
        return "Operator: {}".format(self.name)
    
    
    def __call__(self, alns, *args, **kwargs):
        self.__n_attempts += 1
        if self.__n_attempts % self.__call_frequency == 0:
            return self.__operator(alns, *args, **kwargs)
        else:
            return self.silentCall(alns)
        
    
    
    @property
    def name(self):
        return self.__name
    
    
    @property
    def is_active(self):
        return self.__active
    
    
    @property
    def score(self):
        return self.__score
    
    
    @property
    def accu_score(self):
        return self.__accu_score
    
    
    @property
    def n_attempts(self):
        return self.__n_attempts
    
    
    @property
    def weight(self):
        return self.__weight
    
    
    @property
    def prev_weight(self):
        return self.__prev_weight
    
    
    @property
    def fleet_size_safe(self):
        # if operator will preserve fleet size
        return self.__preserve_n_tour
    
    
    def setWeight(self, w):
        self.__prev_weight = self.__weight
        self.__weight = w
    
    
    def addScore(self, s):
        self.__score += s
        self.__accu_score += s
    
    
    def enable(self):
        self.__active = True
    
    
    def disable(self):
        self.__active = False
    
    
    def resetStat(self):
        self.__n_attempts = 0
        self.__score = 0
    
    
    def reset(self):
        # go to initial status
        self.__active = True
        self.__n_attempts = 0
        self.__score = 0
        self.__accu_score = 0
        self.__weight = 1
        self.__prev_weight = 0
        self.__call_frequency = 1
        
        
    def silentCall(self, alns):
        pass
    
    
    def setCallFrequency(self, fre):
        assert(type(fre) is int)
        
        self.__call_frequency = fre
    

class OperatorGroup:
    
    def __init__(self, group_name):
        self._operator_list = []
        self.__name = group_name
    
    
    @property
    def name(self):
        return self.__name
    
    
    def __iter__(self):
        return self._operator_list.__iter__()
    
    
    def __getitem__(self, ind):
        return self._operator_list[ind]
    
    
    def __contains__(self, v):
        return self._operator_list.__contains__(v)
    
    
    def __str__(self):
        out_expr = 'Operators added:\n'
        for op in self._operator_list:
            out_expr += op.__name__
            out_expr += "\n"
        
        return out_expr
    
    
    def isEmpty(self):
        return False if len(self._operator_list) else True
    
    
    def select(self):
        if self.isEmpty():
            raise Exception("no operator is added!")
            
        # select a operator randomly
        sum_weight = sum(op.weight for op in self if op.is_active)
        target_weight = random.random() * sum_weight
        accu_weight = 0
        select_op = self._operator_list[0]
        
        for op in self:
            if op.is_active:
                accu_weight += op.weight
                if target_weight < accu_weight:
                    break
                else:
                    select_op = op
        
        return select_op
    
    
    def resetAll(self):
        for op in self:
            op.reset()
    
    
    def enableAll(self):
        for op in self:
            op.enable()
    
    
    def addOperators(self, *operators):
        for op in operators:
            self.addOperator_(op)


    def addOperator_(self, op: Operator):
        if op not in self._operator_list:
            self._operator_list.append(op)


    def removeOperator(self, op: Operator):
        if op in self._operator_list:
            self._operator_list.remove(op)
    

#------------------
# operator wrapper
def makeOperator(preserve_number_of_tour):
    def wrapper(method):
        return Operator(method, method.__name__, preserve_number_of_tour)
    return wrapper
#------------------
#------------------
# overrides indicator
def overrides(interface_class):
    def overrider(method):
        assert(method.__name__ in dir(interface_class))
        return method
    return overrider
#------------------


class ALNS(HeurMethodBase): 
    
    def __init__(self, ins: Instance, path_plan=None, default_setting=True):
        super(ALNS, self).__init__(ins, path_plan)
        if default_setting:
            self.setting()
        
        #self.__operator_dict = {}
        self.__iteration_count = 0
        self.__nb_accept = 0
        
        self.__alns_obj = self.defaultALNSObj
        
        ### initialize operator groups
        #   only operators added into the group will be used in ALNS solver
        #   however, added operators may be temporarily disabled by setting
        #       operator.disable()
        #   e.g. disable _destroyOperatorTourRemoval by calling self._destroyOperatorTourRemoval.disable()
        #   re-enable operator: operator.enable()
        
        self.repair_operators = OperatorGroup('repair')
        self.destroy_operators = OperatorGroup('destroy')
        
        ### repair operators
        self.repair_operators.addOperators(self._repairOperatorGreedyInsertWithPerturbation,
                                          self._repairOperatorGreedyInsertNoPerturbation)
        
        ### destroy operators
        self.destroy_operators.addOperators(self._destroyOperatorRandomRemoval,
                                           self._destroyOperatorRelatedRemoval,
                                           self._destroyOperatorLongestTourBreakIntoHalf,
                                           self._destroyOperatorOvercapacitatedTourBreakIntoHalf,
                                           self._destroyOperatorTourRemoval,
                                           self._destroyOperatorWorstRemoval)
        
        
        ### default frequency of local serach operator
        self._localSearchTwoSwapBetweenTours.setCallFrequency(5)

        ### disable two LS operators -> these two are not very efficient
        self._localSearchMoveHeur.disable()
        self._localSearchTwoSwapIntra.disable()
    
    
    def __str__(self):
        # format print output
        
        alns_str = "ALNS - Current WorkPlan:"
        for tour in self.getListOfTours():
            alns_str += "\n" + str(tour)
        
        alns_str += "\n--------------------\n Stat Table: \n"
        if self.params.frozen_vehicles.value:
            print(" ----- Warning -----")
            print(" -- vehicle_fixed_cost is ignored because fleet_size is frozen.")
            print(" ---------------------------")
        score_dict = self.scoreTable()
        for i in score_dict:
            alns_str += " - {0}: {1:.2f}, \n" .format(i, score_dict[i])
  
        alns_str += "----------------------\n"
        
        return alns_str
    
    
    def setting(self,
        Lambda              = 0.3,      # only proceed good candidate with solution value within (1+Lambda)% to local search stage
        non_changing_iterations = 25,   # for the stopping criteria 
        num_iter_before_cooling = 5,   # total inner iterations before changing the cooling factor for simulated annealing
        # parameter from adaptive weight adjustment: 
        weight_segment      = 15,       # part of the adaptive weight adjustment takes in the segment length 
        reaction_factor     = 0.7,      # within [0,1] and shows how reactive/fast the operator weights adapt to their performance
        cooling             = 0.8,      # cooling coefficient, more approaches 1 then slower/harder to get to the optimal solution 
        initial_temperature = 6,        # Greater the value, the greater likelkhood of moving araound the search space
        ):
        
        self.params.setOrAddParameter('lambda_', Lambda)
        self.params.setOrAddParameter('n_non_changing_iter', non_changing_iterations)
        self.params.setOrAddParameter('n_iter_before_cooling', num_iter_before_cooling)
        self.params.setOrAddParameter('weight_segment', weight_segment)
        self.params.setOrAddParameter('reaction_factor', reaction_factor)
        self.params.setOrAddParameter('cooling_factor', cooling)
        self.params.setOrAddParameter('init_temperature', initial_temperature)
    
    
    def setObjective(self, alns_obj, tour_obj, obj_type):
        # set objective function of alns
        # obj = alns_obj(plan) + sum of tour_obj(tour)
        # alns_obj should take a single argument ALNS
        # tour_obj should take a single argument Tour
        # both obj should not modify any class attributes
        # refer to default obj as an example
        
        assert(callable(alns_obj) and callable(tour_obj))

        self.__alns_obj = alns_obj
        self.__alns_obj_type = obj_type
        self._tour_obj = tour_obj
        self._tour_obj_type = obj_type
    
        
    def setALNSObj(self, alns_obj):
        # alns_obj should be scenario-independent
        # TODO: add suport for scenarios
        assert(callable(alns_obj))

        self.__alns_obj = alns_obj
        
    
    def defaultALNSObj(self, alns):
        # penalize imbalance between tours
        
        if len(alns.tour_dict) > 1:
            workload_list = np.array([tour.workload for tour in alns.tour_dict.values()])
            mean_workload = np.mean(workload_list)
            
            coef = 5
            cost = coef * np.sum((np.abs(workload_list-mean_workload)-0.3).clip(min=0)**2)
            
            cost += 100000 if max(tour.worst_workload for tour in alns.tour_dict.values()) > self.ins.params.strict_max_workload else 0
            
            return cost
        
        return 0        
    
    
    @property
    def tourScore(self):
        tour_sum_score = sum(tour.score for tour in self.tour_dict.values())
        if self.params.frozen_vehicles.value:
            tour_sum_score -= self.ins.params.cost_hire * len(self.tour_dict)
        
        return tour_sum_score


    @property
    def score(self):        
        alns_score = self.__alns_obj(self) 

        return  alns_score + self.tourScore
        
    
    def initSolution(self):
        candidate_list = list(self.customer_pool)
        while len(candidate_list):
            node = random.sample(candidate_list, 1)[0]
            candidate_list.remove(node)
            self._greedyInsertNode(node)
        
            
    @overrides(HeurMethodBase)
    def freezeVehicles(self):
        self.params.frozen_vehicles.setValue(1)
        # setting: cannot change number of vehicles
        for op_group in [self.destroy_operators, self.repair_operators]:
            for op in op_group:
                if not op.fleet_size_safe:
                # disable operator which cannot preserve fleet size
                    op.disable()
    
    
    @overrides(HeurMethodBase)
    def defreezeVehicles(self):
        self.params.frozen_vehicles.setValue(0)
        self.destroy_operators.enableAll()
        self.repair_operators.enableAll()
    
    
    ### Solver Definition
    
    def _accept_solution(self, target_obj, acceptor="Simulated_Annealing"):
        current_obj = self.score
        obj_diff = abs(current_obj - target_obj)

        if self.__iteration_count == 0:
                self.__EA = abs(obj_diff)

        if self._allToursAreValid():
            if acceptor == "Hill_Climbing":  #can ignore hill climbling, which only accept better solution 
                if current_obj < target_obj:
                    accept = True
                    credit_status = "better_accept"
                else:
                    accept = False
                    credit_status = "worse_refuse"
                    
            elif acceptor == "Simulated_Annealing": 
                if current_obj > target_obj:
                    p = math.exp(-self.__EA / (self.__temperature))
                    if random.random() < p:                              # make a decision to accept the worse solution or not 
                        accept = True                                    # accept the worse solution current_obj
                        credit_status = "worse_accept"   
                    else: 
                        accept = False                                   # refuse the worse solution current_obj
                        credit_status = "worse_refuse"         
                else: 
                    accept = True                                        # accept the better solution current_obj
                    credit_status = "better_accept"
        else:
            accept=False
            credit_status = "worse_refuse"
        
        if accept:
            self.__nb_accept += 1
            self.__EA = (self.__EA * (self.__nb_accept-1) + obj_diff) / self.__nb_accept
        
        return accept, credit_status
    
    
    def enforceValid(self):
        self._destroyOperatorOvercapacitatedTourBreakIntoHalf(self)
        
    
    def _baseIterate(self, acceptor):
        # run a single iteration
        self._failure = False

        old_obj = self.score
        old_path_plan = self._exportPathResultPlan()
        
        # select operators
        destroy = self.destroy_operators.select()
        repair = self.repair_operators.select()
        
        removed_list = destroy(self) # run selected destroy operator
        repair(self, removed_list) # run selected repair operator
        
        if self._failure:
            self._setPathResultPlan(old_path_plan)
            return
        
        accept, credit_status = self._accept_solution(old_obj, acceptor)
        
        if not accept:
            #roll-back to previou plan
            self._setPathResultPlan(old_path_plan)
        
        ### update operator
        self.rouletteWheelOperatorUpdate(self.destroy_operators, destroy, credit_status)
        self.rouletteWheelOperatorUpdate(self.repair_operators, repair, credit_status)
        
        if self.score <= (self.params.lambda_ + 1) * old_obj:
            # apply local search
            enter_score = self.score
            
            if self._localSearchTwoOpt.is_active:
                self._localSearchTwoOpt(self, max_iter=300)
                after_score = self.score
                self._localSearchTwoOpt.addScore(enter_score-after_score)
                enter_score = after_score
            
            if self._localSearchMoveHeur.is_active:
                self._localSearchMoveHeur(self, max_iter=100)
                after_score = self.score
                self._localSearchMoveHeur.addScore(enter_score-after_score)
                enter_score = after_score
                
            if self._localSearchTwoSwapIntra.is_active:
                self._localSearchTwoSwapIntra(self, max_iter=100)
                after_score = self.score
                self._localSearchTwoSwapIntra.addScore(enter_score-after_score)
                enter_score = after_score
                
            if self._localSearchTourOverlapBreaker.is_active:
                self._localSearchTourOverlapBreaker(self, max_iter=30)
                after_score = self.score
                self._localSearchTourOverlapBreaker.addScore(enter_score-after_score)
                enter_score = after_score
                
            if self._localSearchTwoSwapBetweenTours.is_active:
                self._localSearchTwoSwapBetweenTours(self, max_iter=10)
                after_score = self.score
                self._localSearchTwoSwapBetweenTours.addScore(enter_score-after_score)
                enter_score = after_score
                
        self.__iteration_count += 1
    
    
    @overrides(HeurMethodBase)
    def solve(self, num_to_destroy, max_iter=100, acceptor="Simulated_Annealing", time_limit=1800, show_log=True):
        #-----------------------------
        # Initial condition
        self._solver_start_time = time.time()
        self._solver_num_to_destroy = num_to_destroy
        self.__temperature = self.params.init_temperature
        if (show_log):
            print('---- ALNS start -----\n', 
                  '- initial objective value: {}\n'.format(self.score),
                  '- Acceptor: {}\n'.format(acceptor))
            print('---- Local Search Operator Usage Status ----\n')
            print(f"""
            Two Opt          : {self._localSearchTwoOpt.is_active}
            Two Swap [intra] : {self._localSearchTwoSwapIntra.is_active}
            Two Swap [inter] : {self._localSearchTwoSwapBetweenTours.is_active}
            Overlap Breaker  : {self._localSearchTourOverlapBreaker.is_active}
            Single Move      : {self._localSearchMoveHeur.is_active}
            """)
        #-----------------------------
            
        iter_count = 1
        best_obj = self.score
        self.__no_change_iter_count = 0
        while iter_count < max_iter and self.__no_change_iter_count < self.params.n_non_changing_iter:
            for _ in range(self.params.n_iter_before_cooling.value):
                
                # terminate if exceeds time_limit
                if time.time() - self._solver_start_time > time_limit:
                    print('ALNS stops due to time limit.')
                    break
                
                self._baseIterate(acceptor)

            # outer loop
            obj = self.score
            if best_obj >  obj:
                best_obj =  obj
                self.__no_change_iter_count = 0
            else:
                self.__no_change_iter_count +=1
            
            if acceptor == "Simulated_Annealing":
                self.__temperature *= self.params.cooling_factor

            iter_count += 1              
        
        #-----------------------------
        # Terminate condition
        self.runtime = round(time.time() - self._solver_start_time, 2)
        self._solver_start_time = None
        
        if (show_log):
            print('---- ALNS end -----\n',
                  '- final objective value: {}\n'.format(self.score), 
                  '- time used: {} sec\n'.format(self.runtime))
        
        print(self)
        #-----------------------------
    
        
    def rouletteWheelOperatorUpdate(self, op_group, selected_op, credit_status):
        
        if credit_status == "better_accept":       # accepted a better solution 
            Q = 4                           
        elif credit_status == "worse_accept":      # accepted a worse solution   
            Q = 2                            
        elif credit_status == "worse_refuse":      # rejected a worse solution 
            Q = 0
        
        if self.__iteration_count % self.params.weight_segment == 0:  #update the "time selected" for each operator to  0 
            for op in op_group:
                if op.score != 0 and op.n_attempts != 0:
                    op.setWeight(\
                            (1 - self.params.reaction_factor) * op.prev_weight + \
                            self.params.reaction_factor * (op.weight / op.n_attempts)
                    )
                
                op.resetStat()
                
        else: #keep accumulating data during this segment:
            selected_op.addScore(Q)
    
    ### Local Search Definition
    
    @makeOperator(True)
    def _localSearchMoveHeur(self, max_iter=1e3):
        for tour in self.tour_dict.values():
            tour._moveHeur(max_iter)
    
    
    @makeOperator(True)
    def _localSearchTwoSwapIntra(self, max_iter=1e3):
        for tour in self.tour_dict.values():
            tour._twoSwap(max_iter)
    
    
    @makeOperator(True)
    def _localSearchTwoOpt(self, max_iter=1e4):
        # optimize all tours by two-opt
        for tour in self.tour_dict.values():
            tour.optimize('two_opt', max_iter)
        
    
    @makeOperator(True)
    def _localSearchTwoSwapBetweenTours(self, max_iter=100):
        # optimize by swaping nodes between tours
        is_imporved = True
        iter_count = 0
        c_pair = [(i, j) for i in self.customer_pool for j in self.customer_pool if i.index != j.index]
        while is_imporved and iter_count < max_iter:
            is_imporved = False
            old_score = self.score
            iter_count += 1
            for a, b in c_pair:
                tour_a = self._getTourHavingNode(a)
                tour_b = self._getTourHavingNode(b)
                if tour_a is tour_b:
                    # a & b are in the same tour
                    continue
                
                tour_a_stat = tour_a.exportStat()
                tour_b_stat = tour_b.exportStat()
                
                tour_a.replace(a, b)
                tour_b.replace(b, a)
                
                if self.score < old_score:
                    #apply swap
                    is_imporved = True
                    self._node_to_tour[a.index] = tour_b.name
                    self._node_to_tour[b.index] = tour_a.name
                else:
                    # set back
                    tour_a.replace(b, a, ignore_update=True)
                    tour_a.setStat(tour_a_stat)
                    tour_b.replace(a, b, ignore_update=True)
                    tour_b.setStat(tour_b_stat)
    
    
    @makeOperator(True)
    def _localSearchTourOverlapBreaker(self, max_iter=10):
        is_improved = True
        iter_count = 0
        
        while is_improved and iter_count < max_iter:
            is_improved = False
            old_score = self.score
            tour_name_list = [tour.name for tour in self.tour_dict.values() if not tour.isEmpty()]
            tour_num = len(tour_name_list)
            tour_pair_list = [(tour_name_list[i], tour_name_list[j]) for i in range(tour_num) for j in range(tour_num) if i > j]
            random.shuffle(tour_pair_list)
            for pair in tour_pair_list:
                tour_0 = self.tour_dict[pair[0]]
                tour_1 = self.tour_dict[pair[1]]
                if tour_0.overlap(tour_1):
                    
                    old_tour_0_path_result = tour_0._exportPath()
                    old_tour_1_path_result = tour_1._exportPath()
                    
                    # delete
                    self._deleteTourByName(tour_0.name)
                    self._deleteTourByName(tour_1.name)
                    
                    # create temp tour
                    new_path = old_tour_0_path_result[0] + old_tour_1_path_result[0]
                    temp_tour = self._createTour('_')
                    temp_tour.setPath(new_path)
                    temp_tour.optimize('two_opt')
                    
                    # create two new tour
                    sub_path_1, sub_path_2 = temp_tour.splitPathIntoHalf(2)
                    temp_tour_1 = self._createNewTourByPath(sub_path_1)
                    temp_tour_2 = self._createNewTourByPath(sub_path_2)
                    
                    if self.score < old_score \
                            and self.tourIsValid(temp_tour_1)\
                            and self.tourIsValid(temp_tour_2):
                        is_improved = True
                        break
                    else:
                        self._deleteTourByName(temp_tour_1.name)
                        self._deleteTourByName(temp_tour_2.name)
                        self._createNewTourByPathResult(old_tour_0_path_result, pair[0])
                        self._createNewTourByPathResult(old_tour_1_path_result, pair[1])
            
            iter_count += 1

    ### Repair Operator Definition
    @overrides(HeurMethodBase)
    def _greedyInsertNode(self, node, need_perturbation=False):
        # Find the best tour to insert the given node
        best_tour = None
        tour_score_list = []
        d = self.ins.params.cost_travel
        if need_perturbation:
            d *= random.uniform(0.8, 1.2)
        for tour in self.getListOfTours():
            position_score_list = []
            n_pos = len(tour) + 1
            for pos in range(n_pos):                
                gain, new_workload = self._addGain(tour, pos, node)
                position_score_list.append( (pos, d * gain, new_workload))
                
            if len(position_score_list):
                best_position, best_score, new_workload = min(position_score_list, key=lambda x: x[1])
                if new_workload <= self.ins.params.strict_max_workload.value:
                # consider tour valid only if the total workload satisfies the workload limit
                    tour_score_list.append((tour, best_position, best_score))
        
        if len(tour_score_list):
            best_tour, best_position, best_score = min(tour_score_list, key=lambda x:x[2])            
            
        if not self.params.frozen_vehicles:
            # consider creating a new tour
            temp_tour = self.createNewTourByPath([node])
            new_tour_score = temp_tour.score
            if not len(tour_score_list) or best_score > new_tour_score:
                return
            else:
                self.deleteTourByName(temp_tour.name)
        
        if best_tour is None:
            self._failure = True
            return
        
        self.insertIntoTourByName(best_tour.name, node)
        return # terminate
        
    
    @makeOperator(True)
    def _repairOperatorGreedyInsertNoPerturbation(self, candidate_list):
        return self._repairOperatorGreedyInsert(candidate_list, False)
    
    
    @makeOperator(True)
    def _repairOperatorGreedyInsertWithPerturbation(self, candidate_list):
        return self._repairOperatorGreedyInsert(candidate_list, True)
    
    
    def _repairOperatorGreedyInsert(self, candidate_list, need_perturbation):
        while candidate_list:
            node = random.sample(candidate_list, 1)[0]
            candidate_list.remove(node)
            self._greedyInsertNode(node, need_perturbation)
            if self._failure:
                return
    
    
    ### Destory Operators Definition
  
    @makeOperator(True)
    def _destroyOperatorRandomRemoval(self):
        # remove $num randomly-chosen customers from all tours
        num = self._solver_num_to_destroy
        removed_list = random.sample(self.customer_pool, num)
        for c in removed_list:
            self.removeNodeFromTour(c)
        
        return removed_list
    
    
    @makeOperator(True)
    def _destroyOperatorWorstRemoval(self):
        # remove $num customers with the highest removal gain from all tours
        num = self._solver_num_to_destroy
        score_table = [(c, self._removalGain(c)) for c in self.customer_pool]
        score_table.sort(key=lambda x: x[1], reverse=True)
        removed_list = [c[0] for c in score_table[:num]]
        for c in removed_list:
            self.removeNodeFromTour(c)
        
        return removed_list

    
    @makeOperator(True)
    def _destroyOperatorRelatedRemoval(self):
        # randomly pick 1 customer and remove $num closet neighbors of it (including itself)
        num = self._solver_num_to_destroy
        c = random.sample(self.customer_pool, 1)[0]
        distance_map = [(i, self.ins.arc_dict[c.index, i.index].length) for i in self.customer_pool if i is not c]
        distance_map.sort(key=lambda x: x[1])
        removed_list = [i[0] for i in distance_map[:num]]
        for node in removed_list:
            self.removeNodeFromTour(node)
        
        return removed_list

    
    @makeOperator(False)
    def _destroyOperatorTourRemoval(self):
        # randomly delete a tour
        tour_name = random.sample(self.tour_dict.keys(), 1)[0]        
        removed_list = self.tour_dict[tour_name].path.copy()
        self.deleteTourByName(tour_name)
        
        return removed_list
    
    
    @makeOperator(False)
    def _destroyOperatorLongestTourBreakIntoHalf(self):
        longest_tour = max(self.getListOfTours(), key=lambda x: x.worst_workload)
        sub_path_1, sub_path_2 = longest_tour.splitPathIntoHalf(2)
        self.deleteTourByName(longest_tour.name)
        self.createNewTourByPath(sub_path_1)
        self.createNewTourByPath(sub_path_2)
        
        return [] # no customers have been removed
    
    
    @makeOperator(False)
    def _destroyOperatorOvercapacitatedTourBreakIntoHalf(self):
        tour_list = list(self.tour_dict.values())
        for tour in tour_list:
            if not self.tourIsValid(tour):
                sub_path_1, sub_path_2 = tour.splitPathIntoHalf(2)
                self.deleteTourByName(tour.name)
                self.createNewTourByPath(sub_path_1)
                self.createNewTourByPath(sub_path_2)
        
        return [] # no customers have been removed
    
    
    ### Utils
    
    def _addGain(self, tour, pos, node):
        # compute the gain of adding $node into $tour at $pos
        # less is better
        old_score = self.score
        
        tour.insert(pos, node)
        tour_new_workload = tour.worst_workload
        
        gain = self.score - old_score
        tour.remove(node)
    
        return gain, tour_new_workload
    

    def _removalGain(self, node):
        # compute the gain of removing $node from its tour
        # higher is better
        old_score = self.score
        
        tour = self._getTourHavingNode(node)
        pos = tour.node_position[node.index]
        tour.remove(node)
        
        gain = old_score - self.score
        tour.insert(pos, node)
        
        return gain


    ### Result Print
    
    def scoreTable(self):
        # compute the score (objective value) of the input tour plan
        # if None then compute the invoking plan
        traversal_cost = 0
        vehicle_fixed_cost = 0
        idle_time_cost = 0
        wait_time_cost = 0
        overtime_cost = 0
        
        tour_list = self.getListOfTours()
        for tour in tour_list:
            traversal_cost += self.ins.params.cost_travel_time * tour.travel_time
            vehicle_fixed_cost += self.ins.params.cost_hire
            wait_time_cost += self.ins.params.cost_idle * tour.customer_wait_time
            idle_time_cost += self.ins.params.cost_wait * tour.worker_idle_time
            overtime_cost += self.ins.params.cost_overtime * tour.overtime
        
        # Record everything inside the dataframe as a row: 
        record = {"ALNS_Cost": self.score,
                  "imbalance_cost": self.__alns_obj(self),
                  "objective": self.tourScore,
                  "traversal_cost":  traversal_cost, 
                  "vehicle_fixed_cost":  vehicle_fixed_cost,
                  "idle_cost": idle_time_cost,
                  "wait_cost": wait_time_cost,
                  "overtime_cost": overtime_cost,
                  }
        
        return record
    

    def updateTimeWindow(self, duration=2, refine=True):
        for t in self.tour_dict.values():
            t.setTimeWindowToPathNode(duration, refine)
        
        for t in self.tour_dict.values():
            t.update()

    
    def plot(self):
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
                
        route_num = len(self.tour_dict)
        cmap = plt.get_cmap('jet', route_num*2+1)
        cind = 0
        for route in self.tour_dict.values():
            path = [self.depot_start] + route.path + [self.depot_end]
            for i in range(len(path)-1):
                from_node = path[i]
                to_node = path[i+1]
                diff_x = to_node.x_loc - from_node.x_loc
                diff_y = to_node.y_loc - from_node.y_loc
                ax.arrow(from_node.x_loc+diff_x*0.1, from_node.y_loc+diff_y*0.1, diff_x*0.75, diff_y*0.75, 
                         head_width=1, head_length=2, color=cmap(2*cind), zorder=2)
            cind += 1
        plt.show()



if __name__ == "__main__":
    ins = Instance(50) # create an instance with 100 customers
    for c in ins.customer_dict.values():
        c.setTimeWindow(9, 10)
    
    alns = ALNS(ins) # load instances
    alns.initSolution()
    print("---------- INIT ----------")
    print(alns)
    #alns.freezeVehicles()
    alns.solve(5,100)
    print(alns)