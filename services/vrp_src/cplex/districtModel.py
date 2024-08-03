# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 13:45:40 2021

@author: Yiran
"""

#---------------------
# project scripts
from services.vrp_src.modelComponents import Instance, Tour
from services.vrp_src.cplex.tspModel import TSPModel
#---------------------

#---------------------
# packages
from docplex.mp.model import Model as cpx
#---------------------


class DistModel:
    
    def __init__(self, ins: Instance):
        self.ins = ins
        self.customer_dict = ins.customer_dict
        self.arc_dict = ins.arc_dict
        self._closet_customer_num = min(10, ins.n_customer-1) # default
        self.__initRankMap()

    
    def setClosetCustomerNum(self, num):
        self._closet_customer_num = num
    
    
    def __initRankMap(self):
        self.rank_dict = {} # customer rank map
        for i in self.customer_dict:
            self.rank_dict[i] = [j for j in self.customer_dict if (i,j) in self.arc_dict]
            self.rank_dict[i].sort(key=lambda x: self.arc_dict[i, x].length)
    
    
    def getServiceTeamBound(self, delta = 0.1):
        return self.ins.getServiceTeamBound(delta)
    

    def getLikelihood(self):
        # TODO: likelihood is constant throughout all nodes
        i = 1
        temp_val_list = [(1- self.customer_dict[self.rank_dict[i][j]].cancel_rate) * 
                          (self._closet_customer_num - j + 1)  \
                          for j in range(self._closet_customer_num) ]
        
        sum_val = sum(temp_val_list)
        likelihood = [temp_val_list[i]/sum_val for i in range(self._closet_customer_num)]
        
        return likelihood
    
    
    def getActivityMeasure(self):
        activity_measure = {}
        likelihood = self.getLikelihood()
        for i in self.customer_dict:
            total_t = 0
            for j in range(self._closet_customer_num):
                next_clost_customer = self.rank_dict[i][j]
                temp_t = likelihood[j] * self.arc_dict[i, next_clost_customer].travel_time
                total_t += temp_t
                
            activity_measure[i] = total_t + self.customer_dict[i].service_time
                
        return activity_measure
    
    
    def getModel(self, slots, L, s_min, s_max):
        # x[i, j] = 1 if customer [i] belongs to district [j]
        
        m = cpx('district')
        M = slots * L
        
        activity_measure = self.getActivityMeasure()
        #Miu = sum(activity_measure.values())/len(activity_measure)
                
        prod_customer = [(i, j) for i in self.customer_dict for j in self.customer_dict]
        c_range = self.customer_dict.keys()
        
        x = m.binary_var_dict(prod_customer, name="x")
                
        self.model_var = {'x': x}
        
        # i - customer index
        # j - cluster
        
        m.add_constraints_((m.sum(x[i,j] for j in c_range)== 1 for i in c_range) , 
                           names=["1-cust-1-dist_{}".format(i) for i in c_range])
        
        m.add_constraint_(m.sum(x[i, i] for i in c_range)== M, ctname="number_of_district" )
        
        
        m.add_constraints_((x[i,j] <= x[j,j] for i in c_range for j in c_range))
        
        """
        m.add_constraints_((m.sum(activity_measure[j] * x[j,i] for j in c_range) >= s_min/100*Miu*x[i,i] for i in c_range),
                           names =["size_limit_low_{}".format(i) for i in c_range])
        
        m.add_constraints_((m.sum(activity_measure[j] * x[j,i] for j in c_range) <= s_max/100*Miu*x[i,i] for i in c_range),
                           names =["size_limit_up_{}".format(i) for i in c_range])
        
        m.add_constraints_((m.sum(activity_measure[j] * x[j,i] for j in c_range) <= 10/slots for i in c_range),
                           names =["size_limit_low_{}".format(i) for i in c_range])
        """
        
        obj_coef = {(i,j): self.arc_dict[i,j].length*activity_measure[i] for i, j in self.arc_dict if i in self.customer_dict}
        obj_expr = m.sum(obj_coef[i,j] * x[i,j] for i, j in obj_coef if (i, j) in x)
        
        m.add_kpi(obj_expr) # for report only, not affect the solving procedure
        
        m.minimize(obj_expr)
        
        return m
    
    
    def solve(self, slots, L, s_min, s_max):
        m  = self.getModel(slots, L, s_min, s_max)
        sol = m.solve(log_output=False)
        x = self.model_var['x']
        
        dist_dict = {}
        for i in self.customer_dict:
            for j in self.customer_dict:
                x_val = sol.get_value(x[i,j])
                if x_val > 0.5:
                    #print((i,j))
                    if j in dist_dict:
                        dist_dict[j]['member'].append(self.customer_dict[i])
                    else:
                        dist_dict[j] = {'center': None, 'member': [self.customer_dict[i]]}
                    if i == j:
                        dist_dict[j]['center'] = self.customer_dict[i]
        
        #print(dist_dict)
        return dist_dict
    
    
    def getTour(self, node_list):
        tour = Tour('_', self.invoking_ins)
        tour.setPath(node_list)
        tour._twoOpt(max_iter= 2e6, distance_only=True)
        
        return tour
    
    # change time window - based on the stage 1 solution 
    def assignTimeSlots(self, dist_dict, working_hours, start_working_time, n_team, duration, assign_time=True):
        dist_center_list = [(i, dist_dict[i]['center']) for i in dist_dict]
        depot = self.ins.depot_start
        dist_center_list.sort(key=lambda x: x[1].distanceFrom(depot))
        
        path_dict = {i: [] for i in range(n_team)}
        slot_ind = 0
        sub_count = 0
        for dist in dist_center_list:
            path_dict[sub_count] += dist_dict[dist[0]]['member']
            sub_count += 1
            if sub_count == n_team:
                slot_ind += 1
                sub_count = 0
        
        tspM = TSPModel(self.ins)
        path_plan = {}
        for i in path_dict:
            # tour = self.getTour(path_dict[i])
            ori_path = [c.index for c in path_dict[i]]
            #print('ori: ', ori_path)
            path = tspM.solveToPath(ori_path)
            if path is None:
                return None
            #print('after: ', path)
            tour = Tour('_', self.ins)
            tour.setPath([self.customer_dict[c] for c in path])
            tour.getNodeVisitTime()
            
            if assign_time:
                # assign time windows
                for c in tour:
                    if self.ins.params.cost_idle+self.ins.params.cost_wait > 0:
                        diff = duration * (self.ins.params.cost_idle/(self.ins.params.cost_idle+self.ins.params.cost_wait))
                    else:
                        diff = duration/2
                    e_arrival_time = tour.node_visit_time[c.index] + start_working_time - tour.start_time
                    s_tw = max(start_working_time, e_arrival_time-diff)
                    c.setTimeWindow(s_tw, s_tw+duration)
            path_plan[i] = tour.path
            
        
        return path_plan

    def _assignTimeSlots(self, dist_dict, n_team, start_working_time=0, duration=2, assign_time=True):
        dist_center_list = [(i, dist_dict[i]['center']) for i in dist_dict]
        depot = self.ins.depot_start
        dist_center_list.sort(key=lambda x: x[1].distanceFrom(depot))
        
        path_dict = {i: [] for i in range(n_team)}
        slot_ind = 0
        sub_count = 0
        for dist in dist_center_list:
            path_dict[sub_count] += dist_dict[dist[0]]['member']
            sub_count += 1
            if sub_count == n_team:
                slot_ind += 1
                sub_count = 0
        
        tspM = TSPModel(self.ins)
        path_plan = {}
        for i in path_dict:
            # tour = self.getTour(path_dict[i])
            ori_path = [c.index for c in path_dict[i]]
            print('ori: ', ori_path)
            path = tspM.solveToPath(ori_path) if len(ori_path) > 1 else ori_path
            if path is None:
                return None
            print('after: ', path)
            tour = Tour('_', self.ins)
            tour.setPath([self.customer_dict[c] for c in path])
            
            if assign_time:
                # assign time windows
                tour.setTimeWindowToPathNode(duration)
            path_plan[i] = tour.path
            
        
        return path_plan
    
    
    def quickDist(self, n_team):
        dist_dict = self.solve(1, n_team, 10, 3000)
        
        return dist_dict
        
        
    
if __name__ == "__main__":
    ins = Instance(30)
    dM = DistModel(ins)
    dist_dict = dM.solve(1, 6, 100, 1500)
    dM.assignTimeSlots(dist_dict, 8, 8, 6, 2)
  