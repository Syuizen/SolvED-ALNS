# -*- coding: utf-8 -*-
"""
Created on Tue May  4 02:00:00 2021

@author: Yiran
"""

# project script ----------
from services.vrp_src.modelComponents import Instance
# --------------------------

# package ------------------
from itertools import combinations
from docplex.mp import model as cpx
from cplex.callbacks import LazyConstraintCallback
from docplex.mp.callbacks.cb_mixin import ConstraintCallbackMixin
# --------------------------



class SecCallback(ConstraintCallbackMixin, LazyConstraintCallback):
    
    
    def __init__(self, env):
        ConstraintCallbackMixin.__init__(self)
        LazyConstraintCallback.__init__(self, env)
    
    
    def __call__(self):
        x = self.x
        sol ={ i: self.get_values(x[i].name) for i in x}
        selected = [i for i in sol if sol[i] > 0.5]
        # find the shortest cycle in the selected edge list
        tour = self.subtour(selected, self.node)

        if len(tour) < self.n:
            # add subtour elimination constr. for every pair of cities in tour
            expr = x[tour[-1], tour[0]]
            tour_n = len(tour)
            for i in range(1, tour_n):
                expr += x[tour[i-1], tour[i]]
            ct = expr <= len(tour)-1
            unsats = self.get_cpx_unsatisfied_cts([ct], self.make_complete_solution(), tolerance=1e-6)
            for ct, cpx_lhs, sense, cpx_rhs in unsats:
                self.add(cpx_lhs, sense, cpx_rhs)
            
    
    def subtour(self, edges, node_list):
        unvisited = node_list.copy()
        cycle = node_list.copy() + [node_list[0]]  # initial length has 1 more city
        while unvisited:  # true if list is non-empty
            thiscycle = []
            neighbors = unvisited
            while neighbors:
                current = neighbors[0]
                thiscycle.append(current)
                unvisited.remove(current)
                neighbors = [j for i, j in edges
                             if j in unvisited and i == current]
            if len(cycle) > len(thiscycle):
                cycle = thiscycle
        return cycle


class TSPModel:
    
    
    def __init__(self, ins: Instance):
        self.invoking_ins = ins
        self.arc_dict = ins.arc_dict


    def subtour(self, edges, node_list):
        unvisited = node_list.copy()
        cycle = node_list.copy() + [node_list[0]]  # initial length has 1 more city
        while unvisited:  # true if list is non-empty
            thiscycle = []
            neighbors = unvisited
            while neighbors:
                current = neighbors[0]
                thiscycle.append(current)
                unvisited.remove(current)
                neighbors = [j for i, j in edges
                             if j in unvisited and i == current]
            if len(cycle) > len(thiscycle):
                cycle = thiscycle
        return cycle    
    
    
    def getModel(self, active_node_index_list = None):
        
        ins = self.invoking_ins
        if active_node_index_list is None:
            active_node_index_list = ins.getActiveNodeIndexList()
        
        while ins.depot_start.index in active_node_index_list:
            active_node_index_list.remove(ins.depot_start.index)
        
        while ins.depot_end.index in active_node_index_list:
            active_node_index_list.remove(ins.depot_end.index)
        
        active_node_index_list.insert(0, ins.depot_start.index)
        self._N_NODES =  len(active_node_index_list)
        self.node_list = active_node_index_list.copy()
        arc_list =  [(i, j) for i,j in self.arc_dict.keys() if i in active_node_index_list and 
                                                             j in active_node_index_list 
                                                             and ( (i == ins.depot_start.index and i!=j) or
                                                                i > j)
                                                                ]
        
        dist_dict = {a : self.arc_dict[a].length for a in arc_list}
        m = cpx.Model('tsp')
        x = m.binary_var_dict(arc_list, name="x")
        all_key = set(x.keys())
        for i, j in all_key:
            x[j, i] = x[i, j]
        
        m.add_constraints((m.sum(x[i,j] for i in active_node_index_list if i != j) == 2 for j in active_node_index_list),
                              names=["1-in-1-out:({0})".format(i) for i in active_node_index_list])
        
        obj_expr = m.sum([dist_dict[i,j] * x[i,j] for i, j in arc_list])
        
        m.minimize(obj_expr)
        
        self.model_var = {'x': x}
        cb = m.register_callback(SecCallback)
        cb.n = len(active_node_index_list)
        cb.x = x
        cb.node = active_node_index_list.copy()
        
        return m

    
    def solve(self, active_node_index_list=None):
        #print("tsp: node list:", active_node_index_list)
        m = self.getModel(active_node_index_list)
        self.model = m
        
        sol = m.solve(log_output=False)
        if sol is None:
            return None
        
        tour = None
        x = self.model_var['x']
        vals = {(i,j): sol.get_value(x[i,j]) for i,j in x}
        selected = [(i, j) for i, j in vals.keys() if vals[i, j] > 0.5]
        tour = self.subtour(selected, self.node_list.copy())
        
        return tour
    
    
    def solveToPath(self, active_node_index_list=None):
        tour = self.solve(active_node_index_list)
        if tour is None:
            return None
        
        if self.invoking_ins.depot_start.index == tour[0]:
            tour.remove(self.invoking_ins.depot_start.index)
        else:
            raise Exception("error in TSP")
        
        return tour
    
    
if __name__ == "__main__":
    ins = Instance(5)
    tspM = TSPModel(ins)
    sol = tspM.solve()