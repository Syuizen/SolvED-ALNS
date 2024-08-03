# -*- coding: utf-8 -*-
"""
Created on Tue May 18 01:55:23 2021

@author: Yiran
"""

# this script is built for numba-optimizable ALNS
# replace bottleneck update function in ALNS

from numba.types import Array, float64
import numba
import warnings
import numpy as np

warnings.simplefilter('ignore', category=numba.errors.NumbaDeprecationWarning)
warnings.simplefilter('ignore', category=numba.errors.NumbaPendingDeprecationWarning)

def makeMyUpdateTW(
                 full_node_time,
                 full_arc_time,
                 full_arc_length
                 ):
    
    N = len(full_node_time)
    
    @numba.njit(fastmath=True)
    def getTourStat(tour, full_node_start_t, full_node_end_t):
        
        # INPUT FORMAT:
        #   M : size of tour
        #   have_tw: if contains time_window
        #   tour (including 2 depots)
        #   The first and last node in tour should both be depots
        #   N = len(full_node_time) -- > # node in graph (including exactly 1 depot)
        #   index of arc[i, j] in list --> i*N + j
        
        # WARINING: we sepearte depot into two nodes depot_start and depot_end artificially for modelling purpose
        #   However, this function should NOT include the extra copy of depot
        #   In particular, depot index = 0 is reserved
        
        # return the following statistics as a single list
        # 1. length
        # 2. travel_time
        # 3. service_time
        # 4. workload
        # 5. worker_idle_time
        # 6. customer_wait_time
        # 7. start_time
        # 8. arrival_time_of_node --> size is N (NOT the size of tour)
        #result = Array(float64, 7+N, "C")
        
        current_ind = tour[0] # first customer_node & 0 is the depot
        
        length = full_arc_length[current_ind] # depot -> 1st node
    
        node_tw_start = full_node_start_t[current_ind]
        node_tw_end = full_node_end_t[current_ind]

        #mid_time = (node_tw_start + node_tw_end)/2.0 # arrival time at the 1st node     
        #start_time = mid_time - full_arc_time[current_ind] # eliminate time starting from depot to the 1st node
        #start_time = 0 if start_time < 0 else start_time
        
        start_time = 0.0
        arrival_time = start_time + full_arc_time[current_ind]

        #result[7 + current_ind] = arrival_time # record arrival time
        
        total_travel_time = full_arc_time[current_ind]
        
        #we always arrive on time at the first node
        worker_idle_time = 0.0
        customer_wait_time = 0.0
    
        # start serving
        arrival_time += full_node_time[current_ind]
        total_service_time = full_node_time[current_ind]

        prev_ind = current_ind
        for i in range(1, len(tour)):
            current_ind = tour[i]
            arc_ind = prev_ind * N + current_ind
            # leave to next node
            length += full_arc_length[arc_ind]
            arrival_time += full_arc_time[arc_ind]
            total_travel_time += full_arc_time[arc_ind]
            
            #result[7 + current_ind] = arrival_time # record arrival time
            
            if current_ind > 0:
                node_tw_start = full_node_start_t[current_ind]
                node_tw_end = full_node_end_t[current_ind]
                
                if node_tw_start > arrival_time:
                    # idle
                    time_gap = node_tw_start - arrival_time
                    worker_idle_time += time_gap
                    # team wait till start service time
                    arrival_time += time_gap
                    
                elif arrival_time > node_tw_end:
                    # wait
                    time_gap = arrival_time - node_tw_end
                    customer_wait_time += time_gap
            
            # serve at node
            arrival_time += full_node_time[current_ind]
            total_service_time += full_node_time[current_ind]
            
            # ready to move to next node
            prev_ind = current_ind
        
        # back to depot
        arc_ind = prev_ind * N
        length += full_arc_length[arc_ind]
        arrival_time += full_arc_time[arc_ind]
        total_travel_time += full_arc_time[arc_ind]

        # travel finished

        workload = arrival_time - start_time
        
        #prepare return value
        
        return (length,
                total_travel_time,
                total_service_time,
                workload,
                worker_idle_time,
                customer_wait_time,
                start_time)
    
    return getTourStat



def makeAllSceUpdate(full_sce_node_time,
                 full_sce_arc_time,
                 full_arc_length,
                 full_sce_prob,
                 workload_allowed,
                 max_workload):

    N = len(full_sce_node_time[0])
    M = len(full_sce_prob)

    @numba.njit(fastmath=True)
    def getTourStat(tour, full_node_start_t, full_node_end_t):
        t_length=0
        t_total_travel_time=0
        t_total_service_time=0
        t_workload=0
        t_worker_idle_time =0
        t_customer_wait_time = 0
        t_start_time = 0

        t_worst_workload = 0
        t_overtime = 0
        tour_n = len(tour)
        t_infeas = tour_n

        for s in range(M):
            current_ind = tour[0] # first customer_node & 0 is the depot
            
            length = full_arc_length[current_ind] # depot -> 1st node
        
            node_tw_start = full_node_start_t[current_ind]
            node_tw_end = full_node_end_t[current_ind]

            #mid_time = (node_tw_start + node_tw_end)/2.0 # arrival time at the 1st node     
            #start_time = mid_time - full_arc_time[current_ind] # eliminate time starting from depot to the 1st node
            #start_time = 0 if start_time < 0 else start_time
            
            start_time = 0.0
            arrival_time = start_time + full_sce_arc_time[s, current_ind]

            #result[7 + current_ind] = arrival_time # record arrival time
            
            total_travel_time = full_sce_arc_time[s, current_ind]
            
            #we always arrive on time at the first node
            worker_idle_time = 0.0
            customer_wait_time = 0.0
        
            # start serving
            arrival_time += full_sce_node_time[s, current_ind]
            total_service_time = full_sce_node_time[s, current_ind]

            prev_ind = current_ind
            for i in range(1, tour_n):
                current_ind = tour[i]
                arc_ind = prev_ind * N + current_ind
                # leave to next node
                length += full_arc_length[arc_ind]
                arrival_time += full_sce_arc_time[s, arc_ind]
                total_travel_time += full_sce_arc_time[s, arc_ind]
                
                #result[7 + current_ind] = arrival_time # record arrival time
                
                if current_ind > 0:
                    node_tw_start = full_node_start_t[current_ind]
                    node_tw_end = full_node_end_t[current_ind]
                    
                    if node_tw_start > arrival_time:
                        # idle
                        time_gap = node_tw_start - arrival_time
                        worker_idle_time += time_gap
                        # team wait till start service time
                        arrival_time += time_gap
                        
                    elif arrival_time > node_tw_end:
                        # wait
                        time_gap = arrival_time - node_tw_end
                        customer_wait_time += time_gap
                
                # serve at node
                arrival_time += full_sce_node_time[s, current_ind]
                total_service_time += full_sce_node_time[s, current_ind]
                
                # ready to move to next node
                prev_ind = current_ind

                # check if the tour is feasible
                back_to_depot_time = arrival_time + full_sce_arc_time[s, current_ind * N]
                if back_to_depot_time > max_workload and t_infeas > i:
                    t_infeas = i
            
            # back to depot
            arc_ind = prev_ind * N
            length += full_arc_length[arc_ind]
            arrival_time += full_sce_arc_time[s, arc_ind]
            total_travel_time += full_sce_arc_time[s, arc_ind]

            # travel finished

            workload = arrival_time - start_time

            # add to total value
            prob = full_sce_prob[s]
            t_length += length * prob
            t_total_travel_time += total_travel_time * prob
            t_total_service_time += total_service_time * prob
            t_workload += workload * prob
            t_worker_idle_time += worker_idle_time * prob
            t_customer_wait_time += customer_wait_time * prob
            t_start_time += start_time * prob

            if workload > t_worst_workload:
                t_worst_workload = workload

            if workload > workload_allowed:
                t_overtime += (workload - workload_allowed) * prob

        #prepare return value
        
        return (t_length,
                t_total_travel_time,
                t_total_service_time,
                t_workload,
                t_worker_idle_time,
                t_customer_wait_time,
                t_start_time,
                t_overtime,
                t_worst_workload,
                t_infeas)
    

    return getTourStat

if __name__ == "__main__":
    pass