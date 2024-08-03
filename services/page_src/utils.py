# -*- coding: utf-8 -*-
"""
Created on Tue May 11 15:53:39 2021

@author: Yiran
"""

def time_convertor(t):
    print(f"check: {t}")
    if not t:
        return '-'
    
    if type(t) != str and type(t) != tuple:
        return "{0:02d}:{1:02d}".format(int(t), int(60*(abs(t)%1)))
        
    st = float(t[0])
    et = float(t[1])
    
    o_st = "{0:02d}:{1:02d}".format(int(st), int(60*(abs(st)%1)))
    o_et = "{0:02d}:{1:02d}".format(int(et), int(60*(abs(et)%1)))
    
    return o_st + ' - ' + o_et



def path_plan_to_list(path_plan, ins):
    path_list = list(path_plan.values())
    path_list = [[ins.depot_start] + p + [ins.depot_end] for p in path_list]
    
    return path_list