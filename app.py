# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 01:36:21 2021

@author: Yiran
"""

import os
from flask import Flask, render_template, request, session, redirect, url_for

from services.vrp_src.heuristic.alns import ALNS
from services.vrp_src.modelComponents import Instance, Tour
from services.vrp_src.cplex.districtModel import DistModel
from services.vrp_src.cplex.modelBuilder import ModelBuilder

import pandas as pd

import services.page_src.mapbuilder as fMap
import services.page_src.utils as fUtils

app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = 'uploads'
app.secret_key = "aSecretKey"
app.debug = True


@app.route('/', methods=['POST', 'GET'])
def basePage():
    
    map_div, hdr_txt, script_txt = fMap.render_map()
    
    if 'wrong_file_uploaded' in session and session['wrong_file_uploaded']:
        displayalert = True
    else:
        displayalert = False
        
    return render_template('base.html', 
                           displayalert = displayalert,
                           map_div=map_div, 
                           hdr_txt=hdr_txt, 
                           script_txt=script_txt)


@app.route('/upload', methods=['POST'])
def upload_file():
    # accept csv only
    uploaded_file = request.files['file']
    print(uploaded_file.filename)
    if uploaded_file.filename != '':
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], uploaded_file.filename)
        filetype = uploaded_file.filename.split('.')[-1]
        if filetype == "csv":
            data = pd.read_csv(filepath)
        elif filetype == "xlsx":
            data = pd.read_excel(filepath)
        else:
            session['wrong_file_uploaded'] = True
            return redirect(url_for('basePage'))
        
        #col check
        for col in ['name', 'x', 'y']:
            if col not in data.columns:
                session['wrong_file_uploaded'] = True
                return redirect(url_for('basePage'))
        
        n = len(data)
        ins = Instance()
        for i in range(n):
            row = data.loc[i]
            ins.addCustomer(row['x'].item(), row['y'].item(), str(row['name'].item()))
        ins.build()
        customer_list = [i for i in ins.customer_dict.values()]
        session['ins_var'] = [(i.x_loc, i.y_loc) for i in customer_list]
        map_div, hdr_txt, script_txt = fMap.render_map(customer_list)
        lb, ub = ins.getServiceTeamBound()
        session['min_team'] = lb
        session['max_team'] = ub
        params =  {
        "workload": ins.params.max_workload.value,
        "cancel_rate": ins.params.cancel_rate.value,
        "cost_wait": ins.params.cost_wait.value,
        "cost_idle": ins.params.cost_idle.value,
        "cost_overtime": ins.params.cost_overtime.value,
        "cost_travel": ins.params.cost_travel.value,
        "cost_hire": ins.params.cost_hire.value
        }
        session['n_customer'] = ins.n_customer
        session['params'] = params
        session['ins_var'] = [{'name': i.name, 
                           'loc': i.loc} for i in customer_list]
        return render_template('nodeMapPre.html', params=params,
                                               n_customer=ins.n_customer,
                                               map_div=map_div, 
                                               hdr_txt=hdr_txt, 
                                               script_txt=script_txt)

    return redirect(url_for('basePage'))


@app.route('/nodeMapPreSetting', methods=['POST'])
def nodeMapPrePage():
    n_customer = int(request.form['n_customer'])
    ins = Instance(n_customer)
    customer_list = [i for i in ins.customer_dict.values()]
    params =  {
        "workload": ins.params.max_workload.value,
        "cancel_rate": ins.params.cancel_rate.value,
        "cost_wait": ins.params.cost_wait.value,
        "cost_idle": ins.params.cost_idle.value,
        "cost_overtime": ins.params.cost_overtime.value,
        "cost_travel": ins.params.cost_travel_time.value,
        "cost_hire": ins.params.cost_hire.value
        }
    session['n_customer'] = n_customer
    session['params'] = params
    customer_list = [i for i in ins.customer_dict.values()]
    session['ins_var'] = [{'name': i.name,
                           'loc': i.loc} for i in customer_list]
    map_div, hdr_txt, script_txt = fMap.render_map(customer_list)
    return render_template('nodeMapPre.html', params=params,
                                               n_customer=n_customer,
                                               map_div=map_div, 
                                               hdr_txt=hdr_txt, 
                                               script_txt=script_txt)


@app.route('/distMap')
def distMapPage():
    n_team = int(request.args.get('n_team'))
    n_customer = int(request.args.get('n_customer'))
    tw_width = float(request.args.get('time_window_width'))
    tw_width_2 = float(request.args.get('time_window_width_2'))
    
    print("------- Dist Map ------\n")
    
    print(tw_width)
    customer_list = session.get('ins_var')
    params = session.get('parmas')
    print(params)
    ins = Instance()
    ins.setting(
        cancel_rate = params['cancel_rate'],
        max_workload = params['workload'],
        strict_max_workload = 10,
        cost_wait_time = params['cost_wait'], # per hour
        cost_idle_time = params['cost_idle'], # per hour
        cost_work_overtime = params['cost_overtime'], # per hour
        cost_travel_time = params['cost_travel'], 
        cost_hire_per_team = params['cost_hire']
        )
    print(ins.params)
    for c in customer_list:
        c_loc = c['loc']
        ins.addCustomer(c_loc[0], c_loc[1], c['name'])
    ins.build()
    for arci in ins.arc_dict:
        arc = ins.arc_dict[arci]
    
    ins.formalizeData()
    dM = DistModel(ins)
    dist_dict = dM.quickDist(n_team)
    #path_plan = dM.assignTimeSlots(dist_dict, ins.params.max_workload, 8, n_team, tw_width)
    #
    path_plan = dM.assignTimeSlots(dist_dict, working_hours=8, start_working_time=8, 
                                           n_team=n_team, duration=tw_width, assign_time=False)
    
    print("--- enter ALNS ---")
    alns = ALNS(ins)
    if path_plan is None:
        alns.initSolution()
    else:
        alns.setPathPlan(path_plan.copy())
    alns.solve(max(1, round(0.1*n_customer)), max_iter=1000)
    print(alns)
    alns.freezeVehicles()
    for tour in alns.tour_dict.values():
        tour.start_time = 8
        tour.getNodeVisitTime()
        print(tour.node_visit_time)
        tour.setTimeWindowToPathNode(tw_width, False)
        
    path_plan = alns.exportPathPlan()
    node_to_dist = {}
    for i in dist_dict:
        for c in dist_dict[i]['member']:
            node_to_dist[c] = i
    
    customer_list = list(ins.customer_dict.values())
    to_save_list = [{'name': i.name,
                     'id': i.index,
                     'tw': i.time_window,
                     'active': i.is_active,
                     'loc': i.loc, 
                     'dist': node_to_dist[i]} for i in customer_list]
    session['ins_var'] = to_save_list
    to_save_path_plan = {i: [j.index for j in path_plan[i]] for i in path_plan}
    session['path_plan'] = to_save_path_plan
    
    #print(to_save_list)
    map_div, hdr_txt, script_txt = fMap.render_map(customer_list, dist_dict=dist_dict)
    
    return render_template('distMap.html', n_customer=n_customer,
                                           n_team=n_team,
                                           tw_width=tw_width,
                                           tw_width_2=tw_width_2,
                                           map_div=map_div, 
                                           hdr_txt=hdr_txt, 
                                           script_txt=script_txt)


@app.route('/tableMap', methods=['POST'])
def tableMapPage():
    #print(request.form)
    print("----- table Map ----- \n")
    n_team = int(request.form['n_team'])
    n_customer = int(request.form['n_customer'])
    tw_width = float(request.form['time_window_width'])
    tw_width_2 = float(request.form['time_window_width_2'])
    select_method = int(request.form['select_method'])
    customer_list = session.get('ins_var')
    session['tw_width_2'] = tw_width_2
    dist_dict = {}
    params = session.get('parmas')
    ins = Instance(default_setting=False)
    ins.setting(
        cancel_rate = params['cancel_rate'],
        max_workload = params['workload'],
        strict_max_workload = 10,
        cost_wait_time = params['cost_wait'], # per hour
        cost_idle_time = params['cost_idle'], # per hour
        cost_work_overtime = params['cost_overtime'], # per hour
        cost_travel_time = params['cost_travel'], 
        cost_hire_per_team = params['cost_hire']
        )
    for c in customer_list:
        c_loc = c['loc']
        c_node = ins.addCustomer(c_loc[0], c_loc[1], c['name'], c['id'])
        c_node.setTimeWindow(c['tw'][0], c['tw'][1])
        c_node._old_tw = c['tw']
        if c['dist'] in dist_dict:
            dist_dict[c['dist']]['member'].append(c_node)
        else:
            dist_dict[c['dist']] ={'center': c_node, 'member': [c_node]}
    ins.build()
    ins.formalizeData()
    node_to_dist = {}
    for i in dist_dict:
        for c in dist_dict[i]['member']:
            node_to_dist[c] = i
    customer_list = [i for i in ins.customer_dict.values()]
    if "reschedule" in request.form:
        forbid_cust_list = [i for i in ins.customer_dict.values() if i.name not in request.form ]
    else:
        forbid_cust_list = []
    
    for c in forbid_cust_list:
        c.setActiveStatus(False)
    
    if select_method == 2:
        #heuristic
        alns = ALNS(ins)
        dM = DistModel(ins)
        if True or "reschedule" in request.form:
            path_plan_id = session.get('path_plan')
            alns = ALNS(ins)
            ins.formalizeData()
            for i in path_plan_id:
                c_list = [ins.customer_dict[j] for j in path_plan_id[i] if j in ins.customer_dict if ins.customer_dict[j].is_active]
                tour = alns.createNewTourByPath(c_list, i)
        #else:
        #    path_plan = dM.assignTimeSlots(dist_dict, ins.params.max_workload, 8, n_team, tw_width, False)
        #    if path_plan is None:
        #        alns.initSolution()
        #    else:
        #        alns.setPathPlan(path_plan)
        alns.freezeVehicles()
        print('---- input ----\n')
        print(alns)
        alns.solve(max(1, round(0.1*len(alns.customer_pool))), 20*n_customer)
        print('---- after ALNS --- \n')
        print(alns)
        path_plan = alns.exportPathPlan()
        tour_list = [tour for tour in alns.tour_dict.values()]
    else:
        #exact
        mb = ModelBuilder(ins)
        mb.buildVRP(n_team=n_team, time_window_given=True, uniform_start_time=8)
        mb.getSolution()
        tour_list = [tour for tour in mb.tour_dict.values()]
        path_plan = {i: mb.tour_dict[i].path.copy() for i in mb.tour_dict}
        
    path_list = fUtils.path_plan_to_list(path_plan, ins)
    cust_to_tour = {}
    for i in path_plan:
        for c in path_plan[i]:
            cust_to_tour[c] = i
    
    if "reschedule" in request.form:
        for tour in tour_list:
            tour.getNodeVisitTime()
            tour.setTimeWindowToPathNode(tw_width_2, refine=True)
    
    to_save_path_plan = {i: [j.index for j in path_plan[i]] for i in path_plan}
    session['path_plan'] = to_save_path_plan
    
    map_div, hdr_txt, script_txt = fMap.render_map(customer_list, 
                                                   dist_dict=dist_dict,
                                                   path_list=path_list,
                                                   forbid_node_list=forbid_cust_list,
                                                   cust_to_tour=cust_to_tour)
    
    table = {'header': ['name', 'team', 'location', 'time slot']}
    table['data'] = []
    for c in ins.customer_dict.values():            
        table['data'].append({ 'name': c.name,
            'val':[c.name, 
                   cust_to_tour[c] if c in cust_to_tour else 'cancelled', 
                   c.loc, 
                   fUtils.time_convertor(c.time_window)],
            'disable': False if (c.name in request.form and 'reschedule' in request.form)
                                or 'reschedule' not in request.form else True
            })
    
    
    to_save_list = [{'name': i.name,
                     'id': i.index,
                     'active': i.is_active,
                     'tw': i._old_tw,
                     'e_tw': i.time_window,
                     'dist': node_to_dist[i],
                     'loc': i.loc} for i in customer_list]
    session['ins_var'] = to_save_list
    
    #print(table)
    
    return render_template('tableMap.html', table=table,
                                           n_customer=n_customer,
                                           n_team=n_team,
                                           tw_width=tw_width,
                                           tw_width_2=tw_width_2,
                                           select_method=select_method,
                                           map_div=map_div, 
                                           hdr_txt=hdr_txt, 
                                           script_txt=script_txt)


@app.route('/evaluateMap', methods=['POST'])
def evaluateMapPage():
    tw_width_2 = session.get('tw_width_2')
    customer_list = session.get('ins_var')
    params = session.get('parmas')
    ins = Instance(default_setting=False)
    ins.setting(
        cancel_rate = params['cancel_rate'],
        max_workload = params['workload'],
        strict_max_workload = 10,
        cost_wait_time = params['cost_wait'], # per hour
        cost_idle_time = params['cost_idle'], # per hour
        cost_work_overtime = params['cost_overtime'], # per hour
        cost_travel_per_km = params['cost_travel'], 
        cost_hire_per_team = params['cost_hire']
        )
    for c in customer_list:
        c_loc = c['loc']
        c_node = ins.addCustomer(c_loc[0], c_loc[1], c['name'], c['id'])
        c_node.setActiveStatus(c['active'])
        c_node.setTimeWindow(c['e_tw'][0], c['e_tw'][1])
    ins.build()
    
    forbid_cust_list = [i for i in ins.customer_dict.values() if not i.is_active ]
   
    ins.formalizeData()
    path_plan_id = session.get('path_plan')
    #print(path_plan_id)
    alns = ALNS(ins)
    for i in path_plan_id:
        c_list = [ins.customer_dict[j] for j in path_plan_id[i] if j in ins.customer_dict and ins.customer_dict[j].is_active]
        tour = alns.createNewTourByPath(c_list, i)
        print(tour)
        #tour.setTimeWindowToPathNode(tw_width_2)
    
    
    # create table
    ins._benchmarkBuild(300, 3000, tour_dict=alns.tour_dict)
    # ins.formalizeData()
    print('--- benckmark build succeed ---')
    tour_info = {tour: tour._benckmark() for tour in alns.getListOfTours()}
    atable = {'data': []}
    atable['header'] = ['name', 'team', 'location', 'time slot', 'service level', 'arrival time', 'idle time', 'wait time']
    for i in ins.customer_dict.values():
        if i.is_active:
            tour = alns._getTourHavingNode(i)
            info = tour_info[tour]
            visit_time = fUtils.time_convertor(info['arrival_time'][i.index])
            delay_time = float(i.getDelayTime(info['arrival_time'][i.index]))
            service_level = "{:.2f}%".format(100*info['service_status'][i.index])
            if service_level == "100.00%":
                service_level = "99.99%"
            atable['data'].append(
                 {'val': 
                  [i.name, tour.name, i.loc, fUtils.time_convertor(i.time_window), service_level, visit_time] +\
                (['-', fUtils.time_convertor(delay_time)] if delay_time > 0 else [fUtils.time_convertor(abs(delay_time)), '-'])}
                )
        else:
            atable['data'].append(
                {'val':
                 [i.name, 'cancelled', i.loc, '-', '-', '-', '-', '-']
                    }
                )
    
    team_table = {'data': []}
    team_table['header'] = ['team', 'depart', 'return', 'overtime']
    for tour in alns.getListOfTours():
        info = tour_info[tour]
        over_time = max(0, info['workload'] - ins.params.max_workload)
        team_table['data'].append(
            {'val': [tour.name, fUtils.time_convertor(tour.start_time), fUtils.time_convertor(info['end_time']),  fUtils.time_convertor(over_time)]}
            )
    
    path_plan = alns.exportPathPlan()
    path_list = fUtils.path_plan_to_list(path_plan, ins)
    cust_to_tour = {}
    for i in path_plan:
        for c in path_plan[i]:
            cust_to_tour[c] = i
    
    print('--- finish ---')
    
    customer_list = ins.customer_dict.values()
    map_div, hdr_txt, script_txt = fMap.render_map(customer_list, 
                                                   path_list=path_list,
                                                   forbid_node_list=forbid_cust_list,
                                                   cust_to_tour=cust_to_tour)
    
    return render_template('evalMap.html', atable=atable,
                                           team_table=team_table,
                                           tw_width_2=tw_width_2,
                                           map_div=map_div, 
                                           hdr_txt=hdr_txt, 
                                           script_txt=script_txt)
    
    

@app.route('/nodeMap', methods=['GET', 'POST'])
def nodeMapPage():
    #print(request.form)
    params =  {
        "workload": float(request.form['workload']),
        "cancel_rate": float(request.form['cancel_rate']),
        "cost_wait": float(request.form['cost_wait']),
        "cost_idle": float(request.form['cost_idle']),
        "cost_overtime": float(request.form['cost_overtime']),
        "cost_travel": float(request.form['cost_travel']),
        "cost_hire": float(request.form['cost_hire'])
    }
    session['parmas'] = params
    print('----- Node Map -----\n')
    print(params)
    customer_list = session.get('ins_var')
    ins = Instance(default_setting=False)
    ins.setting(
        cancel_rate = params['cancel_rate'],
        max_workload = params['workload'],
        strict_max_workload = 10,
        cost_wait_time = params['cost_wait'], # per hour
        cost_idle_time = params['cost_idle'], # per hour
        cost_work_overtime = params['cost_overtime'], # per hour
        cost_travel_per_km = params['cost_travel'], 
        cost_hire_per_team = params['cost_hire']
        )
    for c in customer_list:
        c_loc = c['loc']
        ins.addCustomer(c_loc[0], c_loc[1], c['name'])
    ins.build()
    customer_list = [i for i in ins.customer_dict.values()]
    map_div, hdr_txt, script_txt = fMap.render_map(customer_list)
    lb, ub = ins.getServiceTeamBound()
    mb = ModelBuilder(ins)
    n_team_log = mb.searchBestNTeams()
    n_team = min(n_team_log, key=lambda x: x[1])[0]
    session['min_team'] = lb
    session['max_team'] = ub
    session['recom_team'] = n_team
    
    n_customer = int(request.form['n_customer'])
    
    return render_template('nodeMap.html', n_customer=n_customer,
                                           recommend_team=n_team,
                                           min_team=lb,
                                           max_team=ub,
                                           map_div=map_div, 
                                           hdr_txt=hdr_txt, 
                                           script_txt=script_txt)


@app.route('/planMap')
def planMapPage():
    n_customer = int(request.args.get('n_customer'))
    customer_loc_list = session.get('ins_var')
    ins = Instance()
    for c_loc in customer_loc_list:
        ins.addCustomer(c_loc[0], c_loc[1])
    ins.build()
    
    alns = ALNS(ins)
    alns.initSolution()
    alns.solve(3, 100)
    #alns.tourOverlapBreaker()
    path_list = list(alns.exportPathPlan().values())
    path_list = [[ins.depot_start] + p + [ins.depot_end] for p in path_list] 
    customer_list = [i for i in ins.customer_dict.values()]
    map_div, hdr_txt, script_txt = fMap.render_map(customer_list, path_list)
    
    return render_template('nodeMap.html', n_customer=n_customer,
                                           min_team=session['min_team'],
                                           max_team=session['max_team'],
                                           map_div=map_div, 
                                           hdr_txt=hdr_txt, 
                                           script_txt=script_txt)
