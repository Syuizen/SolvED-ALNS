# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 20:03:37 2021

@author: Yiran
"""

import folium
from flask import Markup

import math
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import matplotlib.cm as mplcm
import matplotlib.colors as colors

LEHIGH_UNIVERSITY_COOR = (40.606911, -75.378281)

ICON_COLOR_LIST = ['red', 'blue', 'green', 'purple', 'orange', 'beige', 'darkblue', 'darkgreen', 'cadetblue', 
                  'darkpurple', 'white', 'pink', 'lightblue', 'lightgreen',
                  'gray', 'black']

FORBIDDEN_COLOR =  'lightgray'

def get_N_distinct_color(N):
    cm = plt.get_cmap('gist_rainbow')
    cNorm  = colors.Normalize(vmin=0, vmax=N-1)
    scalarMap = mplcm.ScalarMappable(norm=cNorm, cmap=cm)
    hex_color = [colors.rgb2hex(scalarMap.to_rgba(i)) for i in range(N)]
    return hex_color


def convert_loc_to_geoloc(x_loc, y_loc, geo_center=LEHIGH_UNIVERSITY_COOR):
    R = 6378.1 #Radius of the Earth
    d = norm([x_loc, y_loc]) # Euclidean distance
    brng = math.acos(y_loc/d) if d != 0 else 0 #bearing in radians
    brng = (2 * np.pi - brng) if x_loc < 0 else brng


    c_lat, c_lon = geo_center
    c_rlat = math.radians(c_lat) #Current lat point converted to radians
    c_rlon = math.radians(c_lon) #Current long point converted to radians

    g_rlat = math.asin( math.sin(c_rlat) * math.cos(d/R) +
         math.cos(c_rlat) * math.sin(d/R) * math.cos(brng))

    g_rlon = c_rlon + math.atan2(math.sin(brng) * math.sin(d/R) * math.cos(c_rlat),
                               math.cos(d/R) - math.sin(c_rlat) * math.sin(g_rlat))

    g_lat = math.degrees(g_rlat)
    g_lon = math.degrees(g_rlon)

    return g_lat, g_lon


def render_map(node_list = [], 
               path_list = [], 
               dist_dict={}, 
               forbid_node_list=[],
               cust_to_tour={}):
    # generate a map with node and path displayed

    # map center: lehigh university campus
    m = folium.Map(location=LEHIGH_UNIVERSITY_COOR, zoom_start=10)

    folium.Marker(LEHIGH_UNIVERSITY_COOR,
                  icon  = folium.Icon(color="red", icon="info-sign"),
                  popup = "Lehigh University"
                  ).add_to(m)

    node_to_color = {i.index: ICON_COLOR_LIST[0] for i in node_list}
    color_ct = 1
    for i in dist_dict:
        color = ICON_COLOR_LIST[color_ct%len(ICON_COLOR_LIST)]
        node_s = dist_dict[i]['member']
        for c in node_s:
            node_to_color[c.index] = color
        color_ct += 1
    
    # if path_list is included, then use assign color to each path
    if path_list:
        path_to_color = []
    else:
        path_to_color = get_N_distinct_color(len(path_list))
    
    color_ct = 1
    for path in path_list:
        color = ICON_COLOR_LIST[color_ct%len(ICON_COLOR_LIST)]
        path_to_color.append(color)
        for c in path:
            node_to_color[c.index] = color
        color_ct += 1
        
    # add marks
    for node in node_list:
        #print(node.name, 'color: ', node_to_color[node.index])
        folium.Marker(
            convert_loc_to_geoloc(node.x_loc, node.y_loc),
            icon=folium.Icon(icon = 'home', 
                             prefix='fa',
                             color = node_to_color[node.index] if node not in forbid_node_list else FORBIDDEN_COLOR),
            popup = node.name,
        ).add_to(m)


    # add paths
    n_color_path = len(path_list)
    if n_color_path:
        color_list = path_to_color
        count = 0
        for path in path_list:
            g_arc = [convert_loc_to_geoloc(i.x_loc, i.y_loc) for i in path]
            folium.PolyLine(g_arc, 
                            color=color_list[count],
                            popup= cust_to_tour[path[1]] if path[1] in cust_to_tour else None).add_to(m)
            count += 1

    _ = m._repr_html_()

    # get definition of map in body
    map_div = Markup(m.get_root().html.render())

    # html to be included in header
    hdr_txt = Markup(m.get_root().header.render())

    # html to be included in <script>
    script_txt = Markup(m.get_root().script.render())

    return map_div, hdr_txt, script_txt
