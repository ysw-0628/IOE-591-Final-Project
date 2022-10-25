#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 15 20:47:07 2021

@author: yushuangwai
"""

import random as rand
import gurobipy as gp
from gurobipy import GRB
import numpy as np

data = np.load('/Users/yushuangwai/Downloads/sioux_falls_distance.npy')

small_data = data[0:13,0:13]
m = gp.Model("single_depot")

# Set variables

#L = list of nodes
L = []
for i in range(len(small_data)):
    L.append('%s' % i)
                 
#L_c = list of customers
L_c = L[2:]

#L_d = list of depots
L_d = L[:2]
#arcs, costs = gp.multidict({('i,j'): cost of traveling from i to j})
arcs, costs = gp.multidict({('%s %g' % (i,j)): data[i,j] for i in range(len(small_data)) for j in range(len(small_data[i]))})

#orders, products, quantity = gp.multidict({nodes in L_c}:[products,quantity])
orders,customers,products,quantity = gp.multidict({
    '1' :[ 2, 'Kids & Baby' ,3],
 '2' :[ 3, 'Superstore' , 1],
 '3' :[ 3, 'Others' , 1],
 '4' :[ 4, 'Books' , 2],
 '5' :[ 4, 'Kids & Baby' , 1],
 '6' :[ 5, 'Mens Fashion' , 3],
 '7' :[ 5, 'Mens Fashion' , 4],
 '8' :[ 5, 'Kids & Baby' , 3],
 '9' :[ 6, 'School & Education' , 2],
 '10' :[ 6, 'Soghaat' , 1],
 '11' :[ 7, 'Others' , 1],
 '12' :[ 7, 'Books' , 3],
 '13' :[ 7, 'Kids & Baby' , 5],
 '14' :[ 8, 'Others' , 3],
 '15' :[ 8, 'Entertainment' , 4],
 '16' :[ 8, 'Beauty & Grooming' , 3],
 '17' :[ 9, 'Womens Fashion' , 4],
 '18' :[ 10, 'Books' , 1],
 '19' :[ 10, 'Beauty & Grooming' , 2],
 '20' :[ 10, 'Mens Fashion' , 5],
 '21' :[ 11, 'Kids & Baby' , 4],
 '22' :[ 11, 'School & Education' , 5],
 '23' :[ 12, 'Books' , 1],
    
    })
#ordCust = {node i in L_c: products.getkey() == i }
ordcust = []
for i in range(2,len(L_c)+2):
    ordcust.append([])
    for j in range(1,len(orders)+1):
        if customers['%d' % j] == i:
            ordcust[i-2].append('%s' % j)

#V_num = number of fleets
V_num = range(4)
#V_q = Capacity of each fleet:q
V_q = 22.5
#number of nodes

depot = range(len(L_d))

node = range(len(L))

cust = range(2,len(L))

# Create decision variables

b = m.addVars(node,node,V_num, vtype = GRB.BINARY,name = "traverse")
x = m.addVars(orders, V_num, vtype = GRB.BINARY, name = "assign_car")
u = m.addVars(node,V_num, vtype = GRB.INTEGER,ub = len(L), name = "position" )

#Set Objective

m.setObjective((gp.quicksum(b[i,j,v] * costs['%s %g' %(i,j)] for i in node for j in node for v in V_num)), GRB.MINIMIZE)

#Create Constraints

m.addConstrs((gp.quicksum(b[i,j,v] for j in cust for i in depot ) <= 1 for v in V_num), name = "single-pack")
m.addConstrs((gp.quicksum(b[i,k,v] for i in node) == gp.quicksum(b[k,j,v] for j in node) for k in node for v in V_num ), name = "non-stop")
m.addConstrs((u[i,v]- u[j,v] + len(L) * b[i,j,v] <= len(L_c) for i in node for j in cust for v in V_num), name = 'position-limit')
m.addConstr((u[0,0] == 1),name = "origin-1")
m.addConstr((u[0,1] == 1),name = "origin-2")
m.addConstr((u[1,2] == 1),name = "origin-3")
m.addConstr((u[1,3] == 1),name = "origin-4")
m.addConstrs((gp.quicksum(x[o,v] for v in V_num) == 1 for  o in orders), name = 'surjection-of-order')
m.addConstrs((gp.quicksum(x[o,v] for o in ordcust[j-2]) <= len(orders) * gp.quicksum(b[i,j,v] for i in node) for v in V_num for j in cust),name = "all-goods-delivered")
m.addConstrs((gp.quicksum(x[o,v] * quantity[o] for o in orders) <= V_q  for v in V_num), name = "weight-cap")
m.addConstrs((b[0,j,3] == 0 for j in cust), name = 'depot-10')
m.addConstrs((b[0,j,2] == 0 for j in cust), name = 'depot-1')
m.addConstrs((b[1,j,0] == 0 for j in cust), name = 'depot-12')
m.addConstrs((b[1,j,1] == 0 for j in cust), name = 'depot-13')

m.optimize()
m.write('multi-depot.lp')
             
# print optimal solutions
for v in m.getVars ():
    if v.x != 0 and v.x < len(L):
        print ('%s %g' % (v.varName , v.x))

# print optimal sequence for each vehicles 

print('An optimal route is: ')
for v in V_num:
    print('V%g: ' % v, end = '')
    if v == 0:
        i = 0
        for j in cust:
            if b[i,j,v].X == 1:
                print('%g --> ' % j, end = '')
                i = j
        
            for j in node:
                if b[i,j,v].X == 1 and i != 0:
                    print('%g --> ' % j, end = '')
                    i = j 
                   
    else:
        i = 1
        for j in cust:
            if b[i,j,v].X == 1:
                print('%g --> ' % j, end = '')
                i = j
        
            for j in node:
                if b[i,j,v].X == 1 and i != 1:
                    print('%g --> ' % j, end = '')
                    i = j 
            
print('The traveling distance for each vechicle is ')
for v in V_num:
    print('V%g: ' % v, end=' ')
    q = 0
    for i in node:
        for j in node:
            if b[i, j, v].X == 1:
                q = q + data[i, j]
    print('%g' % q, end=' ')


print('The usage of each vehicle is:')
for v in V_num:
    print('V%g: ' % v, end='')
    q = 0
    for i in orders:
        if x[i, v].X == 1:
            q = q + quantity[i]
    print('%g' % q, end=' ')
