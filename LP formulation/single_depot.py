#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 16 23:27:16 2021

@author: yushuangwai
"""
import random as rand
import gurobipy as gp
from gurobipy import GRB
import numpy as np

data = np.load('/Users/yushuangwai/Downloads/sioux_falls_distance.npy')

small_data = data[0:12,0:12]
m = gp.Model("single_depot")

# Set variables

#L = list of nodes
L = []
for i in range(len(small_data)):
    L.append('%s' % i)
                 
#L_c = list of customers
L_c = L[1:]

#arcs, costs = gp.multidict({('i,j'): cost of traveling from i to j})
arcs, costs = gp.multidict({('%s %g' % (i,j)): data[i,j] for i in range(len(small_data)) for j in range(len(small_data[i]))})

#orders, products, quantity = gp.multidict({nodes in L_c}:[products,quantity])
orders,customers,products,quantity = gp.multidict({
    '1' :[ 1, 'Kids & Baby' , 3],
 '2' :[ 2, 'Superstore' , 1],
 '3' :[ 2, 'Others' , 1],
 '4' :[ 3, 'Books' , 2],
 '5' :[ 3, 'Kids & Baby' , 1],
 '6' :[ 4, 'Mens Fashion' , 3],
 '7' :[ 4, 'Mens Fashion' , 4],
 '8' :[ 4, 'Kids & Baby' , 3],
 '9' :[ 5, 'School & Education' , 2],
 '10' :[ 5, 'Soghaat' , 1],
 '11' :[ 6, 'Others' , 1],
 '12' :[ 6, 'Books' , 3],
 '13' :[ 6, 'Kids & Baby' , 5],
 '14' :[ 7, 'Others' , 3],
 '15' :[ 7, 'Entertainment' , 4],
 '16' :[ 7, 'Beauty & Grooming' , 3],
 '17' :[ 8, 'Womens Fashion' , 4],
 '18' :[ 9, 'Books' , 1],
 '19' :[ 9, 'Beauty & Grooming' , 2],
 '20' :[ 9, 'Mens Fashion' , 5],
 '21' :[ 10, 'Kids & Baby' , 4],
 '22' :[ 10, 'School & Education' , 5],
 '23' :[ 11, 'Books' , 1],
    
    })
#ordCust = {node i in L_c: products.getkey() == i }
ordcust = []
for i in range(1,len(L_c)+1):
    ordcust.append([])
    for j in range(1,len(orders)+1):
        if customers['%d' % j] == i:
            ordcust[i-1].append('%s' % j)

#V_num = number of fleets
V_num = range(3)
#V_q = Capacity of each fleet:q
V_q = 30
#number of nodes


node = range(len(L))

cust = range(1,len(L))

# Create decision variables

b = m.addVars(node,node,V_num, vtype = GRB.BINARY,name = "traverse")
x = m.addVars(orders, V_num, vtype = GRB.BINARY, name = "assign_car")
u = m.addVars(node,V_num, vtype = GRB.INTEGER,ub = len(L), name = "position" )

#Set Objective

m.setObjective((gp.quicksum(b[i,j,v] * costs['%s %g' %(i,j)] for i in node for j in node for v in V_num)), GRB.MINIMIZE)

#Create Constraints

m.addConstrs((gp.quicksum(b[0,j,v] for j in cust ) <= 1 for v in V_num), name = "single-pack")
m.addConstrs((gp.quicksum(b[i,k,v] for i in node) == gp.quicksum(b[k,j,v] for j in node) for k in node for v in V_num ), name = "non-stop")
m.addConstrs((u[i,v]- u[j,v] + len(L) * b[i,j,v] <= len(L_c) for i in node for j in cust for v in V_num), name = 'position-limit')
m.addConstrs((u[0,v] == 1 for v in V_num),name = "origin")
m.addConstrs((gp.quicksum(x[o,v] for v in V_num) == 1 for  o in orders), name = 'surjection-of-order')
m.addConstrs((gp.quicksum(x[o,v] for o in ordcust[j-1]) <= len(orders) * gp.quicksum(b[i,j,v] for i in node) for v in V_num for j in cust),name = "all-goods-delivered")
m.addConstrs((gp.quicksum(x[o,v] * quantity[o] for o in orders) <= V_q  for v in V_num), name = "weight-cap")

m.optimize()
m.write('single-depot.lp')
             
# print optimal solutions
for v in m.getVars ():
    if v.x != 0:
        print ('%s %g' % (v.varName , v.x))

# print optimal sequence for each vehicles 
print('An optimal route is: ')
for v in V_num:
    print('V%g: ' % v, end = '')
    i = 0
    print('%g --> ' % i, end = '')
    for j in cust:
        if b[i,j,v].X == 1:
            print('%g --> ' % j, end = '')
            i = j
    
        for j in node:
            if b[i,j,v].X == 1 and i != 0:
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

print('total visit of customer:')
q = 0
for v in V_num:
    for i in node:
        for j in cust:
            if b[i, j, v].X == 1:
                q = q + 1
print('%g' % q, end=' ')

    
