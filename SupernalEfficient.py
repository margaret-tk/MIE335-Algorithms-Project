import gurobipy as gp
from gurobipy import GRB
import numpy as np
import time
from InputReader import importlist


def Supernal(c,a,b):
    
    
    n = len(c[0])
    
    NDF = []
    
    temp_a = []
    for i in range(n):
        temp = []
        for j in range(len(a)):
            temp.append(a[j][i])
        temp_a.append(temp)
            
    c_orig = c.copy()
    J = len(c)
    temp_c = [item for sublist in c for item in sublist]
    c = np.array(temp_c)
    a = np.array(temp_a)
    b = np.array(b)
    
    cons = len(a[0])
    
    if J == 2:
        supernal = [0,0]
    
    elif J == 3:
        supernal = [0,0,0]
        
    regions1 = [supernal]
    regions2 = []
    
    m = gp.Model("Supernal Method")
    
    Items=list(np.arange(0,n))
    Dimensions= list(np.arange(0,cons))

    # Decision Vars
    x = m.addVars(n,vtype=GRB.BINARY, name="ToTakeOrNotToTake")
    
    #Mute Output Text
    m.Params.OutputFlag = 0
        
    
    #Knapsack Constraints
    for j in Dimensions:
        m.addConstr(gp.quicksum(x[i]*a[i,j] for i in Items) <= b[j], name="j"+str(j))
        
    #Initital Supernal Region
    num_regions = 1
    m.addConstr(gp.quicksum(x[i]*c[i] for i in Items) <= 0, name = "z1")
    m.addConstr(gp.quicksum(x[i]*c[i+n] for i in Items) <= 0, name = "z2")
    
    
    if J == 3:
        l = [0.33,0.33,0.34]
        m.setObjective((gp.quicksum(x[i]*c[i] for i in Items)*l[0] + gp.quicksum(x[i]*c[i+n] for i in Items)*l[1] + gp.quicksum(x[i]*c[i+n*2] for i in Items)*l[2]), GRB.MINIMIZE)
        m.addConstr(gp.quicksum(x[i]*c[i+2*n] for i in Items) <= 0, name = "z3")
    
    elif J ==2:
        l = [0.5,0.5]
        m.setObjective((gp.quicksum(x[i]*c[i] for i in Items)*l[0] + gp.quicksum(x[i]*c[i+n] for i in Items)*l[1]), GRB.MINIMIZE)

    m.optimize()
    
    #Remove Denominated Regions for J = 3
    def deleteDominated(reg):
        nondom = reg.copy()
        for i in range(len(reg)):
            for j in range(len(reg)): 
    
                if i == j:
                    continue
    
                if reg[i][0]<= reg[j][0]:
                    if reg[i][1] <= reg[j][1]:
                        if reg[i][2] <= reg[j][2]:    
                            nondom.remove(reg[i])
                            break
        return nondom
    
    
    def optimize(region):
        
        m.getConstrByName('z1').RHS = region[0]
        m.getConstrByName('z2').RHS = region[1]
        
        if J == 3:
            m.getConstrByName('z3').RHS = region[2]
        
        m.optimize()
        
        Mat = np.zeros((1,n))
        i = 0
        j = 0
        
        try:
            for p in x:
                if j == Mat.shape[1]:
                    j = 0
                    i += 1
                Mat[i,j] = abs(x[p].x)
                j+=1
            
            z = []
            for i in range(J):
                z_temp = 0
                for j in range(n):
                    z_temp += c_orig[i][j]*Mat[0][j]
                z.append(z_temp)
            
            if z not in NDF:
                NDF.append(z)
            
            
            if J == 2:
    
                regions2.append([z[0]-1,region[1]])
                regions2.append([region[0],z[1]-1])        
                
            if J == 3:
                
                regions2.append([z[0]-1,region[1],region[2]])
                regions2.append([region[0],z[1]-1,region[2]])
                regions2.append([region[0],region[1],z[2]-1])                     
                
            return None
            
        except: 
            return None
        
    
    #While there are new regions to be processed in the next layer, continue
    while len(regions1) > 0:
        
        #iterate through all regions to be processed in current layer
        for r in range(len(regions1)):
            optimize(regions1[r])
        
        
        if J == 2:
            regions1 = regions2.copy()
        
        elif J == 3:
            #Remove dominated regions from next processing layer
            regions1 = deleteDominated(regions2)
        
        #Count new regions
        num_regions += len(regions1)
        #Reset empty list of regions for the next processing level
        regions2 = []
        
    ndf_final = NDF.copy() 
    if J == 2:

              
    
        for i in range(len(NDF)):
            for j in range(len(NDF)): 
    
                if i == j:
                    continue
        
                if NDF[i][0]>= NDF[j][0]:
                    if NDF[i][1] >= NDF[j][1]:
                        ndf_final.remove(NDF[i])
                    break
    
    return ndf_final, num_regions


