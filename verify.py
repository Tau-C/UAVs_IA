import gurobipy as gp
from gurobipy import max_
from gurobipy import or_
from gurobipy import GRB
from math import sqrt

center_x = 1100
center_y = 1100
dis_ma = []

def cal_dis(coodi_):
    dis_0 = []
    for i in range(len(coodi_)):
        dis_0_i = sqrt((center_x-coodi_[i][0])**2+(center_y-coodi_[i][1])**2)
        dis_0.append(dis_0_i)
    dis_ma.append(dis_0)
    for i in range(len(coodi_)):
        dis_i = []
        for j in range(len(coodi_)):
            dis_ij = sqrt((coodi_[i][0]-coodi_[j][0])**2+(coodi_[i][1]-coodi_[j][1])**2)
            dis_i.append(dis_ij)
        dis_ma.append(dis_i)


f = open("map_10_1.txt")
node_num = 10
uav_num =5
#define uav_type
#type_1
uav_type_list = []
for i in range(2):
    uav_type_list.append(0)

#type_2
for i in range(3):
    uav_type_list.append(1)

v_s = 10
v_p = 3

node_size = []
node_col = []
coodi_ma = []
node_type_list = []


for i,line in enumerate(f):
    line = line.strip("\n")
    line = line.split(" ")
    y = [float(x) for x in line]
    coodi_ma.append([y[0],y[1]])
    node_size.append(y[2])
    node_col.append(y[3])
    node_type_list.append(y[4])


cal_dis(coodi_ma)
T = 8000
model = gp.Model("AR")
# model.setParam("MIPFocus", 2) 
x = model.addVars(node_num+1,node_num+1,uav_num, vtype = GRB.BINARY, name = "x")
y = model.addVars(node_num+1, vtype=GRB.BINARY,name = "y")
z = model.addVars(node_num+1,uav_num,vtype=GRB.BINARY, name= "z")
E = model.addVars(node_num+1, lb=0.0, ub=T,vtype = GRB.CONTINUOUS,name = "E")
b = model.addVars(node_num+1,uav_num, lb=0.0, ub=T, vtype=GRB.CONTINUOUS,name = "b")
e = model.addVars(node_num+1,uav_num, lb= 0.0, ub = T, vtype = GRB.CONTINUOUS, name = "e")

for k in range(uav_num):
    for j in range(node_num+1):
        model.addConstr((z[j,k]==or_(x.select("*",j,k))),name = "z-0-1")#

for i in range(node_num):
    sum_z = 0
    for k in range(uav_num):
        sum_z +=z[i+1,k]
    model.addConstr((y[i+1]==1)>>(sum_z>=1), name = "y-0-1")#


# sum_st = 0
# for j in range(node_num):
#     for k in range(uav_num):
#         sum_st+=x[0,j+1,k]
# model.addConstr(sum_st==uav_num, name = "start")#

for k in range(uav_num):
    for j in range(node_num):
        sum_ij = 0
        sum_jp = 0
        for i in range(node_num+1):
            sum_ij+=x[i,j+1,k]
        for p in range(node_num+1):
            sum_jp+=x[j+1,p,k]
        model.addConstr(sum_ij==sum_jp, name ="stream")

# for k in range(uav_num):
#     for i in range(node_num+1):
#         for j in range(node_num+1):
#             if(i==j):
#                 model.addConstr(x[i,j,k]==0, name ="non")

for k in range(uav_num):
    for j in range(node_num+1):
        sum_to_j = 0
        for i in range(node_num+1):
            sum_to_j+=x[i,j,k]
        model.addConstr(sum_to_j<=1, name ="non")

#type_cons
for i in range(node_num+1):
    for j in range(node_num):
        for k in range(uav_num):
            if(node_type_list[j]!=2):
                if(node_type_list[j]!=uav_type_list[k]):
                    model.addConstr(x[i,j+1,k]==0,name="type_con")


for k in range(uav_num):
    sum_i = 0
    for i in range(node_num):
        sum_i += x[0,i+1,k]
    model.addConstr(sum_i==1, name = "one_")

for k in range(uav_num):
    sum_i_e = 0
    for i in range(node_num):
        sum_i_e += x[i+1,0,k]
    model.addConstr(sum_i_e==1, name = "one_end")
#协同
tmp_b_ik = model.addVars(node_num, uav_num, vtype = GRB.CONTINUOUS, name ="tmp_b_ik")
tmp_b_ijk = model.addVars(node_num+1,node_num, uav_num, vtype = GRB.CONTINUOUS, name ="tmp_b_jik")
for k in range(uav_num):
    for i in range(node_num):
        sum_b = 0
        for j in range(node_num+1):
            model.addConstr((tmp_b_ijk[j,i,k]==(e[j,k]+dis_ma[j][i]/v_s)*x[j,i+1,k]))
            sum_b+=tmp_b_ijk[j,i,k]
        #     sum_b+=(e[j,k]+dis_ma[j][i+1]/v_s)*x[j,i+1,k]
        model.addConstr(tmp_b_ik[i,k]==sum_b)
        model.addConstr(b[i+1,k]==tmp_b_ik[i,k], name = "b_start")

for j in range(node_num):
    sum_j = 0
    for i in range(node_num+1):
        for k in range(uav_num):
            sum_j+=x[i,j+1,k]
    model.addConstr(sum_j<=node_col[j], name = "col_num")#

flag = model.addVars(node_num,lb =0, vtype = GRB.BINARY,name = "flag")
E_tmp = model.addVars(node_num,lb =0, vtype = GRB.CONTINUOUS, name = "E_tmp")
ttmp = model.addVars(node_num,lb =0, vtype = GRB.CONTINUOUS, name = "ttmp")
for i in range(node_num):
    sum_E = 0
    N_tmp = 0
    model.addConstr(((flag[i]==or_(z.select(i+1,"*")))), name ="flag")
    for k in range(uav_num):
        sum_E += b[i+1,k]*z[i+1,k]
        N_tmp += z[i+1,k]
    model.addConstr(ttmp[i] == (node_size[i]/v_p+sum_E))
    model.addConstr(E_tmp[i] == ttmp[i]*flag[i])
    model.addConstr(((E_tmp[i]==E[i+1]*N_tmp)), name ="col")


# max_inv = model.addVars(node_num, uav_num,lb = 0.0, vtype=GRB.CONTINUOUS)
# t_i_max = model.addVars(node_num, vtype=GRB.CONTINUOUS)
# t_i_min = model.addVars(node_num, vtype=GRB.CONTINUOUS)
# t_delta = model.addVars(node_num, vtype=GRB.CONTINUOUS)
# t_delta_in = model.addVars(node_num, vtype=GRB.CONTINUOUS)
# for i in range(node_num):
#     for k in range(uav_num):
#         model.addConstr(z[i+1,k]==max_inv[i,k]*b[i+1,k])
# model.addConstrs(t_i_max[i] == max_(max_inv.select(i,"*")) for i in range(node_num))
# model.addConstrs(flag[i]==t_i_min[i]*t_i_max[i] for i in range(node_num))
# model.addConstrs(t_delta[i]==E[i+1]-t_i_min[i] for i in range(node_num))
# model.addConstrs(flag[i]==t_delta_in[i]*t_delta[i] for i in range(node_num))

# model.addConstr(E[0]==0, name = "node_start_time")
for k in range(uav_num):
    model.addConstr(e[0,k]==0, name = "uav_start")
    
model.addConstrs((E[i+1]<=T for i in range(node_num)), name = "col_time")

model.addConstrs((e[i+1,k]==max_(E[i+1],b[i+1,k]) for i in range(node_num) for k in range(uav_num)),name = "end_time")


obj = 0
for i in range(node_num):
    obj+=y[i+1]*node_size[i]
model.Params.NonConvex = 2
model.setObjective(obj, GRB.MAXIMIZE)
model.optimize()

max_s = 0

for i in range(node_num):
    print(node_size[i])
    max_s+=node_size[i]

# model.computeIIS()
# model.write("model1.ilp")
if model.status == GRB.OPTIMAL:
    print([y[i+1].x for i in range(node_num)])
    print([E[i+1].x for i in range(node_num)])
    # for k in range(uav_num):
    #     print("uav:"+str(k))
    #     for i in range(node_num+1):
    #         for j in range(node_num+1):
    #             print("from node_"+str(i)+"_to_"+str(j)+":"+str(x[i,j,k].X))

    print("------------")

    # for k in range(uav_num):
    #     print("uav:"+str(k))
    #     for i in range(node_num):
    #         print("start:uav_"+str(k)+"_to node_"+str(i+1)+":"+str(b[1+i,k].x))


    # for k in range(uav_num):
    #     print("uav:"+str(k))
    #     for i in range(node_num):
    #         print("uav_"+str(k)+"_to node_"+str(i+1)+":"+str(z[1+i,k].x))

    # print("-------")
    for k in range(uav_num):
        print("|PATH|~uav:"+str(k))
        path = []
        start = 0
        path.append(start)
        flag = 1
        while(flag == 1):
            fk = 0
            for i in range(node_num):
                if(float(x[start,i+1,k].x)>0.5):
                    path.append(i+1)
                    start = i+1
                    fk = 1
                    break  
            if(fk == 0):
                flag = 0
        path.append(0)
        for j in range(len(path)-1):
            print("node:"+str(path[j])+"-->",end="")
        print("node:"+str(path[len(path)-1]))

    print(model.Runtime)
    # for i in range(node_num):
    #     print(str(t_delta_in[i]))
    
        
