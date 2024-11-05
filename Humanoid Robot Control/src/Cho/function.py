import numpy as np
import pandas as pd
import csv
# 將弧度換算角度進行控制運算
def rad2deg(radius):
    return radius/np.pi*180
def csv2cmd(filename):
    file = open(filename)
    reader = csv.reader(file)
    data_list = list(reader)
    file.close()
    for i in range(len(data_list)):
        for j in range(len(data_list[i])):
            data_list[i][j] = float(data_list[i][j])
        # data_list[i].insert(0,float(0)) #加入trunk角度
    return data_list
def filt_7(step, desired, motion,i):

    if i == 10 and (step<70 or (step>95 and step<165)):
        desired[i]= rad2deg(motion[50][i])
    if i == 10 and (step>=70 and step<=95):
        desired[i]= rad2deg(motion[83][i])

    if i == 10 and ((step>164 and step<206) or (step>240 and step<310)):
        desired[i]= rad2deg(motion[50][i])

    if i == 10 and (step>=206 and step<=240):
        desired[i]= rad2deg(motion[223][i])

    if i == 10 and ((step>=310 and step<346) or (step>380 and step<450)):
        desired[i]= rad2deg(motion[50][i])

    if i == 10 and (step>=346 and step<=380):
        desired[i]= rad2deg(motion[363][i])

    if i == 10 and ((step>=450 and step<487) or (step>521 and step<591)):
        desired[i]= rad2deg(motion[50][i])

    if i == 10 and (step>=487 and step<=521):
        desired[i]= rad2deg(motion[503][i])



    if i == 4 and (step<142 or (step>165 and step<235)):
        desired[i]= rad2deg(motion[50][i])

    if i == 4 and (step>=142 and step<=165):
        desired[i]= rad2deg(motion[154][i])

    if i == 4 and ((step>=235 and step<277) or (step>311 and step<381)):
        desired[i]= rad2deg(motion[50][i])
    if i == 4 and (step>=277 and step<=311):
        desired[i]= rad2deg(motion[294][i])

    if i == 4 and ((step>=381 and step<417) or (step>451 and step<521)):
        desired[i]= rad2deg(motion[50][i])
    if i == 4 and (step>=417 and step<=451):
        desired[i]= rad2deg(motion[434][i])

    if i == 4 and ((step>=521 and step<557) or (step>591 and step<621)):
        desired[i]= rad2deg(motion[50][i])
    if i == 4 and (step>=557 and step<=591):
        desired[i]= rad2deg(motion[574][i])

    if (i == 2 or i == 6 or i == 8 or i == 12) and (step>=124 and step<=127):
        desired[i]= (step-124)*(rad2deg(motion[127][i])-rad2deg(motion[124][i]))/(127-124)+rad2deg(motion[124][i])
    if (i == 2 or i == 6 or i == 8 or i == 12) and (step>=194 and step<=198):
        desired[i]= (step-194)*(rad2deg(motion[198][i])-rad2deg(motion[194][i]))/(198-194)+rad2deg(motion[194][i])
    
    if (i == 2 or i == 6 or i == 8 or i == 12) and (step>=264 and step<=267):
        desired[i]= (step-264)*(rad2deg(motion[267][i])-rad2deg(motion[264][i]))/(267-264)+rad2deg(motion[264][i])
    if (i == 2 or i == 6 or i == 8 or i == 12) and (step>=334 and step<=337):
        desired[i]= (step-334)*(rad2deg(motion[337][i])-rad2deg(motion[334][i]))/(337-334)+rad2deg(motion[334][i])
    # if (i == 2 or i == 6 or i == 8 or i == 12) and (step>=125 and step<=127):
    #     desired[i]= (step-125)*(0-rad2deg(motion[125][i]))/(127-125)+rad2deg(motion[125][i])

    # if i == 3 and (step>=132 and step<=146):
    #     desired[i]= rad2deg(motion[137][i])   
    # if i == 3 and (step>=272 and step<=286):
    #     desired[i]= rad2deg(motion[277][i])
    # if i == 3 and (step>=412 and step<=426):
    #     desired[i]= rad2deg(motion[417][i])
    # if i == 3 and (step>=552 and step<=566):
    #     desired[i]= rad2deg(motion[557][i])



    if i == 3 and (step>=150 and step<=169):
        desired[i]= rad2deg(motion[157][i])   
    if i == 3 and (step>=290 and step<=309):
        desired[i]= rad2deg(motion[297][i])
    if i == 3 and (step>=430 and step<=449):
        desired[i]= rad2deg(motion[437][i])
    if i == 3 and (step>=570 and step<=589):
        desired[i]= rad2deg(motion[577][i])

    # if i == 9 and (step>=61 and step<=75):
    #     desired[i]= rad2deg(motion[66][i])
    # if i == 9 and (step>=201 and step<=215):
    #     desired[i]= rad2deg(motion[206][i])
    # if i == 9 and (step>=341 and step<=355):
    #     desired[i]= rad2deg(motion[346][i])
    # if i == 9 and (step>=481 and step<=495):
    #     desired[i]= rad2deg(motion[486][i])  

    if i == 9 and (step>=78 and step<=97):
        desired[i]= rad2deg(motion[85][i])
    if i == 9 and (step>=218 and step<=237):
        desired[i]= rad2deg(motion[225][i])
    if i == 9 and (step>=358 and step<=377):
        desired[i]= rad2deg(motion[365][i])
    if i == 9 and (step>=498 and step<=517):
        desired[i]= rad2deg(motion[505][i])  

    if (i == 2 or i == 6 or i == 8 or i == 12) and (step>614):
        desired[i]= 0

    
    return desired[i]

def filt_5(step, desired, motion,i):

    if i == 10 and (step<61 or (step>85 and step<135)):
        desired[i]= rad2deg(motion[50][i])
    if i == 10 and (step>=61 and step<=85):
        desired[i]= rad2deg(motion[73][i])

    if i == 10 and ((step>134 and step<161) or (step>185 and step<235)):
        desired[i]= rad2deg(motion[50][i])

    if i == 10 and (step>=161 and step<=185):
        desired[i]= rad2deg(motion[173][i])

    if i == 10 and ((step>234 and step<261) or (step>285 and step<335)):
        desired[i]= rad2deg(motion[50][i])

    if i == 10 and (step>=261 and step<=285):
        desired[i]= rad2deg(motion[273][i])

    if i == 10 and ((step>334 and step<362) or (step>386 and step<436)):
        desired[i]= rad2deg(motion[50][i])

    if i == 10 and (step>=362 and step<=386):
        desired[i]= rad2deg(motion[374][i])



    if i == 4 and (step<111 or (step>135 and step<185)):
        desired[i]= rad2deg(motion[50][i])

    if i == 4 and (step>=111 and step<=135):
        desired[i]= rad2deg(motion[123][i])

    if i == 4 and ((step>184 and step<211) or (step>235 and step<285)):
        desired[i]= rad2deg(motion[50][i])
    if i == 4 and (step>=211 and step<=235):
        desired[i]= rad2deg(motion[223][i])

    if i == 4 and ((step>284 and step<312) or (step>336 and step<386)):
        desired[i]= rad2deg(motion[50][i])
    if i == 4 and (step>=312 and step<=336):
        desired[i]= rad2deg(motion[324][i])

    if i == 4 and ((step>385 and step<412) or (step>436 and step<486)):
        desired[i]= rad2deg(motion[50][i])
    if i == 4 and (step>=412 and step<=436):
        desired[i]= rad2deg(motion[425][i])

    
    if i == 3 and (step>=115 and step<=137):
        desired[i]= rad2deg(motion[125][i])   
    if i == 3 and (step>=215 and step<=237):
        desired[i]= rad2deg(motion[225][i])
    if i == 3 and (step>=315 and step<=337):
        desired[i]= rad2deg(motion[325][i])
    if i == 3 and (step>=415 and step<=437):
        desired[i]= rad2deg(motion[425][i])

    if i == 9 and (step>=65 and step<=87):
        desired[i]= rad2deg(motion[75][i])
    if i == 9 and (step>=165 and step<=187):
        desired[i]= rad2deg(motion[175][i])
    if i == 9 and (step>=265 and step<=287):
        desired[i]= rad2deg(motion[275][i])
    if i == 9 and (step>=365 and step<=387):
        desired[i]= rad2deg(motion[375][i])  

    if (i == 2 or i == 6 or i == 8 or i == 12) and (step>450):
        desired[i]= 0

    
    return desired[i]

def filt_6(step, desired, motion,i):

    
    return desired[i]


def filt_FSF(step, desired, motion,i):

    if i == 2 and (step>=251 and step<=258):
        desired[i]= (step-251)*(rad2deg(motion[258][i])-rad2deg(motion[251][i]))/(258-251)+rad2deg(motion[251][i])
    if i == 6 and (step>=255 and step<=261):
        desired[i]= rad2deg(motion[258][i])
    if i == 8 and (step>=255 and step<=257):
        desired[i]= rad2deg(motion[255][i])
    if i == 12 and (step>=255 and step<=261):
        desired[i]= rad2deg(motion[258][i])
    
    return desired[i]

def Walk(step, desired, motion,end,period):
    if step < len(motion):
        for i in range(len(motion[step])):
            desired[i] = rad2deg(motion[step][i])
            # if period == 60:
            #     desired[i] = filt_6(step, desired, motion,i)
            # elif period == 50:
            #     desired[i] = filt_5(step, desired, motion,i)
            # elif period == 70:
            #     desired[i] = filt_7(step, desired, motion,i)
            # else:
            #     desired[i] = filt_FSF(step, desired, motion,i)
            desired[i] = filt_FSF(step, desired, motion,i)

    else:
        for i in range(len(motion[0])):
            desired[i] = rad2deg(motion[end-1][i])
            # if period == 60:
            #     desired[i] = filt_6(step, desired, motion,i)
            # elif period == 50:
            #     desired[i] = filt_5(step, desired, motion,i)
            # elif period == 70:
            #     desired[i] = filt_7(step, desired, motion,i)
            # desired[i] = filt_FSF(step, desired, motion,i)
    # print(desired)
    return desired

