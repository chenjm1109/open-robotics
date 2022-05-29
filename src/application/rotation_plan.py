import math
import matplotlib.pyplot as plt
import numpy as np
import random


def angle_sort(angle_list):
    sort_list = angle_list.copy()
    dist_list = angle_list.copy()
    for i in range(len(angle_list)):
        if (angle_list[i]<angle_list[0]):
            angle_list[i] += 360
        sort_list[i] = (angle_list[i]-angle_list[0])%360
        if(sort_list[i]>180):
            dist_list[i] = sort_list[i]-360
        else:
            dist_list[i] = sort_list[i]
    

    return dist_list

if __name__ == "__main__":
    # theta_min = -721
    # theta_max = 721
    # angle_list = list(range(theta_min, theta_max))
    # dist_list = angle_sort(angle_list)
    # plt.plot(angle_list, dist_list)
    # plt.plot(angle_list, angle_list)
    # plt.plot([theta_min, theta_max], [180, 180], "--", color='g')
    # plt.plot([theta_min, theta_max], [-180, -180], "--", color='g')
    # plt.savefig("dis.png")

    angle_list = []
    angle_list.append(-270)
    for i in range(4):
        angle_list.append(round(random.random()*360-180))

    fig = plt.figure(figsize=(12, 12), dpi=300)
    
    dist_list = angle_sort(angle_list)
    print('angle_list:')
    print(angle_list)
    print('dist_list:')
    print(dist_list)
    print(np.argsort(dist_list))

    x_circle = []
    y_circle = []
    for i in range(360):
        x_circle.append(1*math.cos(i*math.pi/180))
        y_circle.append(1*math.sin(i*math.pi/180)) 
    plt.plot(x_circle, y_circle, linestyle='--', color='g')

    plt.plot([0, 1.0],[0, 0], linestyle='--', color='g')
    plt.text(0, 1.05, 'y',  color='g', size=20)
    plt.plot([0, 0],[0, 1.0], linestyle='--', color='g')
    plt.text(1.05, 0, 'x',  color='g', size=20)

    sort_index = np.argsort(dist_list)
    index = 0
    for i in sort_index:
        x = 1*math.cos(angle_list[i]*math.pi/180)
        y = 1*math.sin(angle_list[i]*math.pi/180)
        plt.plot([0,x],[0,y], color='gray')
        plt.text(x, y, 
        str(index)+":"+str(angle_list[i])+"°", size=20, color='r')
        index+=1
    
    plt.savefig("dis.png")
    