# -*- coding: utf-8 -*-
"""
Created on Tue Jun  1 23:29:10 2021

@author: maksa
"""
import numpy as np
import matplotlib.pyplot as plt

""" Section 1 creating time steps for simulation, establishing ideal parameter
values and creating empty arrays for plots"""

N = 1000                    # number of time steps within simulation
t = np.linspace(0, 100, N)  # amount of time the simulation will run for
T = 100/N


d_ideal = 5         # Ideal following distance in meters
a_max = 3.5         # Max acceleration of car
a_min = -4.5         # Max Deceleration of car
v_ideal = 22.352    # Cruising speed if following distance allows (50mph)
d_vlarge = 15       # Threshold for Free Ride State
d_large = 7.5       # Threshold for regular acceleration
d_vsmall = 1        # Threshold for max braking

a_front = np.zeros(N)
a_follow = np.zeros(N) #Establishing acceleration array for following car
v_follow = np.zeros(N) #Establishing velocity array for following car

v_0_follow = 4             #initial velocity for following car
v_front = np.zeros(N) + 5   # Establishing vel array for front car 
x_follow = np.zeros(N)      # Establishing following car position array
x_front = np.zeros(N)       # Establishing front car position array

d = np.zeros(N)             # Establishing following distance array


""" Section 2 Car Following Model Translation Gipps"""

def gipps_car_following(d_following): #Function input is following distance
    if i == 0:
        v_follow[0] = v_0_follow
        x_follow[0] = 0
        a_follow[0] = 0
        d[0] = d_ideal
        x_front[0] = 5.0
    else :
        if d[i]>d_vlarge:
            a_follow[i] = a_max
            v_follow[i] = v_follow[i-1] + 2.5*a_max*t[i]*(-(v_follow[i]/v_ideal))*((0.025+(v_follow[i]/v_ideal))**0.5)
        elif d[i] >d_large:
            a_follow[i] = a_follow[i] + 0.1
            v_follow[i] = v_follow[i-1] + 2.5*a_follow[i]*t[i]*(1-(v_follow[i]/v_ideal))*((0.025+(v_follow[i]/v_ideal))**0.5)
        elif d[i]< d_ideal:
            a_follow[i] = a_follow[i] - 1
            v_follow[i] = a_follow[i]*T + (((a_follow[i]*T)**2)-a_follow[i]*(2*(x_follow[i-1]-x_follow[i])-v_follow[i]*T-((v_follow[i-1])**2)/a_follow[i]))**0.5
        elif d[i]< d_vsmall:
            a_follow[i] = a_min
            v_follow[i] = a_min*T + (((a_min*T)**2)-a_min*(2*(x_follow[i-1]-x_follow[i])-v_follow[i]*T-((v_follow[i-1])**2)/a_min))**0.5
        elif d[i]< 0:
            v_follow[i] = 10000000
            
""" Section 3: Application to front car at constant Vel"""

for i in range(t.size):
    d[i] = x_front[i]-x_follow[i]
    gipps_car_following(d[i])
    x_front[i] = v_front[i]*T +x_front[i]
    x_follow[i] = v_follow[i]*T + x_follow[i]
    
    
    
""" Section 4: Plotting Data """

# plot position vs. time, car vel vs. time.  

_, ax = plt.subplots(2,1)

ax[0].plot(t,x_follow,label='Follow Position')
ax[0].plot(t,x_front,label='Lead Position')
ax[0].set_ylabel('x-position')
ax[0].set_xlabel('Time [s]')
ax[0].legend()

ax[1].plot(t,v_follow,label='Follow Velocity')
ax[1].plot(t,v_front,label='Lead Velocity')
ax[1].set_ylabel('Velocity')
ax[1].set_xlabel('Time [s]')
ax[1].legend()

# car vel vs. car position and accel vs. time
_, ax = plt.subplots(2,1)

ax[0].plot(x_follow,v_follow,label='Follow velocity vs. Follow Position')
ax[0].plot(x_front,v_front,label='Front velocity vs. Front Position')
ax[0].set_ylabel('Velocity [m/s]')
ax[0].set_xlabel('x-position [m]')
ax[0].legend()

ax[1].plot(t,a_follow,label='Follow Accel')
ax[1].plot(t,a_front,label='Front Accel')
ax[1].set_ylabel('Acceleration [m/s^2]')
ax[1].set_xlabel('Time [s]')
ax[1].legend()

# accel vs. follow dist.

plt.plot(d, a_follow)
plt.title('Follow Accel vs. Following distance')
plt.ylabel('Acceleration [m/s^2]')
plt.xlabel('Following Distance [m]')
plt.legend()

plt.show()  
    
        
    
         
                                #Function output is new following car distance
                                        
    