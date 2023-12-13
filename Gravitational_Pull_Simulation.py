# -*- coding: utf-8 -*-
"""
Created on Sat Oct 21 17:35:44 2023

@author: Amani
"""

from astropy import units as astr_u, constants as astr_c
import matplotlib.pyplot as plt
from math import pi

#creating units of velocity and acceleration
vel = 1*astr_u.m/astr_u.second
acc = 1*vel/astr_u.second

#Densisty of earth assuming its perfectly spherical
dens_earth = 1*astr_c.M_earth / ((4/3)*pi*(astr_c.R_earth)**3)

#Creating class of an object
class Object:
    def __init__(self, name, mass, pos_x, pos_y, pos_z):
        self.name = name
        self.mass = mass
        self.pos_x, self.pos_y, self.pos_z = pos_x, pos_y, pos_z
        self.a_x = self.a_y = self.a_z = 0*acc  #Initial acceleration
        
        #Initial velocity of objects
        self.v_x = self.v_y = self.v_z = 0*vel
        
        # initially not paired to another object
        self.pair = None
        
        #Assuming object is perfectly spherical in nature
        #Also assuming it has the same density as Earth
        self.radius = (self.mass/((4/3)*pi*dens_earth))**(1/3)        
    
    #Using Newtons method to cultivate motion of massive objects
    def next_position(self, obj, axis, delta_t = 0.5*astr_u.second):
        
        #Counter measure for correct calculations
        if isinstance(delta_t, astr_u.quantity.Quantity):
            pass
        else:
            delta_t = delta_t*astr_u.second
        
        if axis == 'x':
            pos_1 = self.pos_x
            pos_2 = obj.pos_x
            v_i = self.v_x
        elif axis == 'y':
            pos_1 = self.pos_y
            pos_2 = obj.pos_y
            v_i = self.v_y
        elif axis == 'z':
            pos_1 = self.pos_z
            pos_2 = obj.pos_z
            v_i = self.v_z
        
        #If either position values are of type 'Quantity'
        #Get only their integer value to remove further errors
        if isinstance(pos_1, astr_u.quantity.Quantity):
            pos_1 = pos_1.value
        if isinstance(pos_2, astr_u.quantity.Quantity):
            pos_2 = pos_2.value
        
        r = abs(pos_1-pos_2)*astr_u.m
        
        if int(r.value) == 0:
            F = 0*astr_u.newton
        else: 
            F = astr_c.G * self.mass * obj.mass / (r**2)
        # if axis == 'x': print(F)
        
        #Acceleration calculation
        # Assuming the motion of the first object
        #Hence dividing by its mass and not the latter's mass
        a = -F / self.mass
        
        # if axis == 'x': print(a)
        #If the object has already been used to gravitationally pull the other
        if obj.name == self.pair: #Then acceleration is in oppsite direction
            a = -a
        
        #new velocity of object
        if axis == 'x': 
            self.v_x = v_i + a*delta_t
            self.a_x = a
            # print(self.v_x)
        elif axis == 'y': 
            self.v_y = v_i + a*delta_t
            self.a_y = a
        elif axis == 'z': 
            self.v_z = v_i + a*delta_t
            self.a_z = a
        
        #Change in current axis (x, y or z)
        #With the use of Newtons equation of motion
        delta_axis_1 = v_i * delta_t + 0.5 * a * (delta_t**2)
        # print(delta_axis_1)
        
        try:
            pos_next = pos_1.value + delta_axis_1.value
        except AttributeError:
            pos_next = pos_1 + delta_axis_1.value
        return pos_next

#Gets next coords of obj_1 based on gravitational pull of obj_2
def get_next_position(obj_1, obj_2, t):
    
    x = obj_1.next_position(obj_2, 'x', delta_t = t)
    y = obj_1.next_position(obj_2, 'y', delta_t = t)
    z = obj_1.next_position(obj_2, 'z', delta_t = t)
    
    return x, y, z

# Initial position between rock_1 and Moon centers in meters
# x_initial = 384400000  

moon_mass = 7.3e22*astr_u.kg
earth_mass = 1*astr_c.M_earth
dis = 1.4e7*astr_u.m # 3.844e9 *astr_u.m

#Defining the objects
rock_1 = Object(name = 'rock_1', mass = earth_mass , pos_x = 0*astr_u.m,  
                pos_y = dis, pos_z = 0*astr_u.m)
rock_2 = Object('rock_2', earth_mass , pos_x = dis,  pos_y = 0*astr_u.m
                , pos_z = dis)

#Giving initial velocities
rock_1.v_x, rock_1.v_y, rock_1.v_z = 0e1*vel, 3e-2*vel, 0e1*vel
rock_2.v_x, rock_2.v_y, rock_2.v_z = 0e1*vel, -3e-2*vel, 0e1*vel

def get_distance_between(obj_1, obj_2):
    #using distance formula to get distance between rock_1 and rock_2
    x_diff = abs(obj_1.pos_x-obj_2.pos_x)
    y_diff = abs(obj_1.pos_y-obj_2.pos_y)
    z_diff = abs(obj_1.pos_z-obj_2.pos_z)
    d = (x_diff**2 + y_diff**2 + z_diff**2)**0.5
    
    try: #Get only the value of distance if its of type 'Quantity'
        d = d.value
    except AttributeError:
        pass
    
    return d

step_time_init = 5
#Default step size is 1 hour (in secs)
def get_step_size(obj_1, obj_2, step, d_thresh, time = 60*60):
    
    new_step = step
    k = 1 + 1e-1
    # limits of valid stepsizes of motion of objects (in seconds)
    
        #Counter measure for correct calculations
    if isinstance(time, astr_u.quantity.Quantity):
        pass
    else: time = time*astr_u.second
    
    max_step = step_time_init
    min_step = max_step*1e-3
    
    dis = get_distance_between(obj_1, obj_2)
    
    # if there is a significant decrease in the distance
    #and as long as the stepsize is larger than minimum step parametre
    if dis > 1e16 and step > min_step:
        if step/k > min_step:
             new_step = step/k #Reduce the stepsize 
    
    #Increase the time step if the distance between the objects are smaller
    #than the threshold distance. Do this just as long as 
    #the new step doest  not above the maximum
    elif dis < dis_thresh: 
        if new_step < max_step:
            new_step = step*k
            
    return new_step

dis_thresh = (1*astr_c.R_earth).value/2 # Threshold distance between objects in metres

def initiate_simulation():
    # Plotting Trajectories
    q = 12
    fig = plt.figure(figsize = (3*q,q))

    ax_1 = fig.add_subplot(111, projection = '3d')

    ax_1.set_xlabel('x-axis')
    ax_1.set_ylabel('y-axis')
    ax_1.set_zlabel('z-axis')

    n, k = dis.value, 1.2
    ax_1.set_zlim(-2*k*n,2*k*n)
    ax_1.set_xlim(-k*n, k*n)
    ax_1.set_ylim(-k*n, k*n)

    # ax_2 = fig.add_subplot(122)
    # ax_2.set_title('Distance between objects over time')
    # ax_2.set_xlabel('t')
    # ax_2.set_ylabel('r')
    
    return ax_1

def simulate(ax_1, obj_1, obj_2):
    # dist = distance
    # ax_1.set_title(f'Distance between objects: {distance:.9f} $*10^{9}$ metres')
    ax_1.set_title(f'Distance between objects: {distance:,} metres')
    
    sctt_1 = ax_1.scatter(obj_1.pos_x, obj_1.pos_y, obj_1.pos_z, 
                        c = '#000000', s = 150)
    sctt_2 = ax_1.scatter(obj_2.pos_x, obj_2.pos_y, obj_2.pos_z,
                        c = 'b', s = 150)
    
    traj_x_1.append(obj_1.pos_x), traj_x_2.append(obj_2.pos_x)
    traj_y_1.append(obj_1.pos_y), traj_y_2.append(obj_2.pos_y)
    traj_z_1.append(obj_1.pos_z), traj_z_2.append(obj_2.pos_z)
    
    line_1 = ax_1.plot(traj_x_1, traj_y_1, traj_z_1, c = 'r')
    line_2 = ax_1.plot(traj_x_2, traj_y_2, traj_z_2, c = 'r')
    
    plt.show()
    plt.pause(0.1)
    
    sctt_1.remove(), sctt_2.remove() #,sctt_3.remove()
    line_1.clear(), line_2.clear()

def simulation_question():
    if input("Press enter to start the simulation") == '':
        return
    else:
        print("Please follow instructions")
        simulation_question()
        
distance_init = get_distance_between(rock_1, rock_2)
distance = get_distance_between(rock_1, rock_2)

#Pairing mechanism utilized for relative shifting of acceleration direction
#E.g. If rock_1 has positive acceleration, roock_2 will have negative
rock_1.pair = rock_2.name

simulation_question()
ax_1 = initiate_simulation()
traj_x_1, traj_x_2, traj_y_1, traj_y_2, traj_z_1, traj_z_2 = [],[],[],[],[],[]

c = 0

touching = (rock_1.radius+rock_2.radius).value

first_loop = True
#If objects are touching, then end simulation
while distance >= touching:
    
    if first_loop == True:
        step_time = step_time_init
    else:
        #get the new step size
        step_time = get_step_size(rock_1, rock_2, step_time, dis_thresh, step_time)
     
    old_step = step_time #Record prvious_step_size
    
    old_rock_1 = rock_1 
    
    #Get position of rock_1 based on  gravity from rock_2
    rock_1.pos_x, rock_1.pos_y, rock_1.pos_z = get_next_position(rock_1, rock_2, step_time)
    
    #Get position of rock_2 based on gravity from old_rock_1
    rock_2.pos_x, rock_2.pos_y, rock_2.pos_z = get_next_position(rock_2, old_rock_1, step_time)
    
    distance = get_distance_between(rock_1, rock_2)
    
    #If step size of time hasn't changed
    if old_step == step_time:
        #Start from scratch in counting the consecutive number of times
        #that the stepsize has been adjusted to a greater value
        step_change_num = 0
    elif step_time > old_step: 
        step_change_num += 1
        step_time /= 1e15
        continue #Do not display it, but rerun the while loop to get a 
                 #new step_size that doesnt bring "repulsion" characteristics
    
    simulate(ax_1, rock_1, rock_2)
    
    #Variable shifts to state that we are no longer in the first loop
    first_loop = False
    # print(f'{c}')
    # print(f'step_time = {step_time}')
    # print(f'distance = {distance}')
    # a_mag = (rock_1.a_x**2 + rock_1.a_y**2 + rock_1.a_z**2)**0.5
    # print(f'acceleration = {a_mag}')
    # c += 1

else:
    print('______________________________')
    print("Completed running simulation")
