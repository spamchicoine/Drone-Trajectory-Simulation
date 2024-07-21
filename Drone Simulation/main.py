from asyncio.windows_events import NULL
from tkinter import *
import numpy as np
#import matplotlib as mp
from time import *
from drone import *
from sub_area import *

canvas_size = 600

def run_simu(trajectory, radius, num_drones, square_side, drone_coverage, run):

    root = Tk()

    canvas = Canvas(root, width = canvas_size, height = canvas_size, bg="white")

    area = canvas.create_oval(canvas_size/2 - 200, canvas_size/2 - 200, canvas_size/2 + 200, canvas_size/2 + 200)

    scale_factor = canvas_size / (3 * radius)

    #------- Generate Sub areas -------
    
    cursor = 0
    scaled_area_radius = radius * scale_factor
    row_length = scaled_area_radius
    scaled_square_side = square_side * scale_factor
    sub_area_list = []
    
    for i in range(0, int(-(scaled_area_radius // -scaled_square_side))):
        for j in range(0, int(-(row_length//-scaled_square_side))):
            for quad in range(0,4):
                sub_area_list.append( Sub_area(canvas,i,j,scaled_square_side,quad))
        
        cursor += scaled_square_side

        row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor)))

    canvas.pack()
    root.update()

    #------- Generate drones ------

    drone_list = []

    scaled_drone_coverage = drone_coverage * scale_factor

    for i in range(0,num_drones):
        drone_list.append(Drone(trajectory, canvas, i+1, scaled_area_radius, scale_factor, num_drones, scaled_drone_coverage, sub_area_list))

    #------ Begin simulation -------

    t = 0
    tau = 1
    
    while (run):
        t+=1
        
        #----------- Move each drone ----------

        for i, drone in enumerate(drone_list):
            if (trajectory == 1):
                if (t/10  > i and drone.finished == False):  #Each drone starts moving at ith second at .01 sleep time
                    drone.active = True
                    drone.next_radial()
            
            elif (trajectory == 2):
                if drone.finished == False:
                    drone.active = True
                    drone.next_ring()
        
        
        sleep(.1)
        root.update()
        
        if (t>1000):
            run = False
            return sub_area_list
        

    root.mainloop()

def main():
    run = True
    again = True
    trajectory = NULL
    drone_coverage = -1
    radius = -1
    num_drones = -1
    square_side = -1

    while (again==True):

        while (trajectory not in ['1','2']):
            trajectory = input("Select simulation trajectory:\n1: Radial\n2: Ring\n")
        
        trajectory = int(trajectory)

        while (radius < 0 or radius > 299):
            radius = int(input("\nEnter area radius:\n(Must be smaller than 300 with current window size)\n(Units are abritray adjust so your radius is less than 300 units): "))
        
        while (num_drones < 0):
            num_drones = int(input("\nEnter number of drones: "))
        
        while (drone_coverage < 0 or drone_coverage > radius):
            drone_coverage = int(input("\nEnter drone coverage radius:\n(Must be smaller than area radius): "))

        while (square_side < 0 or square_side > radius):
            square_side = int(input("\nEnter the side length of sub-areas\n(Smaller sub areas means more accurate coverage but worse performance)\n(Must be smaller than radius): "))

        coverage_results = run_simu(trajectory, radius, num_drones, square_side, drone_coverage, run)

        for i in coverage_results:
            print(i.covered)


main()