from asyncio.windows_events import NULL
from tkinter import *
import numpy as np
import matplotlib.pyplot as plt
import time
from drone import Drone
from sub_area import *

canvas_size = 600

def run_simu(trajectory, radius, num_drones, square_side, drone_coverage, cycles,run):

    root = Tk()

    canvas = Canvas(root, width = canvas_size, height = canvas_size, bg="white")

    area = canvas.create_oval(canvas_size/2 - canvas_size/3, canvas_size/2 - canvas_size/3, canvas_size/2 + canvas_size/3, canvas_size/2 + canvas_size/3)

    scale_factor = canvas_size / (3 * radius)
    
    scaled_area_radius = radius * scale_factor

    #------- Generate Sub areas -------
    
    sub_area_list = generate_sub_areas(canvas, scaled_area_radius, square_side * scale_factor)
    
    canvas.pack()
    root.update()

    #------- Generate drones ------

    drone_list = []

    scaled_drone_coverage = drone_coverage * scale_factor

    for i in range(0,num_drones):
        drone_list.append(Drone(trajectory, canvas, canvas_size, i+1, scaled_area_radius, scale_factor, num_drones, scaled_drone_coverage, sub_area_list, cycles))

    #------ Begin simulation -------

    t = 0
    tau = 1
    
    while (run):
        start = time.time()
        t+=1
        run = False
        #----------- Move each drone ----------

        for i, drone in enumerate(drone_list):

            if drone.finished == False:

                run = True

                match trajectory:

                    case 1:
                        if (t/10  > i):  #Each drone starts moving at ith second at .01 sleep time
                            drone.active = True
                            drone.next_radial()
                        
                    case 2:
                        drone.active = True
                        drone.next_ring()
                    
                    case 3:
                        drone.active = True
                        drone.next_spiral()

                    case 4:
                        pass
        
        end = time.time()
        if end - start < .01:
            time.sleep(.01 - (end-start))
        root.update()
    
    print((time.time() - start))
    return sub_area_list
    

def generate_sub_areas(canvas, scaled_area_radius, scaled_square_side):
    
    id = 0
    areas_per_quad = 0
    cursor = 0
    row_length = scaled_area_radius

    for i in range(0, int(-(scaled_area_radius // -scaled_square_side))):
        for j in range(0, int(-(row_length//-scaled_square_side))):
            areas_per_quad+=1
            
        cursor += scaled_square_side

        row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor)))

    
    sub_area_list = [None] * (areas_per_quad * 4)
    cursor = 0
    row_length = scaled_area_radius
    
    for i in range(0, int(-(scaled_area_radius // -scaled_square_side))):
        for j in range(0, int(-(row_length//-scaled_square_side))):
            for quad in range(0,4):
                sub_area_list[quad*areas_per_quad + id] = Sub_area(canvas, i, j, scaled_square_side, quad, quad*areas_per_quad + id)
            id+=1
    
        cursor += scaled_square_side

        row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor))) # Rearrangement of pythag to solve for "opposite" edge

    return sub_area_list
    
def main():

    run = True
    again = True

    while (again==True):
        trajectory = -1
        drone_coverage = -1
        radius = -1
        num_drones = -1
        square_side = -1
        cycles = -1

        while (trajectory not in ['1','2','3']):
            trajectory = input("Select simulation trajectory:\n1: Radial\n2: Ring\n3: Spiral\n")
        
        trajectory = int(trajectory)

        while (radius <= 0):
            try:
                radius = int(input("\nEnter area radius: "))
            
            except:
                pass
        
        while (num_drones <= 0):
            try:
                num_drones = int(input("\nEnter number of drones: "))
            except:
                pass
        
        while (drone_coverage < 0 or drone_coverage > radius):
            try:
                drone_coverage = int(input("\nEnter drone coverage radius:\n(Must be smaller than area radius): "))
            except:
                pass

        while (square_side < max(1,radius/(canvas_size/3)) or square_side > radius):
            try:
                square_side = int(input("\nEnter the side length of sub-areas\n(Smaller sub areas means more accurate coverage but worse performance)\n(Must be smaller than radius): "))
            except:
                pass

        while (cycles < 1):
            try:
                cycles = int(input("\nEnter the number of cycles to complete\n(Number of times each drone will repeat its path): "))
            except:
                pass

        coverage_results = run_simu(trajectory, radius, num_drones, square_side, drone_coverage, cycles,run)

        for i in coverage_results:
            print(i.id, i.covered)
        pass

main()
