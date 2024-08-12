from tkinter import *
import numpy as np
import matplotlib.pyplot as plt
import time
from drone import Drone
from sub_area import *

canvas_size = 600 # Size of tkinter window's width/heigth
exit_simulation = False # Used to end simulation early if user presses escape
tick_l = 0.01

# Called when user presses the escape key to let the simulation know to stop early
def stop_simulation():
    global exit_simulation 
    exit_simulation = True

# This handles the creation and updating of all simulation relevant objects and graphics
def run_simulation(trajectory, area_radius, num_drones, square_side, drone_coverage, cycles):

    # Our window object
    root = Tk()

    # Our windows canvas to places objects onto
    canvas = Canvas(root, width = canvas_size, height = canvas_size, bg="white")

    # The circular area the drones will cover
    area = canvas.create_oval(canvas_size/2 - canvas_size/3, canvas_size/2 - canvas_size/3, canvas_size/2 + canvas_size/3, canvas_size/2 + canvas_size/3)

    # Very important this allows the visual size of the area to remain the same with all other objects scaling accordingly
    scale_factor = canvas_size / (3 * area_radius)
    
    # The actual length of the areas radius
    scaled_area_radius = area_radius * scale_factor

    # Binds the escape key to call the stop_simulation function
    root.bind("<Escape>",lambda e:stop_simulation())

    #------- Generate Sub areas -------
    
    # Create a list of our sub area objects that tile our circular area
    sub_area_list = generate_sub_areas(canvas, scaled_area_radius, square_side * scale_factor)
    
    # This loads created canvas and updates it
    canvas.pack()
    root.update()

    #------- Generate drones ------

    drone_list = [None] * num_drones
    
    # Actual size of drone coverage radius
    scaled_drone_coverage = drone_coverage * scale_factor

    # Create drone objects for the number of drones the user inputted
    for i in range(0, len(drone_list)):
        drone_list[i] = Drone(trajectory, canvas, canvas_size, i+1, scaled_area_radius, scale_factor, num_drones, scaled_drone_coverage, cycles)

    #------ Begin simulation -------

    t = 0   # Our "time" variable t, time is not exact its really just about keeping track of the number of updates
    run = True  # Keep track of whether the simulation should continue running or not
    
    while (run):

        # Remember we binded escape to make exit_simulation = True
        if exit_simulation:
            break
        
        else:
            start = time.time() # Used to measure time this update takes
            t+=1    # Incremement t
            run = False   # We initially assume run as False then correct if any of the drones have not finished

            #----------- Move each drone ----------

            for i, drone in enumerate(drone_list):

                if drone.finished == False:

                    run = True # Here we ensure the simulation runs another update if there are still unfinished drones
                    
                    # Depending of which trajectory was chosen will choose which update function should run on the drones
                    match trajectory:

                        # Radial trajectory update
                        case 1:
                            #Each drone starts moving at ith second at .01 sleep time
                            if (t/10  > i):   # This offsets the time at which each drone starts updating
                                drone.active = True    # Although we only need to set this once when the drone starts this is just the easiest way to fit this into the structure but this becomes redundant for the most part setting every update
                                drone.next_radial(sub_area_list)

                        # Ring trajectory update  
                        case 2:
                            drone.active = True
                            drone.next_ring(sub_area_list)
                        
                        # Spiral trajectory update
                        case 3:
                            if (t/10 > 2*drone.tau + drone.T):
                                drone.active = True
                                # We hand in a time value based on the number of ticks our program has been running, this must be adjusted back to 0 every cycle
                                drone.next_spiral(sub_area_list, (((t- 10*(2*drone.tau + drone.T)) - ((drone.totalcycles - drone.cycles) * (drone.tau/tick_l))) * tick_l))
            
            end = time.time()   # End time of the update

            # This ensure that the simulation does not perform an update more than once every 0.01 seconds
            if end - start < tick_l:
                time.sleep(tick_l - (end-start)) # Sleep the remaining difference if the update time is less than 0.01 seconds
            root.update() # Update canvas

    # Once updates are complete create a list to store our coverage results in
    results = [None] * len(sub_area_list)

    # Store coverage results, the index in this list corrosponds to the sub_areas id so no need to store extra information
    for i, area in enumerate(sub_area_list):
        results[i] = area.covered

    return results


# This create the squares that will tile our circular area and allow us to measure what part of the the circle each drone is covering
def generate_sub_areas(canvas, scaled_area_radius, scaled_square_side):
    
    id = 0 # Incrementing ID to be assigned to each new sub area
    areas_per_quad = 0 # This is important for how we will place sub areas into our list
    cursor = 0  # Corrosponds to which row we are currently creating
    row_length = scaled_area_radius # We start at the base of our first quadrant where the row length is the radius

    # This actually calculates the areas per quad, very similar code is used to actually create the sub areas but first we need to know this number
    # The -(x // -y) actually performs a ceiling divide. if the initial row lenght is 100 and the square side length is 30, it will take 4 squares to form that row
    for i in range(0, int(-(scaled_area_radius // -scaled_square_side))):
        areas_per_quad += int(-(row_length//-scaled_square_side))
        
        cursor += scaled_square_side
        row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor)))

    sub_area_list = [None] * (areas_per_quad * 4)   # 4 quadrants * number of sub areas per quadrant to create the correct list size
    cursor = 0  # Reset cursor
    row_length = scaled_area_radius # Reset initial row length
    
    # This iterates the sub area's for a single quadrant row by row
    # Every time we make a sub area we create 3 additional sub areas with flipped coordinates corrosponding to the other quadrants
    # This is why we place each sub area we create in the list according to which quadrant it belongs to to still have the sub areas in a certain order
    for i in range(0, int(-(scaled_area_radius // -scaled_square_side))):
        for j in range(0, int(-(row_length//-scaled_square_side))):
            for quad in range(0,4):
                sub_area_list[quad*areas_per_quad + id] = Sub_area(canvas, i, j, scaled_square_side, quad, quad*areas_per_quad + id)
            id+=1
    
        cursor += scaled_square_side

        row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor))) # Rearrangement of pythag to solve for "opposite" edge

    return sub_area_list

def generate_sub_areas_less(canvas, scaled_area_radius, scaled_square_side):
    
    id = 0 # Incrementing ID to be assigned to each new sub area
    areas_per_quad = 0 # This is important for how we will place sub areas into our list
    cursor = scaled_square_side  # Corrosponds to which row we are currently creating
    row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor))) # We start at the base of our first quadrant where the row length is the radius
    rows = float(np.sqrt(np.absolute(scaled_square_side*scaled_square_side - scaled_area_radius*scaled_area_radius)))
    # This actually calculates the areas per quad, very similar code is used to actually create the sub areas but first we need to know this number
    # The -(x // -y) actually performs a ceiling divide. if the initial row lenght is 100 and the square side length is 30, it will take 4 squares to form that row
    for i in range(0, int(rows // scaled_square_side)):
        areas_per_quad += int(row_length // scaled_square_side)
        
        cursor += scaled_square_side
        row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor)))

    sub_area_list = [None] * (areas_per_quad * 4)   # 4 quadrants * number of sub areas per quadrant to create the correct list size
    cursor = scaled_square_side  # Reset cursor
    row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor))) # Reset initial row length
    
    # This iterates the sub area's for a single quadrant row by row
    # Every time we make a sub area we create 3 additional sub areas with flipped coordinates corrosponding to the other quadrants
    # This is why we place each sub area we create in the list according to which quadrant it belongs to to still have the sub areas in a certain order
    for i in range(0, int(rows // scaled_square_side)):
        for j in range(0, int(row_length // scaled_square_side)):
            for quad in range(0,4):
                sub_area_list[quad*areas_per_quad + id] = Sub_area(canvas, i, j, scaled_square_side, quad, quad*areas_per_quad + id)
            id+=1
    
        cursor += scaled_square_side

        row_length = float(np.sqrt(np.absolute(scaled_area_radius*scaled_area_radius - cursor*cursor))) # Rearrangement of pythag to solve for "opposite" edge

    return sub_area_list

# Handle user inputs and results of the simulation
def main():

    again = True # Bool to keep track of if the user wants to continue running simulations

    while (again==True):

        # These are all the user inputs, they start at negative one for input verification purposes

        # This determines which trajectory the drones will follow
        trajectory = -1

        # This is the radius of each drones coverage circle
        drone_coverage = -1

        # Radius of the circular area the drones will cover
        area_radius = -1
        
        # Number of drones to each complete the trajectory specified
        num_drones = -1

        # Side length of the sub region squares that tile the circular area
        square_side = -1

        # Number of trajectory repetitions to complete
        cycles = -1

        # Whether or to save results
        saveresults = -1

        # The following is the code that gathers user inputs, try/excepts are usually to be avoided
        # but this is what it easiest to make sure users are entering valid inputs.

        while (trajectory not in ['1','2','3']):
            trajectory = input("Select simulation trajectory:\n1: Radial\n2: Ring\n3: Spiral\n")

            if trajectory not in ['1','2','3']:
                print("Invalid Input\n")
        
        trajectory = int(trajectory)

        while (area_radius <= 0):
            try:
                area_radius = int(input("\nEnter area radius: "))
            
            except:
                print("Invalid Input\n")
        
        while (num_drones <= 0):
            try:
                num_drones = int(input("\nEnter number of drones: "))
            except:
                print("Invalid input\n")
        
        while (drone_coverage < 0 or drone_coverage > area_radius):
            try:
                drone_coverage = int(input("\nEnter drone coverage radius:\n(Must be smaller than area radius): "))
            except:
                print("Invalid Input\n")
        
        # we do not want the sub area side lengths to less than a single unit
        while (square_side < max(1, area_radius/(canvas_size/3)) or square_side > area_radius):
            try:
                square_side = int(input("\nEnter the side length of sub-areas\n(Smaller sub areas means more accurate coverage but worse performance)\n(Must be smaller than area radius): "))
            except:
                print("Invalid input\n")

        while (cycles < 1):
            try:
                cycles = int(input("\nEnter the number of cycles to complete\n(Number of times each drone will repeat its path): "))
            except:
                print("Invalid Input\n")

        coverage_results = np.asarray(run_simulation(trajectory, area_radius, num_drones, square_side, drone_coverage, cycles))

        #------ Graphing ------
        sub_area_ids = np.linspace(0, len(coverage_results) - 1, num=len(coverage_results))
        
        fig, ax = plt.subplots()
        ax.bar(sub_area_ids, coverage_results)
        plt.show()

        #---- Save Results ----
        while saveresults not in ["n", "y", "Y", "N"]:
            saveresults = input("Would you like to save result (Y/N): ")

            if saveresults not in ["n", "y", "Y", "N"]:
                print("Invalid Input\n")
        
        if saveresults in ["y", "Y"]:
            savename = input("Enter image name to save to: ") + ".png"
            fig.savefig(savename)
        
        #----- Run Again -----
        while again not in ["n", "y", "Y", "N"]:
            again = input("Would you like to run simulation again (Y/N): ")

            if again not in ["n", "y", "Y", "N"]:
                print("Invalid Input\n")
        
        if again in ["y", "Y"]:
            again = True
        else:
            again = False

main()