from tkinter import *
import numpy as np
import random

class Drone:

    def __init__(self, trajectory, canvas, canvas_size, i, scaled_area_radius, scale_factor, num_drones, scaled_drone_coverage, cycles):
        self.trajectory = trajectory # Which trajectory this drone will use
        self.cycles = cycles    # Repetitions to complete of drone's trajectory
        self.canvas = canvas    # Each drone needs to have reference to the canvas to draw itselfs and create updates
        self.canvas_size = canvas_size  # I wanted to import this but it causes a circular import even if your just specifying the global so I just pass it in instead
        self.drone_num = i  # Drones in the list adopt a number corrosponding to their index + 1
        self.area_radius = scaled_area_radius   # In general tkinter doesnt keep of where objects are located or there dimensions so all the important measurements and coordinates are kept track of
        self.coverage_radius = scaled_drone_coverage    # Radius of the area this drone will cover
        self.scale_factor = scale_factor    # Used to ensure sizing is made consistent
        
        self.active = False     # Indicates whether or not a drone should be updating
        self.finished = False   # Indicates if a drone has completed its cycles of its trajectory

        # This is the part where child classes are sposed to come in but too bad
        match self.trajectory:
            
            # Radial specific attributes
            case 1:
                self.tau = 100  # Used as an inverse scaling factor for velocity
                self.x = self.canvas_size/2    # Drones start at center
                self.y = self.canvas_size/2
                self.rotation = (2*np.pi / num_drones) * i # Angle at which the drone will travel, split between drones spreading them out evenly
                self.flip = False   # Used to indicate if the drone has reach the edge and should begin traveling back
            
            # Ring specific attributes
            case 2:
                self.tau = 100  # Inverse scaling factor for velocity
                self.orbit_radius = np.sqrt(i/(num_drones+1))*self.area_radius  # Radius at which the drone should orbit
                self.x = self.canvas_size/2 + self.orbit_radius # Drone starts at its orbit radius
                self.y = self.canvas_size/2
                self.v = 2*np.pi*self.orbit_radius/self.tau     # Each drone's velocity remains constant so we can just delcare it
                self.coord_index = 0 #Index in coords_list (Unused)
            
            # Spiral specific attributes
            case 3:
                self.tau = 4    # In this case tau will be the amount of time in seconds the spiral will complete in.
                self.T = random.random() * self.tau     # Used for timing the drone
                self.rotation = random.random() * 2*np.pi
                self.theta = 0
                self.pi_squ = np.pi * np.pi # Define this constant here
                self.x = self.canvas_size/2
                self.y = self.canvas_size/2
                self.totalcycles = self.cycles
                
        # This is the distance between the circular areas center and the drones center, used for many checks
        self.R = np.sqrt((self.x -self.canvas_size/2)*(self.x -self.canvas_size/2) + (self.y -self.canvas_size/2)*(self.y -self.canvas_size/2))
        
        # Create the circular object on the canvas representing the drone
        self.drone_point = self.canvas.create_oval(self.x - self.coverage_radius, self.y - self.coverage_radius, self.x + self.coverage_radius, self.y + self.coverage_radius,fill='blue')
        
        
    #------ Update drone for trajectory 1 -------
    def next_radial(self, sub_area_list):
        
        # Coverage is an abritrary measurement of the amount of time a sub area has been within the coverage radius of a drone
        coverage = 0

        # Because velocity is changing relatively continously we perform "mini" updates of 1 unit or less until the coverage has reached a whole unit signifying this update is complete
        while coverage < 1:
            if self.active == True:

                # Distance from drones center to the "origin" (circle areas center)
                self.R = np.sqrt((self.x -self.canvas_size/2)*(self.x -self.canvas_size/2) + (self.y -self.canvas_size/2)*(self.y -self.canvas_size/2))
                
                # In radial velocity is constantly changing based on the distance of the drone's center to the areas center
                v = (self.area_radius * self.area_radius) / (self.tau*(self.R + 1))
                
                # Checks if drone has reached the circle areas edge, if so then the drone will begin moving back
                if (self.R > self.area_radius):
                    self.flip = True
                    
                # Check if drone has returned to the center, if so a cycle has been completed. If all cycles complete then the drone is finished
                if (self.flip == True and self.R < 1):
                    self.cycles -= 1
                    
                    if self.cycles == 0:
                        self.canvas.delete(self.drone_point)
                        self.active = False
                        self.finished = True
                        return
                    
                    self.flip = False
                    
                # We must avoid movements greater than one unit at a time. Thus velocities are broken down into single unit moves then a remainder if neccesary
                # Of course then coverage must be scaled based on the portion of the velocity being processed. Coverage per update must always add up to 1 unit of coverage
                # In this check we are checking if a move of 1 unit will keep the total coverage equal to or less than 1 unit of coverage
                if coverage + 1/v <= 1:
                    
                    # Amounts to change in x and y to move 1 unit
                    v1x = np.cos(self.rotation)
                    v1y = np.sin(self.rotation)*-1

                    self.check_covered_areas_V2(1/v, sub_area_list) # Check which areas are being covered
                    coverage += 1/v    # Wpdate this updates total coverage
                    
                    # If we are moving in the other direction make negative movements
                    if self.flip == True:

                        # Adjust our drones x and y coordinates
                        self.x -= v1x
                        self.y -= v1y

                        self.canvas.move(self.drone_point, -v1x, -v1y)  # Move the circle corrosponding to our drone on our canvas
                    
                    else:

                        # Adjust our drones x and y coordinates
                        self.x += v1x
                        self.y += v1y

                        self.canvas.move(self.drone_point, v1x, v1y) # Move the circle corrosponding to our drone on our canvas

                # Otherwise the remaining coverage corrosponds to a movement less than 1 unit
                else:

                    r = v*(1-coverage)  # The amount to move to make our total coverage equal to 1 after we've done as many movements of 1 unit possible

                    # Amount to move in x and y to move this remaining distance
                    vrx = np.cos(self.rotation)*r
                    vry = np.sin(self.rotation)*r*-1

                    self.check_covered_areas_V2(1-coverage, sub_area_list)    # Check the regions coverage and allocate the last bit of coverage for this update
                    coverage = 1 # Realistically should be coverage += r/v but because of floating point errors that might not actually make our total coverage one but mathimatically it should be so just set it


                    if self.flip == True:

                        # Adjust our drones x and y coordinates
                        self.x -= vrx
                        self.y -= vry

                        self.canvas.move(self.drone_point, -vrx, -vry) # Move the circle corrosponding to our drone on our canvas
                    
                    else:

                        # Adjust our drones x and y coordinates
                        self.x += vrx
                        self.y += vry

                        self.canvas.move(self.drone_point, vrx, vry) # Move the circle corrosponding to our drone on our canvas

    #------ Update drone for trajcetory 2 -------
    def next_ring(self, sub_area_list):
        
        # Make the drone move at most one unit at a time perpendicular to the line formed between itself and the area center
        v = 2*np.pi*self.orbit_radius/self.tau

        vc = v  # keep track of the portion of velocity handled

        # We will work through our velocity at this update 1 unit at time and then a remainder if there is one
        while vc > 0:

            if self.active == True:
                
                # Distance from the center of the area to the center of our drone
                distancex = self.x - 300
                distancey = self.y - 300
                
                # If remaining velocity to process is greater than 1 we will move 1 unit
                # Remember we do not want to move more than one unit at a time we must break an update based on velocity into smaller movements
                if vc >= 1:

                    self.check_covered_areas_V2(1/v, sub_area_list)    # Check areas covered

                    # This has been greatly simplified but this is the distance to move 1 unit along the tanget of the line drawn between the areas center and the drones center, continously moving along this tangent results in an orbit
                    v1x = abs(distancey / self.orbit_radius) 
                    v1y = abs(distancex / self.orbit_radius)

                    # Depending on which quadrant you want to move these amounts either negatively or positively
                    # If distancex is greater than 0 we are in quadrant 1 or 4 in which case y should decrease (remember y axis is flipped)
                    if distancex > 0:
                        self.y -= v1y
                        self.canvas.move(self.drone_point, 0, -v1y)     # Move corrosponding object on canvas
                    
                    elif distancex == 0:
                        pass

                    else:
                        self.y += v1y
                        self.canvas.move(self.drone_point, 0, v1y)
                    
                    if distancey > 0:
                        self.x += v1x
                        self.canvas.move(self.drone_point, v1x, 0)

                    elif distancey == 0:
                        pass

                    else:
                        self.x -= v1x
                        self.canvas.move(self.drone_point, -v1x, 0)
                        
                    vc -= 1   # Decrement vc by the amount we moved
                
                # Handle he remainder if there is one
                elif vc > 0:

                    self.check_covered_areas_V2(vc/v, sub_area_list)   # Check covered sub areas

                    # Distance to move the remainder along the tanget of the line drawn between the areas center and the drones center
                    vrx = abs(distancey / (self.orbit_radius/vc))
                    vry = abs(distancex / (self.orbit_radius/vc))

                    # Add or subtract amounts based on quadrant
                    if distancex > 0:
                        self.y -= vry
                        self.canvas.move(self.drone_point, 0, -vry)   # Move corrosponding object on canvas
                    
                    elif distancex == 0:
                        pass

                    else:
                        self.y += vry
                        self.canvas.move(self.drone_point, 0, vry)
                    
                    if distancey > 0:
                        self.x += vrx
                        self.canvas.move(self.drone_point, vrx, 0)

                    elif distancey == 0:
                        pass

                    else:
                        self.x -= vrx
                        self.canvas.move(self.drone_point, -vrx, 0)
                    
                    vc -= vc

                #Probably a bad way of checking if the drones have compeleted a rotation but it seems to work
                if (((distancex > 0) and ((self.x - 300) > 0)) and ((self.y - 300) * distancey < 0)):
                    
                    self.cycles -= 1    # Decrement cycles every rotation

                    # If all cycles complete then remove the drone
                    if self.cycles == 0:
                        self.active = False
                        self.finished = True
                        self.canvas.delete(self.drone_point)
                        vc = 0 # Stops this update
    
        # Must do something for floating points error as otherwise the drones drift
        # The the drone is supposed to orbit at self.orbit_radius, so see how far off the actual distance is and scale accordingly
        dx = self.x - 300
        dy = self.y - 300

        self.R = np.sqrt(dx*dx + dy*dy)     # Drone distance from center
        
        error_scale = self.orbit_radius/self.R  # How far the drone is off it's orbit scale factor

        self.canvas.move(self.drone_point, (dx * error_scale) - dx, (dy * error_scale) - dy)    # Move the corrosponding object on canvas accordingly

        # Update our x and y coords accordingly
        self.x = (dx * error_scale) + 300
        self.y = (dy * error_scale) + 300

    #------ Update drone for trajectory 3 -------
    def next_spiral(self, sub_area_list, t):

        if (self.active == True):
            
            # Using the time value we get the theta we should be at by time t
            # This functions like velocity but we start with a time value and calculate where the drone should be by that time
            end_theta = 2*np.pi*np.sqrt(np.sqrt(t / self.tau))  # What theta should be at time t
            
            # This is then the arc length we should be at the time value t
            end_arc_l = ((self.area_radius * (((end_theta * end_theta) + 4)**(3/2) - 8) )/ (12 * self.pi_squ))
            
            # We then calculate our current arc length to get the difference of arc length we must travel 
            curr_arc_l = ((self.area_radius * (((self.theta * self.theta) + 4)**(3/2) - 8) )/ (12 * self.pi_squ)) # Current arc length

            arc_move = end_arc_l - curr_arc_l # Arc length to move

            arc_move_left = arc_move    # Extra value to use to keep track of the arc length

            # Now we must move the difference in arc length no more than 1 unit of arc length at a time
            while arc_move_left > 0:

                # Distance of drone's center from area center
                self.R = np.sqrt((self.x - self.canvas_size/2) * (self.x - self.canvas_size/2) + (self.y - self.canvas_size/2) * (self.y - self.canvas_size/2))

                # If drone has reached the edge
                if self.R > self.area_radius:
                    self.cycles-=1  # Compelted a cycle

                    # If all cycles complete remove the drone
                    if self.cycles == 0:
                        self.active = False
                        self.finished = True
                        self.canvas.delete(self.drone_point)
                        return
                    
                    # Otherwise we will reset the drone for the next cycle
                    self.theta = 0     # Reset angle to 0
                    self.rotation = random.random() * 2*np.pi   # Pick a new random rotation
                    self.canvas.move(self.drone_point, 300 - self.x, 300 - self.y)  # Move corrosponding circle on canvas back to center
                    self.x = self.canvas_size/2     # Reset x back to center
                    self.y = self.canvas_size/2     # Reset y back to center
                    return
                
                # If the arc length left for this update is greater than one
                if arc_move_left >= 1:

                    self.check_covered_areas_V2(1/arc_move, sub_area_list)  # Check coverage

                    # Adjust values
                    curr_arc_l += 1
                    arc_move_left -= 1
                
                # Handle remainder arc length to move
                elif arc_move_left > 0:

                    self.check_covered_areas_V2(arc_move_left/arc_move, sub_area_list)  # Check coverage

                    # Adjust values, move should be finished
                    curr_arc_l = end_arc_l
                    arc_move_left = 0
                
                # Theta value of the current arc length
                new_theta = np.sqrt(((12*self.pi_squ*curr_arc_l + self.area_radius*8)/self.area_radius)**(2/3)-4)

                # Distance of drone from center at the new theta
                hypot = (self.area_radius*new_theta**2) / (4 * self.pi_squ)

                # Use distance and theta to get the new x and y values
                new_x = hypot*np.cos(new_theta+self.rotation)
                new_y = hypot*np.sin(new_theta+self.rotation)

                # Get the difference in current and next coordinates
                dx = new_x - (self.x - self.canvas_size/2)
                dy = new_y - (self.y - self.canvas_size/2)

                self.canvas.move(self.drone_point, dx, dy)  # Move the difference

                # Update the drones x and y values
                self.x = new_x + self.canvas_size/2
                self.y = new_y + self.canvas_size/2

                self.theta = new_theta  # Update the drones theta
    

    # This checks which sub areas the drone called on is covering
    def check_covered_areas_V2(self, coverage, sub_area_list):
        sub_area_list_length  = len(sub_area_list)  # We use this value a lot so just declare it here
        indexs = self.get_CCAV2_indexs(sub_area_list_length)   # Get indexs of where in the sub area list we should search
        temp_side_length = sub_area_list[1].side_length   # Side length never changes so reduce number of references just using this

        # This will loop through each pair of indexs (either one or two pairs in our case)
        for pair in range(0,int(len(indexs)/2)):

            # Search the sub area list using each pair of indexs
            for i in range(int(indexs[(1 + 2*pair)-1])-1, int(indexs[(2 + 2*pair)-1])):

                # Check if the sub area we have indexed overlaps with our drone
                if self.check_overlap(sub_area_list[i].centerx, sub_area_list[i].centery, temp_side_length):
                    sub_area_list[i].covered += coverage   # If so add our scaled coverage amount to that sub area

    # Return indexes used to traverse only the parts of our sub area list corrosponding to the quadrants our drone is in
    def get_CCAV2_indexs(self, sub_area_list_length):

        k = self.canvas_size / 1.5
        j = self.canvas_size / 3

        # Get which quadrants the drone is in
        # Note that we are treating each quadrant as a square and not a semi circle, this is okay because the square completely encompasses each quadrant it corrosponds to
        inq1 = self.check_overlap(k, j, j)
        inq2 = self.check_overlap(j, j, j)
        inq3 = self.check_overlap(j, k, j)
        inq4 = self.check_overlap(k, k, j)

        # Might be the worst looking thing I've ever written (One must imagine a certain other simulation programmer) but this determines what part of the sub area list to search depending on which quadrants the drone is in
        # The main trick here is recognizing that checking the combinations of one quadrant greatly reduces the future combinations you check of other quadrants
        if inq1:

            if inq2:

                if inq3:

                    if inq4:
                        return (1, sub_area_list_length) # In quadrants 1,2,3,4
                    else:
                        return (1, sub_area_list_length*0.75) # In quadrants 1,2,3
                else:

                    if inq4:
                        return (1, sub_area_list_length/2, sub_area_list_length*.75 + 1, sub_area_list_length) # In quadrants 1,2,4
                    else:
                        return (1, sub_area_list_length/2) # In quadrants 1,2
            else:

                if inq4:

                    if inq3:
                        return (1, sub_area_list_length/4, sub_area_list_length/2 + 1, sub_area_list_length) # In quadrants 1,3,4
                    else:
                        return (1, sub_area_list_length/4, sub_area_list_length*.75 + 1, sub_area_list_length) # In quadrants 1,4
                else:
                    return (1, sub_area_list_length/4) # In quadrant 1
            
        elif inq2:
            
            if inq3:

                if inq4:
                    return (sub_area_list_length/4 + 1, sub_area_list_length)  # In quadrants 2,3,4
                else:
                    return (sub_area_list_length/4 + 1, sub_area_list_length*.75) # In quadrants 2,3
            else:
                return  (sub_area_list_length/4 + 1, sub_area_list_length/2) # In quadrant 2
        
        elif inq3:

            if inq4:
                return (sub_area_list_length/2 + 1, sub_area_list_length) # In quadrants 3,4
            else:
                return (sub_area_list_length/2 + 1, sub_area_list_length*.75) # In quadrant 3
        
        elif inq4:
            return (sub_area_list_length*.75 + 1, sub_area_list_length) # In quadrant 4

    # Checks if the circular self (drone in this case) overlaps with a square
    def check_overlap(self, squ_centerx, squ_centery, side_length):

        # This assumes self is a circle (drone in this case) and checks if it overlaps a square defined by the other parameters
        w = side_length/2

        # These are absolute meaning it doesnt matter which side the circle is the math will be the same.
        distancex = np.abs(self.x - squ_centerx)
        distancey = np.abs(self.y - squ_centery)
        
        # Easy case, drone cannot possibly be overlapping if its distance in y or x is greater than w plus the radius of the drone
        if ((distancex > (w + self.coverage_radius)) or (distancey > (w + self.coverage_radius))):
            return False
        
        # Then if the distance in x !!OR!! distance in y is less than w we know there is overlap (side case)
        elif ((distancex < w) or (distancey < w)):
            return True
        
        # Then check the literal corner case using the distance from the circle to square's corner
        elif ((distancex - w)*(distancex - w) + (distancey - w)*(distancey - w)) < self.coverage_radius*self.coverage_radius:
            return True