from tkinter import *
import numpy as np

class Drone:

    def __init__(self, trajectory, canvas, i, scaled_area_radius, scale_factor, num_drones, scaled_drone_coverage, sub_area_list):
        self.trajectory = trajectory
        self.sub_area_list = sub_area_list
        self.canvas = canvas
        self.drone_num = i
        self.area_radius = scaled_area_radius
        self.coverage_radius = scaled_drone_coverage
        self.scale_factor = scale_factor
        self.tau = 100

        self.active = False
        self.finished = False

        match self.trajectory:
            
            case 1:
                self.x = 300
                self.y = 300
                self.theta = (2*np.pi / num_drones) * i
                self.flip = False
            
            case 2:
                self.orbit_radius = np.sqrt(i/(num_drones+1))*self.area_radius
                self.x = 300 + self.orbit_radius
                self.y = 300
            
            case 3:
                self.theta = (2*np.pi / num_drones) * i
                self.spiral_coords = self.create_spiral_coords()
                self.x = 300
                self.y = 300
                self.sci = 0 #Index in spiral_coords
                
        self.R = np.sqrt((self.x - 300)*(self.x - 300) + (self.y - 300)*(self.y - 300))
        
        self.drone_point = self.canvas.create_oval(self.x - self.coverage_radius, self.y - self.coverage_radius, self.x + self.coverage_radius, self.y + self.coverage_radius,fill='blue')
        
        
    #------ Update drone for trajectory 1 -------
    def next_radial(self):
        #--- Coverage is an abritrary measurement of the amount of time a sub area has been within the coverage radius of a drone
        coverage = 0

        #--- Amounts to change in x and y to move 1 unit
        v1x = np.cos(self.theta)
        v1y = np.sin(self.theta)*-1

        while coverage < 1:
            if self.active == True:

                #--- Distance from drones center to the "origin" (circle areas center)
                self.R = np.sqrt((self.x - 300)*(self.x - 300) + (self.y - 300)*(self.y - 300))


                v = (self.area_radius * self.area_radius) / (self.tau*(self.R + 1))
                #v = 10 * self.scale_factor
                
                #--- Checks if drone has reached the circle areas edge
                if (self.R + self.coverage_radius > self.area_radius):
                    self.flip = True
                    
                #--- Check if drone has returned to the center, if so that drone paths is complete
                if (self.flip == True and self.R < 1):
                    self.canvas.delete(self.drone_point)
                    self.active = False
                    self.finished = True
                    return
                    
                    #--- We move 1 unit at a time if the velocity is greater than 1
                if coverage + 1/v <= 1:

                    self.check_covered_V2(1/v)
                    coverage += 1/v

                    pass
                    
                    if self.flip == True:
                        self.x -= v1x
                        self.y -= v1y
                        self.canvas.move(self.drone_point, -v1x, -v1y)
                    
                    else:
                        self.x += v1x
                        self.y += v1y
                        self.canvas.move(self.drone_point, v1x, v1y)

                    #--- Obviously is the velocity is no an integer we need to be able to process the remaining velocity once it is below 1 but still above 0
                else:

                    #--- Amounts to change in x and y to move the decimal portion of v, 0 if v is an integer
                    r = v*(1-coverage)

                    vrx = np.cos(self.theta)*r
                    vry = np.sin(self.theta)*r*-1

                    self.check_covered_V2(1-coverage)
                    coverage = 1

                    pass

                    if self.flip == True:
                        self.x -= vrx
                        self.y -= vry
                        self.canvas.move(self.drone_point, -vrx, -vry)
                    
                    else:
                        self.x += vrx
                        self.y += vry
                        self.canvas.move(self.drone_point, vrx, vry)
                    
                    # At each
                pass

    def next_ring(self):
        
        # Make the drone move at most one unit at a time perpendicular to the line formed between itself and the area center
        v = 2*np.pi*self.orbit_radius/self.tau

        vc = v

        while vc > 0:

            if self.active == True:

                distancex = self.x - 300
                distancey = self.y - 300
                
                if vc >= 1:

                    self.check_covered_V2(1/v)

                    v1x = abs(distancey / self.orbit_radius)
                    v1y = abs(distancex / self.orbit_radius)

                    if distancex >= 0:
                        self.y -= v1y
                        self.canvas.move(self.drone_point, 0, -v1y)
                    
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
                        
                    vc -= 1
                
                elif vc > 0:
                    self.check_covered_V2(vc/v)

                    vrx = abs(distancey / (self.orbit_radius/vc))
                    vry = abs(distancex / (self.orbit_radius/vc))

                    if distancex >= 0:
                        self.y -= vry
                        self.canvas.move(self.drone_point, 0, -vry)
                    
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

                if (((distancex > 0) and ((self.x - 300) > 0)) and ((self.y - 300) * distancey < 0)): #Probably a bad way of checking if the drones have compeleted one rotation but it seems to work
                    self.active = False
                    self.finished = True
                    self.canvas.delete(self.drone_point)
                    vc = 0 #Stop loop
    

        # Accounting for floating points error
        dx = self.x - 300
        dy = self.y - 300

        self.R = np.sqrt(dx*dx + dy*dy)
        
        # Basic premise, we know that the the drone is sposed to orbit at self.orbit_radius, so see how far off the actual distance is and scale accordingly
        error_scale = self.orbit_radius/self.R

        self.canvas.move(self.drone_point, (dx * error_scale) - dx, (dy * error_scale) - dy)

        self.x = (dx * error_scale) + 300
        self.y = (dy * error_scale) + 300

    def next_spiral(self):

        newx = self.spiral_coords[self.sci][0] + 300
        newy = self.spiral_coords[self.sci][1] + 300

        dx = self.x - newx
        dy = self.y - newy

        self.canvas.move(self.drone_point, dx, dy)

        self.x = newx
        self.y = newy

        self.sci+=1

    def check_covered_V2(self,coverage):

        for area in self.sub_area_list:

            w = area.side_length/2

            distancex = np.abs(self.x - area.centerx)
            distancey = np.abs(self.y - area.centery)

            cornerdistance = ((distancex - w)*(distancex - w)) + ((distancey - w)*(distancey - w))

            if ((distancex > (w + self.coverage_radius)) or (distancey > (w + self.coverage_radius))):
                area.covered+=0
            
            elif ((distancex < w) or (distancey < w)):
                area.covered+=coverage

            elif cornerdistance < self.coverage_radius*self.coverage_radius:
                area.covered+=coverage

    def create_spiral_coords(self):

        coord_list =[]

        pi_sq = np.pi*np.pi
        tal = self.area_radius * (2*((1+pi_sq)**(3/2) - 1))/(3*pi_sq) # Total arc length

        ctal = 0 #keep track of tal as we work through it

        while tal > 0:

            if tal > 1:
                ctal += 1
                htag = 1
                tal -= 1
            
            elif tal > 0:
                ctal += tal
                htag = tal
                tal -= tal

            theta = np.sqrt(((12*pi_sq*ctal + self.area_radius*8)/self.area_radius)**(2/3)-4) #Theta value of a point ctal arc length into the spiral

            hypot = (self.area_radius*theta**2) / (4*pi_sq)
            x = hypot*np.cos(theta+self.theta)
            y = hypot*np.sin(theta+self.theta)

            coord_list.append((x,y,htag))
        
        return coord_list