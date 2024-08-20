from tkinter import *
import numpy as np

class Sub_area:

    def __init__(self, canvas, canvas_size ,i, j, scaled_square_side, quad, id):
        self.canvas = canvas    # The canvas object to draw our sub area onto
        self.canvas_size = canvas_size  # Size of canvas side length
        self.covered = 0    # The amount of coverage this sub area has recieved
        self.side_length = scaled_square_side   # The side length of our sub area
        self.id = id    # The ID this sub area uses, corrosponding to its index

        # Again probably where child classes come in but uh... no? also beccause the quadrants kindof do a criss cross of what they share
        # Basically this is what actually flips coordinates to more efficiently tile the circular area
        if (quad == 0 or quad == 3):
            self.x1 = self.canvas_size/2 + self.side_length*j
            self.x2 = self.canvas_size/2 + self.side_length*(j+1)
        else:
            self.x1 = self.canvas_size/2 - self.side_length*(j+1)
            self.x2 = self.canvas_size/2 - self.side_length*j
        
        if (quad == 0 or quad == 1):
            self.y1 = self.canvas_size/2 - self.side_length*(i+1)
            self.y2 = self.canvas_size/2 - self.side_length*i
        else:
            self.y1 = self.canvas_size/2 + self.side_length*i
            self.y2 = self.canvas_size/2 + self.side_length*(i+1)
        
        # Top left is (x1,y1), bottom right is (x2, y2) that is just tkinter covnention for creating squares so we will calculate our center
        self.centerx = self.x1 + scaled_square_side/2
        self.centery = self.y1 + scaled_square_side/2

        self.area = self.canvas.create_rectangle(self.x1, self.y1, self.x2, self.y2)    # Create our corrosponding square on the canvas
        self.id_label = self.canvas.create_text(self.centerx, self.centery, text=str(self.id))  # Create a label on our canvas for each sub area