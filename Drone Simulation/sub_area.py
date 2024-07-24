from tkinter import *
import numpy as np

class Sub_area:

    def __init__(self, canvas, i, j, scaled_square_side, quad):
        self.canvas = canvas
        self.covered = 0
        self.side_length = scaled_square_side

        if (quad == 0 or quad == 3):
            self.x1 = 300 + self.side_length*j
            self.x2 = 300 + self.side_length*(j+1)
        else:
            self.x1 = 300 - self.side_length*(j+1)
            self.x2 = 300 - self.side_length*j
        
        if (quad == 0 or quad == 1):
            self.y1 = 300 - self.side_length*(i+1)
            self.y2 = 300 - self.side_length*i
        else:
            self.y1 = 300 + self.side_length*i
            self.y2 = 300 + self.side_length*(i+1)


        self.area = self.canvas.create_rectangle(self.x1, self.y1, self.x2, self.y2)

        self.centerx = self.x1 + scaled_square_side/2
        self.centery = self.y1 + scaled_square_side/2