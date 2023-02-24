from data_structure_library import *
import numpy as np
import matplotlib.pyplot as plt
class Bot():
    def __init__(self, nrow, ncol, color) -> None:
        self.nrow = nrow
        self.ncol = ncol
        self.tree = None
        self.color = color
    
    def conv(self, row):
        return self.nrow - row
    
    def plot(self):
        # plot vertices
        for node in self.tree.V:
            plt.plot(node.col, self.nrow-node.row, 'ro')
        for edge in self.tree.E:
            x_arr = [edge[0].col, edge[1].col]
            y_arr = [self.conv(edge[0].row), self.conv(edge[1].row)]
            plt.plot(x_arr, y_arr, self.color, linestyle="dotted")