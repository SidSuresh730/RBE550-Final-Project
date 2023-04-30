import maze_generation
import numpy as np
from mapper_robot import MapperBot
from rrt_robot import RRTBot
from a_star import ABot
from data_structure_library import Node

def main():
	print("Project Update")
	# ---- Run Maze Generation code
	num_rows = 4 # Number of rows in the maze
	num_cols = 4 # Number of columns in the maze
	num_fires_smol = 1 # Number of 1x1 in the maze
	num_fires_med = 0 # Number of 2x2 in the maze
	num_fires_lrg = 0 # Number of 3x3 in the maze
	num_inside = 8 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = False
	
	[maze, fires, entrances] = maze_generation.generate_maze(num_rows, num_cols, num_inside, num_fires_smol, num_fires_med, num_fires_lrg, num_ent, plot_maze)
	# A* will be provided an even lower rez version of the current maze, use that in the loop
	
	print("Generating Bots")
	terminator = ABot(len(maze), len(maze[0]), color='cyan', x=entrances[0][1], y=entrances[0][0], theta=entrances[0][2])
	
	#terminator.maze = maze
	#terminator.big_maze = maze_generation.maze_expansion(terminator.maze, num_inside)
	terminator.big_maze = maze
	terminator.goal = (fires[0].row, fires[0].col)
	
	# A* Step Function Here
	count = 0
	while 69:
		count +=1
		print(count)
		check = terminator.step()
		if check:
			break
	
	maze_generation.plot(field=terminator.big_maze, path=terminator.loc_path, bot=None, fires=fires)
	
if __name__ == "__main__":
	main()
