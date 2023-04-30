import maze_generation
import numpy as np
from mapper_robot import MapperBot
from rrt_robot import RRTBot
from a_star import ABot
from data_structure_library import Node

def main():
	print("Project Update\n")
	# ---- Run Maze Generation code
	num_rows = 4 # Number of rows in the maze
	num_cols = 4 # Number of columns in the maze
	num_fires_smol = 1 # Number of 1x1 in the maze
	num_fires_med = 0 # Number of 2x2 in the maze
	num_fires_lrg = 0 # Number of 3x3 in the maze
	num_inside = 8 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = False
	
	print("Generating Maze in Final")
	[maze, fires, entrances] = maze_generation.generate_maze(num_rows, num_cols, num_inside, num_fires_smol, num_fires_med, num_fires_lrg, num_ent, plot_maze)
	# A* will be provided an even lower rez version of the current maze, use that in the loop
	
	print("Running A*")
	terminator = ABot(len(maze), len(maze[0]), color='cyan', x=entrances[0][1], y=len(maze[0])-entrances[0][0]-1, theta=entrances[0][2])
	print("Angle:", terminator._theta)
	
	count = 0
	commands = [0, 0]
	beep_boop = list()
	#beeper_booper = list()
	# A* Step Function Here
	while count < 5000:
		print("")
		beep_boop.append(Node(terminator.nrow - terminator._y - 1, terminator._x))
		#beeper_booper.append(terminator._theta)
		print(count)
		if terminator.destination:
			if abs((terminator.nrow - terminator.goal[0] - 1) - terminator._y) < .1 and abs(terminator.goal[1] - terminator._x) < .1:
				break  
			else:
				terminator.motion_primitive()
		elif terminator.goal:
			#terminator.maze = maze
			#terminator.big_maze = maze_generation.maze_expansion(terminator.maze, num_inside)
			terminator.big_maze = maze
			start = (len(terminator.big_maze[0])-terminator._y-1, terminator._x)
			path = terminator.a_star(start, terminator.goal, terminator.big_maze)	
			terminator.path = terminator.local_planner(path, terminator.big_maze)
			terminator.destination = terminator.path.pop()
		if count == 5:
			terminator.goal = (fires[0].row, fires[0].col)		
		count += 1
	# End step function
	
	maze_generation.plot(field=terminator.big_maze, path=beep_boop, bot=None, fires=fires)
	
if __name__ == "__main__":
	main()
