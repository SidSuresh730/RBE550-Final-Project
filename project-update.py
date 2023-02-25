import maze_generation
from mapper_robot import MapperBot
from rrt_robot import RRTBot
from a_star import ABot
from data_structure_library import Node

def main():
	print("Project Update\n")
	# ---- Run Maze Generation code
	num_rows = 10 # Number of rows in the maze
	num_cols = 10 # Number of columns in the maze
	num_fires_smol = 5 # Number of 1x1 in the maze
	num_fires_med = 3 # Number of 2x2 in the maze
	num_fires_lrg = 1 # Number of 3x3 in the maze
	num_inside = 5 # Number of padding inside each cell
	num_ent = 1 # Number of entrances to the maze
	plot_maze = True
	
	print("Generating Maze")
	[maze, fires, entrances] = maze_generation.generate_maze(num_rows, num_cols, num_fires_smol, num_fires_med, num_fires_lrg, num_inside, num_ent, plot_maze)
	
	start = Node(entrances[0][0], entrances[0][1])
	
	print("Running Dijkstra's")
	dora = MapperBot(len(maze)-1, len(maze[0]), 'red')
	dora.djikstra_map(s=start, maze=maze)
	maze_generation.plot(field=maze, path=None, bot=dora)
	
	print("Running RRT")
	(hwalls, vwalls) = maze_generation.get_list_walls(maze)
	ert = RRTBot(epsilon= 1, start=start, nrow=len(maze)-1, ncol=len(maze[0]), color='red')
	rrt_limit = 500
	buffer = .5
	while ert.success<rrt_limit:
		ert.rrt_move(hwalls=hwalls,vwalls=vwalls, buffer=buffer)
	maze_generation.plot(field=maze,path=None, bot=ert)
	
	print("Running A*")
	terminator = ABot(len(maze)-1, len(maze[0]), color='cyan')
	start = [start.row, start.col]
	end = fires[0]
	path = terminator.a_star(start, end, maze)
	maze_generation.plot(field=maze, path=path, bot=terminator)
	
if __name__ == "__main__":
	main()
