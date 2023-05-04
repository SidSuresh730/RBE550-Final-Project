# RBE550-Final-Project
## Final Project for Motion Planning RBE 550
### Setting Up the Python Virtual Environment
1. Download the zip file and extract it to the desired location or pull the repository
2. Run the following command from within the project folder: ```source environment/bin/activate```
3. In the terminal window, each line should now begin with the prefix ```(rbe-500)```
### Running the Code
1. From terminal, run the command ```python simulation.py``` to run the default simulation
2. To customize the simulation run the command ```python simulation.py <nrows> <ncols> <smol> <med> <lrg> <num_inside> <simtime>```
Where ```<nrows>``` is the number of rows in the maze, ```<ncols>``` is the number of columns in the maze ```<smol>``` is the number of small fires, ```<med>``` is the number of medium fires, ```<lrg>``` is the number of large fires, ```<num_inside>``` is the width of a cell of the maze, and ```<simtime>``` is the number of simulation seconds for which to run the simulation.
