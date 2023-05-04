#!/bin/bash

# Test case 1: Small maze, normal corridors
echo "Test Case 1" >> test_data.csv
for i in 1 2 3 4 5
do
    python simulation.py 3 3 5 3 1 4 900
    python simulation.py 3 3 5 3 1 4 1200
    python simulation.py 3 3 5 3 1 4 2400
    python simulation.py 3 3 5 3 1 4 3600
done
# Test case 2: Large maze, Narrow corridors
echo "Test Case 2" >> test_data.csv
for i in 1 2 3 4 5
do
    python simulation.py 8 8 5 3 1 4 900
    python simulation.py 8 8 5 3 1 4 1200
    python simulation.py 8 8 5 3 1 4 2400
    python simulation.py 8 8 5 3 1 4 3600
done
# Test case 3: Medium maze, Wider corridors
echo "Test Case 3" >> test_data.csv
for i in 1 2 3 4 5
do
    python simulation.py 3 3 5 3 1 8 900
    python simulation.py 3 3 5 3 1 8 1200
    python simulation.py 3 3 5 3 1 8 2400
    python simulation.py 3 3 5 3 1 8 3600
done


