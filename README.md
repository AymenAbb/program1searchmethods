# program1searchmethods
Intro to AI- Program 1: Search Methods


Prompts used for ChatGPT o1:
------------------------------------------------------------------------------------------------------------------------------
Canvas Write this in Python, all in one file.

For this assignment, you’ll be doing a small example of a large problem—route-finding.
Given a list of cities and their adjacencies—from city A, what cities are next to it—can a route be found
from city A to city X?


I would like you to implement each of the following search methods.

This exercise will take the form of an experimental report detailing your findings of a comparison of each of the following methods:
1. undirected (blind) brute-force approaches like breadth-first search, depth-first search, and ID-DFS search
2. Heuristic Approaches like best-first search, and A* search
(5 Total Methods)

You'll want to think about the initial conditions and how to check for a "valid" sequence and GOAL check at each step of the search process.

Programming details:
• You’re given 2 data files:
◦ The first (adjacencies.txt) is a list of all the cities we know about in southern Kansas and the latitude and longitude of each.
# Names have been simplified to use underscores instead of a space
◦ A file (coordinates.csv) listing each town (pair) as a related adjacent node. 
# be aware adjacency is symmetric: If A is adjacent to B, then B is adjacent to A. This may not be listed comprehensively if your method requires that bidirectional connections be explicitly stated, you may need to generate additional pairs for the symmetrical connection to work.
--> Be sure to take this into account when setting up your program’s data structures. If adjacency is listed
in either direction, it should be considered present in both directions.
• Your program should:
Ask the user for their starting and ending towns, making sure they’re both towns in the database. If not, tell the user it's an incorrect town and prompt them to submit again.
Ask them then to select the search method they wish to use to find a route to the destination.
If that route exists, the program should then print the route the method found in order, from origin to destination.

Your program should also: 
1. Measure and print the total time needed to find the route (and include a time-out).
2. Calculate and display the total distance (node to node) for the cities visited on the route selected.
3. Return to the search method selection and allow a new method to be selected for comparison.
------------------------------------------------------------------------------------------------------------------------------

