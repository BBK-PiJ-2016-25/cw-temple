# Submission for PiJ Coursework 4: Philip Hammond and the Temple of Gloom (ahawes 02)

My submission for the Philip Hammond and the Temple of Gloom assignment.

# Exploration Phase

In order to find the orb, Philip selected his next tile to visit by looking at his neighbours and prioritising them based first on the number of times they had been visited, followed by their distance from the orb.

# Escape Phase

Philip determined the shortest path to the exit by using an algorithm based on Breadth First Search.

First, he examined each of the tiles in the map in order of the amount of gold on them, and if possible to reach them and get back to the exit in time, he visited them.

Then, he returned to the exit by the shortest path.

In all cases, he picked up the gold on any tile he encountered.


For further implementation details, please see the javadoc provided for all methods.

