# Rover
A rover simulation with python
This project focused on simulating an autonomous rover navigating different terrains using Python. A grid-based environment was created where terrain types were randomly generated with different movement costs representing conditions from smooth to rocky surfaces. The environment was defined using a class containing parameters such as width, height, and terrain cost matrices.

To ensure valid rover movement, boundary conditions were implemented so the rover remained within the grid limits and invalid coordinates were excluded. The rover’s surrounding nodes (neighbors) were then defined in four directions—up, down, left, and right—allowing the rover to perceive its environment and make navigation decisions.

Each grid coordinate was assigned a movement cost, taking into account Python’s row-column indexing system to correctly map terrain positions. For path planning, the Dijkstra algorithm was used together with a priority queue (heapq) to calculate the most efficient path between points based on terrain costs. The implementation was kept separate from the environment class to maintain flexibility and reusability in future projects.
[AUTONOMOUS ROVER REPORT.pdf](https://github.com/user-attachments/files/28198198/AUTONOMOUS.ROVER.REPORT.pdf)
