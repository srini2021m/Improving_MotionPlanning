This project was created in collaboration with a teammate (Heather, pham28h@mtholyoke.edu). We evaluated the currently available Expansive Space Tree's algorithm for motion planning for potential weaknesses and proposed a modification that would result in better runtime and thereby, efficiency. 

<h1> Weakness & Proposed Enhancement </h1>
Problem: EST's use of random directions in a 3D environmentfor determining how the grow the tree's can cause the algorithm to explore unnecessary space and slow the progress of finding a path. This would result in increased opportunity for the start and goal node trees to explore the space locally around its own root. It also increases the possibility of the start and goal trees growing in opposite directions from each other rather than expand towards the counterpart configuration.  

Enhancement: Introducing a directional bias that would increase the likelihood of the random direction chosen for growth of the start tree to head towards the goal tree and vice versa.
Logic: Creating this bias would reduce the chance of unnecessary exploration in opposite directions. Thus, EST would have a better chance of growing the tree in the shortest path, resulting in faster connection and path finding.

<h2> Results and Further Analysis </h2>
Implementing the bias was detrimental to runtime statistics by 128%. In a simple configuration space like the 3D environment, the EST algorithm is trying to expand the tree so the more sparse and randomly it grows, the better. Thus, heading particularly towards the start/goal node wasn't the best solution.   

New proposal: A directional bias would be more benificial in a complex environment such as the Narrow Passage problem.  
Results: A 57% improvement in runtime was observed indicating that more directed growth is benificial to solve narrow passage motion planning problems.
   
