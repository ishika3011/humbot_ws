<----------------markov localization------------>
tracks 3 cooordinates (x,y, theta) 

random sampling done, it extracts a subset of all possible robot positions, 
applying Markov localization to track position of this subset of poses
removes random number of samples,so the robot can get lost


<------------------------------------------------------monte carlo localization --------------------------------------------------------------------->

initial guess-->> measurement update -->> state prediction--
                        ^                                  |
                        |---------------ressampling -------

measurement update then prediction step
considers only a sample subset of poses
cons: require initial guess, approx initial pose where robot is located, and assoc. with covariance


there are various particles, and we try various hypotheses and various poses in world to answer the ques will it make sense if robot is at this point?
for this they gave WEIGHTS to particle(how well sensor reading match if robot was located in the position of particle)
(measurement update step) -->>    matches laser scans of map with the actual map and assigns probability (weight)
(state update) -->> updates the position of each particle based on odom data
resampling -->removing particles having lighter weight, so they are not considered for next itieration



<------------------------------------------------------------amcl use when/not----------------------------------------------------------------------->

If you are building the map live (SLAM or mapping with odom + laser like your node), then:
You still need something like the static_map_odom_broadcaster,
Because AMCL won’t run until a finished map is available.

If you are localizing on an already built map, then:
You don’t need the static broadcaster,
Because AMCL takes over publishing map → odom.



<------------------------------------------------------------Particle filter SLAM-------------------------------------------------------------------->
intial guess --> map update --> measurement update step --> state prediction--
                     ^                                                       |
                     |                                                       |
                  resampling -------------------------------------------------


<-------------------------------------------------------EKFslam-------------------------------------------------------------------------->

obj* - environmental feature

INITIAL GUESS-----> STATE PREDICTION ---------->MEASUREMENT PREDICTION --------> OBTAINED MEASUREMENT -----> DATA ASSOCIATION
(some covariance  (covariance increases)      moving in path                   the prediction is then     due to odometry and localisation errors we need it
assigned to it,     if detects any obj*,       calculation via                  compared with actual       this compares boththe features detected and if it 
in ellipse shape)   assigns covariance        motion model seeing if           measurement                  is same/not, if same then CORRECT ODOM ERRORS
                    to it                     the same obj* is there                                         improving robot'slocalization

<------------------------------------------------------------------Graph Slam----------------------------------------------------------------------->

Vertex (cities) and edge (cost to reach that city)
graph which represents both ROBOT POSITION and Envi Landmarks

ROBOT POSE ---- NEXT ROBOT POSE --------
    Odometry Constraint     \   Measurement constraint
    (representing via        \   (use sensor model)
    odometry motion model)    \  
                            LANDMARK
 


<-----------------------------------------------------------------GRAPH---------------------------------------------------------->
 
 VISIBILITY GRAPH-(minimum distance from obstacle)
 2 nodes as start and end of the graph,
 assigns nodes to each vertex of obstacle - by connecting all pairs of vertices visible to each other

 Cons:
    since using this algo, robot will pass as close as possible to obstacle
    have to add inflated obstacle around the obstacle

    size increases, as assigning, nodes to each vertices

 VORONOI GRAPH- (max distance from obstacle)

 every mid point between obstacles is assigned a node.
 Use case: USED WHEN HUMANS and ROBOTS are in same space

CELL DECOMPOSITION-
free cells in graph are assigned nodes, 
fastest route

cons:
    better graph needs finer sampling but it is computationlly expensive



dijkstra:

works on priority queue (node name, cost) its like BFS if cost is equal


<-----------------------------------------Costmaps------------------------------------------------------>
layered costmap
1.static layer: this gets input from /map 

2. obstacle layer: 
using /scan topic to see if any new obstacle is detected, if yes using "combination_method: 1" which indicates that using the maximum cost among the layers
so that if static layer has said that map is free (cost is 0) and obstacle layer is saying cost is 100 then it takes cost as 100

3. inflation layer:
inflating the periphery of obstacle to create not recomended areas or
in simpleterms " not passing too close to obstacle "
radius = includes both footprint and area around obstacles, not recommended for traversing

<-----------------------------------------------------------------Nav2 Planner--------------------------------------------------------------------->
    INPUT                                                                         OUTPUT

/goal                                                                       /dijkstra/visited_map
/map                               DIJKSTRA PLANNER NODE
/tf                                                                         /dijkstra/path
(for robot position)


<-------------------------------converting path planning algos to Plugins for Nav2 Planner--------------------------------------------------------------->
we use plugins in Nav2 for easy switcing b/w planning algo.
these planner run inside lifecycle nodes, allowing controlled startup and reconfig
to avoid manual changing of each node's state, -->lifecycle manager is used. which automates this process and ensure proper sequencing



<-------------------------------------------------------------Behavior Trees-------------------------------------------------------------------------->
for using black board : tyoe --->  {}


<-------------------------------------------------------------Nav2 BT Navigators-------------------------------------------------------------------------->

implements complex logics/behaviors us BT. 
they are implemented asking 1 or more behaviors provided by behavior server -- like recovery behavior


<-------------------------------------------------------------Nav2 Behaviors-------------------------------------------------------------------------->

implements atomic, independent behaviors.
sedning goal to action servers obtain simple behvior --- rotate, wait, move back some specific distance




<-------------------------------------------------------------Important lectures-------------------------------------------------------------------------->

Module 3 plan and navigation --- lec 145