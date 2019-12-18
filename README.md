## RRT

A Rapidly Expanding Random Tree (RRT) algorithm for path planning can be summarised with the following pseudo-code:

![rrt](/media/rrt_code.png)

* RANDOM_CONFIGURATION selects a random coordinate in the domain.
* NEAREST_VERTEX finds the vertex in the tree closest to the random coordinate, according to some metric, in this case Euclidian.
* NEW_CONFIGURATION generates a new configuration in the tree by moving distance Δ from the nearest vertex to the randomly selected one.

Two versions were developed. First, a goal-oriented RRT was implemented with circular obstacles. Next, an image was used to represent the domain's obstacles, which were avoided by implementing Bresenham's Line Algorithm.

<div class="gallery" data-columns="2">
	<img src="/media/rrt2.gif" style="width: 100%">
	<img src="/media/rrt3.gif" style="width: 100%">
</div>