# Path Planning

In this C++ project the goal is to safely navigate around a virtual highway with
other traffic that is driving +/- 10 mph of the 50 mph speed limit. The simulator
that drives the car is providing the path planner the car's localization and
sensor fusion data; there is also a sparse map list of waypoints around the
highway.

### Model Documentation

The path generation model follows the [project walkthrough](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d)
by Udacity. The following steps are applied (all line references in relationship
to ``main.cpp``):

1. ll. 248-360: All cars from the sensor fusion data are examined and we are
performing a high-level decision whether changing lanes (``my_ref_lane``) or
changing speed (``my_ref_speed``). Influencing the cars decision are:
  * Is there any car in front of us so we have to change lanes or slow down.
  (``bool change_lanes_or_slow_down``)
  * Are there cars obstructing the right or left lanes so we cannot change
  lanes and *must* slow down? This also includes cars that are switching lanes
  and thusly have a horizontal velocity component in Frenet space.
  (``bool dont_go_left`` and ``bool dont_go_right``)
  * Also, we do not change lanes if we are already in a right-most or left-most
  lane; we also don't perform a double-lane-change (checking ``changing_lanes``)   or a lange change below 20mph.
  * Is the distance to the closest car in front of us in the target lane
  greater than the distance to the car in front of us in our lane (by
  examining ``double min_dist_left``, ``double min_dist_right`` and
  ``double min_dist_here``).

2. ll. 362-424: A spline is constructed to sample new trajectory points from
  * Its first control points are the last two points of the non-processed/remaining
  path that is returned by the simulator.
  * If currently no points were returned by the simulator, it starts with the
  current vehicle position.
  * To that, 3 more control points with distance 30, 60 and 90m ahead of the
  last point in Frenet space are added. (ll. 400-406)
  * The spline control points are transformed to vehicle coordinates with x
  pointing ahead, so that the spline is a well-defined function with unique
  x-to-y resolution for all x. (ll. 413-420)

3. ll. 426-463: The spline is sampled to return the trajectory points that the
path should follow.
  * The spline is first filled with the points that were returned from the
  simulator and were not processed yet (ll. 430-434)
  * The spline is always sampled up to 30m ahead (``target_x_car``, ll. 439).
  * The number of points that are sampled (and the distance between the sampled
  points) is determined by the reference velocity, so that the car covers the
  correct distance between each trajectory point of which one is passed every 0.2s.   

In the following subsections I go into detail about the nuances in the code
that make me pass each aspect of the rubric requirement of "no incident occuring".