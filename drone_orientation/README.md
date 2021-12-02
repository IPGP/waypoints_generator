# Drone orientation along one path

Defines the successive positions (orientations) of the drone along one path, to acquire images with a defined overlap ratio, from one image to the next.
In addition, the gimbal pitch is adapted, so that photos are taken parallel to the ground.

![prof1](/drone_orientation/prof1_orientations.svg)

## TODO

* If necessary, adding B/H ratio check between pairs of orientations
* Making the algorithm faster (shifting the orientation to satisfy overlap is probably not the best idea)
