# Improved Kinematic Character Controller
Custom kinematic character controller with the ability to climb steps, for Godot.
This project is a demonstration of my solution to the stepping problem. (I also implement other common features.)

Does not work with Godot Physics!!! Use the Godot Jolt plugin!!!

## Motivaton
The built in character controller doesn't provide support for stair/step climbing. Flat bottomed collision shapes like cylinders and boxes get stuck on the foot of slopes, forcing the user to use a capsule shape (or other round shapes).

The stepping problem is not trivial, and the lack of resources and working implementations makes solving it a difficult task. 

Hiding the problem with invisible ramps creates a lot more work during map creation. (You have to make separate stair/step colliders, one for characters and one for non-character game objects). You only have to come up with a code-based solution once. 

Writing custom collision response is unavoidable, since the stepping needs to occour during the 'move/collide and slide' algorithm. Workarounds using the built in controller will always have unsolvable issues and compromises.

The goal of this project is to provide an efficent solution to the problem, relying on collision data and taking advantage of the 'Jolt 3D' physics engine, avoiding the need to write custom collision detection.

## Main idea
My implementation follows the snapping method. When colliding with a step during 'move and slide', we snap the player up to the height of the step surface, then continue on with the remaining motion. When moving down steps, we simply snap the player to the floor.
The resulting effect is similiar to games like Quake and Half-Life. The camera can be interpolated on the Y axis during stepping, for a more natural look in first person view.

## Features
- Moving and sliding on walls, slopes, and ceilings, with velocity clipping, as you'd expect.
- Flat bottomed collision shapes no longer get stuck on the foot of ramps.
- Snapping down on slopes and stairs.
- Climbing steps.

## Planned features
- Moving platform interactions.
- Rigid body interactions.

## Important notes
- Does not work with Godot Physics!!! Use the Godot Jolt plugin!!!
- This is not a drag-and-drop character controller.
- I've only tested it on low poly terrains.
- Kinematic character controllers should use animatable bodies (sync_to_physics must be set to false!).
- The camera's movement on the Y axis should be smoothly interpolated, to avoid the camera teleporting.

## About the scripts
- Motion testing and ray casting is simplified using objects.

## Addons I used
- Godot Jolt physics engine (You need this!) https://github.com/godot-jolt/godot-jolt
- Qodot for map making. https://github.com/QodotPlugin/Qodot
- Debug Draw 3D for debugging. https://github.com/DmitriySalnikov/godot_debug_draw_3d
