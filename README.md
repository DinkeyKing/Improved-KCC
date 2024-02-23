# Improved Kinematic Character Controller
This project showcases how a custom kinematic character controller (with stair stepping, moving platforms and physics) can be made in Godot. \
(Because not using CharacterBody3D can provide various advantages.)

Does not work with Godot Physics!!! Use the Godot Jolt plugin!!! (This  will change most likely, see below.)

## Current status and issues (!!!)
### Controller status
Still a work in progress with many issues to adress. The current files are outdated, I will update the project once I resolve most issues, and achieve my desired character controller behaviour.

### Physics Engine Differences
I've decided to give Godot Physics a try, and I've found that it behaves a lot differently than Jolt in terms of collision reporting. A lot of issues and strange behaviours I've enccountered during the creation of my character controller only appear when using Godot Jolt, which resulted in a lot of workarounds in the script, making the code more complicated than needed. Godot Physics overall seems to work better for getting the required collision infos, so despite it's inaccuracies, performance issues and other shortcomings, I will be using that for now, without ensuring that the controller script will function properly when used with Jolt.

### Usable Shapes
I don't plan dealing with capsule shapes, so they may not work properly. This means the best shape to use when using Godot Physics is the box shape, since cylinder shapes are not well implemented there.


## Motivaton

The built in character controller is good for quick prototyping and for games that don't require more complex collision response for their characters. I needed stair stepping, non-capsule shapes, and more complex physics interactions, so making a custom character controller was unavoidable.

## Features
- Moving and sliding on walls, slopes, and ceilings, with velocity clipping, as you'd expect.
- Flat bottomed collision shapes no longer get stuck on the foot of ramps.
- Snapping down on slopes and stairs.
- Climbing steps.
- Moving platform interactions.
- Convincing rigid body interactions.

## Important notes
- Does not work with Godot Physics!!! Use the Godot Jolt plugin!!!
- This is not a drag-and-drop character controller.
- I've only tested it on low poly terrains.
- Kinematic character controllers should use animatable bodies.

## Climbing stairs

The stepping problem is not trivial, it can be solved in different ways. 

Hiding the problem with invisible ramps creates a lot more work during map creation. (You have to make separate stair/step colliders, one for characters and one for non-character game objects). You only have to come up with a code-based solution once. 

Writing custom collision response is unavoidable, since the stepping needs to occour during the 'move/collide and slide' algorithm. Workarounds using the built in controller will always have unsolvable issues and compromises.

My implementation follows the snapping method. When colliding with a step during 'move and slide', we snap the player up to the height of the step surface, then continue on with the remaining motion. When moving down steps, we simply snap the player to the floor.
The resulting effect is similiar to games like Quake and Half-Life. The camera can be interpolated on the Y axis during stepping, for a more natural look in first person view. (Same can be done with models.)

## About the scripts
- Motion testing and ray casting is simplified using objects.

## Addons I used
- Godot Jolt physics engine (You need this!) https://github.com/godot-jolt/godot-jolt
- Qodot for map making. https://github.com/QodotPlugin/Qodot
- Debug Draw 3D for debugging. https://github.com/DmitriySalnikov/godot_debug_draw_3d

## Assets
- Dev textures from : https://kenney.nl/assets/prototype-textures
