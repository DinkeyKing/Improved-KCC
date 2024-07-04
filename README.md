# Improved Kinematic Character Controller

The IKCC class functions as a replacement for ```CharacterBody3D```'s ```move_and_slide``` collision response. It implements all of it's features with some changes and improvements, and provides new ones, while keeping the easy to use interface. New features include climbing steps and other low geometry, and special interactions with kinematic bodies (moving floors, -walls, and -ceilings) and dynamic rigid bodies.

I highly recommend using the Godot Jolt addon to make collision checks more accurate and bug-free, and to make cylinder shapes work! https://github.com/godot-jolt/godot-jolt

Demo project includes example controllers and a test environment.

## Current status
Not yet fully complete. I'm still working towards a release version.

## Motivation

- I wanted to have features for my characters' collision response that are not provided by the built in ```move_and_slide``` function (step climbing for example). There
are also a few of issues with it I wanted to fix.
- It's better to have collision response code for characters in scripts than in engine code, because character collision response is gameplay specific, and scripts are easier to write, test and modify.

## What's different?

### Fixes
- Fixed corner jittering.
- Removed logic that causes the character to halt when colliding with multiple walls with ```floor_block_on_wall``` turned on.
- Fixed sliding up "wall floor supports" due to recovery when ```floor_block_on_wall``` is on.

### Improvements
- Walls can now cancel out to ceiling, similar to how they can cancel out to floor.
- Floor snap safe margin can be set seperately from the safe margin used for slide collisions.

### New features
- Step/stair climbing with adjustable maximum step height. Surface normals can also be used to allow the character to climb over a range of low obstacles.
- The chatacter can now detach from a moving platform, if the platform suddenly changes speed in the opposite direction of it's movement vertical to it's surface. The speed change threshold can be set.
- Added the ability to slide the motion from platform velocity.
- Moving walls and ceilings are reworked. They now accelarate the character based on their velocities.
- Added optional rigid body interactions, which include contact impulses, weight force, and additional options for rigid body platforms.

### Other changes
- When ```floor_stop_on_slope``` is false, velocity is slid on the floor normal, instead of fully removing vertical velocity. This is useful for slippery slopes for example.
- When ```floor_block_on_wall``` is false, the ```wall_min_slide_angle``` only affects collisions with fully vertical walls, which I think is a better behaviour.
- Velocity will now be slid on walls
when ```motion_mode``` is set to ```MotionMode.FLOATING```, or when ```floor_block_on_wall``` is false. I can't think of a reason why I wouldn't modify the velocity on a wall.

## How to use
- Optional, but recommended: set 3D physics engine implementation to Godot Jolt. (https://github.com/godot-jolt/godot-jolt)
- Place 'ikcc.gd' somewhere in your project folder ('addons' folder for example).
- Your character controller scripts should be attached to ```CharacterBody3D``` nodes, like normal.
- In your script, instead of extending ```CharacterBody3D```, extend the ```IKCC``` class.
- Assign the character body's ```CollisionShape3D``` node to the ```collider``` property.
- As with the built in interface, set up collision response parameters, set the desired velocity in ```_physics_process```  and call ```move_and_slide``` to move the body.
### Getting collision info
You can reference the state variables (like ```is_on_floor```) and the collision result objects directly in the controller script, since in GDScript there are no access modifiers.
### Using rigid body interactions
- If you wish to use rigid body interactions, make sure the character body is on a collision layer that is excluded from the rigid bodies' collision mask! (The chatacter script detects the collision and applies the impulses to both bodies.) This is important, because otherwise the rigid body would be colliding with a body with infinite mass!
#### Limitations
- The contact impulses are estimations from the available collision data.
- When calculating the impulse between the character and a rigid body, only those two bodies are considered.
