# Improved Kinematic Character Controller

The `IKCC` class is a general purpose 3D kinematic/virtual body implemented in GDScript, which acts as an alternative to the built-in `CharacterBody3D` class. It implements all of it's features, provides new ones, and is just as easy to use.

Some logic in character controllers written for `CharacterBody3D` might not translate well to `IKCC`, see the [`guide`][gde] for details.

I highly recommend using Godot Jolt (Jolt Physics) for the physics server implementation!
https://github.com/godot-jolt/godot-jolt

The demo project includes an [`example character controller`][xpl] implemented with `IKCC`, as well as some simple platforming levels, and a sandbox environment.

I took a lot of inspiration from Jolt Physics' CharacterVirtual:
https://github.com/jrouwe/JoltPhysics/blob/d6e015372b08f28c3c62e9aaab9a5cb2af667e54/Jolt/Physics/Character/CharacterVirtual.cpp

## List of new features and improvements

- Physically accurate algorithm for velocity and position solving:
	- Evades the bugs and weird behaviours of `CharacterBody3D`'s velocity and position calculations.
	- The velocity used for the displacement is always the actual velocity of the body.
	- Ensures that no corner jittering occours, and that acute corner scenarios are solved in a single iteration.
	- Constraint based velocity solving, which takes into account:
		- Constraint velocities (this means accurate velocity resolutions when colliding with moving kinematic bodies)
		- Constant speed modifier of constraints
		- Constant horizontal direction modifier of constraints
		- Minimum slide angle property of constraints
- Stair stepping with adjustable maximum step height (works with all collider shapes)
- The character body detects and resolves incoming collision's from other bodies.
- Status variables like `is_on_floor` and `current_floor_normal` is set based on what the character is touching at it's final calculated position, as opposed to getting set by any collision check during the movement simulation. (You can still check wether the character collided with a floor or not using `collided with floor`.)
- Floor snapping is reworked:
	- When floor snap is applied, the velocity will be made parallel to the floor.
	- You can set the minimum velocity length needed in the direction of the floor normal in order for the body to detach from the floor.
- Walls can cancel out to ceiling, similiar to how they can cancel out to floor.
- The applied motion from the platform velocity can follow the configured sliding behaviour.
- If vertical accelerations/decelerations of moving platforms are larger than a given threshold, than the character body can detach from it.
- Interactions with dynamic rigid bodies:
	- Contact impulses
	-  Weight force

\* As far as I know, accurate collisions with dynamic bodies is not possible without manually managing the physics simulation, which you can't do inside of Godot (as of now). Aside from that, it's not a priority of this project.

## How to get started

- Optional, but recommended: set 3D physics server implementation to Godot Jolt. (https://github.com/godot-jolt/godot-jolt)
- Place [`ikcc.gd`][spt] somewhere in your project folder
- Your character controller scripts should be attached to `CharacterBody3D` nodes, like normal.
- In your script, instead of extending `CharacterBody3D`, extend the `IKCC` class.
- Assign the character body's `CollisionShape3D` node to the `collider` property.
- As with `CharacterBody3D`, set the desired movement parameters before calling `move_and_slide` in `_physics_process` to move the body.

## Demo project assets and addons

https://www.kenney.nl/assets/prototype-textures

https://distantlantern.itch.io/allsky-free

https://github.com/func-godot/func_godot_plugin

https://godotengine.org/asset-library/asset/1766

https://github.com/godot-jolt/godot-jolt

[spt]: IKCC/ikcc.gd
[gde]: IKCC/guide/guide.md
[xpl]: entities/actor/player/player_body.gd
