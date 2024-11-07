
## Implements a complex 'move and slide' type collision response for character bodies.
class_name IKCC
extends PhysicsBody3D



#==================================================================================================
# -----------------------
# | CONSTANTS AND ENUMS |
# -----------------------


const MAX_SLIDES                : int   = 6        # Default: 6
const MAX_SLIDE_COLLISIONS      : int   = 6        # Default: 6
const MAX_SNAP_COLLISIONS       : int   = 6        # Default: 6
const CMP_EPSILON               : float = 0.00001  # Default: 0.0001
const ANGLE_CMP_EPSILON         : float = 0.01     # Default: 0.01
const RAYCAST_OFFSET_LENGTH     : float = 0.001    # Default: 0.001
const RECOVERY_FACTOR           : float = 2.0      # Default: 2.0
const WALL_VERTICAL_APPROX_CMP  : float = 0.01     # Default: 0.01

# INFO RECOVERY_FACTOR : The amount of recovery can be more than the safe margin,
# so that has to be accounted for.

## Different modes for sliding collision response.
enum MotionMode {
	## Suitable for characters that move on the ground.
	GROUNDED,
	## Suitable for flying or swimming movement.
	FLOATING
}

## Defines how to modify the velocity when leaving a moving platform.
enum PlatformLeaveAction {
	## Does nothing when leaving a platform.
	DO_NOTHING,
	## Adds the last platform velocity to [member velocity] when leaving a platform.
	ADD_VELOCITY,
	## Adds the last platform velocity without the downward component to [member velocity].
	ADD_VELOCITY_NO_DOWNWARDS
}

enum FloorType {
	NONE,
	COLLISION,
	SURFACE
}




#==================================================================================================
# --------------
# | PROPERTIES |
# --------------


## Sets the motion mode which defines the behavior of [method move_and_slide]. 
## See [enum MotionMode] constants for available modes.
@export var motion_mode := MotionMode.GROUNDED
## Vector pointing upwards, used to determine what is a wall and what is a floor (or a ceiling)
## when calling [method move_and_slide].
@export var up_direction : Vector3 = Vector3.UP :
	set(value) : up_direction = value.normalized()

@export_group("Assign me a reference!")
## Reference to the collider belonging to the character body.
@export var collider : CollisionShape3D

@export_group("Floor")
## If [code]true[/code], the body will be able to move on the floor only.
## This option avoids to be able to walk on walls, it will however allow to slide down along them.
@export var floor_block_on_wall : bool = true
## If [code]false[/code], the [member velocity] will slide on the floor, instead of fully removing vertical velocity.
## This is useful for slippery slopes.
@export var floor_stop_on_slope : bool = true
## If [code]true[/code], the body will move at a constant speed on floors, regardless of the slope. [br]
## If [code]false[/code], the motion will slide on the floor, resulting in decreased speed,
## and when going down slopes, snapping will cause an increased speed.
@export var floor_constant_speed : bool = false
## Sets a maximum snapping distance. When set to a value different from [code]0.0[/code], the body is kept attached to
## slopes when calling [method move_and_slide]. The snapping vector is determined by the given distance
## along the opposite direction of the [member up_direction].
@export_range(0.0, 1.0, 0.01, "suffix:m") var max_snap_length : float = 0.26 :
	set(value) : max_snap_length = absf(value)
## Maximum angle where a slope is still considered a floor (or a ceiling), rather than a wall.
@export_range(0.0, 180.0, 0.1, "degrees")
var _floor_max_angle : float = 45.0 :
	set (value) : 
		floor_max_angle = value * (PI/180.0)  # Set radian variable
		_floor_max_angle = value  # Set degree variable

@export_group("Wall")
## If [code]true[/code], the [member velocity] will slide on walls, if the horizontal component is
## looking away from the wall. This only matters when [member floor_block_on_wall] is [code]true[/code].
@export var wall_slide_vertical_only_collision : bool = false
## If the angle of a wall collision is smaller than this angle, the body will stop
## instead of sliding. When [enum MotionMode] is set to [code]MotionMode.GROUNDED[/code], only the horizontal movement is affected.
## If [member floor_block_on_wall] is [code]false[/code], this only affects collisions with fully vertical walls.
@export_range(0.0, 180.0, 0.1, "degrees") 
var _wall_min_slide_angle : float = 15.0 :
	set (value) : 
		wall_min_slide_angle = value * (PI/180.0)  # Set radian variable
		_wall_min_slide_angle = value  # Set degree variable

@export_group("Ceiling")
## If [code]false[/code], upwards [member velocity] and motion will be removed on ceiling collisions,
## which prevents sliding up ceilings.
@export var slide_on_ceiling : bool = true

@export_group("Moving Obstacles")
## If [code]true[/code], the applied motion from the platform velocity will also follow the sliding behaviour,
## instead of stopping on collisions.
@export var slide_platform_motion : bool = true
## Sets the behavior to apply when you leave a moving platform. By default, to be physically accurate,
## when you leave the last platform velocity is applied. See [enum PlatformLeaveAction] constants for available behavior.
@export var platform_leave_action := PlatformLeaveAction.ADD_VELOCITY
## Collision layers that will be included for detecting floor bodies that will act as moving platforms
## to be followed by the body.
@export_flags_3d_physics var moving_platform_layers : int = 1
## Collision layers that will be included for detecting wall and ceiling bodies that will 
## accelarate the body.
@export_flags_3d_physics var moving_wall_and_ceiling_layers : int = 1
## The character detaches from a moving platform, if the platform suddenly changes speed in the
## opposite direction of it's movement vertical to it's surface, and the change in speed is larger
## than this threshold.
@export_range(0.01, 100, 0.01, "suffix:m/s") var platform_vertical_detach_threshold : float = 1.0 :
	set(value) : platform_vertical_detach_threshold = maxf(0.0, value)

@export_group("Stepping")
## If [code]true[/code], the character can climb all kinds low geometry with floor surfaces, not just steps. [br]
## [b]IMPORTANT NOTICE:[/b] To make stepping work for capsule and sphere colliders, this must be set to [code]true[/code]!
@export var use_surface_normals : bool = true
## Sets the maximum height of the steps (and other low obstacles) the character can climb.
@export_range(0.0, 1.0, 0.01, "suffix:m") var max_step_height : float = 0.26 :
	set(value) : max_step_height = absf(value)

@export_group("Rigid Body Interactions")
## If [code]false[/code], rigid bodies collisions will be treated as collisions with static bodies. [br]
## If [code]true[/code], contact impulses and weight force will be simulated when colliding with
## rigid bodies. In this case, it's [b]important[/b] that the collision mask of the rigid bodies that
## will be interacted with has the character's collision layer excluded!
@export var interact_with_rigid_bodies : bool = true
## The gravity acceleration used when applying a weight force.
## The default value is read from the project settings.
@export_range(0.0, 100, 0.01, "suffix:m/s^2") var gravity_acceleration : float = ProjectSettings.get_setting("physics/3d/default_gravity") :
	set(value) : gravity_acceleration = absf(value)
## The mass of the character body.
@export_range(0.001, 1000.0, 0.001, "suffix:kg") var mass : float = 1.0 :
	set(value) : 
		mass = maxf(CMP_EPSILON, value)
		inverse_mass = 1.0 / mass
## The body's bounciness (only applies to rigid body collisions). Values range from 0 (no bounce) to 1 (full bounciness).
@export_range(0.0, 1.0, 0.01) var physics_material_bounce: float = 0.0 :
	set(value) : physics_material_bounce = clampf(value, 0.0, 1.0)
## If the character's horizontal distance from the rigid body platform's centre of mass is less than
## this threshold, the applied impulse will always be central, meaning no tourqe is applied.
@export_range(0.0, 64, 0.01, "suffix:m") var rigid_body_platform_central_impulse_threshold : float = 0.2 :
	set(value) : rigid_body_platform_central_impulse_threshold = absf(value)
## The fraction of the tourqe to be eliminated from the impulses applied to the rigid body platform,
## when the resulting velocity from the collision impulse points upwards. [br]
## This is used to get rid of the excessive rotations caused by the estimated impulses.
@export_range(0.0, 1.0, 0.01) var rigid_body_platform_counter_torque_factor : float = 0.75 :
	set(value) : rigid_body_platform_counter_torque_factor = clampf(value, 0.0, 1.0)
## If the length difference between the previously picked up horizontal (friction) platform velocity
## and the current platform's horizontal velocity exceeds this threshold, the horizontzal platform
## velocity will be interpolated instead of instantly picked up. [br]
## This useful for stopping excessive amounts of accelarations on rigid bodies. [br]
## (Horizontal means horizontal relative to the platform surface.)
@export_range(0.0, 64, 0.01, "suffix:m/s") var rigid_body_platform_horizontal_stick_threshold : float = 4.0 :
	set(value) : rigid_body_platform_horizontal_stick_threshold = absf(value)
## How fast to interpolate the friction component of the platform velocity to the actual velocity of the platform,
## when the [member rigid_body_platform_horizontal_stick_threshold] is exceeded. Note that the physics
## material of the rigid body also affects the interpolation speed.
@export var rigid_body_platform_friction_strength : float = 1.0 :
	set(value) : rigid_body_platform_friction_strength = absf(value)
## If the length difference between the previously picked up vertical platform velocity
## and the current platform's vertical velocity exceeds this threshold, the previous vertical
## component is kept, meaning the character no longer sticks to the rigid body. [br]
## (Vertical means vertical relative to the platform surface.)
@export_range(0.01, 100, 0.01, "suffix:m/s") var rigid_body_platform_vertical_stick_threshold : float = 1.0 :
	set(value) : rigid_body_platform_vertical_stick_threshold = absf(value)

@export_group("Collision")
## Extra margin used for recovery and during slide collisons. [br] [br]
## If the body is at least this close to another body, it will consider them to be colliding 
## and will be pushed away before performing the actual motion. [br]
## A higher value means it's more flexible for detecting collision, which helps with consistently
## detecting walls and floors. [br]
## A lower value forces the collision algorithm to use more exact detection, so it can be used in
## cases that specifically require precision, e.g at very low scale to avoid visible jittering,
## or for stability with a stack of character bodies
@export_range(0.001, 256.0, 0.001, "suffix:m") var safe_margin : float = 0.001 :
	set(value) : safe_margin = absf(value)
## Extra margin used for recovery when snapping to the floor.
@export_range(0.001, 256.0, 0.001, "suffix:m") var snap_safe_margin : float = 0.001 :
	set(value) : snap_safe_margin = absf(value)




#==================================================================================================
# --------------
# | VARIABLES  |
# --------------


var is_on_floor                     : bool
var is_on_wall                      : bool
var is_on_ceiling                   : bool
var is_on_wall_floor                : bool
var has_stepped                     : bool
var prev_on_floor                   : bool
var prev_on_floor_surface           : bool
var prev_on_wall_floor              : bool
var prev_had_floor_collider_saved   : bool
var can_apply_constant_speed        : bool
var sliding_enabled                 : bool
var slide_count                     : int
var velocity                        : Vector3
var real_velocity                   : Vector3
var prev_floor_normal               : Vector3
var current_floor_normal            : Vector3
var current_wall_normal             : Vector3
var current_ceiling_normal          : Vector3
var platform_velocity               : Vector3
var floor_impact_velocity           : Vector3
var remaining_floor_impact_velocity : Vector3
var total_travel                    : Vector3
var initial_motion_slide_up         : Vector3
var previous_position               : Vector3
var current_floor_collider_encoded  : EncodedObjectAsID
var current_floor_collider_data     : ColliderData
var collider_datas                  : Array[ColliderData]
var collision_results               : Array[CollisionState]

# Translate angle properties to radian
@onready var wall_min_slide_angle : float = _wall_min_slide_angle * (PI/180.0)
@onready var floor_max_angle      : float = _floor_max_angle * (PI/180.0)

# Precalculate inverse mass
@onready var inverse_mass : float = 1.0 / mass




#==================================================================================================
# ------------------
# | PUBLIC METHODS |
# ------------------



## Moves the body with the current velocity and the requested motion mode.
## Returns true if collision happened.
func move_and_slide() -> bool :
	# "Hack in order to work with calling from _process as well as from _physics_process; calling from thread is risky"
	var delta_t : float = get_physics_process_delta_time() if Engine.is_in_physics_frame() else get_process_delta_time()
	previous_position = global_position
	
	# Take axis lock into account
	if axis_lock_linear_x :
		velocity.x = 0.0
	if axis_lock_linear_y :
		velocity.y = 0.0
	if axis_lock_linear_z :
		velocity.z = 0.0
	
	# Save previous states
	prev_on_floor = is_on_floor
	prev_on_wall_floor = is_on_wall_floor
	prev_floor_normal = current_floor_normal
	
	# Reset state variables
	is_on_floor = false
	is_on_wall = false
	is_on_ceiling = false
	is_on_wall_floor = false
	has_stepped = false
	current_floor_normal = Vector3.ZERO
	current_wall_normal = Vector3.ZERO
	current_ceiling_normal = Vector3.ZERO
	collision_results.clear()
	
	# Move with requested motion mode
	if motion_mode == MotionMode.GROUNDED :
		grounded_move(delta_t)
	else :
		floating_move(delta_t)
	
	# Add platform velocity if left floor
	if platform_leave_action != PlatformLeaveAction.DO_NOTHING and not platform_velocity.is_zero_approx() and not current_floor_collider_encoded :
		if platform_leave_action == PlatformLeaveAction.ADD_VELOCITY_NO_DOWNWARDS and platform_velocity.dot(up_direction) < 0.0 :
			platform_velocity = platform_velocity.slide(up_direction)
		
		velocity += platform_velocity
		platform_velocity = Vector3.ZERO
	
	# Compute de facto velocity from new and old position
	real_velocity = (global_position - previous_position) / delta_t
	
	if collision_results.size() > 0 :
		return true
	
	return false




## Snaps the character to the floor, if there is floor below within 'p_snap_length' meters away
func snap_to_floor(p_snap_length : float) -> void :
	
	prev_on_floor_surface = false  # Set flag false by default
	
	# Already on floor, no need to snap
	if is_on_floor :
		return
	
	# Snap by at least safe margin to keep floor state consistent
	var snap_length : float = maxf(snap_safe_margin, p_snap_length)
	
	var collided    : bool
	var snap_motion := -up_direction * snap_length
	var m_result    := PhysicsTestMotionResult3D.new()
	
	collided = _move_and_collide(m_result, snap_motion, true, true, MAX_SNAP_COLLISIONS, false, snap_safe_margin)
	
	if not collided :
		return
	
	var result_state := CollisionState.new()
	
	set_collision_state(m_result, result_state)
	
	if not (result_state.s_floor) :
		# Check surface normal, if it's floor, allow another snap attempt on next frame
		# Fixes case when walking down step and reported collision normal is wall
		# NOTE : This triggers the snap method more often, but makes snapping more reliable.
		var feet_pos : Vector3 = get_feet_position()
		var max_surface_height : float = feet_pos.dot(up_direction)
		var floor_surface_found : bool = search_for_floor_surface_normal(m_result, max_surface_height)
		if floor_surface_found :
			prev_on_floor_surface = true
		return
	
	# Only set floor collision flag(s)
	update_overall_state(result_state, true, false, false)
	collision_results.append(result_state)
	
	var travel : Vector3 = m_result.get_travel()
	
	# 'move_and_collide' may stray the body a bit because of pre un-stucking
	# so only ensure that motion happens on floor direction in this case.
	if travel.length() > snap_safe_margin :
		travel = up_direction * up_direction.dot(travel)
	else :
		travel = Vector3.ZERO
	
	# Don't slide down floor walls due to recovery when standing on them
	# NOTE : Use the modified travel's length. Only apply, when wall floor normal is not 
	# vertical up, to keep floor state consistent.
	if result_state.s_wall_floor_support and up_direction.dot(result_state.floor_normal) < 1.0 - CMP_EPSILON and travel.length() < snap_safe_margin * RECOVERY_FACTOR + CMP_EPSILON :
		travel = Vector3.ZERO
	
	global_position += travel




## Applies an impulse to the character body
func apply_impulse(p_impulse : Vector3) -> void :
	velocity += p_impulse * inverse_mass




#==================================================================================================
# -------------------
# | PRIVATE METHODS |
# -------------------


## Performs the core logic of a grounded movement
func grounded_move(p_delta_t : float) -> void :
	
	# Get platform velocity
	var prev_platform_velocity : Vector3 = platform_velocity
	var current_floor_is_rigidbody : bool = false
	var detach_from_platform : bool = false
	platform_velocity = Vector3.ZERO
	if current_floor_collider_encoded :
		var floor_collider_object : Object = instance_from_id(current_floor_collider_encoded.object_id)
		
		if floor_collider_object is PhysicsBody3D :
			var floor_collider := floor_collider_object as PhysicsBody3D
			var platform_rid : RID = floor_collider.get_rid()
			var excluded : bool = (moving_platform_layers & PhysicsServer3D.body_get_collision_layer(platform_rid)) == 0
			
			if platform_rid.is_valid() and not excluded :
				var platform_body_state : PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(platform_rid)
				
				if platform_body_state :
					var local_position : Vector3 = global_position - platform_body_state.transform.origin
					platform_velocity = platform_body_state.get_velocity_at_local_position(local_position)
					# NOTE : Static body constant velocity is also taken into account.
					
					if interact_with_rigid_bodies and floor_collider_object is RigidBody3D :
						current_floor_is_rigidbody = true
						
						var rigidbody : RigidBody3D = floor_collider_object as RigidBody3D
						var collision_normal : Vector3 = current_floor_collider_data.collision_normal
						var horizontal_platform_velocity : Vector3 = platform_velocity.slide(collision_normal)
						
						# Handle push component
						var platform_push_velocity : Vector3
						var platform_impulse_data : CollisionImpulseData = calculate_impulse_from_collision(rigidbody, current_floor_collider_data, prev_platform_velocity + remaining_floor_impact_velocity + velocity, true)
						if not platform_impulse_data.bodies_separating :
							var impulse : Vector3 = platform_impulse_data.impulse
							
							platform_push_velocity = (prev_platform_velocity + remaining_floor_impact_velocity).dot(collision_normal) * collision_normal + (-impulse * inverse_mass)
							apply_impulse_to_rb_platform(platform_impulse_data, rigidbody, platform_body_state)
							
						elif platform_impulse_data.relative_speed_towards_normal > rigid_body_platform_vertical_stick_threshold :
							platform_push_velocity = (prev_platform_velocity + remaining_floor_impact_velocity).dot(collision_normal) * collision_normal # Keep previous vertical component
							detach_from_platform = true  # Don't apply snap to floor later
						else :
							platform_push_velocity = platform_velocity - horizontal_platform_velocity  # Keep vertical stick component of platform velocity
						
						# Handle friction component
						var platform_friction_velocity : Vector3 = horizontal_platform_velocity
						var prev_platform_friction_velocity : Vector3 = prev_platform_velocity.slide(collision_normal)
						# If friction velocity has turned around and is not too large, instantly pick up the new velocity
						if not (platform_friction_velocity.dot(prev_platform_friction_velocity) < 0.0 and platform_friction_velocity.length_squared() < rigid_body_platform_horizontal_stick_threshold ** 2.0) :
							var length_diff : float = absf((platform_friction_velocity - prev_platform_friction_velocity).length())
							if length_diff > rigid_body_platform_horizontal_stick_threshold :
								var lerp_weight : float = (rigidbody.physics_material_override.friction if rigidbody.physics_material_override else 1.0) * rigid_body_platform_friction_strength * p_delta_t
								
								platform_friction_velocity = prev_platform_friction_velocity.lerp(platform_friction_velocity, lerp_weight)
						
						# Combine push and friction components
						platform_velocity = platform_push_velocity + platform_friction_velocity
						
						# Gravity is taken into account when moving upwards
						var platform_velocity_facing_up : bool = platform_velocity.dot(up_direction) > 0.0
						if platform_velocity_facing_up :
							platform_velocity += gravity_acceleration * p_delta_t * -up_direction
							
							var platform_vel_dot_up : float = platform_velocity.dot(up_direction)
							if platform_vel_dot_up < 0.0 :
								platform_velocity -= platform_vel_dot_up * up_direction
		
		if not interact_with_rigid_bodies or not current_floor_is_rigidbody :
			detach_from_platform = prev_had_floor_collider_saved and (prev_platform_velocity - platform_velocity).dot(prev_floor_normal) > platform_vertical_detach_threshold
			
			if detach_from_platform :
				platform_velocity = platform_velocity.slide(prev_floor_normal) + prev_platform_velocity.dot(prev_floor_normal) * prev_floor_normal
		
		# FIXME : Because only one floor collider is taken into account,
		# the transition between two floors with velocity can be jittery.
		# To solve this, one needs to be definetively selected over the other, or their
		# velocity need to be summed some way.
		# Alternatively, one of the floors can be raised
		# (or lowered) a little to get around this issue.
	
	# Take axis lock into account for platform velocity
	if axis_lock_linear_x :
		platform_velocity.x = 0.0
	if axis_lock_linear_y :
		platform_velocity.y = 0.0
	if axis_lock_linear_z :
		platform_velocity.z = 0.0
	
	
	# Clear saved collision entity datas
	prev_had_floor_collider_saved = current_floor_collider_encoded != null
	current_floor_collider_encoded = null
	
	current_floor_collider_data = null 
	collider_datas.clear()
	
	
	# Get data of current collisions before moving
	# HACK (EXTERNAL) : The method 'body_test_motion' (regardless of the physics server) does not work as advertised
	# since it does not actually report the collisions from recoveries, so an extra call is needed
	# unfortunately to get the current collisions before moving with move_and_slide.
	var res := PhysicsTestMotionResult3D.new()
	var rest_result_state := CollisionState.new()
	_move_and_collide(res, Vector3.ZERO, true, true, MAX_SNAP_COLLISIONS, false, safe_margin, false)
	set_collision_state(res, rest_result_state, true)
	collision_results.append(rest_result_state)
	update_overall_state(rest_result_state, true, true, true)
	
	var snap_on_platform : bool = is_on_floor
	# NOTE: Slide velocity on floor later, when a moving platform collides with the character
	# from below that is faster towards the collision normal than the character.
	
	
	# Move with platform motion first
	var platform_velocity_zero : bool = platform_velocity.is_zero_approx()
	if not platform_velocity_zero :
		
		var platform_motion : Vector3 = platform_velocity * p_delta_t
		
		if slide_platform_motion :
			sliding_enabled = not floor_stop_on_slope
			can_apply_constant_speed = sliding_enabled
			slide_count = 0
			total_travel = Vector3.ZERO
			initial_motion_slide_up = platform_motion.slide(up_direction)
			
			move_and_slide_grounded(platform_motion, true)
		else :
			var platform_res := PhysicsTestMotionResult3D.new()
			var platform_result_state := CollisionState.new()
			
			_move_and_collide(platform_res, platform_motion, false, false, MAX_SLIDE_COLLISIONS, true, safe_margin, true)
			set_collision_state(platform_res, platform_result_state)
			collision_results.append(platform_result_state)
			update_overall_state(platform_result_state, true, true, true)
	
	
	# Move with motion
	var motion : Vector3 = velocity * p_delta_t
	sliding_enabled = not floor_stop_on_slope
	can_apply_constant_speed = sliding_enabled
	slide_count = 0
	total_travel = Vector3.ZERO
	initial_motion_slide_up = motion.slide(up_direction)
	
	move_and_slide_grounded(motion)
	
	
	# Add velocity from moving walls and ceilings
	add_wall_and_ceiling_push_velocity()
	
	
	# Save floor impact velocity before snapping and sliding it
	floor_impact_velocity = Vector3.ZERO
	remaining_floor_impact_velocity = Vector3.ZERO
	if is_on_floor and not prev_on_floor :
		floor_impact_velocity = velocity
	
	
	# Snap logic
	var velocities_facing_up : bool = (velocity + platform_velocity).dot(up_direction) > CMP_EPSILON
	var velocity_facing_up : bool = velocity.dot(up_direction) > CMP_EPSILON
	
	# Check if hit floor while sliding
	var hit_floor_during_sliding : bool = false
	for collision_state in collision_results :
		if collision_state == rest_result_state :
			continue
		if collision_state.s_floor :
			hit_floor_during_sliding = true
			break
	
	var slid_velocity_on_floor : bool = false
	if (not velocity_facing_up and not detach_from_platform) or snap_on_platform :
		if (prev_on_floor or prev_on_floor_surface) and not hit_floor_during_sliding :
			snap_to_floor(max_snap_length)
		
		if is_on_floor :
			if floor_stop_on_slope :
				velocity = velocity.slide(up_direction)  # Fully clip gravity
			else :
				velocity = velocity.slide(current_floor_normal)  # Slide down on slope
			
			slid_velocity_on_floor = true
	
	
	# Process rigid body collisions
	if interact_with_rigid_bodies :
		for collider_data in collider_datas :
			var id : int = collider_data.collider_id
			if is_instance_id_valid(id) :
				var collider_object : Object = instance_from_id(id)
				if collider_object is RigidBody3D :
					var rigidbody := collider_object as RigidBody3D
					
					# Handle wall and ceiling collisions
					if collider_data.collision_type != ColliderData.CollisionType.FLOOR :
						var impulse_data : CollisionImpulseData = calculate_impulse_from_collision(rigidbody, collider_data, velocity + platform_velocity)
						if not impulse_data.bodies_separating :
							var impulse : Vector3 = impulse_data.impulse
							var impulse_position : Vector3 = impulse_data.impulse_position
							
							apply_impulse(-impulse)
							rigidbody.apply_impulse(impulse, impulse_position)
					
					# Apply weight force and land impact impulse to floor rigid bodies
					if collider_data.collision_type == ColliderData.CollisionType.FLOOR :
						var rigidbody_body_state : PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(rigidbody.get_rid())
						
						# Apply weight force
						if not velocities_facing_up and prev_on_floor :
							var avarage_collision_point : Vector3 = Math.calculate_vector3_array_avarage(collider_data.collision_points)
							var collision_normal : Vector3 = collider_data.collision_normal
							var normal_dot_up : float = collision_normal.dot(up_direction)
							if normal_dot_up > 0.0 :
								var relative_velocity : Vector3 = prev_platform_velocity - platform_velocity
								var downward_relative_velocity : float = relative_velocity.dot(-up_direction)
								if downward_relative_velocity > 0.0 :
									var current_downward_accel : float = downward_relative_velocity / p_delta_t
									if current_downward_accel < gravity_acceleration :
										var accel_diff : float = gravity_acceleration - current_downward_accel
										var impulse_length : float = mass * accel_diff * normal_dot_up * p_delta_t
										var impulse : Vector3 = impulse_length * -up_direction
										var impulse_position : Vector3 = avarage_collision_point - rigidbody.global_position
										
										rigidbody.apply_impulse(impulse, impulse_position)
						
						# Apply floor impact impulse
						if not prev_on_floor :
							var impact_impulse_data : CollisionImpulseData = calculate_impulse_from_collision(rigidbody, collider_data, floor_impact_velocity)
							if not impact_impulse_data.bodies_separating :
								var impulse : Vector3 = impact_impulse_data.impulse
								
								remaining_floor_impact_velocity = floor_impact_velocity + (-impulse * inverse_mass)
								
								if remaining_floor_impact_velocity.dot(up_direction) > 0.0 :
									# Apply impulse with counter torque only if the resulting velocity points upwards
									apply_impulse_to_rb_platform(impact_impulse_data, rigidbody, rigidbody_body_state)
								else :
									# Apply impulse without counter torque
									var impulse_position : Vector3 = impact_impulse_data.impulse_position
									rigidbody.apply_impulse(impulse, impulse_position)
	
		# Keep floor state stable when pushing bodies on ground
		if slid_velocity_on_floor :
			if floor_stop_on_slope :
				velocity = velocity.slide(up_direction)  # Fully clip gravity
			else :
				velocity = velocity.slide(current_floor_normal)  # Slide down on slope




## Performs the core logic of a floating movement
func floating_move(p_delta_t : float) -> void :
	
	# Clear saved collision entity datas
	prev_had_floor_collider_saved = current_floor_collider_encoded != null
	current_floor_collider_encoded = null
	
	current_floor_collider_data = null 
	collider_datas.clear()
	
	
	# Get data of current collisions before moving
	# HACK (EXTERNAL) : The method 'body_test_motion' (regardless of the physics server) does not work as advertised
	# since it does not actually report the collisions from recoveries, so an extra call is needed
	# unfortunately to get the current collisions before moving with move_and_slide.
	var res := PhysicsTestMotionResult3D.new()
	var result_state := CollisionState.new()
	_move_and_collide(res, Vector3.ZERO, true, true, MAX_SNAP_COLLISIONS, false, safe_margin, false)
	set_collision_state(res, result_state)
	collision_results.append(result_state)
	update_overall_state(result_state, false, true, false)
	
	
	# Move with motion
	var motion : Vector3 = velocity * p_delta_t
	slide_count = 0
	
	move_and_slide_floating(motion)
	
	
	# Add velocity from moving walls and ceilings
	add_wall_and_ceiling_push_velocity()
	
	
	# Process rigid body collisions
	if interact_with_rigid_bodies :
		for collider_data in collider_datas :
			var id : int = collider_data.collider_id
			if is_instance_id_valid(id) :
				var collider_object : Object = instance_from_id(id)
				if collider_object is RigidBody3D :
					var rigidbody := collider_object as RigidBody3D
					var impulse_data : CollisionImpulseData = calculate_impulse_from_collision(rigidbody, collider_data, velocity + platform_velocity)
					if not impulse_data.bodies_separating :
						var impulse : Vector3 = impulse_data.impulse
						var impulse_position : Vector3 = impulse_data.impulse_position
						
						apply_impulse(-impulse)
						rigidbody.apply_impulse(impulse, impulse_position)




## Adds the push velocity from moving walls and ceiling
func add_wall_and_ceiling_push_velocity() -> void :
	var wall_push_velocity := Vector3.ZERO
	for collider_data : ColliderData in collider_datas :
		if collider_data.collision_type == ColliderData.CollisionType.FLOOR :
			continue
		
		var id : int = collider_data.collider_id
		
		if is_instance_id_valid(id) :
			var wall_collider_object : Object = instance_from_id(id)
			
			if wall_collider_object is StaticBody3D or wall_collider_object is AnimatableBody3D :
				var wall_collider := wall_collider_object as PhysicsBody3D
				var wall_rid : RID = wall_collider.get_rid()
				var excluded : bool = (moving_wall_and_ceiling_layers & PhysicsServer3D.body_get_collision_layer(wall_rid)) == 0
				
				if wall_rid.is_valid() and not excluded:
					var wall_body_state : PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(wall_rid)
					if wall_body_state :
						var local_position : Vector3 = global_position - wall_body_state.transform.origin
						var wall_velocity  : Vector3 = wall_body_state.get_velocity_at_local_position(local_position)
						var wall_normal    : Vector3 = collider_data.collision_normal
						
						# Skip if wall normal is not facing the wall's velocity or wall velocity is zero
						var wall_velocity_dot_normal : float = wall_velocity.dot(wall_normal)
						if wall_velocity.is_zero_approx() or wall_velocity_dot_normal < 0.0 :
							continue
						
						# Apply less velocity, the less the wall is facing it's velocity
						wall_push_velocity += wall_velocity_dot_normal * wall_normal
	
	# Modify velocity wall push velocity
	if not wall_push_velocity.is_zero_approx() :
		var push_dir      : Vector3 = wall_push_velocity.normalized()
		var wished_speed  : float   = wall_push_velocity.length()
		var current_speed : float   = (velocity + platform_velocity).dot(push_dir)  # Also account for platform velocity
		
		if current_speed < wished_speed :
			var velocity_to_add : Vector3 = (wished_speed - current_speed) * push_dir
			
			velocity += velocity_to_add




## Does a motion test, setting the given result object, optionally adjusts motion to cancel sliding, 
## optionally moves the body to the end position, returns true if collision happened.
# NOTICE : The built in 'move_and_collide' does not suffice because :
# - Slide cacelling can't be turned off
# - The returned KinematicCollision3D object provides too little information about the collision
func _move_and_collide(p_res : PhysicsTestMotionResult3D, p_motion : Vector3, p_test_only : bool = false, p_rec : bool = false, p_max_col : int = 1, p_cancel_sliding : bool = false, p_margin : float = safe_margin, p_platform_pass : bool = false) -> bool :
	var params := PhysicsTestMotionParameters3D.new()
	
	params.motion = p_motion
	params.from = global_transform
	params.margin = p_margin
	params.max_collisions = p_max_col
	params.recovery_as_collision = p_rec
	
	# Exclude platform object, when applying platform motion
	if p_platform_pass and current_floor_collider_encoded :
		params.exclude_objects = [current_floor_collider_encoded.object_id]
	
	var collided      : bool = PhysicsServer3D.body_test_motion(self, params, p_res)
	var travel        : Vector3 = p_res.get_travel()
	var modify_travel : bool = false 
	
	
	# Cancel sliding caused be recovery, if needed
	if p_cancel_sliding :
		var motion_length : float = p_motion.length()
		var precision     : float = CMP_EPSILON
		
		# No slide cancel if collision depth is too large
		if collided :
			var unsafe_fraction : float = p_res.get_collision_unsafe_fraction()
			var safe_fraction   : float = p_res.get_collision_safe_fraction()
			
			precision += motion_length * (unsafe_fraction - safe_fraction)
			
			if p_res.get_collision_depth(0) > safe_margin + precision :
				p_cancel_sliding = false
		
		if p_cancel_sliding :
			# When motion is null, recovery is the resulting motion
			var motion_normal : Vector3
			
			if motion_length > CMP_EPSILON :
				motion_normal = p_motion / motion_length
			
			# Check depth of recovery
			var projected_length : float   = travel.dot(motion_normal)
			var recovery         : Vector3 = travel - motion_normal * projected_length
			var recovery_length  : float   = recovery.length()
			
			# Fixes cases when cancelling slide causes the motion to go too deep into the ground,
			# because we're only taking rest information into account and not general recovery.
			if recovery_length < safe_margin + precision :
				# Apply adjustment to motion
				travel = motion_normal * projected_length
				modify_travel = true
	
	# Take axis linear lock into account
	if axis_lock_linear_x : travel.x = 0.0
	if axis_lock_linear_y : travel.y = 0.0
	if axis_lock_linear_z : travel.z = 0.0
	
	if modify_travel :
		# Set meta data with modified results (Because setting the properties is not possible.)
		p_res.set_meta("travel", travel)
		p_res.set_meta("remainder", p_motion - travel)
	
	if not p_test_only :
		global_position += travel
	
	return collided




## 'move and slide' collision response for the grounded movement
func move_and_slide_grounded(p_motion : Vector3, p_platform_pass : bool = false) -> void :
	
	# Done maximum allowed amount of iterations
	# NOTE : this shouldn't really happen, 6 slides should be enough for most cases
	if slide_count > MAX_SLIDES :
		slide_count = MAX_SLIDES  # Set accurate value for debugging
		return
	
	var m_result := PhysicsTestMotionResult3D.new()
	
	# No sliding on first attempt to avoid sliding down on slopes due to recovery motion
	# NOTICE : This can causes an extra iteration when colliding with floor!
	# INFO : Using a safe margin larger than 0 can cause the character to slowly slide down slopes,
	# because the recovery motion will push the character directly away from the surface, 
	# which is not straight up in the case of slopes.
	# Make sure sliding is enabled after first iteration
	if not sliding_enabled and slide_count > 0 :
		sliding_enabled = true
	
	# Perform recovery, move body along motion, stop if collided
	var collided : bool = _move_and_collide(m_result, p_motion, false, true, MAX_SLIDE_COLLISIONS, not sliding_enabled, safe_margin, p_platform_pass)
	
	# Moved all the way, no collision to respond to
	if not collided :
		# Apply constant speed when moving down slope, if needed
		var _new_motion : Vector3
		if floor_constant_speed and slide_count == 0 and on_floor_if_snapped(velocity.dot(up_direction) > 0.0) :
			var _travel : Vector3 = m_result.get_meta("travel") if m_result.has_meta("travel") else m_result.get_travel()
			var motion_slide_norm : Vector3 = up_direction.cross(p_motion).cross(prev_floor_normal).normalized()
			
			global_position -= _travel
			_new_motion = motion_slide_norm * p_motion.slide(up_direction).length()
			collided = true
		
		if not collided or _new_motion.is_zero_approx() :
			return
		
		# New iteration
		slide_count += 1
		move_and_slide_grounded(_new_motion, p_platform_pass)
		return
	
	var result_state := CollisionState.new()
	
	# Store info about collision
	set_collision_state(m_result, result_state)
	collision_results.append(result_state)
	
	# Update state with collision
	var prev_iteration_on_wall : bool = is_on_wall
	update_overall_state(result_state, true, true, true)
	
	# Motion was zero, no need to slide
	if p_motion.is_zero_approx() :
		return
	
	
	# Set new motion for next iteration, also modify velocity
	var apply_default_sliding : bool    = true
	var velocity_facing_up    : bool    = velocity.dot(up_direction) > 0.0
	var horizontal_motion     : Vector3 = p_motion.slide(up_direction)
	var collision_normal      : Vector3 = m_result.get_collision_normal(0)  # Get deepest collision normal
	var remaining_motion      : Vector3
	var travel                : Vector3
	
	# Get the modified values caused by slide cancelling if needed
	if not sliding_enabled and m_result.has_meta("travel") and m_result.has_meta("remainder") :
		remaining_motion = m_result.get_meta("remainder")
		travel = m_result.get_meta("travel")
	else :
		remaining_motion = m_result.get_remainder()
		travel = m_result.get_travel()
	
	var new_motion : Vector3 = remaining_motion
	
	# Strictly downward velocity will not move the body when on floor
	# NOTE : This can save a slide iteration.
	if floor_stop_on_slope and result_state.s_floor and (velocity.normalized() + up_direction).length_squared() < 0.0001 :
		if travel.length() <= safe_margin + CMP_EPSILON :
			global_position -= travel  # Revert motion
		
		var collided_with_rigidbody : bool = interact_with_rigid_bodies and m_result.get_collider(result_state.deepest_floor_index) is RigidBody3D
		if not p_platform_pass and not collided_with_rigidbody : velocity = Vector3.ZERO
		
		return
	
	# Wall specific response, when moving horizontally against it
	if result_state.s_wall and horizontal_motion.dot(result_state.wall_normal) < 0.0 :
		var horizontal_normal       : Vector3 = result_state.wall_normal.slide(up_direction).normalized()
		# If rigid body interaction is enabled, don't modify velocity on wall if it belongs to a rigid body
		var collided_with_rigidbody : bool = interact_with_rigid_bodies and m_result.get_collider(result_state.deepest_wall_index) is RigidBody3D
		# Don't slide velocity, if it's facing away from the wall!
		var slide_velocity_on_wall  : bool = not p_platform_pass and velocity.dot(result_state.wall_normal) < 0.0 and not collided_with_rigidbody
		
		# Handle stepping and obstacle climbing
		var wall_is_vertical_approx : bool = absf(result_state.wall_normal.dot(up_direction)) < WALL_VERTICAL_APPROX_CMP
		# INFO : An integer is used, so that floor collision and floor surface can be differentiated.
		# If 'use_surface_normals' is enabled, the 'is_min_distance_to_floor' method should always run, 
		# to get floor surface status.
		var close_to_floor : int = (prev_on_floor or is_on_floor or is_min_distance_to_floor(max_step_height)) as int if not use_surface_normals else is_min_distance_to_floor(max_step_height)
		if not is_zero_approx(max_step_height) and (prev_on_floor or close_to_floor) :
			# Handle special cases for surface normal stepping, when floor is surface only.
			if use_surface_normals and close_to_floor == FloorType.SURFACE and not wall_is_vertical_approx :
				var wall_surface_normal : Vector3 = search_for_wall_surface_normal_below(m_result, result_state.wall_collision_indexes)
				var horizontal_wall_surface_normal : Vector3 = wall_surface_normal.slide(up_direction).normalized() if not wall_surface_normal == Vector3.ZERO else Vector3.ZERO
				
				# INFO : Only allow climbing if the horizontal motion faces opposite of the wall's surface normal.
				# This stops the climbing from triggering, when colliding with a ledge from above.
				var moving_away_from_ledge : bool = not wall_surface_normal == Vector3.ZERO and horizontal_motion.dot(horizontal_wall_surface_normal) > 0.0
				
				# Try and slide up to stable ground
				if not moving_away_from_ledge and horizontal_motion.slide(result_state.wall_normal).dot(up_direction) > 0.0 :
					# Slide using the intersection between the motion plane and the wall plane,
					# in order to keep the direction intact
					# Add offset to motion to help move to stable floor at low velocities
					var offset_dir : Vector3 = -horizontal_normal
					new_motion = new_motion.slide(up_direction) + offset_dir * safe_margin * RECOVERY_FACTOR
					
					# Retain horizontal magnitude of remaining motion
					var wished_horizontal_length : float = new_motion.length()
					
					new_motion = up_direction.cross(new_motion).cross(result_state.wall_normal)
					
					var current_horizontal_length : float = new_motion.slide(up_direction).length()
					if not is_zero_approx(current_horizontal_length) : 
						new_motion *= (wished_horizontal_length / current_horizontal_length)
					
					# Reset gravity accumulation
					if slide_velocity_on_wall and not velocity_facing_up: 
						velocity = velocity.slide(up_direction)
					
					# Start new iteration
					slide_count += 1
					move_and_slide_grounded(new_motion, p_platform_pass)
					return
			# INFO : When surface normal stepping is allowed, the vertical wall check optimization
			# is ignored to make surface normal stepping work on non vertical walls.
			elif not has_stepped and (wall_is_vertical_approx or use_surface_normals) :
				var step_height        : float   = 0.0
				var h_remaining_motion : Vector3 = remaining_motion.slide(up_direction)
				var step_motion_offset : Vector3 = -horizontal_normal * safe_margin * RECOVERY_FACTOR
				var step_motion        : Vector3 = h_remaining_motion
				# INFO : Step motion is extended, so that recoveries won't cancel out the motion towards
				# the wall. This ensures stepping will always happen when collided with step facing towards it,
				# regardless of the remaining motion.
				
				step_height = test_for_step(step_motion + step_motion_offset, horizontal_normal)
				
				if not is_zero_approx(step_height) :
					var up : Vector3 = (step_height + safe_margin) * up_direction
					
					has_stepped = true
					global_position += up  # Move up to step height
					is_on_wall = prev_iteration_on_wall  # Don't set wall flag, since a step is not considered a wall
					
					# Find and remove wall reference in stored data, since a step is not considered a wall
					var collider_index : int = m_result.get_collider_id(result_state.deepest_wall_index)
					var array_index    : int = ColliderData.find_by_id(collider_datas, collider_index) 
					if array_index != -1 :
						collider_datas.remove_at(array_index)
					
					new_motion = step_motion + step_motion_offset
					
					# Start new iteration
					slide_count += 1
					move_and_slide_grounded(new_motion, p_platform_pass)
					return
		
		# Stop jittering in corners
		if result_state.wall_normals.size() > 1 :
			for wall_normal in result_state.wall_normals :
				
				if wall_normal.is_equal_approx(result_state.wall_normal) :
					continue
				
				# Collision is corner case
				if wall_normal.dot(result_state.wall_normal) > 0.0 :
					var h_wall_normal : Vector3 = wall_normal.slide(up_direction).normalized()
					
					# If the slides would go against each other, stop horizontal motion
					if p_motion.slide(horizontal_normal).dot(p_motion.slide(h_wall_normal)) < 0.0 :
						new_motion = new_motion.dot(up_direction) * up_direction
						
						if slide_velocity_on_wall : velocity = velocity.dot(up_direction) * up_direction
						
						# Cancel travel if not too much
						if travel.length() < safe_margin * RECOVERY_FACTOR :
							global_position -= travel
						
						if new_motion.is_zero_approx() :
							return
						
						# Start new iteration
						slide_count += 1
						move_and_slide_grounded(new_motion, p_platform_pass)
						return
		
		if result_state.s_wall_floor_support :
			new_motion = new_motion.slide(result_state.floor_normal)  # Also slide along wall floor
			
			# Stop sliding up wall floor due to recovery
			if floor_block_on_wall and not velocity_facing_up and horizontal_motion.dot(result_state.floor_normal) < -CMP_EPSILON:
				if slide_velocity_on_wall : velocity = velocity.dot(up_direction) * up_direction
				global_position -= travel
				return
		
		# Don't slide up on steep slopes when on floor and velocity is not facing up
		if floor_block_on_wall :
			apply_default_sliding = false
			
			# Set new motion
			if (prev_on_floor or is_on_floor) and not velocity_facing_up :
				# Revert motion
				# NOTE : This is code is from the og move_and_slide, but is this right?
				var travel_length : float = travel.length()
				var cancel_dist_max : float = minf(0.1, safe_margin * 20)
				if travel_length <= safe_margin + CMP_EPSILON :
					global_position -= travel
					travel = Vector3.ZERO
				elif travel_length < cancel_dist_max :
					global_position -= travel.slide(up_direction)
					new_motion = p_motion.slide(up_direction)
					travel = Vector3.ZERO
				
				# Don't snap if stepped, or the character will be back on previous floor
				if not has_stepped : 
					snap_to_floor(max_snap_length)
				
				new_motion = new_motion.slide(horizontal_normal)
				
			else :
				new_motion = new_motion.slide(result_state.wall_normal)
			
			# Scales the horizontal velocity according to the wall slope.
			if slide_velocity_on_wall :
				if velocity_facing_up :
					var slide_vel : Vector3 = velocity.slide(result_state.wall_normal)
					velocity = up_direction * velocity.dot(up_direction) + slide_vel.slide(up_direction)
				else :
					velocity = velocity.slide(horizontal_normal)
		
		else :
			if slide_velocity_on_wall : velocity = velocity.slide(result_state.wall_normal)
		
		# Stop horizontal motion when under wall slide threshold.
		var wall_is_fully_vertical : bool = result_state.wall_normal.is_equal_approx(horizontal_normal)
		if wall_min_slide_angle > 0.0 and (floor_block_on_wall or wall_is_fully_vertical) and not velocity_facing_up:
			var motion_angle : float = absf(acos(-horizontal_normal.dot(horizontal_motion.normalized())))
			
			if motion_angle < wall_min_slide_angle :
				new_motion = up_direction * new_motion.dot(up_direction)
				
				# Revert motion caused by recovery
				if travel.length() < safe_margin * RECOVERY_FACTOR + CMP_EPSILON :
					global_position -= travel
					travel = Vector3.ZERO
				
				if slide_velocity_on_wall : velocity = up_direction * velocity.dot(up_direction)
	
	# Other wall collisions, where not horizontally moving against it (so only vertically moving against it)
	elif result_state.s_wall and velocity.dot(result_state.wall_normal) < 0.0 :
		if wall_slide_vertical_only_collision or not floor_block_on_wall :
			var collided_with_rigidbody : bool = interact_with_rigid_bodies and m_result.get_collider(result_state.deepest_wall_index) is RigidBody3D
			var slide_velocity : bool = not p_platform_pass and velocity.dot(result_state.wall_normal) < 0.0 and not collided_with_rigidbody
			
			if slide_velocity : velocity = velocity.slide(result_state.wall_normal)
	
	
	# Ceiling specific response
	if result_state.s_ceiling and p_motion.dot(result_state.ceiling_normal) < 0.0 :
		var collided_with_rigidbody : bool = interact_with_rigid_bodies and m_result.get_collider(result_state.deepest_ceiling_index) is RigidBody3D
		var slide_velocity : bool = not p_platform_pass and velocity.dot(result_state.ceiling_normal) < 0.0 and not collided_with_rigidbody
		
		if slide_on_ceiling or not velocity_facing_up :
			if slide_velocity : velocity = velocity.slide(result_state.ceiling_normal)
		else :
			# Remove vertical component
			if slide_velocity : velocity = velocity.slide(up_direction)
			new_motion = new_motion.slide(up_direction)
	
	
	# Return if no motion remains, after handling special cases
	if new_motion.is_zero_approx() :
		return
	
	if apply_default_sliding :
		var slide_motion : Vector3 = new_motion.slide(collision_normal)
		
		if sliding_enabled or not is_on_floor:
			# Try to keep horizontal motion direction when sliding on floor and not on wall
			if result_state.s_floor and not result_state.s_wall and not horizontal_motion.is_zero_approx() :
				# Slide using the intersection between the motion plane and the floor plane,
				# in order to keep the direction intact
				var motion_length : float  = slide_motion.length()
				slide_motion = up_direction.cross(remaining_motion).cross(result_state.floor_normal)
				
				# Keep original length to slow down when going up slopes
				slide_motion = slide_motion.normalized() * motion_length
			
			# Slide only if slide motion is facing towards velocity direction
			if slide_motion.dot(velocity) > 0.0 :
				new_motion = slide_motion
			else :
				new_motion = Vector3.ZERO
		
		else :
			new_motion = remaining_motion
	
	total_travel += travel
	
	# Apply constant speed
	if floor_constant_speed and can_apply_constant_speed and prev_on_floor and is_on_floor and not new_motion.is_zero_approx() :
		var travel_slide_up : Vector3 = total_travel.slide(up_direction)
		new_motion = new_motion.normalized() * maxf(0.0, initial_motion_slide_up.length() - travel_slide_up.length())
	
	can_apply_constant_speed = not can_apply_constant_speed and not sliding_enabled
	sliding_enabled = true
	
	# Iterate
	slide_count += 1
	move_and_slide_grounded(new_motion, p_platform_pass)




func move_and_slide_floating(p_motion : Vector3) -> void :
	# Done maximum allowed amount of iterations
	# NOTE : this shouldn't really happen, 6 slides should be enough for most cases
	if slide_count > MAX_SLIDES :
		slide_count = MAX_SLIDES  # Set accurate value for debugging
		return
	
	var m_result := PhysicsTestMotionResult3D.new()
	
	# Perform recovery, move body along motion, stop if collided
	var collided : bool = _move_and_collide(m_result, p_motion, false, true, MAX_SLIDE_COLLISIONS, false, safe_margin)
	
	# Moved all the way, no collision to respond to
	if not collided :
		return
	
	
	var result_state := CollisionState.new()
	
	# Store info about collision and update state with collision
	set_collision_state(m_result, result_state)
	collision_results.append(result_state)
	update_overall_state(result_state, false, true, false)
	
	# Motion was zero, no need to slide
	if p_motion.is_zero_approx() :
		return
	
	
	# Set new motion for next iteration, also modify velocity
	var remaining_motion      : Vector3 = m_result.get_remainder()
	var travel                : Vector3 = m_result.get_travel()
	var new_motion            : Vector3 = remaining_motion
	
	# If rigid body interaction is enabled, don't modify velocity on wall if it belongs to a rigid body
	var collided_with_rigidbody : bool = interact_with_rigid_bodies and m_result.get_collider(result_state.deepest_wall_index) is RigidBody3D
	# Don't slide velocity, if it's facing away from the wall!
	var slide_velocity_on_wall  : bool = velocity.dot(result_state.wall_normal) < 0.0 and not collided_with_rigidbody
	
	if wall_min_slide_angle > 0.0 and acos(result_state.wall_normal.dot(-velocity.normalized())) < wall_min_slide_angle + ANGLE_CMP_EPSILON :
		# Stop motion and velocity when under wall slide threshold
		# Revert motion caused by recovery
		if travel.length() < safe_margin * RECOVERY_FACTOR + CMP_EPSILON :
			global_position -= travel
			travel = Vector3.ZERO
		
		if slide_velocity_on_wall : 
			velocity = Vector3.ZERO
		
		return
	elif slide_count == 0 :
		var motion_slide_norm : Vector3 = remaining_motion.slide(result_state.wall_normal).normalized()
		new_motion = motion_slide_norm * (p_motion.length() - travel.length())
		if slide_velocity_on_wall : 
			velocity = velocity.slide(result_state.wall_normal)
	else :
		new_motion = remaining_motion.slide(result_state.wall_normal)
		if slide_velocity_on_wall : 
			velocity = velocity.slide(result_state.wall_normal)
	
	if new_motion.dot(velocity) <= 0.0 :
		return
	
	if new_motion.is_zero_approx() :
		return
	
	# Iterate
	slide_count += 1
	move_and_slide_floating(new_motion)




## Checks if the character would be on the floor when snapped
func on_floor_if_snapped(p_velocity_facing_up : bool) -> bool :
	
	if up_direction == Vector3.ZERO or is_on_floor or not prev_on_floor or p_velocity_facing_up :
		return false
	
	var snap_length : float = maxf(max_snap_length, snap_safe_margin)
	
	var collided    : bool
	var snap_motion := -up_direction * snap_length
	var m_result    := PhysicsTestMotionResult3D.new()
	
	collided = _move_and_collide(m_result, snap_motion, true, true, 4, false, snap_safe_margin)
	
	if not collided :
		return false
	
	var result_state := CollisionState.new()
	
	set_collision_state(m_result, result_state)
	
	return result_state.s_floor




## Checks if the body is at least the given distance away from the floor.
## Returns 0 if no floor found.
## Rerurns 1 if floor collision is found or already on floor.
## Returns 2 if no floor collision is found, but a floor surface is found.
func is_min_distance_to_floor(p_min_distance : float) -> FloorType :
	
	var min_distance : float = maxf(snap_safe_margin, p_min_distance)
	
	var collided    : bool
	var test_motion := Vector3.DOWN * min_distance
	var m_result    := PhysicsTestMotionResult3D.new()
	
	collided = _move_and_collide(m_result, test_motion, true, true, 4, false, safe_margin)
	
	if not collided :
		return FloorType.NONE
	
	var result_state := CollisionState.new()
	
	set_collision_state(m_result, result_state)
	
	if result_state.s_floor :
		return FloorType.COLLISION
	
	if use_surface_normals and result_state.s_wall :
		var feet_pos            : Vector3 = get_feet_position()
		var max_surface_height  : float = feet_pos.dot(up_direction)
		var floor_surface_found : bool = search_for_floor_surface_normal(m_result, max_surface_height)
		
		if floor_surface_found :
			return FloorType.SURFACE
	
	return FloorType.NONE





## If a step is detected, this method will return the height of the step (always positive),
## otherwise it will return 0, if any of the tests fail.
func test_for_step(p_horizontal_motion : Vector3, p_horizontal_wall_normal : Vector3) -> float :
	
	if p_horizontal_motion.is_zero_approx() :
		return 0.0
	
	# NOTE : floor condition is checked outside.
	
	# Check if step can be climbed
	var motion_tester  := MotionTester.new()
	var test_transform : Transform3D = global_transform
	
	# UP
	var up := up_direction * max_step_height
	
	motion_tester.test_motion(self, test_transform, up, safe_margin, false, 0)
	# NOTE : We only need the travel of the motion, so no collision report is needed.
	# NOTE : Safe margin should always be high enough to avoid tunneling!
	# Recovery will affect the motion, but can be compensated.
	
	# FORWARD
	test_transform.origin = motion_tester.endpos
	var forward : Vector3 = p_horizontal_motion
	
	motion_tester.test_motion(self, test_transform, forward, safe_margin, false, 0)
	
	# DOWN
	var down := -up_direction * max_step_height
	test_transform.origin = motion_tester.endpos
	
	motion_tester.test_motion(self, test_transform, down, safe_margin, true, 4)
	
	if not motion_tester.hit :
		return 0.0
	
	var step_height : float = (motion_tester.endpos - global_position).dot(up_direction)
	
	if step_height < 0.0 or is_zero_approx(step_height) :
		return 0.0
	
	var result_state := CollisionState.new()
	set_collision_state(motion_tester.res, result_state)
	
	if not result_state.s_floor :
		if use_surface_normals :
			var floor_found : bool
			var feet_pos : Vector3 = get_feet_position()
			var max_surface_height : float = feet_pos.dot(up_direction) + step_height
			
			floor_found = search_for_floor_surface_normal(motion_tester.res, max_surface_height, motion_tester.endpos + collider.position)
			if not floor_found :
				return 0.0
		else :
			return 0.0
	
	# Check if climbed step, and not a slope
	# INFO : We check the angle of the horizontal summed motion and the wall we are trying to climb.
	# If the motion direction is looking away from the wall, or is parallel to the wall,
	# the character did not climb it.
	var h_motion_sum : Vector3 = (motion_tester.endpos + (-up_direction) * step_height) - global_position
	
	# NOTE : When sliding parallel along a flat wall, the horizontal motion towards the wall normal
	# can be less than zero due to recoveries, so an epsilon is needed. Otherwise steps can be
	# falsely detected.
	if h_motion_sum.dot(p_horizontal_wall_normal) >= -safe_margin :
		return 0.0
	
	return step_height





## Calculates information about given collisions
## and stores it in the given 'CollisionState' object.
func set_collision_state(p_motion_result : PhysicsTestMotionResult3D, p_col_state : CollisionState, rest_check : bool = false) -> void :
	
	# Check for null reference
	if not p_motion_result :
		printerr("PhysicsTestMotionResult3D object is null!")
		return
	
	p_col_state.motion_result = p_motion_result
	
	var collision_count      : int = 0
	var wall_collision_count : int = 0
	
	# Collision normals will be prioratized by collision depth.
	var max_wall_depth       : float = -1.0
	var max_floor_depth      : float = -1.0
	var max_ceiling_depth    : float = -1.0
	
	var wall_normal_sum      := Vector3.ZERO
	var tmp_wall_normal      := Vector3.ZERO  # Avoid duplicate on avarage calculation
	
	collision_count = p_motion_result.get_collision_count()
	
	for i in collision_count :
		var collision_normal : Vector3 = p_motion_result.get_collision_normal(i)
		var collision_depth  : float = p_motion_result.get_collision_depth(i)
		
		# NOTE: When checking collisions in a resting positions, we only want the ones where
		# The orher body is moving towards the character, and the character is not moving towards
		# the other body.
		if rest_check :
			var collider_velocity : Vector3 = p_motion_result.get_collider_velocity(i)
			var character_summed_velocity : Vector3 = velocity + platform_velocity
			if velocity.dot(collision_normal) < 0.0 or collision_normal.dot(character_summed_velocity - collider_velocity) > 0.0 :
				continue
		
		if motion_mode == MotionMode.GROUNDED :
			# Check if floor collision
			var floor_angle : float = acos(collision_normal.dot(up_direction))
			if floor_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
				p_col_state.s_floor = true
				p_col_state.floor_collision_indexes.append(i)
				
				if collision_depth > max_floor_depth :
					max_floor_depth = collision_depth
					p_col_state.floor_normal = collision_normal
					p_col_state.deepest_floor_index = i
				continue
			
			# Check if ceiling collision
			var ceiling_angle : float = acos(collision_normal.dot(-up_direction))
			if ceiling_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
				p_col_state.s_ceiling = true
				p_col_state.ceiling_collision_indexes.append(i)
				
				if collision_depth > max_ceiling_depth : 
					max_ceiling_depth = collision_depth
					p_col_state.ceiling_normal = collision_normal
					p_col_state.deepest_ceiling_index = i
				continue
		
		p_col_state.s_wall = true  # Collision is wall by default
		p_col_state.wall_collision_indexes.append(i)  # Store wall collision index
		
		if collision_depth > max_wall_depth :
			max_wall_depth = collision_depth
			p_col_state.wall_normal = collision_normal
			p_col_state.deepest_wall_index = i
		
		# Collect wall normals for calculating avarage
		if motion_mode == MotionMode.GROUNDED and not collision_normal.is_equal_approx(tmp_wall_normal) :
			tmp_wall_normal = collision_normal
			wall_normal_sum += collision_normal
			wall_collision_count += 1
			
			# Store unique wall normals
			p_col_state.wall_normals.append(collision_normal)
	
	if motion_mode == MotionMode.GROUNDED and wall_collision_count > 1 :
		var combined_wall_normal : Vector3 = wall_normal_sum.normalized()
		var on_regular_floor : bool = (is_on_floor or prev_on_floor) and not prev_on_wall_floor
		
		if not p_col_state.s_floor and not on_regular_floor :
			# Check if wall normals cancel out to floor support
			# Only register wall floors, if not on regular floor, to avoid sliding up on walls when
			# standing on regular floor.
			var floor_angle : float = acos(combined_wall_normal.dot(up_direction))
			if floor_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
				p_col_state.s_floor = true
				p_col_state.s_wall_floor_support = true
				p_col_state.floor_normal = combined_wall_normal
				# NOTE : Keep wall state for proper sliding!
				return
		
		if not p_col_state.s_ceiling :
			# Check if wall normals cancel out to ceiling support
			var ceiling_angle : float = acos(combined_wall_normal.dot(-up_direction))
			if ceiling_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
				p_col_state.s_ceiling = true
				p_col_state.ceiling_normal = combined_wall_normal




## Updates the overall state with the given collision state, assuming it's the latest.
func update_overall_state(p_col_state : CollisionState, p_set_floor : bool, p_set_wall : bool, p_set_ceiling : bool) -> void :
	
	# Floor
	if p_set_floor and p_col_state.s_floor :
		is_on_floor = true
		is_on_wall_floor = is_on_wall_floor or p_col_state.s_wall_floor_support
		current_floor_normal = p_col_state.floor_normal
		
		# Store unique floor collider datas
		var deepest_floor_array_index : int = -1
		for index in p_col_state.floor_collision_indexes :
			var array_index : int = ColliderData.append_or_update(collider_datas, p_col_state, index, ColliderData.CollisionType.FLOOR)
			if index == p_col_state.deepest_floor_index :
				deepest_floor_array_index = array_index
				
				# Store floor collider
				if not current_floor_collider_encoded :
					current_floor_collider_encoded = EncodedObjectAsID.new()
				current_floor_collider_encoded.object_id = p_col_state.motion_result.get_collider_id(p_col_state.deepest_floor_index)
				
		# Wall floor support does not have a collider, we take it into account
		if deepest_floor_array_index != -1 :
			current_floor_collider_data = collider_datas[deepest_floor_array_index]
	
	# Wall
	if p_set_wall and p_col_state.s_wall :
		is_on_wall = true
		current_wall_normal = p_col_state.wall_normal
		
		# Store unique wall collider datas
		for index in p_col_state.wall_collision_indexes :
			ColliderData.append_or_update(collider_datas, p_col_state, index, ColliderData.CollisionType.WALL)
	
	# Ceiling
	if p_set_ceiling and p_col_state.s_ceiling :
		is_on_ceiling = true
		current_ceiling_normal = p_col_state.ceiling_normal
		
		# Store unique ceiling collider datas
		for index in p_col_state.ceiling_collision_indexes :
			ColliderData.append_or_update(collider_datas, p_col_state, index, ColliderData.CollisionType.CEILING)
	
	# HACK : Make sure the floor collider's type and normal is not overwritten to wall or ceiling
	if current_floor_collider_data and current_floor_collider_data.collision_type != ColliderData.CollisionType.FLOOR :
		current_floor_collider_data.collision_normal = current_floor_normal
		current_floor_collider_data.collision_type = ColliderData.CollisionType.FLOOR




## Returns the centre-bottom (feet) position of the collider
func get_feet_position() -> Vector3 :
	
	if collider.shape is CylinderShape3D :
		var cylinder := collider.shape as CylinderShape3D
		return collider.global_position + (cylinder.height * 0.5) * -up_direction
	
	if collider.shape is BoxShape3D :
		var box := collider.shape as BoxShape3D
		return collider.global_position + (box.size.y * 0.5) * -up_direction
	
	if collider.shape is CapsuleShape3D :
		var capsule := collider.shape as CapsuleShape3D
		return collider.global_position + (capsule.height * 0.5) * -up_direction
	
	if collider.shape is SphereShape3D :
		var sphere := collider.shape as SphereShape3D
		return collider.global_position + (sphere.radius) * -up_direction
	
	printerr("Shape not handled by get_feet_position()!")
	
	return collider.global_position




## Returns the radius of the collider, if the collider is rounded, else it returns 0.0
func get_collider_radius() -> float :
	
	if collider.shape is CapsuleShape3D :
		var capsule := collider.shape as CapsuleShape3D
		return capsule.radius
	
	if collider.shape is SphereShape3D :
		var sphere := collider.shape as SphereShape3D
		return sphere.radius
	
	return 0.0  # Collider is not round




## Checks if the collision result includes a floor surface normal using raycasts
func search_for_floor_surface_normal(p_m_result : PhysicsTestMotionResult3D, p_max_height : float = 0.0, p_ray_origin : Vector3 = collider.global_position) -> bool :
	var raycast            := RayCast.new()
	var direct_space_state : PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	
	for i in p_m_result.get_collision_count() :
		var col_point  : Vector3 = p_m_result.get_collision_point(i)
		
		# If the surface is too high, it's not a floor
		var collider_radius : float = get_collider_radius()
		if col_point.dot(up_direction) > p_max_height + collider_radius :
			continue
		
		var col_normal : Vector3 = p_m_result.get_collision_normal(i)
		var offset_dir : Vector3 = -col_normal
		var dest       : Vector3 = col_point + offset_dir * RAYCAST_OFFSET_LENGTH
		
		raycast.intersect(p_ray_origin, dest, self, collision_mask, direct_space_state)
		
		var floor_angle : float = acos(raycast.normal.dot(up_direction))
		if raycast.hit and floor_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
			return true
	
	return false




## Checks if the collision result includes a wall surface normal
## below a wall collision point using raycasts
func search_for_wall_surface_normal_below(p_m_result : PhysicsTestMotionResult3D, p_wall_indexes : PackedInt32Array) -> Vector3 :
	var raycast            := RayCast.new()
	var direct_space_state : PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	
	for i in p_wall_indexes :
		var col_point  : Vector3 = p_m_result.get_collision_point(i)
		var col_normal : Vector3 = p_m_result.get_collision_normal(i)
		var dest       : Vector3 = col_point + (-up_direction) * RAYCAST_OFFSET_LENGTH  # Offset downward to find wall surface below
		var origin     : Vector3 = col_point + col_normal * RAYCAST_OFFSET_LENGTH
		
		raycast.intersect(origin, dest, self, collision_mask, direct_space_state)
		
		if not raycast.hit :
			continue
		
		var floor_angle : float = acos(raycast.normal.dot(up_direction))
		if floor_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
			continue
		
		var ceiling_angle : float = acos(col_normal.dot(-up_direction))
		if ceiling_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
			continue
		
		return raycast.normal
	
	return Vector3.ZERO




## Returns information about a collision between the character body and a rigid body
func calculate_impulse_from_collision(p_rigidbody : RigidBody3D, p_collider_data : ColliderData, p_character_velocity : Vector3 = velocity, p_recheck_for_points : bool = false) -> CollisionImpulseData :
	var impulse_data := CollisionImpulseData.new()
	var rigidbody_body_state : PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(p_rigidbody.get_rid())
	
	var average_point  : Vector3 = Math.calculate_vector3_array_avarage(p_collider_data.collision_points) if not p_recheck_for_points else Vector3.ZERO
	var average_normal : Vector3 = -p_collider_data.collision_normal
	# NOTE : This is not actually the avarage normal, but the latest collision's.
	# INFO : The calculations assume the collision normal is facing the rigid body.
	
	impulse_data.collision_normal = -average_normal  # Saved normal faces the character body
	
	if p_recheck_for_points :
		var collision_points : PackedVector3Array = calculate_collision_points_with_body(p_rigidbody)
		if collision_points.size() > 0 : 
			average_point = Math.calculate_vector3_array_avarage(collision_points)
		else  :
			average_point = Math.calculate_vector3_array_avarage(p_collider_data.collision_points)
	
	var m1 : float = mass
	var m2 : float = p_rigidbody.mass
	var I2_inv : Vector3 = rigidbody_body_state.inverse_inertia
	
	var r2 : Vector3 = average_point - p_rigidbody.global_transform.origin
	
	impulse_data.impulse_position = r2
	
	var v1 : Vector3 = p_character_velocity
	var v2 : Vector3 = p_rigidbody.linear_velocity
	
	var omega2 : Vector3 = p_rigidbody.angular_velocity
	
	var relative_velocity : Vector3 = (v2 + omega2.cross(r2)) - v1
	var v_rel_norm : float = relative_velocity.dot(average_normal)
	
	impulse_data.relative_speed_towards_normal = v_rel_norm
	
	if v_rel_norm > 0.0:
		impulse_data.bodies_separating = true
		return impulse_data  # No impulse needed if bodies are separating
		
	impulse_data.bodies_separating = false
	
	var restitution : float = minf(physics_material_bounce, p_rigidbody.physics_material_override.bounce if p_rigidbody.physics_material_override else 0.0)
	
	var j_denom : float = (1.0 / m1) + (1.0 / m2) + (average_normal.dot((r2.cross(average_normal) * I2_inv).cross(r2)))
	var j : float= -(1.0 + restitution) * v_rel_norm / j_denom
	var impulse : Vector3 = j * average_normal
	
	impulse_data.impulse = impulse
	
	return impulse_data




## Returns the collision points between the given body object and the character body
func calculate_collision_points_with_body(p_other_body : PhysicsBody3D) -> PackedVector3Array :
	var direct_space_state := get_world_3d().direct_space_state
	var params := PhysicsShapeQueryParameters3D.new()
	params.shape = collider.shape
	params.margin = safe_margin
	params.transform = global_transform
	params.collision_mask = 1 << 31
	params.exclude = [self]
	
	var original_body_layer : int = p_other_body.collision_layer
	p_other_body.collision_layer = 1 << 31
	var collision_points : PackedVector3Array = direct_space_state.collide_shape(params, 16)
	p_other_body.collision_layer = original_body_layer
	
	return collision_points




## Applies an impulse with reduced or eliminated torque, based on the given impulse's position
func apply_impulse_to_rb_platform(p_impulse_data : CollisionImpulseData, p_rigidbody : RigidBody3D, p_platform_body_state : PhysicsDirectBodyState3D) -> void :
	var impulse : Vector3 = p_impulse_data.impulse
	var impulse_position : Vector3 = p_impulse_data.impulse_position
	var collision_normal : Vector3 = p_impulse_data.collision_normal
	var rigidbody_center_of_mass : Vector3 = p_platform_body_state.center_of_mass - p_rigidbody.global_position
	var impulse_distance_from_center_of_mass : float = (impulse_position - rigidbody_center_of_mass).slide(collision_normal).length()
	
	if impulse_distance_from_center_of_mass < rigid_body_platform_central_impulse_threshold :
		p_rigidbody.apply_central_impulse(impulse)
	else :
		p_rigidbody.apply_impulse(impulse, impulse_position)
		
		 # Calculate and apply counter-torque if necessary
		var induced_torque : Vector3 = impulse_position.cross(impulse * rigid_body_platform_counter_torque_factor)
		p_rigidbody.apply_torque_impulse(-induced_torque)




#==================================================================================================
# ------------------------------
# | STRUCTS AND HELPER CLASSES |
# ------------------------------


## Stores imformation about about motion result collisions
class CollisionState extends RefCounted :
	var s_floor                   : bool
	var s_wall                    : bool
	var s_ceiling                 : bool
	var s_wall_floor_support      : bool
	var deepest_floor_index       : int
	var deepest_wall_index        : int
	var deepest_ceiling_index     : int
	var wall_normal               : Vector3
	var floor_normal              : Vector3
	var ceiling_normal            : Vector3
	var wall_normals              : PackedVector3Array
	var floor_collision_indexes   : PackedInt32Array
	var wall_collision_indexes    : PackedInt32Array
	var ceiling_collision_indexes : PackedInt32Array
	var motion_result             : PhysicsTestMotionResult3D


## Stores info about a collision with a rigid body
class CollisionImpulseData extends RefCounted :
	var bodies_separating             : bool
	var relative_speed_towards_normal : float
	var collision_normal              : Vector3
	var impulse                       : Vector3
	var impulse_position              : Vector3


## Stores a collider and collision data that belongs to it
# INFO : Collision type and normal belongs to the latest collision with the collider.
# WARNING : This might not work well with concave shapes.
class ColliderData extends RefCounted :
	var collider_id       : int
	var collision_type    : CollisionType
	var collision_normal  : Vector3
	var collision_points  : PackedVector3Array
	
	enum CollisionType {FLOOR, WALL, CEILING}
	
	func _init(p_id : int, p_normal : Vector3, p_point : Vector3, p_collision_type : CollisionType) -> void :
		collider_id = p_id
		collision_type = p_collision_type
		collision_normal = p_normal
		collision_points.append(p_point)
	
	static func find_by_id(p_array : Array[ColliderData], p_id : int) -> int :
		var index : int = 0
		for collider_data in p_array :
			if collider_data.collider_id == p_id :
				return index
			index += 1
		return -1
	
	static func append_or_update(p_collider_data_array : Array[ColliderData] , p_col_state : CollisionState, p_index : int, p_collision_type : CollisionType) -> int :
		var _collision_point   : Vector3 = p_col_state.motion_result.get_collision_point(p_index)
		var _collision_normal  : Vector3 = p_col_state.motion_result.get_collision_normal(p_index)
		var _collider_id : int = p_col_state.motion_result.get_collider_id(p_index)
		var index : int = find_by_id(p_collider_data_array, _collider_id)
		if index == -1 :
			p_collider_data_array.append(ColliderData.new(_collider_id, _collision_normal, _collision_point, p_collision_type))
			index = p_collider_data_array.size() - 1
			return index
		else :
			p_collider_data_array[index].collision_points.append(_collision_point)
			p_collider_data_array[index].collision_type = p_collision_type
			p_collider_data_array[index].collision_normal = _collision_normal
			return index


## Class for storing common math operations
class Math :
	static func calculate_vector3_array_avarage(vector_array : PackedVector3Array) -> Vector3 :
		var avarage_vector : Vector3
		var vector_array_size : int = vector_array.size()
		
		if vector_array_size > 1 :
			var summed_vector := Vector3.ZERO
			for vector in vector_array :
				summed_vector += vector
				
			avarage_vector = summed_vector / vector_array_size
		else :
			avarage_vector = vector_array[0]
		
		return avarage_vector


## Simplifies motion testing
class MotionTester extends RefCounted :
	var hit    : bool = false
	var endpos : Vector3
	var res    := PhysicsTestMotionResult3D.new()
	
	func test_motion(body : RID, from : Transform3D, motion : Vector3, margin : float = 0.001, rec : bool = false, max_cols : int = 1) -> void:
		var params := PhysicsTestMotionParameters3D.new()
		
		params.from = from
		params.motion = motion
		params.margin = margin
		params.exclude_bodies = [body]
		params.recovery_as_collision = rec
		params.max_collisions = max_cols
		
		hit = false
		res = PhysicsTestMotionResult3D.new()
		
		hit = PhysicsServer3D.body_test_motion(body, params, res)
		
		endpos = from.origin + res.get_travel()


## Simplifies raycasting
class RayCast extends RefCounted :
	var hit    : bool
	var normal : Vector3
	
	func intersect(origin : Vector3 , dest : Vector3, exclude : RID, mask : int, space_state : PhysicsDirectSpaceState3D) -> void:
		var ray_query := PhysicsRayQueryParameters3D.new()
		
		ray_query.from = origin
		ray_query.to = dest
		ray_query.collision_mask = mask
		ray_query.exclude = [exclude]
		
		var results : Dictionary = space_state.intersect_ray(ray_query)
		
		hit = false
		
		if results.is_empty():
			return
		
		hit = true
		normal = results["normal"]
