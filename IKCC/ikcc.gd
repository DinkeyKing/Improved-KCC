
## Implements a kinematic/virtual character body.
class_name IKCC
extends PhysicsBody3D



#==================================================================================================
# -----------------------
# | CONSTANTS AND ENUMS |
# -----------------------


## The maximum slide iterations allowed.
const MAX_SLIDES                : int   = 5             # Default: 5
## The maximum collision reports per slide iteration.
const MAX_SLIDE_COLLISIONS      : int   = 6             # Default: 6
## The maximum collision reports per floor snaps.
const MAX_SNAP_COLLISIONS       : int   = 4             # Default: 4
## The maximum collision reports for motionless collision checks.
const MAX_REST_COLLISIONS       : int   = 4             # Default: 4
## The maximum number of constraint iterations
const MAX_CONSTRAINT_ITERATIONS : int   = 7             # Default: 7 (Jolt's CharacterVirtual's is 15)
## General epsilon used to compare floats.
const CMP_EPSILON               : float = 0.00001       # Default: 0.00001
## Epsilon used when checking if given planes are equal to each other.
const PLANES_CMP_EPSILON        : float = 0.00001       # Default: 0.00001
## Epsilon used when classifying collisions by the angle of their normals.
const ANGLE_CMP_EPSILON         : float = 0.01          # Default: 0.01
## Offset length used when offsetting raycasts.
const RAYCAST_OFFSET_LENGTH     : float = 0.001         # Default: 0.001
## The length of a recovery can approximately be no more than safe margin times this number.
const RECOVERY_FACTOR           : float = 2.0           # Default: 2.0
## The factor used to increase the safe margin for the final touch collision check.
const TOUCH_SAFE_MARGIN_FACTOR  : float = 2.0           # Default: 2.0


## Different modes for sliding collision response.
enum MotionMode {
	## Suitable for moving along the ground.
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
## Maximum angle where a slope is still considered a floor (or a ceiling), rather than a wall.
@export_range(0.0, 180.0, 0.1, "degrees")
var floor_max_angle_degrees : float = 45.0 :
	set (value) : 
		floor_max_angle = value * (PI/180.0)  # Set radian variable
		floor_max_angle_degrees = value  # Set degree variable
## If [code]true[/code], removes the sliding caused by the downward component of [member velocity] 
## when landing from air onto a sloped floor. Additionally, [member velocity] will keep it's horizontal
## direction when solving a floor constraint. (The horizontal plane is defined by
## [member up_direction].) [br]
## It's [b]important[/b] to note that if you want gravity and other downwards and towards the floor
## accelerations to drag the body down on floor slopes, you should set this to [code]false[/code], 
## (because keeping horizontal direction is undesired in that scenario)!
@export var floor_stop_on_slope : bool = true
## If [code]true[/code], [member velocity] will keep it's original magnitude when solving a floor
## constraint.
@export var floor_constant_speed : bool = false
## Sets a maximum snapping distance. When set to a value different from [code]0.0[/code], the body is
## kept attached to slopes when calling [method move_and_slide]. The snapping vector is determined by
## the given distance along the opposite direction of the [member up_direction]. If the body snaps
## to the floor, [member velocity] will be modified so that it parallels the floor plane.
@export_range(0.0, 1.0, 0.01, "suffix:m") var max_snap_length : float = 0.26 :
	set(value) : max_snap_length = absf(value)
## If [member velocity] has a magnitude of at least [floor_detach_threshold] in the opposite
## direction of a floor, the body will not snap to it, otherwise it won't.
@export var floor_detach_threshold : float = 2.0

@export_group("Wall")
## If [code]true[/code], the body will not slide up wall slopes when colliding with them on floor.
@export var floor_block_on_wall : bool = true
## If the angle of a wall collision is smaller than this angle, the body will stop
## instead of sliding along it.
@export_range(0.0, 180.0, 0.1, "degrees") 
var wall_min_slide_angle_degrees : float = 15.0 :
	set (value) : 
		wall_min_slide_angle = value * (PI/180.0)  # Set radian variable
		wall_min_slide_angle_degrees = value  # Set degree variable
## If [code]true[/code], [member velocity] will keep it's original magnitude when solving a wall
## constraint. 
@export var wall_constant_speed : bool = false

@export_group("Ceiling")
## If [code]false[/code], the body will not slide up ceilings.
@export var slide_up_ceilings : bool = true
## If [code]true[/code], [member velocity] will keep it's original magnitude when solving a ceiling
## constraint. 
@export var ceiling_constant_speed : bool = false

@export_group("Moving Platforms")
## If [code]true[/code], the applied motion from [member platform_velocity] will also follow the sliding behaviour,
## instead of stopping on collisions.
@export var slide_platform_motion : bool = true
## Sets the behavior to apply when you leave a moving platform. By default, to be physically accurate,
## when you leave the last platform velocity is applied. See [enum PlatformLeaveAction] constants for available behavior.
@export var platform_leave_action := PlatformLeaveAction.ADD_VELOCITY
## If the vertical platform velocity change is larger than this threshold, the body will detach
## from the platform. Note that this only works if the leave velocity large enough to bypass
## the floor snap. (See [member floor_detach_threshold].)
@export var platform_detach_treshold : float = 1.0
## Collision layers that will be included for detecting floor bodies that will act as moving platforms
## to be followed by the body.
@export_flags_3d_physics var moving_platform_layers : int = 1

@export_group("Stepping")
## Sets the maximum height of the walls the character can step on.
@export_range(0.0, 1.0, 0.01, "suffix:m") var max_step_height : float = 0.26 :
	set(value) : max_step_height = absf(value)
## Sets the mimimum distance the character will move towards a wall when attempting to step on it.
@export_range(0.0, 0.01, 0.001) var min_step_forward_distance : float = 0.001
## If stepping will not result in the body landing on a floor, an additional test will be made to
## see if there is floor further along the step direction, the distance to travel forward in this
## test is [member step_floor_check_distance]. If a floor is found, the step move will be succesful. 
@export_range(0.0, 0.5, 0.01) var step_floor_check_distance : float = 0.3

@export_group("Rigid Body Interactions")
## If [code]false[/code], rigid bodies collisions will be treated as collisions with static bodies. [br]
## If [code]true[/code], contact impulses and weight force will be simulated when colliding with
## rigid bodies. In this case, it's [b]important[/b] that the collision mask of the rigid bodies that
## will be interacted with has the character's collision layer excluded!
@export var interact_with_rigid_bodies : bool = true
## The mass of the character body.
@export_range(0.001, 1000.0, 0.001, "suffix:kg") var mass : float = 1.0 :
	set(value) : 
		mass = maxf(CMP_EPSILON, value)
		inverse_mass = 1.0 / mass

@export_group("Collision")
## Extra margin used for recovery and during slide collisons. [br] [br]
## If the body is at least this close to another body, it will consider them to be colliding 
## and will be pushed away before performing the actual motion. [br]
## A higher value means it's more flexible for detecting collision, which helps with consistently
## detecting walls and floors. [br]
## A lower value forces the collision algorithm to use more exact detection, so it can be used in
## cases that specifically require precision, e.g at very low scale to avoid visible jittering,
## or for stability with a stack of character bodies.
@export_range(0.001, 0.010, 0.001, "suffix:m") var safe_margin : float = 0.001 :
	set(value) : safe_margin = absf(value)




#==================================================================================================
# --------------
# | VARIABLES  |
# --------------


## [code]true[/code] if the body is touching a floor. Set during [method move_and_slide]. [br]
var is_on_floor                     : bool
## [code]true[/code] if the body is touching a wall. Set during [method move_and_slide].
var is_on_wall                      : bool
## [code]true[/code] if the body is touching a ceiling. Set during [method move_and_slide].
var is_on_ceiling                   : bool
## [code]true[/code] if the body collided with a floor. Set during [method move_and_slide].
var collided_with_floor             : bool
## [code]true[/code] if the body collided with a wall. Set during [method move_and_slide].
var collided_with_wall              : bool
## [code]true[/code] if the body collided with a ceiling. Set during [method move_and_slide].
var collided_with_ceiling           : bool
## [code]true[/code] if the body is touching a floor surface. Set during [method move_and_slide].
var is_on_floor_surface             : bool
## [code]true[/code] if the body performed a step move. Set during [method move_and_slide].
var has_stepped                     : bool
## [code]true[/code] if the body is touched a floor in the previous call to [method move_and_slide].
## Set during [method move_and_slide].
var prev_on_floor                   : bool
## [code]true[/code] if the body is touched a floor surface in the previous call to [method move_and_slide].
## Set during [method move_and_slide].
var prev_on_floor_surface           : bool
## [code]true[/code] if platform velocity was valid in the previous call to [method move_and_slide].
var prev_platform_velocity_valid    : bool
## The linear velocity of the body.
var velocity                        : Vector3
## The position delta divided by delta time.
## Set during [method move_and_slide].
var real_velocity                   : Vector3
## The travel vector from the last call to [method move_and_slide].
var delta_position                  : Vector3
## The normal of the deepest active floor collision. If the body is not touching a floor, it's a zero vector.
## Set during [method move_and_slide].
var current_floor_normal            : Vector3
## The normal of the deepest active wall collision. If the body is not touching a wall, it's a zero vector.
## Set during [method move_and_slide].
var current_wall_normal             : Vector3
## The normal of the deepest active ceiling collision. If the body is not touching a ceiling, it's a zero vector.
## Set during [method move_and_slide].
var current_ceiling_normal          : Vector3
## The normal of the latest floor collision. If the body did not collide with a floor, it's a zero vector.
## Set during [method move_and_slide].
var last_floor_normal               : Vector3
## The normal of the latest wall collision. If the body did not collide with a wall, it's a zero vector.
## Set during [method move_and_slide].
var last_wall_normal                : Vector3
## The normal of the latest ceiling collision. If the body did not collide with a ceiling, it's a zero vector.
## Set during [method move_and_slide].
var last_ceiling_normal             : Vector3
## The velocity of the floor collider at the position of the character body. If the body is not on
## a floor, it's a zero vector.
## Set during [method move_and_slide].
var platform_velocity               : Vector3
## The velocity the body hit the ground with arriving from air. If that didn't happen, it's a zero vector.
## Set during [method move_and_slide].
var floor_impact_velocity           : Vector3
## The global position of the body before the latest call to [method move_and_slide].
## Set during [method move_and_slide].
var previous_position               : Vector3
## Stores the id of the current floor collider the body is touching.
## Set during [method move_and_slide].
var current_floor_collider_encoded  : EncodedObjectAsID
## The RID of the current floor collider the body is touching.
## Set during [method move_and_slide].
var current_floor_collider_rid      : RID
## An array of data objects that holds collision data sorted by colliders.
## Set during [method move_and_slide].
var collider_datas                  : Array[ColliderData]
## An array of data objects that stores information about the collisions that happened
## during the latest call to [method move_and_slide].
## Does not include collision data of the final touch collision check and floor snaps.
var collision_states                : Array[CollisionState]
## ## The collision data of the latest floor snap.
## Set during [method move_and_slide] and [method snap_to_floor].
var snap_collision_state            : CollisionState
## The collision data of the last collision check at the final position of the body.
## Set during [method move_and_slide].
var touch_collision_state           : CollisionState

# Translate angle properties to radian when initialising
## The minimum wall slide angle in radians.
@onready var wall_min_slide_angle : float = wall_min_slide_angle_degrees * (PI/180.0)
## The maximum wall angle in radians.
@onready var floor_max_angle      : float = floor_max_angle_degrees * (PI/180.0)

# Precalculate inverse mass
## The inverse mass of the body.
@onready var inverse_mass : float = 1.0 / mass




#==================================================================================================
# ------------------
# | PUBLIC METHODS |
# ------------------



## Call this in '_physics_process' to simulate body movement.
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
	prev_on_floor_surface = is_on_floor_surface
	
	# Reset state variables
	collided_with_floor = false
	collided_with_wall = false
	collided_with_ceiling = false
	is_on_floor = false
	is_on_wall = false
	is_on_ceiling = false
	is_on_floor_surface = false
	has_stepped = false
	#last_floor_normal = Vector3.ZERO <-- We will read this later in grounded move, so don't clear it yet
	last_wall_normal = Vector3.ZERO
	last_ceiling_normal = Vector3.ZERO
	current_floor_normal = Vector3.ZERO
	current_wall_normal = Vector3.ZERO
	current_ceiling_normal = Vector3.ZERO
	floor_impact_velocity = Vector3.ZERO
	collision_states.clear()
	collider_datas.clear()
	snap_collision_state = null
	touch_collision_state = null
	
	# Handle contacts before moving, and get their constraints
	var pre_move_constraints : Array[KinematicConstraint] = handle_pre_move_contacts()
	
	# Move with requested motion mode
	if motion_mode == MotionMode.GROUNDED :
		grounded_move(delta_t, pre_move_constraints)
	else :
		last_floor_normal = Vector3.ZERO
		floating_move(delta_t, pre_move_constraints)
	
	# Add platform velocity if left floor
	if platform_leave_action != PlatformLeaveAction.DO_NOTHING and not platform_velocity.is_zero_approx() and not current_floor_collider_encoded :
		if platform_leave_action == PlatformLeaveAction.ADD_VELOCITY_NO_DOWNWARDS :
			platform_velocity -= minf(0.0, platform_velocity.dot(up_direction)) * up_direction
		
		velocity += platform_velocity
		platform_velocity = Vector3.ZERO
	
	# Compute delta position and de facto velocity
	delta_position = global_position - previous_position
	real_velocity = delta_position / delta_t
	
	# Handle contacts at final position
	handle_contacts_at_final_position()
	
	if collision_states.is_empty() and not touch_collision_state and not snap_collision_state :
		return false
	
	return true




## Snaps the character to the floor, if there is floor below within 'p_snap_length' meters away.
func snap_to_floor(p_snap_length : float) -> bool :
	# Snap by at least safe margin to keep floor state consistent
	var snap_length : float = maxf(safe_margin, p_snap_length)
	
	var collided      : bool
	var snap_motion   := -up_direction * snap_length
	var motion_result := PhysicsTestMotionResult3D.new()
	var motion_params := PhysicsTestMotionParameters3D.new()
	motion_params.from = global_transform
	motion_params.motion = snap_motion
	motion_params.margin = safe_margin
	motion_params.recovery_as_collision = true
	motion_params.max_collisions = MAX_SNAP_COLLISIONS
	
	# NOTE: Test only, because we only move the body if a floor collision happened
	collided = _move_and_collide(motion_result, motion_params, true)
	
	if not collided :
		return false
	
	var travel : Vector3 = motion_result.get_meta("travel", motion_result.get_travel()) as Vector3
	var result_state := CollisionState.new()
	set_collision_state(motion_result, result_state)
	
	if not (result_state.s_floor or result_state.s_wall_floor) :
		# Check surface normal, if it's floor, allow another snap attempt on next frame
		# Fixes case when walking down step and reported collision normal is wall
		# NOTE : This triggers the snap method more often, but makes snapping more reliable.
		var snap_position : Vector3 = global_position + travel
		if collided_with_floor_surface(result_state, snap_position) :
			is_on_floor_surface = true
		return false
	
	# Determine constraints for velocity, in order to make it parallel to floors,
	# as to not move away from them
	var kinematic_constraints : Array[KinematicConstraint]
	for i : int in result_state.floor_collision_indexes :
		var floor_normal : Vector3 = motion_result.get_collision_normal(i)
		
		# Filter out floors we are detaching from
		var d : float = velocity.dot(floor_normal)
		if d > floor_detach_threshold :
			continue
		
		# Constant speed is applied, so only direction will be adjusted
		# NOTE: This will redirect vertical velocity towards floor direction!
		append_kinematic_constraint_if_unique_approx(
			kinematic_constraints,
			KinematicConstraint.new(-floor_normal, Vector3.ZERO, 0.0, true, true)
		)
	# Handle wall floors
	if result_state.s_wall_floor :
		var d : float = velocity.dot(result_state.wall_floor_normal)
		if d < floor_detach_threshold :
			append_kinematic_constraint_if_unique_approx(
			kinematic_constraints,
			KinematicConstraint.new(-result_state.wall_floor_normal, Vector3.ZERO, 0.0, true, true)
		)
	
	var is_travel_significant : bool = travel.length_squared() > safe_margin**2
	var detaching_from_floor : bool = kinematic_constraints.size() == 0
	
	# Update overall state
	# NOTE: If floor is within safe margin, floor state is applied even if
	# velocity is not facing the floor. We need this since floor snapping acts
	# as the floor detector.
	if not detaching_from_floor or (detaching_from_floor and not is_travel_significant) :
		update_overall_state(result_state, true, false, false, true)
		snap_collision_state = result_state
	
	# If we are detaching from all floors, don't snap to floor
	if detaching_from_floor :
		return false
	
	# Only move the body if we are not close enough already.
	# INFO: This helps avoid sliding caused by recovery.
	if is_travel_significant :
		global_position += travel
	
	velocity = solve_kinematic_constraints(kinematic_constraints, velocity, up_direction)
	
	return true




## Applies an impulse to the character body.
func apply_impulse(p_impulse : Vector3) -> void :
	velocity += p_impulse * inverse_mass




#==================================================================================================
# -------------------
# | PRIVATE METHODS |
# -------------------



## Called before moving the body. Handles contacts at the starting position,
## returns the constraints from these contacts.
func handle_pre_move_contacts() -> Array[KinematicConstraint] :
	# TODO: A better method is needed for checking shape intersections. Replace
	# this collision check for that one, once available.
	# Get current collisions before moving
	# NOTE: Currently only the 'body_test_motion' (called inside _move_and_collide)
	# is suitable for collision checks, since it's the only way to retrieve the necessary data of all contacts.
	var pre_move_motion_result := PhysicsTestMotionResult3D.new()
	var motion_params := PhysicsTestMotionParameters3D.new()
	motion_params.from = global_transform
	motion_params.motion = Vector3.ZERO
	motion_params.margin = safe_margin
	motion_params.recovery_as_collision = true
	motion_params.max_collisions = MAX_REST_COLLISIONS
	# Disable slide cancelling, because we are not moving the body
	_move_and_collide(pre_move_motion_result, motion_params, true, false)
	
	# Determine collision state with collisions from character motion ignored,
	# only collisions resulting from motion of other bodies are registered.
	var incoming_collision_state := CollisionState.new()
	set_collision_state(pre_move_motion_result, incoming_collision_state, true)
	collision_states.push_back(incoming_collision_state)
	
	# Update overall state with incoming collisions
	if motion_mode == MotionMode.FLOATING :
		update_overall_state(incoming_collision_state, false, true, false)
	else :
		update_overall_state(incoming_collision_state, true, true, true)
	
	# Determine constraints from this state
	var kinematic_constraints : Array[KinematicConstraint]
	var dynamic_constraints   : Array[DynamicConstraint]
	if motion_mode == MotionMode.FLOATING :
		determine_floating_move_constraints(
			incoming_collision_state, kinematic_constraints, dynamic_constraints
		)
	else :
		determine_grounded_move_constraints(
			incoming_collision_state, kinematic_constraints, dynamic_constraints
		)
	
	# Solve velocity for these constraints
	velocity = solve_dynamic_constraints(dynamic_constraints, velocity, inverse_mass)
	velocity = solve_kinematic_constraints(kinematic_constraints, velocity, up_direction)
	
	# Determine collision state with all current contacts before moving
	var pre_move_collision_state := CollisionState.new()
	set_collision_state(pre_move_motion_result, pre_move_collision_state, false)
	# NOTE: The overall state is not updated with collisions that are not the
	# result of motion from either the character, or other bodies. We register
	# touching collision only at the final position.
	
	# Determine constraints from this state
	# NOTE: The incoming collisions are a subset of the current contacts, so
	# we can safely clear the array of constraints.
	kinematic_constraints.clear()
	
	if motion_mode == MotionMode.FLOATING :
		determine_floating_move_constraints(
			pre_move_collision_state, kinematic_constraints, dynamic_constraints
		)
	else :
		determine_grounded_move_constraints(
			pre_move_collision_state, kinematic_constraints, dynamic_constraints
		)
	
	return kinematic_constraints




## Called when done moving, handles contacts at the final position.
func handle_contacts_at_final_position() -> void :
	# TODO: A better method is needed for checking shape intersections. Replace
	# this collision check for that one, once available.
	# Check for all current contacts at the final position
	var touch_motion_result := PhysicsTestMotionResult3D.new()
	var motion_params := PhysicsTestMotionParameters3D.new()
	# NOTE: When getting touch collisions we need an inflated safe margin, because the character is
	# pushed away from collisions, so it will no longer collide if we use the unaltered safe margin.
	var touch_safe_margin : float = safe_margin * TOUCH_SAFE_MARGIN_FACTOR 
	motion_params.from = global_transform
	motion_params.motion = Vector3.ZERO
	motion_params.margin = touch_safe_margin
	motion_params.recovery_as_collision = true
	motion_params.max_collisions = MAX_REST_COLLISIONS
	# Disable slide cancelling, because we are not moving the body
	_move_and_collide(touch_motion_result, motion_params, true, false)
	
	# Determine collision state with these contacts
	touch_collision_state = CollisionState.new()
	set_collision_state(touch_motion_result, touch_collision_state)
	
	# Update overall state and also set current state properties
	if motion_mode == MotionMode.FLOATING :
		update_overall_state(touch_collision_state, false, true, false, true)
	else :  # Grounded move
		# We don't want to set the floor status here, because floor snapping
		# already determined the floor status
		# FIXME: This does mean certain wall floors don't get detected...
		update_overall_state(touch_collision_state, false, true, true, true)




## Performs a grounded movement.
func grounded_move(p_delta_t : float, p_pre_move_constraints : Array[KinematicConstraint]) -> void :
	# Determine platform velocity
	var prev_platform_velocity : Vector3 = platform_velocity
	var floor_collider_valid   : bool = false
	platform_velocity = Vector3.ZERO
	if current_floor_collider_rid.is_valid() :
		var platform_layer : int = PhysicsServer3D.body_get_collision_layer(current_floor_collider_rid)
		var excluded : bool = (moving_platform_layers & platform_layer) == 0
		if not excluded :
			var body_state : PhysicsDirectBodyState3D = PhysicsServer3D.body_get_direct_state(current_floor_collider_rid)
			var local_position : Vector3 = global_position - body_state.transform.origin
			
			platform_velocity = body_state.get_velocity_at_local_position(local_position)
			
			if prev_platform_velocity_valid :
				var platform_vertical_velocity_change_length : float = (prev_platform_velocity - platform_velocity).dot(last_floor_normal)
				if platform_vertical_velocity_change_length > platform_detach_treshold and prev_platform_velocity.dot(last_floor_normal) > floor_detach_threshold :
					velocity += prev_platform_velocity
					platform_velocity = Vector3.ZERO
			
			floor_collider_valid = true
	
	prev_platform_velocity_valid = floor_collider_valid
	
	# Now we can clear last_floor_normal
	last_floor_normal = Vector3.ZERO
	
	# Move with platform velocity
	if not platform_velocity.is_zero_approx() :
		if slide_platform_motion :
			# Perform a sliding move with platform velocity
			var platform_move_io := MoveShapeIO.new(global_transform, platform_velocity, p_delta_t)
			move_shape(platform_move_io, prev_on_floor, p_pre_move_constraints,[current_floor_collider_rid])
			
			# Update position
			global_transform = platform_move_io.transform
			
			# Update overall state with collisions
			for collision_state : CollisionState in platform_move_io.collision_states :
				collision_states.push_back(collision_state)
				update_overall_state(collision_state, true, true, true)
		else :
			# Move with platform velocity until collision
			var motion_result : = PhysicsTestMotionResult3D.new()
			var motion_params : = PhysicsTestMotionParameters3D.new()
			motion_params.from = global_transform
			motion_params.motion = platform_velocity * p_delta_t
			motion_params.margin = safe_margin
			motion_params.recovery_as_collision = true
			motion_params.max_collisions = MAX_SLIDE_COLLISIONS
			motion_params.exclude_bodies = [current_floor_collider_rid]
			
			var collided : bool = _move_and_collide(motion_result, motion_params)
			if collided :
				var result_state := CollisionState.new()
				set_collision_state(motion_result, result_state)
				update_overall_state(result_state, true, true, true)
				collision_states.push_back(result_state)
		
		# Clear pre move constraints, since we have moved
		p_pre_move_constraints.clear()
	
	# Clear floor collider data
	current_floor_collider_encoded = null
	current_floor_collider_rid = RID()
	
	# Move with velocity
	var move_io := MoveShapeIO.new(global_transform, velocity, p_delta_t)
	move_shape(move_io, prev_on_floor, p_pre_move_constraints)
	
	# Update position and velocity
	global_transform = move_io.transform
	velocity = move_io.velocity
	
	# Update overall state with collisions
	for collision_state : CollisionState in move_io.collision_states :
		collision_states.push_back(collision_state)
		update_overall_state(collision_state, true, true, true)
	
	# Set step status flag and floor impact velocity
	has_stepped = move_io.has_stepped
	floor_impact_velocity = move_io.floor_impact_velocity
	
	# Snap to floor if character was on the floor in the previous frame, or has
	# touched floor during movement.
	if prev_on_floor or move_io.touched_floor or prev_on_floor_surface :
		snap_to_floor(max_snap_length)
	
	# Apply gravity force to floor collider
	if interact_with_rigid_bodies :
		apply_gravity_force_to_floor(p_delta_t)




## Applies gravity force to the floor collider.
func apply_gravity_force_to_floor(p_delta_t : float) -> void :
	if not (current_floor_collider_encoded and is_body_dynamic(current_floor_collider_rid)) :
		return
	
	# Create dynamic constraints
	var index : int = ColliderData.find_by_collider_id(collider_datas, current_floor_collider_encoded.object_id)
	var floor_collider_data : ColliderData = collider_datas[index]
	var dynamic_constraints : Array[DynamicConstraint]
	for i : int in floor_collider_data.collision_count :
		dynamic_constraints.push_back(
			DynamicConstraint.new(
				floor_collider_data.collider_rid,
				floor_collider_data.collision_normals[i],
				floor_collider_data.collision_points[i]
			)
		)
	
	# Calculate velocity from gravity acceleraation
	var gravity_velocity : Vector3 = get_gravity() * p_delta_t
	# We have to take platform velocity into account
	
	gravity_velocity += platform_velocity * (
		# If the floor platform is "pulling", reverse the added velocity
		-1.0 if platform_velocity.dot(up_direction) < 0.0 else 1.0
		)
	
	# Apply the impulses
	solve_dynamic_constraints(dynamic_constraints, gravity_velocity, inverse_mass)




## Performs a floating movement.
func floating_move(p_delta_t : float, p_pre_move_constraints : Array[KinematicConstraint]) -> void :
	
	# Clear floor collider data
	current_floor_collider_encoded = null
	current_floor_collider_rid = RID()
	
	# Move with velocity
	var move_io := MoveShapeIO.new(global_transform, velocity, p_delta_t)
	move_shape(move_io, prev_on_floor, p_pre_move_constraints)
	
	# Update position and velocity
	global_transform = move_io.transform
	velocity = move_io.velocity
	
	# Update overall state with collisions
	for collision_state : CollisionState in move_io.collision_states :
		collision_states.push_back(collision_state)
		update_overall_state(collision_state, false, true, false)




## Simulates body movement using collision checks.
func move_shape(
	p_io                         : MoveShapeIO,
	p_prev_on_floor              : bool,
	p_prev_kinematic_constraints : Array[KinematicConstraint] = [],
	p_excluded_bodies            : Array[RID] = [],
	p_step_move_enabled          : bool = true
	) -> void :
	# Check if done maximum allowed amount of iterations
	if p_io.iteration_count > MAX_SLIDES :
		# NOTE : this shouldn't happen
		printerr("IKCC: Max slides surpassed! (%d)" % MAX_SLIDES)
		return
	
	# If simulation time provided is zero (or less), treat it as an error and bail
	if is_zero_approx(p_io.time_remaining) or p_io.time_remaining <= 0.0 :
		printerr("IKCC: Simulation time provided to 'move_shape' is zero!")
		return
	
	# Increase iteration counter
	p_io.iteration_count += 1
	
	# Perform recovery, move body along motion, stop if collided
	var motion        : Vector3 = p_io.velocity * p_io.time_remaining
	var motion_result :         = PhysicsTestMotionResult3D.new()
	var motion_params :         = PhysicsTestMotionParameters3D.new()
	motion_params.from = p_io.transform
	motion_params.motion = motion
	motion_params.margin = safe_margin
	motion_params.recovery_as_collision = true
	motion_params.max_collisions = MAX_SLIDE_COLLISIONS
	motion_params.exclude_bodies = p_excluded_bodies
	var collided : bool = _move_and_collide(motion_result, motion_params, true)
	
	# Modify output position by adding the safe fraction of the motion
	var travel : Vector3 = motion_result.get_meta("travel", motion_result.get_travel()) as Vector3
	p_io.transform.origin += travel
	
	# Reduce remaining time by time spent moving
	p_io.time_remaining -= p_io.time_remaining * motion_result.get_collision_safe_fraction()
	
	if not collided :
		return  # Moved all the way, no collision to respond to
	
	# Calculate collision info
	var result_state := CollisionState.new()
	set_collision_state(motion_result, result_state)
	
	# Store collision info
	p_io.collision_states.push_back(result_state)
	
	if motion.is_zero_approx() :
		return  # Motion was zero, no need to slide
	
	# Determine constraints
	var kinematic_constraints : Array[KinematicConstraint] = []
	var dynamic_constraints   : Array[DynamicConstraint]   = []
	
	# Only add previous constraints if we haven't moved a significant amount
	# NOTE: Recovery can be more than the safe margin, we have to take it into account.
	var is_travel_significant : bool = travel.length_squared() > (safe_margin * RECOVERY_FACTOR)**2
	if not is_travel_significant :
		kinematic_constraints += p_prev_kinematic_constraints
	
	# Remember floor stauts of this iteration (Only used for grounded movement.)
	# If collision includes floor, we can set this to true and skip the floor collision check.
	var touching_floor : bool = result_state.s_floor or result_state.s_wall_floor
	
	if motion_mode == MotionMode.FLOATING :
		determine_floating_move_constraints(result_state, kinematic_constraints, dynamic_constraints)
	else :  # Grounded motion mode
		var floor_below : bool = result_state.s_floor
		var floor_surface_below : bool = false
		var floor_check_travel := Vector3.ZERO
		
		# If no floor collision is reported, we need an extra collision check at our current position.
		if not touching_floor :
			var floor_state := CollisionState.new()
			if check_floor_status(p_io.transform, max_step_height, floor_state, true) :
				floor_below = floor_state.s_floor or floor_state.s_wall_floor
				floor_surface_below = floor_state.s_floor_surface_only
				floor_check_travel = floor_state.motion_result.get_meta("travel", floor_state.motion_result.get_travel()) as Vector3
				# NOTE: Recovery can be more than the safe margin, we have to take it into account.
				touching_floor = floor_below and floor_check_travel.length_squared() <= (safe_margin * RECOVERY_FACTOR)**2
		
		# Set flag if we are touching the floor
		p_io.touched_floor = p_io.touched_floor or touching_floor
		
		# Check if we collided with a step
		if p_step_move_enabled and not is_zero_approx(max_step_height) and (floor_below or floor_surface_below) and result_state.s_wall :
			# Retrieve IO objects of forward move, in case step move fails
			var step_forward_ios : Array[MoveShapeIO]
			if step_move(p_io, result_state, p_excluded_bodies, step_forward_ios) :
				# The character succesfully stepped with remaning time, so we are out of it.
				return
			# If the step move failed, cancel any impulse applied to bodies during the forward move
			for move_shape_io : MoveShapeIO in step_forward_ios :
				move_shape_io.restore_body_states()
		
		# Determine if we want to block horizontal movement on steep slopes
		var slide_up_steep_slopes : bool = not (floor_block_on_wall and touching_floor and result_state.s_wall)
		
		# If floor check distance is not too much, we apply it to the body's position,
		# in order to make floor status more reliable when sliding along steep slopes
		if not slide_up_steep_slopes and touching_floor :
			p_io.transform.origin += floor_check_travel
		
		determine_grounded_move_constraints(
			result_state, kinematic_constraints, dynamic_constraints, slide_up_steep_slopes, slide_up_ceilings
		)
	
	# The user may wish to know the velocity the character hit the ground with, so save it
	var landed_from_air : bool = not p_prev_on_floor and result_state.s_floor
	if landed_from_air and p_io.floor_impact_velocity.is_zero_approx() :
		p_io.floor_impact_velocity = p_io.velocity
	
	# Determine new velocity
	var new_velocity : Vector3 = p_io.velocity
	
	# This feature prevents sliding caused by accumulated velocity from gravity
	# when landing on sloped floors.
	# NOTE: Disabling gravity when on floor is the user's responsibility!
	# When the character lands on the floor from air...
	if floor_stop_on_slope and landed_from_air :
		# and vertical component would cause sliding down the slope...
		if p_io.velocity.dot(up_direction) < 0.0 :
			# First remove downward vertical (gravity) component...
			new_velocity = p_io.velocity.slide(up_direction)
			
			# ... then adjust current velocity so that it parallels the floor planes, by
			# solving the constraints defined below.
			var floor_dir_constraints : Array[KinematicConstraint]
			for i : int in result_state.floor_collision_indexes :
				var floor_normal : Vector3 = motion_result.get_collision_normal(i)
				
				# Constant speed is applied, so only direction will be adjusted.
				append_kinematic_constraint_if_unique_approx(
					floor_dir_constraints,
					KinematicConstraint.new(-floor_normal, Vector3.ZERO, 0.0, true, true)
				)
			
			new_velocity = solve_kinematic_constraints(floor_dir_constraints, new_velocity, up_direction)
	
	# Solve constraints
	new_velocity = solve_dynamic_constraints(dynamic_constraints, new_velocity, inverse_mass, p_io)
	new_velocity = solve_kinematic_constraints(kinematic_constraints, new_velocity, up_direction)
	
	# Set new output velocity
	p_io.velocity = new_velocity
	
	# Cancel motion caused by recovery, when needed
	if not is_travel_significant :
		var cancelled_travel := Vector3.ZERO
		for constraint : KinematicConstraint in kinematic_constraints :
			if constraint.is_slide_cancelled :
				var travel_horizontal_to_plane : Vector3 = travel.slide(constraint.plane_normal)
				cancelled_travel += travel_horizontal_to_plane
				travel -= travel_horizontal_to_plane
				constraint.is_slide_cancelled = false  # reset flag
		# Modify output position
		p_io.transform.origin -= cancelled_travel
	
	if new_velocity.is_zero_approx() :
		return # There's not enough velocity left
	
	# Stop simulating if new velocity still faces a constraint plane
	for constraint : Constraint in (kinematic_constraints + dynamic_constraints) as Array[Constraint] :
		if new_velocity.dot(constraint.plane_normal) <= -CMP_EPSILON :
			return
	
	if is_zero_approx(p_io.time_remaining) :
		return  # Not enough time left to be simulated, bail
	
	# Iterate
	move_shape(
		p_io,
		touching_floor,
		kinematic_constraints,
		p_excluded_bodies,
		p_step_move_enabled
	)




## Determines constraints for floating movement according to input collision state.
func determine_floating_move_constraints(
	p_collision_state       : CollisionState,
	p_kinematic_constraints : Array[KinematicConstraint],
	p_dynamic_constraints   : Array[DynamicConstraint]
	) -> void :
	# NOTE: All collisions are walls if motion mode is floating.
	for i : int in p_collision_state.wall_collision_indexes :
		var wall_normal   : Vector3 = p_collision_state.motion_result.get_collision_normal(i)
		var wall_velocity : Vector3 = p_collision_state.motion_result.get_collider_velocity(i)
		var collider_rid  : RID     = p_collision_state.motion_result.get_collider_rid(i)
		
		if interact_with_rigid_bodies and is_body_dynamic(collider_rid) :
			var collision_point : Vector3 = p_collision_state.motion_result.get_collision_point(i)
			p_dynamic_constraints.push_back(DynamicConstraint.new(collider_rid, wall_normal, collision_point))
			continue
		
		append_kinematic_constraint_if_unique_approx(
			p_kinematic_constraints,
			KinematicConstraint.new(wall_normal, wall_velocity, wall_min_slide_angle, wall_constant_speed)
		)




## Determines constraints for grounded movement according to input collision state.
func determine_grounded_move_constraints(
	p_collision_state       : CollisionState,
	p_kinematic_constraints : Array[KinematicConstraint],
	p_dynamic_constraints   : Array[DynamicConstraint] = [],
	p_slide_up_steep_slopes : bool = true,
	p_slide_up_ceilings     : bool = true,
	) -> void :
	# Floor collisions
	for i : int in p_collision_state.floor_collision_indexes :
		var floor_normal   : Vector3 = p_collision_state.motion_result.get_collision_normal(i)
		
		# We don't add the collider velocity to the floor constraint,
		# since floor movement is handled through platform_velocity.
		# We also treat all floor constraints as kinematic.
		append_kinematic_constraint_if_unique_approx(
			p_kinematic_constraints,
			KinematicConstraint.new(floor_normal, Vector3.ZERO, 0.0, floor_constant_speed, floor_stop_on_slope)
		)
	
	# Wall collisions
	for i : int in p_collision_state.wall_collision_indexes :
		var wall_normal   : Vector3 = p_collision_state.motion_result.get_collision_normal(i)
		var wall_velocity : Vector3 = p_collision_state.motion_result.get_collider_velocity(i)
		var collider_rid  : RID     = p_collision_state.motion_result.get_collider_rid(i)
		
		if interact_with_rigid_bodies and is_body_dynamic(collider_rid) :
			var collision_point : Vector3 = p_collision_state.motion_result.get_collision_point(i)
			p_dynamic_constraints.push_back(DynamicConstraint.new(collider_rid, wall_normal, collision_point))
			continue
		
		# Convert to constraint
		var wall_constraint := KinematicConstraint.new(wall_normal, wall_velocity, wall_min_slide_angle, wall_constant_speed)
		
		# If not allowed to slide up slopes, add an additional constraint that
		# holds the character back
		if not p_slide_up_steep_slopes :
			# Jolt: "Only take planes that point up."
			var dot : float = wall_normal.dot(up_direction)
			if dot > CMP_EPSILON :
				# Jolt: "Make horizontal normal"
				var horizontal_normal : Vector3 = (wall_normal - dot * up_direction).normalized()
				# Jolt: "Project the contact velocity on the new normal so that both planes push at an equal rate"
				var projected_velocity : Vector3 = wall_velocity.dot(horizontal_normal) * horizontal_normal
				
				# Jolt: "Create a secondary constraint that blocks horizontal movement"
				append_kinematic_constraint_if_unique_approx(
					p_kinematic_constraints,
					KinematicConstraint.new(horizontal_normal, projected_velocity, wall_min_slide_angle, wall_constant_speed)
				)
		
		append_kinematic_constraint_if_unique_approx(p_kinematic_constraints,wall_constraint)
	
	# Ceiling collisions
	for i : int in p_collision_state.ceiling_collision_indexes :
		var ceiling_normal : Vector3 = p_collision_state.motion_result.get_collision_normal(i)
		var ceiling_velocity : Vector3 = p_collision_state.motion_result.get_collider_velocity(i)
		var collider_rid  : RID     = p_collision_state.motion_result.get_collider_rid(i)
		
		if interact_with_rigid_bodies and is_body_dynamic(collider_rid) :
			var collision_point : Vector3 = p_collision_state.motion_result.get_collision_point(i)
			p_dynamic_constraints.push_back(DynamicConstraint.new(collider_rid, ceiling_normal, collision_point))
			continue
		
		# Create a secondary constraint that blocks upward movement
		if not p_slide_up_ceilings :
			var down_normal : Vector3 = -up_direction
			var projected_velocity : Vector3 = ceiling_velocity.dot(down_normal) * down_normal
			
			append_kinematic_constraint_if_unique_approx(
					p_kinematic_constraints,
					KinematicConstraint.new(down_normal, projected_velocity, 0.0, ceiling_constant_speed)
				)
		
		append_kinematic_constraint_if_unique_approx(
			p_kinematic_constraints,
			KinematicConstraint.new(ceiling_normal, ceiling_velocity, 0.0, ceiling_constant_speed)
		)




## Appends the given kinematic constraint to the given array if none of the constraints
## in the array are approximately equal to it.
static func append_kinematic_constraint_if_unique_approx(
	p_constraints : Array[KinematicConstraint],
	p_constraint  : KinematicConstraint
	) -> bool :
	for other_constraint : KinematicConstraint in p_constraints :
		var equal_component_count : int = 0
		for i : int in 3 :
			if p_constraint.plane_normal[i] > other_constraint.plane_normal[i] - PLANES_CMP_EPSILON and p_constraint.plane_normal[i] < other_constraint.plane_normal[i] + PLANES_CMP_EPSILON :
				equal_component_count += 1
			else : 
				break
		if equal_component_count == 3 and other_constraint.velocity.is_equal_approx(p_constraint.velocity) :
			return false
	p_constraints.push_back(p_constraint)
	return true




## Solves kinematic constraints.
static func solve_kinematic_constraints(
	p_constraints  : Array[KinematicConstraint],
	p_velocity     : Vector3,
	p_up_direction : Vector3
	) -> Vector3 :
	var new_velocity    : Vector3 = p_velocity  # if the velocity does not need modifying, the original is returned
	var last_velocity   : Vector3 = p_velocity
	var numplanes       : int     = p_constraints.size()
	var iteration_count : int  # Keep track for debugging
	
	# If there are no constraints, return input velocity
	if numplanes == 0:
		return p_velocity
	
	# INFO: This is a modified version of the algorithm found in Jolt's CharacterVirtual.cpp
	for i : int in MAX_CONSTRAINT_ITERATIONS :
		iteration_count = i
		
		# Get the constraint we violate the most
		var constraint : KinematicConstraint = get_constraint_with_highest_penetration(p_constraints, new_velocity)
		
		if not constraint :
			break  # new velocity does not violate any constraints
		
		# Solve velocity on this contraint
		new_velocity = solve_constraint(new_velocity, constraint, p_up_direction)
		
		# Find the constraint we will violate the most if we move in this new direction
		var other_constraint : KinematicConstraint = null
		var highest_penetration : float = CMP_EPSILON
		for c : KinematicConstraint in p_constraints :
			if c == constraint :
				continue
				
			var penetration : float = (c.velocity - new_velocity).dot(c.plane_normal)
			if penetration > highest_penetration :
				# Jolt: "We don't want parallel or anti-parallel normals as that will cause our
				# cross product below to become zero. Slack is approx 10 degrees."
				var dot : float = c.plane_normal.dot(constraint.plane_normal)
				if dot < 0.984 and dot > -0.984 :
					highest_penetration = penetration
					other_constraint = c
		
		if other_constraint :
			# Jolt: "Cancel the constraint velocity in the other constraint plane's direction so that we won't try to apply it again and keep ping ponging between planes"
			# ... but only if the constraint velocity is pushing.
			if constraint.velocity.dot(constraint.plane_normal) >= 0.0 :
				constraint.velocity -= minf(0.0, constraint.velocity.dot(other_constraint.plane_normal)) * other_constraint.plane_normal
			if other_constraint.velocity.dot(other_constraint.plane_normal) >= 0.0 :
				other_constraint.velocity -= minf(0.0, other_constraint.velocity.dot(constraint.plane_normal)) * constraint.plane_normal
			
			# If applying one constaint velocity solves the other's, don't add them together to avoid duplicate push velocity
			var combined_constraint_velocity : Vector3
			if (constraint.velocity - other_constraint.velocity).dot(other_constraint.plane_normal) >= -CMP_EPSILON :
				combined_constraint_velocity = constraint.velocity
			elif (other_constraint.velocity - constraint.velocity).dot(constraint.plane_normal) >= -CMP_EPSILON :
				combined_constraint_velocity = other_constraint.velocity
			else :
				combined_constraint_velocity = constraint.velocity + other_constraint.velocity
			
			# Calculate relative velocity in the crease direction
			var slide_dir : Vector3 = constraint.plane_normal.cross(other_constraint.plane_normal).normalized()
			var relative_velocity : Vector3 = new_velocity - combined_constraint_velocity
			var relative_velocity_in_slide_dir : Vector3 = relative_velocity.dot(slide_dir) * slide_dir
			
			# Handle constant speed feature, if the other constraint needs it
			# NOTE: If only the first is marked, than we already handled it previously,
			# because we are projecting the new velocity onto the crease.
			if other_constraint.constant_speed :
				if constraint.constant_speed :
					# If both constrains are marked, modify the projected relative velocity to have the original's length
					relative_velocity_in_slide_dir = relative_velocity_in_slide_dir.normalized() * relative_velocity.length()
				else :
					# If only the other constraint is marked, make relative velocity parallel to the marked
					# constraint without changeing its length, than project that onto the crease direction.
					var constraint_slide_dir : Vector3 = relative_velocity.slide(other_constraint.plane_normal).normalized()
					var constraint_slide_vel : Vector3 = constraint_slide_dir * relative_velocity.length()
					
					relative_velocity_in_slide_dir = constraint_slide_vel.dot(slide_dir) * slide_dir
			
			# Add all components together
			new_velocity = relative_velocity_in_slide_dir + combined_constraint_velocity
		
		if new_velocity.is_zero_approx():
			break
		
		# Jolt: "If the constraint has velocity we accept the new velocity, otherwise
		# check that we didn't reverse velocity"
		# NOTE: This solves triple plane interactions, where we've lost all 3
		# degrees of freedom.
		if not constraint.velocity.is_zero_approx() :
			last_velocity = constraint.velocity
		elif new_velocity.dot(last_velocity) < 0.0 :
			return Vector3.ZERO  # Stop dead if velocity turned around. (No more degrees of freedom left.)
	
	# Check if we reached maximum amount of iterations. Maximum value may have to be increased
	# for complex scenarios.
	if iteration_count == MAX_CONSTRAINT_ITERATIONS - 1 :
		printerr('IKCC: Looped for maximum amount of constraint iterations. (%d)' % MAX_CONSTRAINT_ITERATIONS)
	
	return new_velocity




## Solves a single constraint.
static func solve_constraint(
	p_velocity     : Vector3,
	p_constraint   : KinematicConstraint,
	p_up_direction : Vector3
	) -> Vector3 :
	var relative_velocity : Vector3 = p_velocity - p_constraint.velocity
	
	# Handle min angle feature
	if is_angle_too_perpendicular(p_constraint.plane_normal, p_constraint.min_slide_angle, relative_velocity) :
		p_constraint.is_slide_cancelled = true
		return p_constraint.velocity
	
	# Handle special cases where direction and length needs to be calculated differently
	if p_constraint.constant_speed or p_constraint.keep_horizontal_dir :
		var slide_dir : Vector3
		var slide_length : float
		
		if p_constraint.keep_horizontal_dir :
			# Slide using the intersection of input velocity and the constraint plane
			slide_dir = p_up_direction.cross(relative_velocity).cross(p_constraint.plane_normal).normalized()
			# If the plane normal is pointing downward, we need to reverse it
			if p_constraint.plane_normal.dot(p_up_direction) < 0.0 :
				slide_dir *= -1
		else :
			slide_dir = relative_velocity.slide(p_constraint.plane_normal).normalized()
		
		if p_constraint.constant_speed :
			# Keep original length
			slide_length = relative_velocity.length()
		else :
			slide_length = relative_velocity.slide(p_constraint.plane_normal).length()
		
		return slide_dir * slide_length + p_constraint.velocity
	
	# Calculate new velocity if we cancel the relative velocity in the normal direction
	var new_velocity : Vector3 = p_velocity - relative_velocity.dot(p_constraint.plane_normal) * p_constraint.plane_normal
	
	return new_velocity




## Determines if slide angle is too steep or not on a given plane.
static func is_angle_too_perpendicular(
	p_plane_normal    : Vector3,
	p_min_slide_angle : float,
	p_velocity        : Vector3
	) -> bool :
	if not p_min_slide_angle :
		return false  # no minimum angle is given
	
	# Calculate the cosine of the angle between velocity and plane normal
	var cosine_angle: float = p_velocity.normalized().dot(p_plane_normal)
	var cosine_min_slide_angle: float = cos(p_min_slide_angle)
	
	if absf(cosine_angle) > cosine_min_slide_angle :
		return true  # angle is too perpendicular
	
	return false




## Returns the constraint that the input velocity violates the most.
## Returns null, if the input velocity doesn't interact with any input constraint.
static func get_constraint_with_highest_penetration(
	p_constraints : Array[KinematicConstraint],
	p_velocity    : Vector3
	) -> KinematicConstraint :
	var constraint : KinematicConstraint = null
	var highest_penetration : float = CMP_EPSILON
	for c : KinematicConstraint in p_constraints :
		var penetration : float = (c.velocity - p_velocity).dot(c.plane_normal)
		if penetration > highest_penetration :
			highest_penetration = penetration
			constraint = c
	return constraint




## Solves dynamic constraints.
static func solve_dynamic_constraints(
		p_constraints  : Array[DynamicConstraint],
		p_velocity     : Vector3,
		p_inverse_mass : float,
		p_io           : MoveShapeIO = null
	) -> Vector3 :
	var new_velocity : Vector3 = p_velocity  # if velocity is not facing any plane, return original
	
	# Check and store how many collisions we have per body
	var bodies_collision_counts : Dictionary
	for constraint in p_constraints :
		var rid_id : int = constraint.collider_rid.get_id()
		if bodies_collision_counts.has(rid_id) :
			bodies_collision_counts[rid_id] += 1
		else :
			bodies_collision_counts[rid_id] = 1
	
	for constraint : DynamicConstraint in p_constraints :
		var body_state := PhysicsServer3D.body_get_direct_state(constraint.collider_rid)
		
		var normal : Vector3 = -constraint.plane_normal
		
		var m1_inv : float   = p_inverse_mass
		var m2_inv : float   = body_state.inverse_mass
		var I2_inv : Vector3 = body_state.inverse_inertia
		
		var r2 : Vector3 = constraint.collision_point - body_state.transform.origin
		
		var v1     : Vector3 = new_velocity
		var v2     : Vector3 = body_state.linear_velocity
		var omega2 : Vector3 = body_state.angular_velocity
		
		var relative_velocity : Vector3 = (v2 + omega2.cross(r2)) - v1
		var v_rel_norm        : float   = relative_velocity.dot(normal)
		
		if v_rel_norm > 0.0 :
			continue
		
		var restitution : float = 0.0
		
		var j_denom : float   = (m1_inv) + (m2_inv) + (normal.dot((r2.cross(normal) * I2_inv).cross(r2)))
		var j       : float   = -(1.0 + restitution) * v_rel_norm / j_denom
		var impulse : Vector3 = j * normal
		
		# Divide impulse by how many collisions happened with the given body
		impulse /= bodies_collision_counts[constraint.collider_rid.get_id()]
		
		new_velocity -= impulse * p_inverse_mass
		
		# Save the originhal state of the body
		if p_io and not BodySaveState.array_contains_rid(p_io.body_save_states, constraint.collider_rid) :
			p_io.body_save_states.push_back(
				BodySaveState.new(
					constraint.collider_rid,
					body_state.linear_velocity,
					body_state.angular_velocity
				)
			)
		
		body_state.apply_impulse(impulse, r2)
	
	return new_velocity




## Checks if the body is at least the given distance away downward from a floor collision or surface.
## Sets the given CollisionState object.
func check_floor_status(
	p_from                     : Transform3D,
	p_check_distance           : float,
	p_result_state             : CollisionState,
	p_check_for_surface_normal : bool = false
	) -> bool :
	var check_distance : float   = maxf(safe_margin, p_check_distance)
	var test_motion    : Vector3 = -up_direction * check_distance
	var motion_result  :         = PhysicsTestMotionResult3D.new()
	var motion_params  :         = PhysicsTestMotionParameters3D.new()
	motion_params.from = p_from
	motion_params.motion = test_motion
	motion_params.margin = safe_margin
	motion_params.recovery_as_collision = true
	motion_params.max_collisions = MAX_SNAP_COLLISIONS
	# We are only testing in this function, but we might apply the travel outside, so enable slide cancelling
	var collided : bool = _move_and_collide(motion_result, motion_params, true, true)
	
	if not collided :
		return false
	
	set_collision_state(motion_result, p_result_state)
	
	if p_result_state.s_floor or p_result_state.s_wall_floor :
		return true
	
	if p_check_for_surface_normal and p_result_state.s_wall and collided_with_floor_surface(p_result_state, p_from.origin) :
		p_result_state.s_floor_surface_only = true
		return true
	
	return false





## Attempts to perform a step move, and if it succeeds, set's the output data and returns true.
func step_move(
	p_io              : MoveShapeIO,
	p_collision_state : CollisionState,
	p_excluded_bodies : Array[RID] = [],
	step_forward_ios  : Array[MoveShapeIO] = []
	) -> bool :
	var horizontal_velocity : Vector3 = p_io.velocity.slide(up_direction)
	
	if horizontal_velocity.is_zero_approx() :
		return false  # We won't step if our horizontal velocity is near zero
	
	# Collect wall normals we are pushing into, and determine extra forward motion needed
	var wall_normals : PackedVector3Array
	# NOTE: We need to move towards the walls with at least the safe margin
	# (because it is possible we've hit a wall only with the safe margin inflated shape),
	# and also some little amount, so that we can reach on the top of the ledge.
	var min_forward_length : float = (safe_margin + min_step_forward_distance)
	var base_forward_motion : Vector3 = horizontal_velocity * p_io.time_remaining
	var extra_forward_motion := Vector3.ZERO
	for i : int in p_collision_state.wall_collision_indexes :
		var wall_velocity : Vector3 = p_collision_state.motion_result.get_collider_velocity(i)
		var wall_normal : Vector3 = p_collision_state.motion_result.get_collision_normal(i)
		if (horizontal_velocity - wall_velocity).dot(wall_normal) < 0.0 :
			wall_normals.push_back(wall_normal)
			
			var horizontal_wall_normal : Vector3 = wall_normal.slide(up_direction).normalized()
			var motion_length_towards_wall : float = -(base_forward_motion + extra_forward_motion).dot(horizontal_wall_normal)
			extra_forward_motion += maxf(0.0, min_forward_length - motion_length_towards_wall) * -horizontal_wall_normal
	
	if wall_normals.is_empty() :
		return false  # There are no walls we are pushing into, bail
	
	# UP
	var step_move_transform : Transform3D = p_io.transform
	var up_motion_result := PhysicsTestMotionResult3D.new()
	var up_motion_params := PhysicsTestMotionParameters3D.new()
	up_motion_params.from = step_move_transform
	up_motion_params.motion = up_direction * max_step_height
	up_motion_params.margin = safe_margin
	up_motion_params.recovery_as_collision = false
	up_motion_params.max_collisions = 0
	_move_and_collide(up_motion_result, up_motion_params, true)
	# NOTE : We only need the travel of the motion, so no collision report is needed.
	
	# See how much we travelled up
	var up_travel := up_motion_result.get_meta("travel", up_motion_result.get_travel()) as Vector3
	var up_travel_length : float = up_travel.dot(up_direction)
	if up_travel_length <= safe_margin :
		return false  # We couldn't move up a significant distance, bail
	
	# Update step move position with upward travel
	step_move_transform.origin += up_travel
	var up_transform : Transform3D = step_move_transform
	
	# FORWARD
	# Move with the extra forward motion if needed,
	# so that the forward move travels at least the minimum distance
	if not extra_forward_motion.is_zero_approx() :
		# NOTE: v = s / t, if t = 1, then v = s
		var extra_forward_move_io := MoveShapeIO.new(step_move_transform, extra_forward_motion, 1.0)
		move_shape(extra_forward_move_io, false, [],  p_excluded_bodies, false)
		# Update step move position
		step_move_transform = extra_forward_move_io.transform
		# Store reference to IO
		step_forward_ios.push_back(extra_forward_move_io)
	
	# Remember this position
	var offsetted_up_transform : Transform3D = step_move_transform
	
	# Move with horizontal velocity, using the remaining time
	var forward_move_io := MoveShapeIO.new(step_move_transform, horizontal_velocity, p_io.time_remaining)
	if not is_zero_approx(p_io.time_remaining) :
		move_shape(forward_move_io, false, [], p_excluded_bodies, false)
		# Update step move position
		step_move_transform = forward_move_io.transform
		# Store reference to IO
		step_forward_ios.push_back(forward_move_io)
	
	# Check if we made progress towards any of the walls
	var made_progress : bool = false
	var forward_move_travel : Vector3 = step_move_transform.origin - up_transform.origin
	for wall_normal : Vector3 in wall_normals :
		if wall_normal.dot(forward_move_travel) < -safe_margin :
			made_progress = true
			break
	if not made_progress :
		return false
	
	# DOWN
	var down_motion_result := PhysicsTestMotionResult3D.new()
	var down_motion_params := PhysicsTestMotionParameters3D.new()
	down_motion_params.from = step_move_transform
	# NOTE: We move down the same amount we moved up.
	down_motion_params.motion = -up_direction * up_travel_length
	down_motion_params.margin = safe_margin
	down_motion_params.recovery_as_collision = true
	down_motion_params.max_collisions = MAX_SNAP_COLLISIONS
	
	if not _move_and_collide(down_motion_result, down_motion_params, true) :
		return false  # In the air, stepping failed
	
	# Update step move position with downward travel
	var down_travel := down_motion_result.get_meta("travel", down_motion_result.get_travel()) as Vector3
	step_move_transform.origin += down_travel
	
	var down_result_state := CollisionState.new()
	set_collision_state(down_motion_result, down_result_state)
	
	# Check if downward move collided with a floor
	if not down_result_state.s_floor :
		# Ground normal is too steep, test at a further distance to see if there is floor further
		# along the ground
		# Default check direction is the inverted horizontal steep ground normal we landed on
		# Use the wall surface normal if available
		var ground_normal : Vector3 = down_result_state.deepest_wall_normal
		var floor_check_forward_dir : Vector3 = -ground_normal.slide(up_direction).normalized()
		# If wall surface normal is available, we use it for the step angle check, so we can
		# better single out the problematic step scenarios and we don't have to set the angle
		# threshold to a value above zero.
		# Because collision could be a corner, we only use the surface normal for the angle check
		# and not for the forward floor check direction.
		var wall_surface_normal : Vector3 = get_wall_surface_normal_below(down_result_state, horizontal_velocity)
		var step_compare_dir    : Vector3 = (
			-wall_surface_normal.slide(up_direction).normalized()
			if not wall_surface_normal == Vector3.ZERO
			else floor_check_forward_dir
		)
		# If the horizontal velocity does not face this direction, we use the horizontal velocity
		# direction instead of the ground normal.
		var horizontal_velocity_dir : Vector3 = horizontal_velocity.normalized()
		if horizontal_velocity_dir.dot(step_compare_dir) < 0.0 :
			floor_check_forward_dir = horizontal_velocity_dir
		# Set length to the preset value
		var floor_check_forward_motion : Vector3 = floor_check_forward_dir * step_floor_check_distance
		
		# Move forward with this motion
		var floor_check_forward_io := MoveShapeIO.new(offsetted_up_transform, floor_check_forward_motion, 1.0)
		move_shape(floor_check_forward_io, false, [], p_excluded_bodies, false)
		
		# Move down
		var floor_check_down_result := PhysicsTestMotionResult3D.new()
		down_motion_params.from = floor_check_forward_io.transform
		if _move_and_collide(floor_check_down_result, down_motion_params, true) :
			
			# INFO: Debug draw to visualize where the floor check lands
			#DebugDraw3D.draw_sphere(down_motion_result.get_meta("travel", down_motion_result.get_travel()) as Vector3 + floor_check_forward_io.transform.origin, 0.2)
			
			var floor_check_result_state := CollisionState.new()
			set_collision_state(floor_check_down_result, floor_check_result_state)
			if not floor_check_result_state.s_floor :
				return false  # No stable floor found
		else :
			return false  # In the air
	
	# Step detected, check step height
	var step_height : float = (step_move_transform.origin - p_io.transform.origin).dot(up_direction)
	if is_zero_approx(step_height) :
		return false  # If we are on the same level, we didn't step
	
	# Move to final position
	p_io.transform.origin = step_move_transform.origin + safe_margin * up_direction
	# Set remaining time to zero, since we have used up the remaining time with the step move,
	# or we didn't have any to begin with
	p_io.time_remaining = 0.0
	# Update velocity with the output horizontal move velocity, and add back the upwards component
	# of the original velocity
	p_io.velocity = forward_move_io.velocity + maxf(0.0, p_io.velocity.dot(up_direction)) * up_direction
	# Append forward move collision states
	p_io.collision_states.append_array(forward_move_io.collision_states)
	# Set step flag
	p_io.has_stepped = true
	
	return true




## Does a motion test, sets the given result object, optionally adjusts travel to cancel sliding
## caused by recovery, optionally moves the body to the end position,
## returns true if collision happened.
## It's important to note that collision check method 'body_test_motion' discards contacts
## that the motion doesn't face (sometimes not if it's parallel to the surface)!
# NOTICE : The built in 'move_and_collide' does not suffice because :
# - Slide cancelling can't be turned off.
# - The returned KinematicCollision3D object provides too little information about the collision.
func _move_and_collide(
	p_res                      : PhysicsTestMotionResult3D,
	p_params                   : PhysicsTestMotionParameters3D,
	p_test_only                : bool  = false,
	p_cancel_sliding           : bool  = true
	) -> bool :
	var collided      : bool = PhysicsServer3D.body_test_motion(self, p_params, p_res)
	var travel        : Vector3 = p_res.get_travel()
	var modify_travel : bool = false 
	
	# Cancel sliding caused be recovery, if needed
	if p_cancel_sliding :
		var motion_length : float = p_params.motion.length()
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
				motion_normal = p_params.motion / motion_length
			
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
		# Set meta data with modified results (Because the result object is read-only in GDScript.)
		p_res.set_meta("travel", travel)
		p_res.set_meta("remainder", p_params.motion - travel)
	
	if not p_test_only :
		global_position += travel
	
	return collided




## Updates the overall state with the given collision state, assuming it's the latest.
func update_overall_state(
	p_col_state       : CollisionState,
	p_set_floor       : bool,
	p_set_wall        : bool,
	p_set_ceiling     : bool,
	p_set_final_state : bool = false
	) -> void :
	
	# Floor
	if p_set_floor and p_col_state.s_floor :
		collided_with_floor = true
		last_floor_normal = p_col_state.deepest_floor_normal
		
		if p_set_final_state :
			# Store floor collider
			if not current_floor_collider_encoded :
				current_floor_collider_encoded = EncodedObjectAsID.new()
			current_floor_collider_encoded.object_id = p_col_state.motion_result.get_collider_id(p_col_state.deepest_floor_index)
			current_floor_collider_rid = p_col_state.motion_result.get_collider_rid(p_col_state.deepest_floor_index)
			
			is_on_floor = true
			current_floor_normal = p_col_state.deepest_floor_normal
	
	# Wall
	if p_set_wall and p_col_state.s_wall :
		collided_with_wall = true
		last_wall_normal = p_col_state.deepest_wall_normal
		
		if p_set_final_state :
			is_on_wall = true
			current_wall_normal = p_col_state.deepest_wall_normal
	
	# Ceiling
	if p_set_ceiling and p_col_state.s_ceiling :
		collided_with_ceiling = true
		last_ceiling_normal = p_col_state.deepest_ceiling_normal
		
		if p_set_final_state :
			is_on_ceiling = true
			current_ceiling_normal = p_col_state.deepest_ceiling_normal
	
	# Wall-floor
	# NOTE : Wall-floors are only registered in collision state,
	# if collision includes no regular floor contacts, so that doesn't need to be checked here.
	if p_set_floor and p_col_state.s_wall_floor :
		collided_with_floor = true
		last_floor_normal = p_col_state.wall_floor_normal
		
		if p_set_final_state :
			# Invalidate floor collider
			current_floor_collider_encoded = null
			
			is_on_floor = true
			current_floor_normal = p_col_state.wall_floor_normal
	
	# Wall-ceiling
	# NOTE : Wall-ceilings are only registered in collision state,
	# if collision includes no regular floor contacts, so that doesn't need to be checked here.
	if p_set_ceiling and p_col_state.s_wall_ceiling :
		collided_with_ceiling = true
		last_ceiling_normal = p_col_state.wall_ceiling_normal
		
		if p_set_final_state :
			is_on_ceiling = true
			current_ceiling_normal = p_col_state.wall_ceiling_normal
	
	# Merge and store collider datas
	for collider_data : ColliderData in p_col_state.collider_datas:
		var array_index : int = ColliderData.find_by_collider_id(collider_datas, collider_data.collider_id)
		if array_index == -1:
			# Collider not in array, append it
			collider_datas.push_back(collider_data)
		else :
			# Collider is in array, update it
			collider_datas[array_index].concat_data_unique_only(collider_data)




## Calculates information about given collisions and stores it in the given 'CollisionState' object.
func set_collision_state(
	p_motion_result           : PhysicsTestMotionResult3D,
	p_col_state               : CollisionState,
	p_ignore_character_motion : bool = false
	) -> void :
	
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
	
	for i : int in collision_count :
		var collision_normal : Vector3 = p_motion_result.get_collision_normal(i)
		var collision_depth  : float = p_motion_result.get_collision_depth(i)
		
		# Handle case when we only want to register collisions that happened only
		# because of motion from other bodies
		if p_ignore_character_motion :
			var collider_velocity : Vector3 = p_motion_result.get_collider_velocity(i)
			var character_summed_velocity : Vector3 = velocity + platform_velocity
			var relative_velocity : Vector3 = character_summed_velocity - collider_velocity
			if velocity.dot(collision_normal) < -CMP_EPSILON or relative_velocity.dot(collision_normal) > 0.0 :
				continue  # Skip collision
		
		# Store collider data
		var collider_id : int = p_motion_result.get_collider_id(i)
		var collider_rid : RID = p_motion_result.get_collider_rid(i)
		var collision_point : Vector3 = p_motion_result.get_collision_point(i)
		var array_index : int = ColliderData.find_by_collider_id(p_col_state.collider_datas, collider_id)
		if array_index == -1 :
			p_col_state.collider_datas.push_back(
				ColliderData.new(collider_id, collider_rid, collision_normal, collision_point)
			)
		else :
			var collider_data : ColliderData = p_col_state.collider_datas[array_index]
			if not collider_data.contains_point(collision_point) :
				collider_data.collision_points.push_back(collision_point)
				collider_data.collision_normals.push_back(collision_normal)
				collider_data.collision_count += 1
		
		if motion_mode == MotionMode.GROUNDED :
			# Check if floor collision
			if is_floor(collision_normal) :
				p_col_state.s_floor = true
				p_col_state.floor_collision_indexes.push_back(i)
				
				if collision_depth > max_floor_depth :
					max_floor_depth = collision_depth
					p_col_state.deepest_floor_normal = collision_normal
					p_col_state.deepest_floor_index = i
				
				continue
			
			# Check if ceiling collision
			if is_ceiling(collision_normal) :
				p_col_state.s_ceiling = true
				p_col_state.ceiling_collision_indexes.push_back(i)
				
				if collision_depth > max_ceiling_depth : 
					max_ceiling_depth = collision_depth
					p_col_state.deepest_ceiling_normal = collision_normal
					p_col_state.deepest_ceiling_index = i
				
				continue
		
		p_col_state.s_wall = true  # Collision is wall by default
		p_col_state.wall_collision_indexes.push_back(i)
		
		if collision_depth > max_wall_depth :
			max_wall_depth = collision_depth
			p_col_state.deepest_wall_normal = collision_normal
			p_col_state.deepest_wall_index = i
		
		# Collect wall normals for calculating avarage
		if motion_mode == MotionMode.GROUNDED and not collision_normal.is_equal_approx(tmp_wall_normal) :
			tmp_wall_normal = collision_normal
			wall_normal_sum += collision_normal
			wall_collision_count += 1
	
	if motion_mode == MotionMode.GROUNDED and wall_collision_count > 1 :
		var combined_wall_normal : Vector3 = wall_normal_sum.normalized()
		
		if not p_col_state.s_floor :
			# Check if wall normals cancel out to floor support
			if is_floor(combined_wall_normal) :
				p_col_state.s_wall_floor = true
				p_col_state.wall_floor_normal = combined_wall_normal
				# NOTE : Keep wall state for proper sliding!
				return
		
		if not p_col_state.s_ceiling :
			# Check if wall normals cancel out to ceiling support
			if is_ceiling(combined_wall_normal) :
				p_col_state.s_wall_ceiling = true
				p_col_state.wall_ceiling_normal = combined_wall_normal
				# NOTE : Keep wall state for proper sliding!




## Calculates whether given normal is floor or not.
func is_floor(p_normal : Vector3) -> bool:
	var floor_angle : float = acos(p_normal.dot(up_direction))
	if floor_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
		return true
	return false




## Calculates whether given normal is ceiling or not.
func is_ceiling(p_normal : Vector3) -> bool:
	var ceiling_angle : float = acos(p_normal.dot(-up_direction))
	if ceiling_angle <= floor_max_angle + ANGLE_CMP_EPSILON :
		return true
	return false




## Calculates whether given normal is wall or not.
func is_wall(p_normal : Vector3) -> bool:
	return not (is_floor(p_normal) or is_ceiling(p_normal))




## Returns the centre-bottom (feet) position of the collider.
func get_feet_position(p_global_position : Vector3) -> Vector3 :
	var collider_global_position : Vector3 = collider.position + p_global_position
	
	if collider.shape is CylinderShape3D :
		var cylinder := collider.shape as CylinderShape3D
		return collider_global_position + (cylinder.height * 0.5) * -up_direction
	
	if collider.shape is BoxShape3D :
		var box := collider.shape as BoxShape3D
		return collider_global_position + (box.size.y * 0.5) * -up_direction
	
	if collider.shape is CapsuleShape3D :
		var capsule := collider.shape as CapsuleShape3D
		return collider_global_position + (capsule.height * 0.5) * -up_direction
	
	if collider.shape is SphereShape3D :
		var sphere := collider.shape as SphereShape3D
		return collider_global_position + (sphere.radius) * -up_direction
	
	printerr("IKCC: Collider shape not handled by get_feet_position()! (%s)" % collider.shape.get_class())
	
	return collider_global_position




## Returns the radius of the collider, if the collider is round, else it returns 0.0.
func get_collider_radius() -> float :
	
	if collider.shape is CapsuleShape3D :
		var capsule := collider.shape as CapsuleShape3D
		return capsule.radius
	
	if collider.shape is SphereShape3D :
		var sphere := collider.shape as SphereShape3D
		return sphere.radius
	
	return 0.0  # Collider is not round




## Checks if the collision result includes a floor surface normal using raycasts.
func collided_with_floor_surface(p_collision_state : CollisionState, p_global_position : Vector3) -> bool :
	var raycast            := RayCast.new()
	var direct_space_state : PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	var max_surface_height : float = get_feet_position(p_global_position).dot(up_direction) + get_collider_radius()
	
	for i : int in p_collision_state.wall_collision_indexes :
		var col_point  : Vector3 = p_collision_state.motion_result.get_collision_point(i)
		
		# If the surface is too high, it's not a floor
		if col_point.dot(up_direction) > max_surface_height :
			continue
		
		var ray_origin : Vector3 = p_global_position + collider.position
		var col_normal : Vector3 = p_collision_state.motion_result.get_collision_normal(i)
		var offset_dir : Vector3 = -col_normal
		var dest       : Vector3 = col_point + offset_dir * RAYCAST_OFFSET_LENGTH
		
		raycast.intersect(ray_origin, dest, self, collision_mask, direct_space_state)
		
		if raycast.hit and is_floor(raycast.normal) :
			return true
	
	return false




## Checks if the collision result includes a wall surface normal
## below a wall collision point using raycasts.
## Returns the wall surface normal if found, else a zero vector.
func get_wall_surface_normal_below(p_collision_sate : CollisionState, p_velocity : Vector3) -> Vector3 :
	var raycast            := RayCast.new()
	var direct_space_state : PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	
	for i : int in p_collision_sate.wall_collision_indexes :
		var col_normal : Vector3 = p_collision_sate.motion_result.get_collision_normal(i)
		if p_velocity.dot(col_normal) > 0.0 :
			continue
		
		var col_point  : Vector3 = p_collision_sate.motion_result.get_collision_point(i)
		var offset     : Vector3 = (col_normal + up_direction) * -RAYCAST_OFFSET_LENGTH
		var dest       : Vector3 = col_point + offset
		var origin     : Vector3 = col_point + col_normal * RAYCAST_OFFSET_LENGTH
		
		raycast.intersect(origin, dest, self, collision_mask, direct_space_state)
		
		if raycast.hit and is_wall(raycast.normal):
			return raycast.normal
	
	return Vector3.ZERO




## Returns true if given body is in a dynamic mode, else it returns false.
static func is_body_dynamic(rid : RID) -> bool :
	var body_mode := PhysicsServer3D.body_get_mode(rid)
	return body_mode == PhysicsServer3D.BodyMode.BODY_MODE_RIGID or body_mode == PhysicsServer3D.BodyMode.BODY_MODE_RIGID_LINEAR




#==================================================================================================
# ------------------------------
# | STRUCTS AND HELPER CLASSES |
# ------------------------------


## Stores imformation about motion test collisions.
class CollisionState extends RefCounted :
	var s_floor                   : bool
	var s_wall                    : bool
	var s_ceiling                 : bool
	var s_wall_floor              : bool
	var s_wall_ceiling            : bool
	var s_floor_surface_only      : bool
	var deepest_floor_index       : int
	var deepest_wall_index        : int
	var deepest_ceiling_index     : int
	var deepest_wall_normal       : Vector3
	var deepest_floor_normal      : Vector3
	var deepest_ceiling_normal    : Vector3
	var wall_floor_normal         : Vector3
	var wall_ceiling_normal       : Vector3
	var floor_collision_indexes   : PackedInt32Array
	var wall_collision_indexes    : PackedInt32Array
	var ceiling_collision_indexes : PackedInt32Array
	var motion_result             : PhysicsTestMotionResult3D
	var collider_datas            : Array[ColliderData]


## Input/output and output variables for the 'move_shape' function.
class MoveShapeIO extends RefCounted :
	# Input/output
	var transform             : Transform3D
	var velocity              : Vector3
	var time_remaining        : float
	
	# Output only
	var collision_states      : Array[CollisionState]
	var touched_floor         : bool
	var has_stepped           : bool
	## Keeps track of recursion depth.
	var iteration_count       : int
	var body_save_states      : Array[BodySaveState]
	var floor_impact_velocity : Vector3
	
	func _init(p_transform : Transform3D, p_velocity : Vector3, p_time_remaining : float) -> void :
		transform = p_transform
		velocity = p_velocity
		time_remaining = p_time_remaining
	
	func restore_body_states() -> void :
		for save_state : BodySaveState in body_save_states :
			var body_state := PhysicsServer3D.body_get_direct_state(save_state.body_rid)
			
			body_state.linear_velocity = save_state.linear_velocity
			body_state.angular_velocity = save_state.angular_velocity


## Base class for constraints.
class Constraint extends RefCounted :
	var plane_normal : Vector3


## Stores info about a kinematic constraint.
class KinematicConstraint extends Constraint :
	var velocity            : Vector3
	## Angle in radians.
	var min_slide_angle     : float
	var constant_speed      : bool
	var keep_horizontal_dir : bool
	
	## Marked by constraint solver.
	var is_slide_cancelled : bool
	
	func _init(
		p_plane               : Vector3,
		p_velocity            : Vector3,
		p_min_slide_angle     : float = 0.0,
		p_constant_speed      : bool = false,
		p_keep_horizontal_dir : bool = false
		) -> void :
		plane_normal = p_plane
		velocity = p_velocity
		min_slide_angle = p_min_slide_angle
		constant_speed = p_constant_speed
		keep_horizontal_dir = p_keep_horizontal_dir


## Stores info about a dynamic constraint.
class DynamicConstraint extends Constraint :
	var collider_rid    : RID
	var collision_point : Vector3
	
	func _init(p_collider_rid : RID, p_plane : Vector3, p_collision_point : Vector3) -> void :
		collider_rid = p_collider_rid
		plane_normal = p_plane
		collision_point = p_collision_point


## Stores data of a body's physics state, so it can be restored.
class BodySaveState extends RefCounted :
	var body_rid : RID
	var linear_velocity : Vector3
	var angular_velocity : Vector3
	
	func _init(p_body_rid : RID, p_linear_velocity : Vector3, p_angular_velocity : Vector3) -> void :
		body_rid = p_body_rid
		linear_velocity = p_linear_velocity
		angular_velocity = p_angular_velocity
	
	static func array_contains_rid(p_array : Array[BodySaveState], p_rid : RID) -> bool :
		for save_state : BodySaveState in p_array :
			if save_state.body_rid.get_id() == p_rid.get_id() :
				return true
		return false


## Stores a collider and collision data that belongs to it.
class ColliderData extends RefCounted :
	var collider_id       : int
	var collider_rid      : RID
	var collision_normals : PackedVector3Array
	var collision_points  : PackedVector3Array
	var collision_count   : int
	
	func _init(p_id : int, p_rid : RID, p_normal : Vector3, p_point : Vector3) -> void :
		collider_id = p_id
		collider_rid = p_rid
		collision_normals = [p_normal]
		collision_points = [p_point]
		collision_count = 1
	
	static func find_by_collider_id(p_array : Array[ColliderData], p_collider_id : int) -> int :
		var index : int = 0
		for collider_data : ColliderData in p_array :
			if collider_data.collider_id == p_collider_id :
				return index
			index += 1
		return -1
	
	func contains_point(p_point : Vector3) -> bool :
		for point : Vector3 in collision_points :
			if point.is_equal_approx(p_point) :
				return true
		return false
	
	func concat_data_unique_only(p_collider_data : ColliderData) -> void :
		for i : int in p_collider_data.collision_count :
			if not contains_point(p_collider_data.collision_points[i]) :
				collision_points.push_back(p_collider_data.collision_points[i])
				collision_normals.push_back(p_collider_data.collision_normals[i])
				collision_count += 1


## Simplifies raycasting.
class RayCast extends RefCounted :
	var hit    : bool
	var normal : Vector3
	
	func intersect(
		origin       : Vector3,
		dest        : Vector3,
		exclude     : RID,
		mask        : int,
		space_state : PhysicsDirectSpaceState3D
		) -> void:
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
