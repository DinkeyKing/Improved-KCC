extends RigidBody3D
class_name Player

##############
# PROPERTIES #
##############

@export_group("General moving")
@export var run_speed : float = 8.0
@export var ground_lerp_speed : float = 10.0
@export var air_lerp_speed : float = 5.0

@export_group("Jumping")
@export var jump_velocity_length : float = 6.0

@export_group("Moving platforms")
@export var add_velocity_on_leave : bool = true

@export_group("Physics")
@export var kin_mass : float = 0.3
	# Kinematic mass, does not correlate with the mass used in the physics engine.
@export var impulse_lerp_speed : float = 7.0  # The friction for impulses can be different


#########
# NODES #
#########

@onready var head := $PivotY as Node3D
@onready var collider := $CollisionShape as CollisionShape3D
@onready var jump_input_buffer := $JumpInputBuffer as InputBuffer


#############
# CONSTANTS #
#############

# Move and slide
const MAX_SLIDES : int = 5                   # Default: 5
const CHECK_FLOOR_DISTANCE : float = 0.08    # Default: 0.08
const SAFE_MARGIN : float = 0.001            # Default: 0.001
const SNAP_SAFE_MARGIN : float = 0.003       # Default: 0.003
const COLLISION_SHAPE_MARGIN : float = 0.04  # Default: 0.04
const MIN_FLOOR_Y : float = 0.7              # Default: 0.7
const MAX_CEILING_Y : float = -0.3           # Default: -0.3
const STEP_HEIGHT : float = 0.26             # Default: 0.26
const LEDGE_DETECT_OFFSET : float = 0.003    # Default: 0.003

# -1 < MAX_CEILING_Y < MIN_FLOOR_Y < 1
# |CEILING|        |WALL|       |FLOOR|

# Physics
const PUSH_FACTOR : float = 0.5               # Default: 0.5
const FRICTION_FACTOR : float = 15.0           # Default: 15.0
const WEIGHT_FACTOR : float = 10.0             # Default: 10.0
const IMPACT_FACTOR : float = 2.0             # Default: 2.0
const JUMP_FACTOR : float = 2.0               # Default: 2.0
const IMPULSE_CHANGE_FACTOR : float = 0.08    # Default: 0.08
const RB_VEL_LIMIT : float = 10.0             # Default: 10.0


#############
# VARIABLES #
#############

var velocity := Vector3.ZERO
var platform_velocity := Vector3.ZERO
var impulse_velocity := Vector3.ZERO
var floor_impact_velocity := Vector3.ZERO
var real_move_velocity := Vector3.ZERO
var speed : float = 0.0
var gravity : float = ProjectSettings.get_setting("physics/3d/default_gravity")
var slide_depth : int = 0
var last_rigid_body_collision : KinematicCollision3D

var is_jumping : bool = false

#var linear_velocity := Vector3.ZERO


#############
# FUNCTIONS #
#############

func _ready() -> void :
	rotation = Vector3.ZERO  # Important! (Since only the head rotates.)


func get_input_direction() -> Vector3 :
	var input_dir_vec2 : Vector2 = Input.get_vector("left", "right", "up", "down")
	var input_dir_vec3 : Vector3 = head.transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	return input_dir_vec3


func set_initial_velocity(move_dir : Vector3, delta: float) -> void :
	var lerp_speed : float = ground_lerp_speed
	
	if is_on_floor() :
		lerp_speed = ground_lerp_speed
		
		# Apply friction to impulse velocity
		impulse_velocity.y = 0.0
		
		impulse_velocity.x = lerpf(impulse_velocity.x, 0.0, delta * impulse_lerp_speed)
		impulse_velocity.z = lerpf(impulse_velocity.z, 0.0, delta * impulse_lerp_speed)
		
		if jump_input_buffer.is_input_just_pressed() :
			velocity.y = jump_velocity_length  # Add jump
			is_jumping = true
			
			apply_jump_impulse_to_objects()
			
		elif velocity.y <= 0.0 :  # On the floor, jump not pressed
			is_jumping = false
			
	else : # Not on floor
		lerp_speed = air_lerp_speed
		
		velocity.y -= gravity * delta  # Add gravity to kinematic velocity
	
	# Apply friction to horizontal velocity
	velocity.x = lerpf(velocity.x, move_dir.x * speed, delta * lerp_speed)
	velocity.z = lerpf(velocity.z, move_dir.z * speed, delta * lerp_speed)


func get_feet_position() -> Vector3 :
	var feet_pos : Vector3 = global_position
	feet_pos.y -= (collider.shape as CylinderShape3D).height / 2.0  # Cylinder shape is assumed
	#feet_pos.y -= (collider.shape as BoxShape3D).size.y / 2.0
	
	return feet_pos


func apply_kinematic_impulse(impulse : Vector3) -> void :
	impulse_velocity += impulse / kin_mass


func get_collision_below() -> KinematicCollision3D :
	var col : KinematicCollision3D
	col = move_and_collide(Vector3.DOWN * CHECK_FLOOR_DISTANCE, true, SAFE_MARGIN, true)
	
	return col


func apply_push_impulse_to_objects(col : KinematicCollision3D) -> void :
	if not col : return
	
	for i in col.get_collision_count() :
		if col.get_collider(i) is RigidBody3D :
			var rigidbody := col.get_collider(i) as RigidBody3D
			var collision_normal : Vector3 = col.get_normal(i)
			var vertical_factor : float = absf(1 - collision_normal[1])  # How vertical the surface is
			var horizontal_factor : float = absf(collision_normal[1])  # How horizontal the surface is
			
			# Horizontal
			var h_vel : Vector3 = velocity + impulse_velocity
			h_vel.y = 0.0
			
			var impulse_length : float = h_vel.length() * vertical_factor * PUSH_FACTOR
				# The more horizontal the push surface is,
				# the less effective the horizontal push will become.
			impulse_length *= rigidbody.mass / (kin_mass * speed)
				# The impulse length is normalized to achieve smooth pushing.
			var impulse : Vector3 = impulse_length * -collision_normal
			var impulse_pos : Vector3 = col.get_position(i)
			
			# Set impulse height
			if rigidbody.global_position.y <= global_position.y :
				impulse_pos.y = rigidbody.global_position.y
			else :
				impulse_pos.y = global_position.y
			
			# Apply horizontal pushing impulse
			rigidbody.apply_impulse(impulse, impulse_pos - rigidbody.global_position)
			
			# Vertical
			var v_vel := Vector3.ZERO
			v_vel.y = velocity.y + impulse_velocity.y
			
			impulse_length = v_vel.length() * horizontal_factor * PUSH_FACTOR
				# The more vertical the push surface is,
				# the less effective the vertical push will become.
			impulse_length *= rigidbody.mass / (kin_mass * jump_velocity_length)
				# The impulse length is normalized to achieve smooth pushing.
			impulse = impulse_length * -collision_normal
			impulse_pos = col.get_position(i)
			
			# Apply vertical pushing impulse
			rigidbody.apply_impulse(impulse, impulse_pos - rigidbody.global_position)
			
			# Floor impact impulse (vertical only)
			if collision_normal[1] <= MIN_FLOOR_Y : return # Proceed only if on top of rigidbody surface
			
			var v_impact_vel := Vector3(0.0, minf(floor_impact_velocity.y, 0.0), 0.0)
				# Only non-positive y value
			impulse = v_impact_vel * kin_mass * horizontal_factor * IMPACT_FACTOR
			# The more horizontal the impact surface is,
			# the less effective the impact impulse will become.
			impulse_pos = get_feet_position()
			
			# Apply horizontal impact friction impulse
			rigidbody.apply_impulse(impulse, impulse_pos - rigidbody.global_position)


func apply_weight_force_to_objects() -> void :
	var col : KinematicCollision3D
	
	col = get_collision_below()
	
	if not col : return
	
	for i in col.get_collision_count() :
		if col.get_collider(i) is RigidBody3D :
			var rigidbody := col.get_collider(i) as RigidBody3D
			var collision_normal : Vector3 = col.get_normal(i)
			var horizontal_factor : float = absf(collision_normal[1])  # How horizontal the surface is
			
			var h_vel : Vector3 = velocity + impulse_velocity
			h_vel.y = 0.0
			
			var weight_force := -collision_normal * gravity * WEIGHT_FACTOR
			var friction_force : Vector3 = h_vel * FRICTION_FACTOR
			var force : Vector3 = (weight_force + friction_force) * kin_mass * horizontal_factor
			var force_pos : Vector3 = get_feet_position()
			
			# Apply weight and friction force
			rigidbody.apply_force(force, force_pos - rigidbody.global_position)
			
			# Sligtly get pushed from the surface to allow objects to topple over
			move_and_collide(collision_normal * 0.04)


func apply_jump_impulse_to_objects() -> void :
	var col : KinematicCollision3D
	
	col = get_collision_below()
	
	if not col : return
	
	for i in col.get_collision_count() :
		if col.get_collider(i) is RigidBody3D :
			var rigidbody := col.get_collider(i) as RigidBody3D
			var collision_normal : Vector3 = col.get_normal(i)
			var horizontal_factor : float = absf(collision_normal[1])  # How horizontal the surface is
			var impulse := Vector3.DOWN * jump_velocity_length * kin_mass * horizontal_factor * JUMP_FACTOR
			var impulse_pos : Vector3 = get_feet_position()
			
			rigidbody.apply_impulse(impulse, impulse_pos - rigidbody.global_position)


# Rigid bodies are handled differently than moving platforms, because
# you can get unreasonably large velocites on top of them.
func modify_velocity_on_rigid_bodies(col : KinematicCollision3D) -> void :
	if col and col.get_collider() is RigidBody3D :
		var rigidbody := col.get_collider() as RigidBody3D
		var collision_normal : Vector3 = col.get_normal()
		
		if collision_normal[1] < MIN_FLOOR_Y : return
		
		var horizontal_factor : float = absf(collision_normal[1])  # How horizontal the surface is
		
		# The platform velocity is applied with friction
		var weight : float = ground_lerp_speed * get_physics_process_delta_time()
		platform_velocity = platform_velocity.lerp(rigidbody.linear_velocity * horizontal_factor, weight)
		
		# Limit velocity
		platform_velocity.limit_length(RB_VEL_LIMIT)


func limit_horizontal_velocity_on_rigid_body(rigidbody_mass : float) -> void :
	var h_vel : Vector3 = velocity
	h_vel.y = 0.0
	
	# The horizontal velocity is limited based on object mass and player speed
	h_vel = h_vel.limit_length( (1.0 / rigidbody_mass) * kin_mass * speed )
	velocity.x = h_vel.x
	velocity.z = h_vel.z
	
	# Decrease impulse velocity
	var h_imp_vel : Vector3 = impulse_velocity
	impulse_velocity.y = 0.0
	
	var lerp_weight : float = (rigidbody_mass / kin_mass) * IMPULSE_CHANGE_FACTOR
	lerp_weight = minf(1.0 , lerp_weight)  # Make sure not to change direction
	
	h_imp_vel = h_imp_vel.lerp(Vector3.ZERO, lerp_weight)
	
	impulse_velocity.x = h_imp_vel.x
	impulse_velocity.z = h_imp_vel.z


func limit_vertical_velocity_on_rigid_body(rigidbody_mass : float) -> void :
	var lerp_weight : float = (rigidbody_mass / kin_mass) * IMPULSE_CHANGE_FACTOR
	lerp_weight = minf(1.0, lerp_weight)  # Make sure not to change direction
	
	velocity.y = lerpf(velocity.y, 0.0, lerp_weight)
	impulse_velocity.y = lerpf(impulse_velocity.y, 0.0, lerp_weight)


func is_on_floor() -> bool :
	var col : KinematicCollision3D
	
	# If CHECK_FLOOR_DISTANCE is too small, this won't work.
	col = move_and_collide(Vector3.DOWN * CHECK_FLOOR_DISTANCE, true, SAFE_MARGIN, true)
	if col and col.get_normal()[1] > MIN_FLOOR_Y :
		return true
	
	return false


func get_floor_normal() -> Vector3 :
	var col : KinematicCollision3D
	var floor_normal := Vector3.ZERO
	
	col = move_and_collide(Vector3.DOWN * CHECK_FLOOR_DISTANCE, true, SAFE_MARGIN, true)
	if col : floor_normal = col.get_normal()
	
	return floor_normal


func snap_to_floor(snap_distance : float) -> void :
	var down := Vector3.DOWN * snap_distance
	var motion_tester := MotionTester.new()
	
	motion_tester.test_motion(self, global_transform, down, SAFE_MARGIN, true)
	
	if motion_tester.hit and motion_tester.normal[1] > MIN_FLOOR_Y :
		var travel : Vector3 = motion_tester.travel
		# Remove horizontal component
		travel.x = 0.0
		travel.z = 0.0
		
		# The safe margin has to be large enough when snapping, to avoid any sliding.
		move_and_collide(travel, false, SNAP_SAFE_MARGIN , true)


func set_platform_velocity() -> void :
	var col : KinematicCollision3D
	
	col = move_and_collide(Vector3.DOWN * CHECK_FLOOR_DISTANCE, true, SAFE_MARGIN, true, 5)
	
	if col and col.get_collider() is RigidBody3D :
		modify_velocity_on_rigid_bodies(col)
		return
	 
	if not col or not col.get_collider() is AnimatableBody3D :
		if add_velocity_on_leave  :  # Just left platform
			platform_velocity.y = maxf(0.0, platform_velocity.y)  # Make sure it's not negative
			impulse_velocity += platform_velocity
			
		platform_velocity = Vector3.ZERO
		return
	
	var platform_collider := col.get_collider() as PhysicsBody3D
	
	if col.get_normal()[1] >= MIN_FLOOR_Y :
		var platform_body_state := PhysicsServer3D.body_get_direct_state(platform_collider)
		
		var pos : Vector3 = global_position - platform_collider.global_position
		
		var plat_vel_goal : Vector3 = platform_body_state.get_velocity_at_local_position(pos)
		
		platform_velocity.x = plat_vel_goal.x
		platform_velocity.z = plat_vel_goal.z
		
		# The vertical component is lerped to allow platforms to launch the player up
		var weight : float = ground_lerp_speed * get_physics_process_delta_time()
		platform_velocity.y = lerpf(platform_velocity.y,plat_vel_goal.y, weight)
		
	else : platform_velocity = Vector3.ZERO
	
	return


# The 'move and slide' algorithm in it's most basic form. This method is unused.
func move_and_slide_basic(motion : Vector3) -> void :
	var col : KinematicCollision3D
	
	# Commit to move
	col = move_and_collide(motion, false, SAFE_MARGIN)
	
	# Algorithm exit point
	if not col or slide_depth > MAX_SLIDES : return
	
	var collision_normal : Vector3 = col.get_normal()
	
	# Set new motion
	var remaining_motion : Vector3 = col.get_remainder()
	motion = remaining_motion.slide(collision_normal)
	
	# Iterate
	slide_depth += 1
	move_and_slide_basic(motion)


func move_and_slide(motion : Vector3) -> void :
	var col : KinematicCollision3D
	
	# Commit to move
	col = move_and_collide(motion, false, SAFE_MARGIN, false, 4)
	
	# Snap to floor on slopes and stairs
	var just_left_ground : bool = motion.y <= 0.0 and motion.y > -0.01  # <- Magic number :(
	if just_left_ground and not is_jumping :
		snap_to_floor(STEP_HEIGHT)
	
	# Algorithm exit point
	if not col or slide_depth > MAX_SLIDES : return
	
	# Variables determined by the collision
	var collision_normal : Vector3 = col.get_normal()
	var collision_position : Vector3 = col.get_position()
	var remaining_motion : Vector3 = col.get_remainder()
	var collided_with_wall := collision_normal[1] < MIN_FLOOR_Y and collision_normal[1] > MAX_CEILING_Y
	var collided_with_ceiling := collision_normal[1] < MAX_CEILING_Y  # Between MAX_CEILING_Y and -1
	var collided_with_floor := collision_normal[1] > MIN_FLOOR_Y  # Between MIN_FLOOR_Y and 1
	
	# Store rigid body collision
	var rigidbody : RigidBody3D = null
	for i in col.get_collision_count():
		if col.get_collider(i) is RigidBody3D:
			rigidbody = col.get_collider(i) as RigidBody3D
			last_rigid_body_collision = col
			break
	
	# Handle stepping
	# Ledge detection
	var raycast := RayCast.new()
	var space_state : PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	var col_pos_offset : Vector3 = collision_position - collision_normal * LEDGE_DETECT_OFFSET
		# If the collision point is not offsetted slightly inwards the ledge,
		# the raycast will miss the surface in some cases.
	var ray_origin : Vector3 = col_pos_offset + Vector3.UP * STEP_HEIGHT
	var ray_dest : Vector3 = col_pos_offset
	
	raycast.intersect(ray_origin, ray_dest, self, collision_mask, space_state)
	
	if is_on_floor() and collided_with_wall and raycast.hit :  # Ledge detected
		var ledge_pos : Vector3 = raycast.endpos
		var feet_pos : Vector3 = get_feet_position()
		var collision_height : float = ledge_pos.y - feet_pos.y
		
		if collision_height > 0.0 and collision_height <= STEP_HEIGHT :
			# Check if surface can be stepped on, by estimating the step position
			var up := Vector3.UP * collision_height
			var motion_dir : Vector3 = remaining_motion.normalized()
			var motion_tester := MotionTester.new()
			
			# UP
			var test_transform := global_transform
			motion_tester.test_motion(self, test_transform, up, SAFE_MARGIN, false)
			var hit_ceiling : bool = motion_tester.hit  # Makes sure there is enough space to move up.
			
			# FORWARD
			test_transform.origin = motion_tester.endpos
			# It's important to extend the forward motion by some amount! 
			# (Because near ledges the reported normal is often incorrect.)
			var forward : Vector3 = remaining_motion + motion_dir * 0.1  # <- Magic number :(
			motion_tester.test_motion(self, test_transform, forward, SAFE_MARGIN, false)
			
			# SLIDE (if needed)
			for i in range(3) :
				if motion_tester.hit and motion_tester.normal[1] < MIN_FLOOR_Y :
					test_transform.origin = motion_tester.endpos
					var slide_motion : Vector3 = motion_tester.remainder.slide(motion_tester.normal)
					motion_tester.test_motion(self, test_transform, slide_motion, SAFE_MARGIN, false)
			
			# DOWN
			test_transform.origin = motion_tester.endpos
			var down := Vector3.DOWN * CHECK_FLOOR_DISTANCE
			motion_tester.test_motion(self, test_transform, down, SAFE_MARGIN, true)
			
			if motion_tester.hit and motion_tester.normal[1] > MIN_FLOOR_Y and not hit_ceiling :
				move_and_collide(up, false, SAFE_MARGIN)
				
				# The motion is extended by the collision shape's margin to make sure
				# the step is fully stepped, so as not to fall down at lower velocities.
				# The motion is aslo extended by the platform motion to avoid falling back
				# after stepping on moving platforms.
				motion = remaining_motion + motion_dir * COLLISION_SHAPE_MARGIN
				
				# Iterate
				slide_depth += 1
				move_and_slide(motion)
				return
	
	# Check if stuck on the foot of a slope
	# If yes, move up a bit and don't modify the motion or velocity!!!
	if collided_with_wall :
		var motion_tester := MotionTester.new()
		
		var up := Vector3.UP * COLLISION_SHAPE_MARGIN
		
		var test_motion : Vector3 = motion
		test_motion.y = 0.0  # Remove vertical component
		test_motion = test_motion.normalized() * 0.1  # Set the length to an arbitrary number
		
		var test_transform := Transform3D(basis, global_position + up)
		
		motion_tester.test_motion(self, test_transform, test_motion, SAFE_MARGIN, false)
		
		if motion_tester.hit and motion_tester.normal[1] > MIN_FLOOR_Y :
			move_and_collide(up, false, SAFE_MARGIN)
			
			# Iterate
			slide_depth += 1
			move_and_slide(motion)
			return
	
	# New horizontal motion
	var h_motion : Vector3 = remaining_motion
	h_motion.y = 0.0
	
	if collided_with_wall :
		# Project horizontal motion on a fully vertical wall's normal, no matter the steepness
		var wall_normal : Vector3 = collision_normal
		wall_normal.y = 0.0
		wall_normal = wall_normal.normalized()
		
		h_motion = h_motion.slide(wall_normal)
		
		if rigidbody :  # Collided with a rigid body
			limit_horizontal_velocity_on_rigid_body(rigidbody.mass)
		
		else :  # Static collision
			velocity = velocity.slide(wall_normal)  # Clip velocity
			impulse_velocity = impulse_velocity.slide(wall_normal)  # Clip impulse velocity
			platform_velocity = platform_velocity.slide(wall_normal) # Clip platform velocity
		
	else :
		h_motion = h_motion.slide(collision_normal)
		
	# New vertical motion
	var v_motion : Vector3 = Vector3.ZERO
	v_motion.y = remaining_motion.y
	
	if collided_with_floor :
		floor_impact_velocity = velocity + impulse_velocity + linear_velocity  # Store impact velocity.
		# (Fall damage could be applied here.)
		
		# These help to stop sliding on slope
		v_motion.y = 0.0
		if not is_jumping : velocity.y = 0.0  # Clip vertical velocity
		
	else :
		v_motion = v_motion.slide(collision_normal)
	
	if collided_with_ceiling :
		if rigidbody :  # Collided with a rigid body
			limit_vertical_velocity_on_rigid_body(rigidbody.mass)
			limit_horizontal_velocity_on_rigid_body(rigidbody.mass)
		
		else :  # Static collision
			velocity = velocity.slide(collision_normal)  # Clip velocity
			impulse_velocity = impulse_velocity.slide(collision_normal)  # Clip impulse velocity
			platform_velocity = platform_velocity.slide(collision_normal) # Clip platform velocity
	
	# Sum motions
	motion = h_motion + v_motion
	
	# Iterate
	slide_depth += 1
	move_and_slide(motion)


func _physics_process(delta : float) -> void :	
	var input_dir : Vector3 = get_input_direction()
	
	speed = run_speed
	
	if Input.is_action_just_pressed("impulse") :
		apply_kinematic_impulse(-head.transform.basis.z * 3.0)  # Dash forward
	
	set_initial_velocity(input_dir, delta)  # Jump impulse is also applied here
	
	set_platform_velocity()
	
	# Do the 'move and slide'
	var movement_motion : Vector3 = velocity * delta
	var impulse_motion : Vector3 = impulse_velocity * delta
	var platform_motion : Vector3 = platform_velocity * delta
	
	floor_impact_velocity = Vector3.ZERO
	last_rigid_body_collision = null
	slide_depth = 0
	
	move_and_slide(movement_motion + impulse_motion + platform_motion)
	
	# The real move velocity is the modified velocity. For things like camera tilt.
	real_move_velocity = velocity
	
	# Kinematic physics interaction
	apply_push_impulse_to_objects(last_rigid_body_collision)
	
	apply_weight_force_to_objects()


# Physics state control
func _integrate_forces(state : PhysicsDirectBodyState3D) -> void :
	if state.linear_velocity == Vector3.ZERO : return
	
	if is_on_floor() :
		state.linear_velocity.y = maxf(0.0, state.linear_velocity.y)
		
		if platform_velocity.is_zero_approx() :  # Not on a moving platform
			# Apply friction
			var weight : float = state.step * impulse_lerp_speed
			state.linear_velocity = state.linear_velocity.lerp(Vector3.ZERO, weight)
			
		else  : state.linear_velocity = Vector3.ZERO
