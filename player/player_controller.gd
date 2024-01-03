extends PhysicsBody3D
class_name Player

##############
# PROPERTIES #
##############

@export_group("General moving")
@export var run_speed : float = 8.0
@export var ground_lerp_speed : float = 10.0
@export var air_lerp_speed : float = 5.0

@export_group("Jumping")
@export var jump_velocity : float = 6.0


#########
# NODES #
#########

@onready var head : Node3D = $PivotY as Node3D
@onready var collider : CollisionShape3D = $CollisionShape as CollisionShape3D
@onready var jump_input_buffer : InputBuffer = $JumpInputBuffer as InputBuffer

#############
# CONSTANTS #
#############

const MAX_SLIDES : int = 5                   # Default: 5
const CHECK_FLOOR_DISTANCE : float = 0.08    # Default: 0.08
const SAFE_MARGIN : float = 0.001            # Default: 0.001
const SNAP_SAFE_MARGIN : float = 0.003       # Default 0.003
const MIN_FLOOR_Y : float = 0.7              # Default: 0.7
const MAX_CEILING_Y : float = -0.3           # Default: -0.3
const STEP_HEIGHT : float = 0.25             # Default: 0.25
const COLLISION_SHAPE_MARGIN : float = 0.04  # Default: 0.04
const LEDGE_DETECT_OFFSET : float = 0.003    # Default 0.003

# -1 < MAX_CEILING_Y < MIN_FLOOR_Y < 1
# |CEILING|        |WALL|       |FLOOR|

#############
# VARIABLES #
#############

var velocity := Vector3.ZERO
var actual_velocity := Vector3.ZERO
var gravity : float = ProjectSettings.get_setting("physics/3d/default_gravity")
var slide_depth : int = 0

var is_jumping : bool = false

#############
# FUNCTIONS #
#############

func _ready() -> void:
	if Engine.is_editor_hint():
		return
	
	rotation = Vector3.ZERO # Important! (Since only the head rotates.)


func get_input_direction() -> Vector3:
	var input_dir_vec2 : Vector2 = Input.get_vector("left", "right", "up", "down")
	var input_dir_vec3 : Vector3 = head.transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	return input_dir_vec3


func set_initial_velocity(move_dir : Vector3, speed : float, delta: float) -> void:
	var lerp_speed : float = ground_lerp_speed
	
	# Vertical Velocity
	if is_on_floor():
		lerp_speed = ground_lerp_speed
		
		if jump_input_buffer.is_input_just_pressed():
			velocity.y = jump_velocity # Add jump
			is_jumping = true
		elif velocity.y <= 0.0: # On the floor, jump not pressed
			is_jumping = false
			
	else : # Not on floor
		lerp_speed = air_lerp_speed
		velocity.y -= gravity * delta # Add gravity
	
	# Horizontal velocity
	velocity.x = lerp(velocity.x, move_dir.x * speed, delta * lerp_speed)
	velocity.z = lerp(velocity.z, move_dir.z * speed, delta * lerp_speed)


func is_on_floor() -> bool:
	var col : KinematicCollision3D
	
	# If CHECK_FLOOR_DISTANCE is too small, this doesn't work.
	col = move_and_collide(Vector3.DOWN * CHECK_FLOOR_DISTANCE, true, SAFE_MARGIN, true)
	if col and col.get_normal()[1] > MIN_FLOOR_Y:
		return true
		
	return false


func get_floor_normal() -> Vector3:
	var col : KinematicCollision3D
	var floor_normal := Vector3.ZERO
	
	col = move_and_collide(Vector3.DOWN * CHECK_FLOOR_DISTANCE, true, SAFE_MARGIN, true)
	if col : floor_normal = col.get_normal()
	
	return floor_normal


func snap_to_floor(snap_distance : float) -> void:
	var down := Vector3.DOWN * snap_distance
	var motion_tester := MotionTester.new()
	
	motion_tester.test_motion(self, global_transform, down, SAFE_MARGIN, true)
	
	if motion_tester.hit and motion_tester.normal[1] > MIN_FLOOR_Y:
		var travel : Vector3 = motion_tester.travel
		# Remove horizontal component
		travel.x = 0.0
		travel.z = 0.0
		# The safe margin has to be large enough when snapping, to avoid any sliding.
		move_and_collide(travel, false, SNAP_SAFE_MARGIN , true)


# The 'move and slide' algorithm in it's most basic form.
func move_and_slide_basic(motion : Vector3) -> void:
	var col : KinematicCollision3D
	
	# Commit to move
	col = move_and_collide(motion, false, SAFE_MARGIN)
	
	# Algorithm exit point
	if not col or slide_depth > MAX_SLIDES:
		return
	
	var collision_normal : Vector3 = col.get_normal()
	
	# Set new motion
	var remaining_motion : Vector3 = col.get_remainder()
	motion = remaining_motion.slide(collision_normal)
	
	# Iterate
	slide_depth += 1
	move_and_slide_basic(motion)


func move_and_slide(motion : Vector3) -> void:
	var col : KinematicCollision3D
	
	# Commit to move
	col = move_and_collide(motion, false, SAFE_MARGIN)
	
	# Snap to floor on slopes and stairs
	var just_left_ground : bool = motion.y <= 0.0 and motion.y > -0.01 # <- Magic number :(
	if just_left_ground and not is_jumping:
		snap_to_floor(STEP_HEIGHT)
	
	# Algorithm exit point
	if not col or slide_depth > MAX_SLIDES:
		return
	
	# Variables determined by the collision
	var collision_normal : Vector3 = col.get_normal()
	var collision_position : Vector3 = col.get_position()
	var remaining_motion : Vector3 = col.get_remainder()
	var collided_with_wall := collision_normal[1] < MIN_FLOOR_Y and collision_normal[1] > MAX_CEILING_Y
	var collided_with_ceiling := collision_normal[1] < MAX_CEILING_Y # Between MAX_CEILING_Y and -1
	var collided_with_floor := collision_normal[1] > MIN_FLOOR_Y # Between MIN_FLOOR_Y and 1

	# Handle stepping
	# Ledge detection
	var raycast := RayCast.new()
	var space_state : PhysicsDirectSpaceState3D = get_world_3d().direct_space_state
	# If the collision point is not offsetted slightly inwards the ledge,
	# the raycast will miss the surface in some cases.
	var col_pos_offset : Vector3 = collision_position - collision_normal * LEDGE_DETECT_OFFSET
	var ray_origin = col_pos_offset + Vector3.UP * STEP_HEIGHT
	var ray_dest = col_pos_offset

	raycast.intersect(ray_origin, ray_dest, self, collision_mask, space_state)

	if is_on_floor() and collided_with_wall and raycast.hit: # Ledge detected
		var ledge_pos = raycast.endpos
	
		var feet_pos : Vector3 = global_position
		feet_pos.y -= collider.shape.height / 2.0
	
		var collision_height : float = ledge_pos.y - feet_pos.y
	
		if collision_height > 0.0 and collision_height < STEP_HEIGHT:
			# Check if surface can be stepped on, by estimating the step position
			var up := Vector3.UP * collision_height
			var motion_dir : Vector3 = remaining_motion.normalized()
			var motion_tester := MotionTester.new()
			
			# UP
			var test_transform := global_transform
			motion_tester.test_motion(self, test_transform, up, SAFE_MARGIN, false)
			var hit_ceiling : bool = motion_tester.hit # Makes sure there is enough space to move up.
			
			# FORWARD
			test_transform.origin = motion_tester.endpos
			# Important to extend the forward motion by some amount! 
			# (Because near ledges the reported normal is often incorrect.)
			var forward : Vector3 = remaining_motion + motion_dir * 0.1 # <- Magic number :(
			motion_tester.test_motion(self, test_transform, forward, SAFE_MARGIN, false)
			
			# SLIDE (if needed)
			if motion_tester.hit:
				test_transform.origin = motion_tester.endpos
				var slide_motion : Vector3 = motion_tester.remainder.slide(motion_tester.normal)
				motion_tester.test_motion(self, test_transform, slide_motion, SAFE_MARGIN, false)
			
			# DOWN
			test_transform.origin = motion_tester.endpos
			var down := Vector3.DOWN * CHECK_FLOOR_DISTANCE
			motion_tester.test_motion(self, test_transform, down, SAFE_MARGIN, true)
			
			if motion_tester.hit and motion_tester.normal[1] > MIN_FLOOR_Y and not hit_ceiling:
				move_and_collide(up, false, SAFE_MARGIN)
				
				# The motion is extended by the collision shape's margin to make sure
				# the step is fully stepped, so as not to fall down at lower velocities.
				motion = remaining_motion + motion_dir * COLLISION_SHAPE_MARGIN
				motion.y = 0.0
				
				# Iterate
				slide_depth += 1
				move_and_slide(motion)
				return
	
	# Check if stuck on the foot of a slope
	# If yes, move up a bit and don't modify the motion or velocity!!!
	if collided_with_wall:
		var motion_tester := MotionTester.new()
		var up := Vector3.UP * COLLISION_SHAPE_MARGIN
		var test_motion : Vector3 = motion
		test_motion.y = 0 # Remove vertical component
		test_motion = test_motion.normalized() # Extend the test motion to unit length
		var test_transform := Transform3D(basis, global_position + up)
		
		motion_tester.test_motion(self, test_transform, test_motion, SAFE_MARGIN, false)
		
		if motion_tester.hit and motion_tester.normal[1] > MIN_FLOOR_Y:
			move_and_collide(up, false, SAFE_MARGIN)
			
			# Iterate
			slide_depth += 1
			move_and_slide(motion)
			return
	
	# New horizontal motion
	var h_motion : Vector3 = remaining_motion
	h_motion.y = 0.0
	
	if collided_with_wall:
		# Project horizontal motion on a fully vertical wall's normal, no matter the steepness
		var wall_normal : Vector3 = collision_normal
		wall_normal.y = 0.0
		wall_normal = wall_normal.normalized()
		
		h_motion = h_motion.slide(wall_normal)
		velocity = velocity.slide(wall_normal) # Clip velocity
	else :
		h_motion = h_motion.slide(collision_normal)
	
	# New vertical motion
	var v_motion : Vector3 = Vector3.ZERO
	v_motion.y = remaining_motion.y
	
	if collided_with_floor :
		# Stop sliding on slope
		v_motion.y = 0.0
		velocity.y = 0.0 # Clip vertical velocity
	else :
		v_motion = v_motion.slide(collision_normal)
	
	if collided_with_ceiling:
		velocity = velocity.slide(collision_normal) # Clip velocity
		
	# Sum motions
	motion = h_motion + v_motion
	
	# Iterate
	slide_depth += 1
	move_and_slide(motion)


func _physics_process(delta : float) -> void:
	var input_dir : Vector3 = get_input_direction()
	var speed : float = run_speed
	
	set_initial_velocity(input_dir, speed, delta)
	
	slide_depth = 0
	move_and_slide(velocity * delta)
	
	# The actual velocity is the modified velocity.
	actual_velocity = velocity
