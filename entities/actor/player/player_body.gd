
@tool
extends IKCC
class_name PlayerBody




#==================================================================================================
# --------------
# | PROPERTIES |
# --------------


@export_group("Grounded movement")
@export var max_ground_speed           : float = 25.0
@export var _max_air_speed             : float = 1.0
@export var _ground_acceleration       : float = 40.0
@export var _air_acceleration          : float = 7.0
@export var ground_stop_friction_speed : float = 7.0
@export var ground_move_friction_speed : float = 5.0
@export var air_stop_friction_speed    : float = 0.0
@export var air_move_friction_speed    : float = 0.0
@export var constantly_apply_gravity   : bool = false :
	set(value) :
		constantly_apply_gravity = value
		floor_stop_on_slope = not constantly_apply_gravity
@export_subgroup("Jumping")
@export var jump_impulse_length : float = 7.0
@export var coyote_time_length  : int = 10


@export_group("Flying movement")
@export var f_air_stop_friction_speed : float = 7.0
@export var f_air_move_friction_speed : float = 5.0
@export var max_fly_speed             : float = 25.0
@export var _fly_acceleration         : float = 40.0
@export var faster_fly_acceleration   : float = 80.0




#==================================================================================================
# -------------------
# | NODE REFERENCES |
# -------------------

@export_group("")
@export var head                    : PlayerCamera
@export var jump_input_buffer       : InputBuffer




#==================================================================================================
# --------------
# | VARIABLES  |
# --------------

var movement_disabled : bool
var coyote_timer : int





#==================================================================================================
# -----------
# | METHODS |
# -----------


## _ready
func _ready() -> void :
	motion_mode = MotionMode.GROUNDED




## _physics_process
func _physics_process(p_delta_t : float) -> void :
	if Engine.is_editor_hint(): return
	
	# Update coyote timer
	if is_on_floor :
		coyote_timer = coyote_time_length
	elif coyote_timer:
		coyote_timer -= 1
	
	set_input_velocity(p_delta_t)
	
	move_and_slide()




## Sets the input velocity for 'move_and_slide'
func set_input_velocity(p_delta_t : float) -> void :
	# Determine input and floor directions
	var input_dir : Vector3 = get_input_direction() if not movement_disabled else Vector3.ZERO
	
	# Determine wished speed and acceleration
	if motion_mode == MotionMode.GROUNDED :
		if is_on_floor :
			# NOTE: At this point our velocity is parallel to the ground, because
			# floor snapping has ensured that.
			
			var ground_accel : float = _ground_acceleration
			# Make the acceleration direction parallel to the floor
			var ground_accel_dir : Vector3 = input_dir.slide(current_floor_normal).normalized()
			
			# Floor is not slippery by default
			constantly_apply_gravity = false
			floor_stop_on_slope = true
			
			# Check floor material properties, and apply changes if needed
			var ground_friction_factor : float = 1.0
			if current_floor_collider_encoded :
				var floor_collider_object : Object = instance_from_id(current_floor_collider_encoded.object_id)
				var geo_material := floor_collider_object.get("geo_material") as GeoMaterial
				if geo_material :
					if geo_material.slippery :
						constantly_apply_gravity = true
						floor_stop_on_slope = false
					if geo_material.override_default_friction :
						ground_friction_factor = geo_material.friction_speed
						ground_accel *= ground_friction_factor
			
			if not constantly_apply_gravity :
				# Apply gravity in the floor direction to accelarate/decelerate on slopes
				var gravity_vector : Vector3 = get_gravity() * p_delta_t
				velocity += gravity_vector.dot(ground_accel_dir) * ground_accel_dir
			else :
				# Apply gravity
				velocity += get_gravity() * p_delta_t
			
			# Apply ground friction
			var ground_friction_speed : float = (
				ground_stop_friction_speed if input_dir == Vector3.ZERO
				else ground_move_friction_speed
			)
			ground_friction_speed *= ground_friction_factor
			velocity = apply_friction(velocity, ground_friction_speed, p_delta_t)
			
			# Apply ground acceleration
			velocity = accelerate(
				velocity, ground_accel, ground_accel_dir, max_ground_speed, p_delta_t
			)
		else :  # Air movement
			# Apply gravity
			velocity += get_gravity() * p_delta_t
			
			# Seperate vertical and horizontal velocity components
			var vertical_velocity : Vector3 = velocity.project(up_direction)
			var horizontal_velocity : Vector3 = velocity - vertical_velocity
			
			# Apply air friction
			var air_friction_speed : float = (
				air_stop_friction_speed if input_dir == Vector3.ZERO
				else air_move_friction_speed
			)
			horizontal_velocity = apply_friction(horizontal_velocity, air_friction_speed, p_delta_t)
			
			# Apply air acceleration
			horizontal_velocity = accelerate(
				horizontal_velocity, _air_acceleration, input_dir, _max_air_speed, p_delta_t
			)
			
			# Combine horizontal and vertical components
			velocity = horizontal_velocity + vertical_velocity
		
		# Handle jumping
		if coyote_timer and jump_input_buffer.is_input_just_pressed() and not movement_disabled :
			# Make sure veolcity does not point downwards before jumping
			velocity -= minf(0.0, velocity.dot(up_direction)) * up_direction
			velocity += jump_impulse_length * up_direction
	else :  # Floating mode
		# Apply air friction
		var air_friction_speed : float = (
			f_air_stop_friction_speed if input_dir == Vector3.ZERO
			else f_air_move_friction_speed
		)
		velocity = apply_friction(velocity, air_friction_speed, p_delta_t)
		
		# Apply fly acceleration
		var fly_accel_speed : float = (
			_fly_acceleration if not Input.is_action_pressed("run")
			else faster_fly_acceleration
		)
		velocity = accelerate(velocity, fly_accel_speed, input_dir, max_fly_speed, p_delta_t)




## Returns the input direction in Vector3
func get_input_direction() -> Vector3 :
	var input_dir_vec2 : Vector2 = Input.get_vector("left", "right", "up", "down")
	var input_dir_vec3 : Vector3 = head.transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	
	if motion_mode == MotionMode.GROUNDED :
		input_dir_vec3 = head.transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	else :  # Floating mode
		input_dir_vec3 = head.camera.global_transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	
	return input_dir_vec3




## Applies friction to the given velocity
static func apply_friction(
	p_velocity       : Vector3,
	p_friction_speed : float,
	p_delta_t        : float
	) -> Vector3 :
	if is_zero_approx(p_friction_speed) :
		return p_velocity
	
	var friction : Vector3 = -p_velocity * p_friction_speed * p_delta_t
	var new_velocity : Vector3 = p_velocity + friction
	
	return new_velocity




## Applies acceleration to the given velocity
static func accelerate(
	p_velocity  : Vector3,
	p_accel     : float,
	p_accel_dir : Vector3,
	p_max_speed : float,
	p_delta_t   : float
	) -> Vector3 :
	if p_accel_dir.is_zero_approx() :
		return p_velocity
	
	# Add acceleration
	var accel_speed  : float = p_accel * p_delta_t
	var new_velocity : Vector3 = p_velocity + p_accel_dir * accel_speed
	
	# Liimt length
	var max_speed : float = maxf(p_max_speed, p_velocity.length())
	new_velocity = new_velocity.limit_length(max_speed)
	
	return new_velocity
