
@tool
extends IKCC
class_name PlayerBody




#==================================================================================================
# --------------
# | PROPERTIES |
# --------------


@export_group("Grounded movement")
@export var max_ground_speed                   : float = 25.0
@export var _max_air_speed                      : float = 1.0
@export var _ground_acceleration               : float = 40.0
@export var _air_acceleration                   : float = 7.0
@export var _default_floor_stop_friction_speed : float = 7.0
@export var default_floor_move_friction_speed  : float = 5.0
@export var air_stop_friction_speed            : float = 0.0
@export var air_move_friction_speed            : float = 0.0
@export var constantly_apply_gravity           : bool = false :
	set(value) :
		constantly_apply_gravity = value
		floor_stop_on_slope = not constantly_apply_gravity
@export_subgroup("Jumping")
@export var jump_impulse_length : float = 7.0
@export var coyote_time_length  : int = 10


@export_group("Flying movement")
@export var f_air_stop_friction_speed : float = 7.0
@export var f_air_move_friction_speed : float = 5.0
@export var fly_speed                  : float = 10.0
@export var _faster_fly_speed          : float = 20.0
@export var _fly_acceleration          : float = 40.0




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

var delta_t : float    # Physics process frame time
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
	
	delta_t = p_delta_t  # Set global physics frame time
	
	if is_on_floor :
		coyote_timer = coyote_time_length
	elif coyote_timer:
		coyote_timer -= 1
	
	set_input_velocity()
	
	move_and_slide()




## Sets the input velocity for 'move_and_slide'
func set_input_velocity() -> void :
	# Determine input and floor directions
	var wish_dir   : Vector3 = get_input_direction() if not movement_disabled else Vector3.ZERO
	var vertical_dir : Vector3 = current_floor_normal if is_on_floor else up_direction
	
	# Apply gravity if needed
	if motion_mode == MotionMode.GROUNDED and (not is_on_floor or constantly_apply_gravity) :
		velocity += get_gravity() * delta_t
	
	# Determine wished speed and acceleration
	var max_horizontal_speed  : float = 0.0
	var accel                 : float = 0.0
	var friction_factor       : float = 1.0
	if motion_mode == MotionMode.GROUNDED :
		if is_on_floor :
			max_horizontal_speed = max_ground_speed
			accel = _ground_acceleration
			
			# Floor is not slippery by default
			constantly_apply_gravity = false
			floor_stop_on_slope = true
			
			# Check floor material properties, and apply changes if needed
			if current_floor_collider_encoded :
				var floor_collider_object : Object = instance_from_id(current_floor_collider_encoded.object_id)
				var geo_material := floor_collider_object.get("geo_material") as GeoMaterial
				if geo_material :
					if geo_material.slippery :
						constantly_apply_gravity = true
						floor_stop_on_slope = false
					if geo_material.override_default_friction :
						friction_factor = geo_material.friction_speed
						accel *= friction_factor
			
			# Make the acceleration direction parallel to the floor
			wish_dir = wish_dir.slide(current_floor_normal).normalized()
			# Apply gravity in the floor direction to accelarate/decelerate on slopes
			if not constantly_apply_gravity :
				var gravity_vector : Vector3 = get_gravity() * delta_t
				velocity += gravity_vector.dot(wish_dir) * wish_dir
		else :
			max_horizontal_speed = _max_air_speed
			accel = _air_acceleration
	else :  # Floating mode
		max_horizontal_speed = fly_speed if not Input.is_action_pressed("run") else _faster_fly_speed
		accel = _fly_acceleration
	
	# Apply friction
	apply_friction(not wish_dir == Vector3.ZERO, vertical_dir, friction_factor)
	
	# Apply acceleration
	accelerate(accel, wish_dir, max_horizontal_speed, vertical_dir)
	
	# Handle jumping
	if motion_mode == MotionMode.GROUNDED and coyote_timer and jump_input_buffer.is_input_just_pressed() and not movement_disabled :
		# Make sure veolcity does not point downwards before jumping
		velocity -= minf(0.0, velocity.dot(up_direction)) * up_direction
		velocity += jump_impulse_length * up_direction




## Applies damping to velocity
func apply_friction(p_player_is_moving : bool, p_vertical_dir : Vector3, p_friction_factor : float) -> void :
	# Floating mode air friction
	if motion_mode == MotionMode.FLOATING and not p_player_is_moving :
		var air_friction_speed : float = f_air_move_friction_speed if p_player_is_moving else f_air_stop_friction_speed
		velocity -= velocity * air_friction_speed * delta_t
		return
	
	# Grounded mode floor and air friction
	var friction_speed : float
	if is_on_floor :
		friction_speed = _default_floor_stop_friction_speed if not p_player_is_moving else default_floor_move_friction_speed
	else :
		friction_speed = air_stop_friction_speed if not p_player_is_moving else air_move_friction_speed
	
	var vertical_velocity : Vector3 = velocity.project(p_vertical_dir)
	var horizontal_velocity : Vector3 = velocity - vertical_velocity
	var friction : Vector3 = horizontal_velocity * friction_speed * delta_t * p_friction_factor
	var new_horizontal_velocity : Vector3 = horizontal_velocity - friction
	
	velocity = new_horizontal_velocity + vertical_velocity




## Returns the input direction in Vector3
func get_input_direction() -> Vector3 :
	var input_dir_vec2 : Vector2 = Input.get_vector("left", "right", "up", "down")
	var input_dir_vec3 : Vector3 = head.transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	
	if motion_mode == MotionMode.GROUNDED :
		input_dir_vec3 = head.transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	else :
		input_dir_vec3 = head.camera.global_transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	
	return input_dir_vec3





## Accelerates the body based on player input
func accelerate(p_accel : float, p_wish_dir : Vector3, p_max_speed : float, p_vertical_dir : Vector3) -> void :
	if p_wish_dir.is_zero_approx() :
		return
	
	var accel_speed : float = p_accel * delta_t
	
	if motion_mode == MotionMode.GROUNDED :
		var vertical_velocity : Vector3 = velocity.project(p_vertical_dir)
		var horizontal_velocity : Vector3 = velocity - vertical_velocity
		var horizontal_velocity_length : float = horizontal_velocity.length()
		var new_horizontal_velocity : Vector3 = horizontal_velocity
		
		# Apply acceleration
		new_horizontal_velocity += p_wish_dir * accel_speed
		
		# Limit horizontal speed
		var max_speed : float = maxf(p_max_speed, horizontal_velocity_length)
		new_horizontal_velocity = new_horizontal_velocity.limit_length(max_speed)
		
		# Add horizontal and vertical components together
		velocity = new_horizontal_velocity + vertical_velocity
	else :  # Floating mode
		var old_velocity_length : float = velocity.length()
		
		velocity += p_wish_dir * accel_speed
		
		var max_speed : float = maxf(p_max_speed, old_velocity_length)
		velocity = velocity.limit_length(max_speed)
