
## Example grounded player controller using the IKCC collision response.
extends IKCC




#==================================================================================================
# --------------
# | PROPERTIES |
# --------------



@export_group("Ground movement")
@export var run_speed             : float = 8.0
@export var air_speed             : float = 5.0
@export var ground_acceleration   : float = 7.0
@export var ground_friction_speed : float = 7.0
@export var wall_friction_speed   : float = 1.0
@export var constantly_apply_gravity : bool = false


@export_group("Air movement")
@export var air_friction_speed    : float = 0.0
@export var air_acceleration      : float = 2.0


@export_group("Jumping")
@export var jump_impulse_length   : float = 7.0




#==================================================================================================
# -------------------
# | NODE REFERENCES |
# -------------------


@onready var head                   := $PivotY                   as Node3D
@onready var jump_input_buffer      := $JumpInputBuffer          as InputBuffer
@onready var speed_label            := $GUI/SpeedLabel           as Label
@onready var horizontal_speed_label := $GUI/HorizontalSpeedLabel as Label




#==================================================================================================
# --------------
# | VARIABLES  |
# --------------

var delta_t : float    # Physics process frame time





#==================================================================================================
# -----------
# | METHODS |
# -----------


## _ready
func _ready() -> void :
	rotation = Vector3.ZERO
	motion_mode = MotionMode.GROUNDED




## _process
func _process(_delta : float) -> void :
	speed_label.text = "de facto speed: %0.2f u/s" % real_velocity.length()
	horizontal_speed_label.text = "horizontal de facto speed: %0.2f u/s" % real_velocity.slide(up_direction).length()




## _physics_process
func _physics_process(p_delta_t : float) -> void :
	
	delta_t = p_delta_t  # Set global physics frame time
	
	set_initial_velocity()
	
	move_and_slide()




## Sets the velocity before calling 'move_and_slide'
func set_initial_velocity() -> void :
	
	apply_friction()
	
	# Apply gravity
	if not is_on_floor or constantly_apply_gravity :
		velocity += gravity_acceleration * delta_t * -up_direction
	
	# Player movement
	var wish_dir   : Vector3 = get_input_direction()
	var wish_speed : float = 0.0
	var accel      : float = 0.0
	
	if is_on_floor :
		wish_speed = run_speed
		accel = ground_acceleration
	else :
		wish_speed = air_speed
		accel = air_acceleration
	
	accelerate(accel, wish_dir, wish_speed)
	
	if is_on_floor and jump_input_buffer.is_input_just_pressed() :
		velocity += jump_impulse_length * up_direction




## Applies damping to velocity
func apply_friction() -> void :
	
	var friction_speed : float = 0.0
	
	if is_on_floor :
		friction_speed = ground_friction_speed
	elif is_on_wall :
		friction_speed = wall_friction_speed
	
	# Adjust horizontal velocity
	if not is_zero_approx(friction_speed) :
		velocity.x = lerpf(velocity.x, 0.0, friction_speed * delta_t)
		velocity.z = lerpf(velocity.z, 0.0, friction_speed * delta_t)




## Returns the input direction in Vector3
func get_input_direction() -> Vector3 :
	
	var input_dir_vec2 : Vector2 = Input.get_vector("left", "right", "up", "down")
	var input_dir_vec3 : Vector3 = head.transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	
	return input_dir_vec3





## Accelerates the velocity in the wished direction with the given acceleration factor,
## until the wished speed is reached in the wished direction.
func accelerate(accel : float, wish_dir : Vector3, wish_speed : float) -> void :
	
	var current_speed : float = velocity.dot(wish_dir)
	var add_speed : float = wish_speed - current_speed
	
	if add_speed <= 0 : 
		return
	
	var accel_speed : float = accel * wish_speed * delta_t
	
	if accel_speed > add_speed : 
		accel_speed = add_speed
	
	velocity += accel_speed * wish_dir
