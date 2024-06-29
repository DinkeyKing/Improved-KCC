
## Example floating player controller using the IKCC collision response.
extends IKCC




#==================================================================================================
# --------------
# | PROPERTIES |
# --------------



@export_group("Movement")
@export var move_speed             : float = 8.0
@export var acceleration   : float = 7.0
@export var wall_friction_speed   : float = 2.0
@export var air_friction_speed    : float = 7.0




#==================================================================================================
# -------------------
# | NODE REFERENCES |
# -------------------


@onready var head                   := $PivotY                   as PlayerCamera
@onready var speed_label            := $GUI/SpeedLabel           as Label




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
	motion_mode = MotionMode.FLOATING




## _process
func _process(_delta : float) -> void :
	speed_label.text = "de facto speed: %0.2f u/s" % real_velocity.length()




## _physics_process
func _physics_process(p_delta_t : float) -> void :
	
	delta_t = p_delta_t  # Set global physics frame time
	
	set_initial_velocity()
	
	move_and_slide()




## Sets the velocity before calling 'move_and_slide'
func set_initial_velocity() -> void :
	
	apply_friction()
	
	# Player movement
	var wish_dir   : Vector3 = get_input_direction()
	var wish_speed : float = 0.0
	var accel      : float = 0.0
	
	wish_speed = move_speed
	accel = acceleration
	
	accelerate(accel, wish_dir, wish_speed)




## Applies damping to velocity
func apply_friction() -> void :
	
	var friction_speed : float = air_friction_speed
	
	if is_on_wall :
		friction_speed += wall_friction_speed
	
	# Damp velocity
	if not is_zero_approx(friction_speed) :
		velocity = velocity.lerp(Vector3.ZERO, friction_speed * delta_t)




## Returns the input direction in Vector3
func get_input_direction() -> Vector3 :
	
	var input_dir_vec2 : Vector2 = Input.get_vector("left", "right", "up", "down")
	var input_dir_vec3 : Vector3 = head.camera.global_transform.basis * Vector3(input_dir_vec2.x, 0.0, input_dir_vec2.y)
	
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
