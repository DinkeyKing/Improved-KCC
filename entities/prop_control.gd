extends RigidBody3D

# This script is for testing purposes.

func _physics_process(_delta : float) -> void :
	if Input.is_action_just_pressed("crate_impulse") :
		#apply_central_impulse(global_transform.basis.z * 20.0 * mass)
		apply_central_impulse(Vector3.UP * 20.0 * mass)
		#apply_central_impulse(Vector3.UP * 5.0 * mass)
