extends Node3D
class_name MotionTester

#############
# CONSTANTS #
#############

const MAX_COL : int = 4


#############
# VARIABLES #
#############

var res := PhysicsTestMotionResult3D.new()
var normal : Vector3
var collision_point : Vector3
var travel : Vector3
var remainder : Vector3
var endpos : Vector3
var hit : bool = false

#############
# FUNCTIONS #
#############

func test_motion(body : RID, from : Transform3D, motion : Vector3, margin : float, rec : bool) -> void:
	var params := PhysicsTestMotionParameters3D.new()
	
	params.from = from
	params.motion = motion
	params.margin = margin
	params.exclude_bodies = [body]
	params.recovery_as_collision = rec
	params.max_collisions = MAX_COL
	
	hit = false
	
	var collided : bool = PhysicsServer3D.body_test_motion(body, params, res)
	if collided:
		hit = true
		normal = res.get_collision_normal()
		collision_point = res.get_collision_point()
	
	travel = res.get_travel()
	remainder = res.get_remainder()
	endpos = from.origin + (motion - remainder)
