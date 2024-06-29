extends RefCounted
#class_name MotionTester




#############
# VARIABLES #
#############


var hit              : bool = false
var endpos           : Vector3
var res              := PhysicsTestMotionResult3D.new()




#############
# FUNCTIONS #
#############

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
