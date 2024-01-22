extends Node3D
class_name Trace

"""
trace.gd

- Uses direct_space_state functions for 3D shape collision testing
- Various functions for specific collision tests
- Treated as an object to simplify getting collision info
"""

const MARGIN : float = 0.001

var endpos : Vector3
var fraction : float
var normal : Vector3
var hit : bool
var colpos : Vector3
var linear_velocity : Vector3
var rid : RID
var id : int

"""
===============
new
===============
"""
func new():
	endpos = Vector3.ZERO
	fraction = 0.0
	normal = Vector3.ZERO
	hit = false
	colpos = Vector3.ZERO
	linear_velocity = Vector3.ZERO
	
	return self

"""
===============
motion
===============
"""
func motion(origin : Vector3, dest : Vector3, shape : Shape3D, e : RID, space_state : PhysicsDirectSpaceState3D):
	var params : PhysicsShapeQueryParameters3D
	
	params = PhysicsShapeQueryParameters3D.new()
	params.set_shape(shape)
	params.transform.origin = origin
	params.collide_with_bodies = true
	params.exclude = [e]
	params.motion = dest - origin
	params.margin = MARGIN
	
	var results : PackedFloat32Array = space_state.cast_motion(params)
	fraction = results[0]

"""
===============
rest
===============
"""
func rest(origin : Vector3, shape : Shape3D, e : RID, mask : int, space_state : PhysicsDirectSpaceState3D):
	var params : PhysicsShapeQueryParameters3D
	
	params = PhysicsShapeQueryParameters3D.new()
	params.set_shape(shape)
	params.set_collision_mask(mask)
	params.transform.origin = origin
	params.collide_with_bodies = true
	params.exclude = [e]
	params.margin = MARGIN
	
	hit = false
	
	var results : Dictionary = space_state.get_rest_info(params)
	
	if results.is_empty() : return
	
	hit = true
	normal = results.get("normal")
	colpos = results.get("point")
	linear_velocity = results.get("linear_velocity")
	rid = results.get("rid")
	id = results.get("collider_id")
	
	
"""
===============
standard
===============
"""
func standard(origin : Vector3, dest : Vector3, shape : Shape3D, e : RID, mask : int, space_state : PhysicsDirectSpaceState3D):
	var params : PhysicsShapeQueryParameters3D
	var results_arr : PackedFloat32Array
	var results_dic : Dictionary
	
	# Create collision parameters
	params = PhysicsShapeQueryParameters3D.new()
	params.set_shape(shape)
	params.transform.origin = origin
	params.collide_with_bodies = true
	params.exclude = [e]
	params.collision_mask = mask
	params.motion = dest - origin
	params.margin = MARGIN
	
	hit = false
	
	# Get distance fraction and position of first collision
	results_arr = space_state.cast_motion(params)
	
	if not results_arr.size() == 0:
		fraction = results_arr[0]
		endpos = origin + (dest - origin).normalized() * (origin.distance_to(dest) * fraction)
	else:
		fraction = 1
		endpos = dest
		return # didn't hit anything
	
	hit = true
	
	# Set next parameter position to endpos
	# Edit: The position must be calculated with the unsafe fraction, otherwise there is no collision
	var unsafe_fraction : float = results_arr[1]
	params.transform.origin = origin + (dest - origin).normalized() * (origin.distance_to(dest) * unsafe_fraction)
	
	# Get collision normal
	results_dic = space_state.get_rest_info(params)
	
	if not results_dic.is_empty():
		normal = results_dic.get("normal")
		colpos = results_dic.get("point")
		linear_velocity = results_dic.get("linear_velocity")
	else:
		normal = Vector3.UP
