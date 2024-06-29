extends RefCounted
#class_name RayCast




#############
# VARIABLES #
#############

var endpos : Vector3
var normal : Vector3
var collider : CollisionObject3D
var hit : bool




#############
# FUNCTIONS #
#############

func intersect(origin : Vector3 , dest : Vector3, exclude : RID, mask : int, space_state : PhysicsDirectSpaceState3D) -> void:
	var ray_query := PhysicsRayQueryParameters3D.new()
	
	ray_query.from = origin
	ray_query.to = dest
	ray_query.collision_mask = mask
	ray_query.exclude = [exclude]
	
	var results : Dictionary = space_state.intersect_ray(ray_query)
	
	hit = false
	
	if results.is_empty():
		return
	
	hit = true
	endpos = results["position"]
	normal = results["normal"]
	collider = results["collider"]
