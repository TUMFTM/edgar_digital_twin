# About
This section describes the [edgar.urdf](edgar.urdf) of the EDGAR research vehicle.

# URDF
A URDF (Unified Robot Description Format) is a file used to represent the kinematic and dynamic description of a robot. It enables the modeling of robots using joints and links among other attributes. 

The edgar.urdf is structured as follows: 

1. Robot element: 
	- The `robot` as the main container for the robot description

2. The link elements represent a rigid body in a robot model. In this particular case the links defined in the URDF are:

	Reference frames:

	- [base_link](edgar.urdf#L15)
	- [axle_edgar_rearcenterground](edgar.urdf#L5)
	- [rack_edgar_rearcenter](edgar.urdf#L16)
	- [axle_edgar_rearcenter](edgar.urdf#L17)

	The respective sensors installed on EDGAR, i.e.:

	- [antenna_novatel_left](edgar.urdf#L19)
	- [imu_novatel_center](edgar.urdf#L22)
	- [lidar_ouster_frontleft](edgar.urdf#L29)
	- [radar_continental_frontleft](edgar.urdf#L34)
	- [camera_basler_frontleft](edgar.urdf#L41)
	- ...
	
	Links can also be visualized using geometric features. We use a [3d model](../3d_model/high_res/EDGAR_T7.fbx) for EDGAR (see first section in the [edgar.urdf](edgar.urdf) file).  

3. The joint elements describe the relative transformation between two links with the origin being the relative position of the child link with respect to the parent link. All [joints](edgar.urdf#L64) in the EDGAR URDF are defined as fixed.

You can find an exemplary integration of the EDGAR URDF file and 3d model using the [edgar_state_publisher](../../tools/edgar_state_publisher). 

For a more detailed tutorial on URDF please refer to http://wiki.ros.org/urdf/Tutorials. 
