#pragma once

namespace helmoro_description {

enum class JointEnum: unsigned int {
	base_link_to_RIGHT_FRONT_WHEEL, 
	base_link_to_LEFT_FRONT_WHEEL, 
	base_link_to_RIGHT_BACK_WHEEL,
	base_link_to_LEFT_BACK_WHEEL,

	NrJoints // = 4
};

enum class ActuatorEnum: unsigned int {
	base_link_to_RIGHT_FRONT_WHEEL, 
	base_link_to_LEFT_FRONT_WHEEL, 
	base_link_to_RIGHT_BACK_WHEEL,
	base_link_to_LEFT_BACK_WHEEL,

	NrActuators // = 4
};

enum class LinkEnum: unsigned int {
	base_link,
	RIGHT_FRONT_WHEEL, 
	LEFT_FRONT_WHEEL, 
	RIGHT_BACK_WHEEL,
	LEFT_BACK_WHEEL,
	CAMERA_LINK,

	NrLinks // = 6
};

enum class ActuatorModeEnum: unsigned int {
	MODE_SHUTOFF=0,         // Passive joints -> no torque
	MODE_JOINTPOSITION=1,   // Joint position ref
	MODE_JOINTVELOCITY=2,   // Joint velocity ref
	MODE_JOINTTORQUE=3,     // Joint torque ref
};



} // namespace helmoro_description
