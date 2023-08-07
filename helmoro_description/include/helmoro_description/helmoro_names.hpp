#pragma once

#include <string>
#include "helmoro_description/enums/enums.hpp"

namespace helmoro_description{

namespace HelmoroJointNames {
constexpr char const *names[] = {
		"base_link_to_RIGHT_FRONT_WHEEL", 
		"base_link_to_LEFT_FRONT_WHEEL", 
		"base_link_to_RIGHT_BACK_WHEEL",
  		"base_link_to_LEFT_BACK_WHEEL"};

constexpr const char* getName(const unsigned int joint) { return names[joint]; }
constexpr const char* getName(const helmoro_description::JointEnum joint) { return names[static_cast<const unsigned int>(joint)]; }
}


namespace ManipulatorActuatorNames {
constexpr char const *names[] = {
	"base_link_to_RIGHT_FRONT_WHEEL", 
	"base_link_to_LEFT_FRONT_WHEEL", 
	"base_link_to_RIGHT_BACK_WHEEL",
	"base_link_to_LEFT_BACK_WHEEL"};

constexpr const char* getName(const unsigned int actuator) { return names[actuator]; }
constexpr const char* getName(const helmoro_description::ActuatorEnum actuator) { return names[static_cast<const unsigned int>(actuator)]; }
}

namespace HelmoroLinkNames {
constexpr char const *names[] = {
		"base_link",
		"RIGHT_FRONT_WHEEL", 
		"LEFT_FRONT_WHEEL", 
		"RIGHT_BACK_WHEEL",
  		"LEFT_BACK_WHEEL", 
		"CAMERA_LINK"};

constexpr const char* getName(const unsigned int link) { return names[link]; }
constexpr const char* getName(const helmoro_description::LinkEnum link) { return names[static_cast<const unsigned int>(link)]; }
}

} // namespace manipulator:description
