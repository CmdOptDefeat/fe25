/**
 * @brief Struct for storing vehicle commands
 * @author DIY Labs
 * @note Negative values in an *unsigned* int16! Demonic behavior and potentially undefined behavior
 */

#pragma once

#include <cstdint>
#include "vehicleinstruction.hpp"

struct VehicleCommand{

  int16_t targetSpeed; // Note from DIY: Putting negative values into an *unsigned* int16 was not a good idea
  uint16_t targetYaw;
  VehicleInstruction instruction = NO_INSTRUCTION;  // initialized by default to NO_INSTRUCTION

};