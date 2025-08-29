#pragma once

#include <cstdint>

struct hwrev2_rf24_telem_block1{

  float oriX;

  uint16_t lidarLeft;
  uint16_t lidarFront;
  uint16_t lidarRight;
  uint16_t lidarBack;

  uint16_t commandedSpeed;
  uint16_t commandedSteer;

  uint32_t millis;

  bool roundDirectionCW;

};

struct hwrev2_rf24_cmd_block1{

  int16_t targetSpeed;
  uint16_t targetYaw;  

};


const uint8_t TLM_PIPE_0[] = "TLM00";
const uint8_t CAR_PIPE_0[] = "CAR00";