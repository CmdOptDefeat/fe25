/**
 * @brief Header for hwrev2 WS2812 LED driver
 * @author DIY Labs
 */

#pragma once

#include <ILED.hpp>
#include <config.hpp>
#include <Adafruit_NeoPixel.h>

class hwrev2_RGBLED : public ILED{
public:
  hwrev2_RGBLED(VehicleConfig cfg);
  void init(ILogger *logger) override;
  void setStaticColor(RGBColor col) override;
  void limitBrightness(uint8_t lim) override;

  // TODO: move colors to interface to standardize
  static const RGBColor RED;
  static const RGBColor GREEN;
  static const RGBColor BLUE;
  static const RGBColor AMBER;
  static const RGBColor CYAN;
  static const RGBColor BLACK;

private:
  VehicleConfig _config;
  ILogger *_logger;
  Adafruit_NeoPixel *_led;

};