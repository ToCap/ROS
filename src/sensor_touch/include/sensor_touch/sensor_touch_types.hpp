#pragma once

/**
 * @file touch_types.hpp
 * @brief Shared definitions for touch sensors (types and measured values).
 */

namespace sensor_touch_abstraction
{


/**
 * @brief Possible measured values for a touch sensor.
 *
 * 0 = Released
 * 1 = Pressed
 * 2 = Bumped
 */
enum class TouchMeasuredValue
{
  RELEASED = 0,
  PRESSED  = 1,
  BUMPED   = 2
};

}  // namespace sensor_touch_abstraction
