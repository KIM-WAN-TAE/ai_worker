// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOYSTICK_CONTROLLER__TACT_SWITCH_GATE_HPP_
#define JOYSTICK_CONTROLLER__TACT_SWITCH_GATE_HPP_

#include <cmath>

namespace joystick_controller
{

enum class TactSwitchAction
{
  NONE,
  SHORT_PRESS,
  LONG_PRESS,
};

// Keep the left and right tact switches independent. Invalid hardware samples
// do not change the edge state, so a transient bus read failure cannot look
// like a release and create a false toggle.
class TactSwitchGate
{
public:
  TactSwitchAction observe(
    bool sample_valid, bool pressed, double now_sec, double long_press_duration)
  {
    if (!sample_valid || !std::isfinite(now_sec) ||
      !std::isfinite(long_press_duration) || long_press_duration <= 0.0)
    {
      return TactSwitchAction::NONE;
    }

    if (!initialized_) {
      initialized_ = true;
      pressed_ = pressed;
      // A switch already held while the controller starts is not a fresh press.
      suppress_until_release_ = pressed;
      return TactSwitchAction::NONE;
    }

    if (!pressed_) {
      if (pressed) {
        pressed_ = true;
        press_start_sec_ = now_sec;
        long_press_triggered_ = false;
        suppress_until_release_ = false;
      }
      return TactSwitchAction::NONE;
    }

    if (!pressed) {
      const bool emit_short = !suppress_until_release_ && !long_press_triggered_;
      pressed_ = false;
      long_press_triggered_ = false;
      suppress_until_release_ = false;
      return emit_short ? TactSwitchAction::SHORT_PRESS : TactSwitchAction::NONE;
    }

    if (!suppress_until_release_ && !long_press_triggered_ &&
      now_sec >= press_start_sec_ &&
      now_sec - press_start_sec_ >= long_press_duration)
    {
      long_press_triggered_ = true;
      return TactSwitchAction::LONG_PRESS;
    }

    return TactSwitchAction::NONE;
  }

private:
  bool initialized_ = false;
  bool pressed_ = false;
  bool suppress_until_release_ = false;
  bool long_press_triggered_ = false;
  double press_start_sec_ = 0.0;
};

}  // namespace joystick_controller

#endif  // JOYSTICK_CONTROLLER__TACT_SWITCH_GATE_HPP_
