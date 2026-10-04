/*
  Copyright (C) 2026 Andrew Dunstan
  This file is part of teensy4_usbhost.

  teensy4_usbhost is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/pgmspace.h>
#include "gamepad.h"

const char* Gamepad::getButtonName(uint8_t btn) const {
  if (impl) {
    auto name = impl->getButtonName(btn);
    if (name) return name;
  }
  switch (btn) {
    case Button::DPAD_UP:
      return PSTR("DPAD_UP");
    case Button::DPAD_DOWN:
      return PSTR("DPAD_DOWN");
    case Button::DPAD_LEFT:
      return PSTR("DPAD_LEFT");
    case Button::DPAD_RIGHT:
      return PSTR("DPAD_RIGHT");
    case Button::START:
      return PSTR("START");
    case Button::SELECT:
      return PSTR("SELECT");
    case Button::LEFT_STICK:
      return PSTR("L3");
    case Button::RIGHT_STICK:
      return PSTR("R3");
    case Button::LEFT_BUMPER:
      return PSTR("LB");
    case Button::RIGHT_BUMPER:
      return PSTR("RB");
    case Button::SYSTEM:
      return PSTR("SYSTEM");
    case Button::FACE_BOTTOM:
      return PSTR("FACE_BOTTOM");
    case Button::FACE_RIGHT:
      return PSTR("FACE_RIGHT");
    case Button::FACE_LEFT:
      return PSTR("FACE_LEFT");
    case Button::FACE_TOP:
      return PSTR("FACE_TOP");
    case Button::LEFT_TRIGGER:
      return PSTR("LEFT_TRIGGER");
    case Button::RIGHT_TRIGGER:
      return PSTR("RIGHT_TRIGGER");
  }
  return PSTR("Unknown");
}

const char* Gamepad::getStickName(uint8_t stk) const {
  if (impl) {
    auto name = impl->getStickName(stk);
    if (name) return name;
  }
  switch (stk) {
    case Stick::LEFT_X:
      return PSTR("LEFT_X_AXIS");
    case Stick::LEFT_Y:
      return PSTR("LEFT_Y_AXIS");
    case Stick::RIGHT_X:
      return PSTR("RIGHT_X_AXIS");
    case Stick::RIGHT_Y:
      return PSTR("RIGHT_Y_AXIS");
  }
  return PSTR("Unknown");
}
