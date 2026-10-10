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

#ifndef _USB_GAMEPAD_H
#define _USB_GAMEPAD_H

#include <cstddef>

class Gamepad {
public:
  enum Button {
    DPAD_UP = 0,
    DPAD_DOWN,
    DPAD_LEFT,
    DPAD_RIGHT,
    START,
    SELECT,
    LEFT_STICK,
    RIGHT_STICK,
    LEFT_BUMPER,
    RIGHT_BUMPER,
    SYSTEM,
    FACE_BOTTOM = 12,  // XBOX = A, PS = CROSS, Nintendo = B
    FACE_RIGHT,        // XBOX = B, PS = CIRCLE, Nintendo = A
    FACE_LEFT,         // XBOX = X, PS = SQUARE, Nintendo = Y
    FACE_TOP,          // XBOX = Y, PS = TRIANLE, Nintendo = X
    LEFT_TRIGGER,
    RIGHT_TRIGGER,

    BUTTON_CUSTOM,

    BUTTON_COUNT = 64
  };
  enum Stick {
    LEFT_X = 0,
    LEFT_Y,
    RIGHT_X,
    RIGHT_Y,

    STICK_CUSTOM,

    STICK_COUNT = 16
  };

private:
  struct {
    uint64_t buttons;
    int16_t sticks[STICK_COUNT];
    uint8_t button_level[BUTTON_COUNT];
  } latched, current = {};
  uint64_t old_buttons;

public:
  class Impl {
  protected:
    Gamepad& pad;
    Impl(Gamepad& _pad) : pad(_pad) { pad.impl = this; reset_padstate(); }
    ~Impl() { reset_padstate(); pad.impl = NULL; }

    void reset_padstate() { pad.current = pad.latched = {}; pad.old_buttons = 0; }
    void update_padbutton(uint8_t btn, uint8_t val) {
      uint64_t mask = 1llu << btn;
      pad.current.button_level[btn] = val;
      if (val) pad.current.buttons |= mask;
      else pad.current.buttons &= ~mask;
    }
    void update_padstick(uint8_t stk, int16_t val) {
      pad.current.sticks[stk] = val;
    }

  public:
    virtual bool isReady() = 0;
    virtual void setPlayerLED(uint8_t) {}
    virtual void setLED(uint32_t) {}
    virtual void setRumble(uint8_t, uint8_t) {}
    virtual const char* getButtonName(uint8_t) const { return NULL; }
    virtual const char* getDeviceType() const = 0;
    virtual const char* getStickName(uint8_t) const { return NULL; }
  };

private:
  class Impl* impl = NULL;

public:
  virtual ~Gamepad() {}
  operator bool() const { return impl && impl->isReady(); }

  // set player indicator: 0-3
  virtual void setPlayerLED(uint8_t id) { if (impl) return impl->setPlayerLED(id); }
  // do something controller-specific with the LED
  virtual void setLED(uint32_t led_value) { if (impl) return impl->setLED(led_value); }
  // supports separate heavy and light motors
  virtual void setRumble(uint8_t heavy, uint8_t light=0) { if (impl) return impl->setRumble(heavy, light); }
  // return a text name for the button
  virtual const char* getButtonName(uint8_t btn) const;
  // returns the type of controller
  virtual const char* getDeviceType() const { return impl ? impl->getDeviceType() : "None"; }
  // return a text name for a stick axis
  virtual const char* getStickName(uint8_t stk) const;

  // capture the current button state
  uint64_t update() { old_buttons = latched.buttons; latched = current; return latched.buttons; }
  // buttons that stayed down since last update
  uint64_t held() const { return old_buttons & latched.buttons; }
  // buttons that went up or down since last update
  uint64_t changed() const { return old_buttons ^ latched.buttons; }
  // buttons that went down since last update
  uint64_t pressed() const { return ~old_buttons & latched.buttons; }
  // buttons that went up since last update
  uint64_t released() const { return old_buttons & ~latched.buttons; }

  uint64_t buttons() const { return latched.buttons; }
  uint8_t triggerL() const { return latched.button_level[LEFT_TRIGGER]; }
  uint8_t triggerR() const { return latched.button_level[RIGHT_TRIGGER]; }
  int stickLX() const { return latched.sticks[LEFT_X]; }
  int stickLY() const { return latched.sticks[LEFT_Y]; }
  int stickRX() const { return latched.sticks[RIGHT_X]; }
  int stickRY() const { return latched.sticks[RIGHT_Y]; }
  uint8_t getButton(uint8_t btn) const { return (btn < BUTTON_COUNT) ? latched.button_level[btn] : 0; }
  int getStick(uint8_t stk) const { return (stk < STICK_COUNT) ? latched.sticks[stk] : 0; }
};

#define PADBUTTON_DPAD_UP      (1llu << Gamepad::Button::DPAD_UP)
#define PADBUTTON_DPAD_DOWN    (1llu << Gamepad::Button::DPAD_DOWN)
#define PADBUTTON_DPAD_LEFT    (1llu << Gamepad::Button::DPAD_LEFT)
#define PADBUTTON_DPAD_RIGHT   (1llu << Gamepad::Button::DPAD_RIGHT)
#define PADBUTTON_START        (1llu << Gamepad::Button::START)
#define PADBUTTON_SELECT       (1llu << Gamepad::Button::SELECT)
#define PADBUTTON_L3           (1llu << Gamepad::Button::LEFT_STICK)
#define PADBUTTON_R3           (1llu << Gamepad::Button::RIGHT_STICK)
#define PADBUTTON_LB           (1llu << Gamepad::Button::LEFT_BUMPER)
#define PADBUTTON_RB           (1llu << Gamepad::Button::RIGHT_BUMPER)
#define PADBUTTON_SYSTEM       (1llu << Gamepad::Button::SYSTEM)
#define PADBUTTON_A            (1llu << Gamepad::Button::FACE_BOTTOM)
#define PADBUTTON_B            (1llu << Gamepad::Button::FACE_RIGHT)
#define PADBUTTON_X            (1llu << Gamepad::Button::FACE_LEFT)
#define PADBUTTON_Y            (1llu << Gamepad::Button::FACE_TOP)

#endif // _USB_GAMEPAD_H
