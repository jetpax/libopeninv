/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef DIGIO_H_INCLUDED
#define DIGIO_H_INCLUDED

#include <libopencm3/stm32/gpio.h>
#include "digio_prj.h"
#include "busio.h"


namespace PinMode {
   enum PinMode
   {
       INPUT_PD,
       INPUT_PD_INV, //Logic Inverted
       INPUT_PU,
       INPUT_PU_INV, //Logic Inverted
       INPUT_FLT,
       INPUT_FLT_INV, //Logic Inverted
       INPUT_AIN,
       OUTPUT,
       OUTPUT_OD,
       OUTPUT_ALT,
       LAST
   };
}

//--------------------------------------
// DigIo class: real STM32 GPIO pins
//--------------------------------------
class DigIo
{
public:
    // Macro pass to declare real pins -> `static DigIo name;`
    // and MCP pins -> `static McpIo name;`
    #define DIG_IO_ENTRY(name, port, pin, mode)     static DigIo name;
    #define BUS_IO_ENTRY(name, type, channel, pm) static BusIo name;
    
    DIG_IO_LIST  // expands all real & MCP pins into static members

    #undef DIG_IO_ENTRY
    #undef BUS_IO_ENTRY


   /** Map GPIO pin object to hardware pin.
    * @param[in] port port to use for this pin
    * @param[in] pin port-pin to use for this pin
    * @param[in] mode pinmode to use
    * @param[in] invert input or not to use
    */
   void Configure(uint32_t port, uint16_t pin, PinMode::PinMode pinMode);

   /**
   * Get pin value
   *
   * @param[in] io pin index
   * @return pin value
   */
   bool Get() { return (gpio_get(_port, _pin) > 0) ^ _invert; }

   /**
   * Set pin high
   *
   * @param[in] io pin index
   */
   void Set() { gpio_set(_port, _pin); }

   /**
   * Set pin low
   *
   * @param[in] io pin index
   */
   void Clear() { gpio_clear(_port, _pin); }

   /**
   * Toggle pin
   *
   * @param[in] io pin index
   */
   void Toggle() { gpio_toggle(_port, _pin); }

private:
   uint32_t _port;
   uint16_t _pin;
   bool _invert;
};

#define DIG_IO_ENTRY(name, port, pin, mode)        DigIo::name.Configure(port, pin, mode);
#define BUS_IO_ENTRY(name, busType, channel, pMode) DigIo::name.Configure(busType, channel, pMode);

#define DIG_IO_CONFIGURE(LIST) LIST  // configures on-chip and & Bus pins

#endif // DIGIO_H_INCLUDED
