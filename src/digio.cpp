/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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
#include "digio.h"

#define DIG_IO_OFF 0
#define DIG_IO_ON  1

#undef DIG_IO_ENTRY
#define DIG_IO_ENTRY(name, port, pin, mode) DigIo DigIo::name;

#ifdef BUSIO_ENABLED    
#undef  BUS_IO_ENTRY
#define BUS_IO_ENTRY(name, busType, channel, mode) BusIo DigIo::name;
#endif

DIG_IO_LIST

void DigIo::Configure(uint32_t port, uint16_t pin, PinMode::PinMode pinMode)
{
    uint32_t pupd = GPIO_PUPD_NONE;
    uint32_t mode = GPIO_MODE_INPUT;
    uint32_t af = 0; // Alternate function (default to none)
    uint16_t val = DIG_IO_OFF;

    _port = port;
    _pin = pin;
    _invert = 0;

    switch (pinMode)
    {
        default:
        case PinMode::INPUT_PD:
            pupd = GPIO_PUPD_PULLDOWN;
            break;
        case PinMode::INPUT_PD_INV:
            pupd = GPIO_PUPD_PULLDOWN;
            _invert = 1;
            break;
        case PinMode::INPUT_PU:
            pupd = GPIO_PUPD_PULLUP;
            val = DIG_IO_ON;
            break;
        case PinMode::INPUT_PU_INV:
            pupd = GPIO_PUPD_PULLUP;
            val = DIG_IO_ON;
            _invert = 1;
            break;
        case PinMode::INPUT_FLT:
            pupd = GPIO_PUPD_NONE;
            break;
        case PinMode::INPUT_FLT_INV:
            pupd = GPIO_PUPD_NONE;
            _invert = 1;
            break;
        case PinMode::INPUT_AIN:
#ifdef STM32F1
            pupd = GPIO_CNF_INPUT_ANALOG;
#else
            mode = GPIO_MODE_ANALOG;
#endif
            break;
        case PinMode::OUTPUT:
            mode = GPIO_MODE_OUTPUT;
#ifdef STM32F1
            pupd = GPIO_CNF_OUTPUT_PUSHPULL;
#else
            pupd = GPIO_PUPD_NONE;
#endif
            break;
        case PinMode::OUTPUT_OD:
            mode = GPIO_MODE_OUTPUT;
#ifdef STM32F1
            pupd = GPIO_CNF_OUTPUT_OPENDRAIN;
#else
            pupd = GPIO_PUPD_NONE;
#endif
            val = DIG_IO_ON;
            break;
        case PinMode::OUTPUT_ALT:
#ifdef STM32F1
            mode = GPIO_MODE_OUTPUT_50_MHZ;
            pupd = GPIO_CNF_OUTPUT_ALTFN_PUSHPULL;
#else
            mode = GPIO_MODE_AF;
            af = GPIO_AF0; 
#endif
            break;
    }

    if (DIG_IO_ON == val)
    {
        gpio_set(port, pin);
    }

#ifdef STM32F1
    gpio_set_mode(port, mode, pupd, pin);
#else
    gpio_mode_setup(port, mode, pupd, pin);
    if (mode == GPIO_MODE_AF)
    {
        gpio_set_af(port, af, pin); 
    }
#endif
}

