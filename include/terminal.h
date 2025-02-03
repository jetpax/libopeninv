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
#ifndef TERMINAL_H
#define TERMINAL_H
#include <stdint.h>
#include "printf.h"

class Terminal;

typedef struct
{
   char const *cmd;
   void (*CmdFunc)(Terminal*, char*);
} TERM_CMD;

class Terminal: public IPutChar
{
public:
   Terminal(uint32_t usart, const TERM_CMD* commands, bool remap = false, bool echo = true);
   void SetNodeId(uint8_t id);
   void Run();
   void PutChar(char c);
   void SendBinary(const uint8_t* data, uint32_t length);
   void SendBinary(const uint32_t* data, uint32_t length);
   bool KeyPressed();
   void FlushInput();
   void DisableTxDMA();
   static Terminal* defaultTerminal;

private:
   struct HwInfo
   {
#ifdef STM32F1
      uint32_t usart;       // USART base address
      uint32_t dmactl;      // DMA controller base address
      uint8_t dmatx;        // DMA channel for TX (STM32F1 only)
      uint8_t dmarx;        // DMA channel for RX (STM32F1 only)
      uint32_t port;        // GPIO port for TX
      uint16_t pin;         // GPIO pin for TX
      uint32_t port_re;     // GPIO port for remapped TX
      uint16_t pin_re;      // GPIO pin for remapped TX      
#else
      uint32_t usart;       // USART base address
      uint32_t dmactl;      // DMA controller base address
      uint8_t dmatx;        // DMA stream for TX (STM32F405)
      uint8_t dmarx;        // DMA stream for RX (STM32F405)
      uint32_t dmaChannel;   // DMA channel for both TX and RX (STM32F405)
      uint32_t port_tx;     // GPIO port for TX
      uint16_t pin_tx;      // GPIO pin for TX
      uint32_t port_rx;     // GPIO port RX
      uint16_t pin_rx;      // GPIO pin RX
#endif      
   };

   void ResetRxDMA();
   void ResetTxDMA();
   const TERM_CMD *CmdLookup(char *buf);
   void EnableUart(char* arg);
   void FastUart(char* arg);
   void Echo(char* arg);
   void Send(const char *str);
   void SendCurrentBuffer(uint32_t len);

   static const int bufSize = 128;
   static const HwInfo hwInfo[];
   const HwInfo* hw;
   uint32_t usart;
   bool remap;
   const TERM_CMD* termCmds;
   uint8_t nodeId;
   bool enabled;
   bool txDmaEnabled;
   const TERM_CMD *pCurCmd;
   int lastIdx;
   uint8_t curBuf;
   uint32_t curIdx;
   bool firstSend;
   bool echo;
   char inBuf[bufSize];
   char outBuf[2][bufSize]; //double buffering
   char args[bufSize];
   uint8_t txBuffer[bufSize];
   uint8_t rxBuffer[bufSize];
};

#endif // TERMINAL_H
