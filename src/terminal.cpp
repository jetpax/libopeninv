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

#include "my_string.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include "terminal.h"
#include "printf.h"
#include <cstring>

#define HWINFO_ENTRIES (sizeof(hwInfo) / sizeof(struct HwInfo))

#ifndef USART_BAUDRATE
#define USART_BAUDRATE 115200
#endif // USART_BAUDRATE


#ifdef STM32F1
const Terminal::HwInfo Terminal::hwInfo[] =
{
   { USART1, DMA1, DMA_CHANNEL4, DMA_CHANNEL5, GPIOA, GPIO_USART1_TX, GPIOB, GPIO_USART1_RE_TX },
   { USART2, DMA1, DMA_CHANNEL7, DMA_CHANNEL6, GPIOA, GPIO_USART2_TX, GPIOD, GPIO_USART2_RE_TX },
   { USART3, DMA1, DMA_CHANNEL2, DMA_CHANNEL3, GPIOB, GPIO_USART3_TX, GPIOC, GPIO_USART3_PR_TX },
   { UART4,  DMA2, DMA_CHANNEL5, DMA_CHANNEL3, GPIOC, GPIO_UART4_TX,  GPIOC, GPIO_UART4_TX },
};

#else
const Terminal::HwInfo Terminal::hwInfo[] =
{
    { USART1, DMA2, DMA_STREAM7, DMA_STREAM2, DMA_SxCR_CHSEL_4, GPIOA, GPIO9, GPIOA, GPIO10},      // USART1 TX: PA9, RX: PA10
    { USART2, DMA1, DMA_STREAM6, DMA_STREAM5, DMA_SxCR_CHSEL_4, GPIOA, GPIO2, GPIOA, GPIO3 },      // USART2 TX: PA2, RX: PA3
    { USART3, DMA1, DMA_STREAM3, DMA_STREAM1, DMA_SxCR_CHSEL_4, GPIOB, GPIO10, GPIOB, GPIO11 },    // USART3 TX: PB10, RX: PB11
};
#endif

Terminal* Terminal::defaultTerminal;

Terminal::Terminal(uint32_t usart, const TERM_CMD* commands, bool remap, bool echo)
:  usart(usart),
   remap(remap),
   termCmds(commands),
   nodeId(1),
   enabled(true),
   txDmaEnabled(true),
   pCurCmd(NULL),
   lastIdx(0),
   curBuf(0),
   curIdx(0),
   firstSend(true),
   echo(echo)
{
   //Search info entry
   hw = hwInfo;
   for (uint32_t i = 0; i < HWINFO_ENTRIES; i++)
   {
      if (hw->usart == usart) break;
      hw++;
   }

   defaultTerminal = this;

#ifdef STM32F1

    gpio_set_mode(remap ? hw->port_re : hw->port, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, remap ? hw->pin_re : hw->pin);

#else

    gpio_mode_setup(hw->port_tx, GPIO_MODE_AF, GPIO_PUPD_NONE, hw->pin_tx);
    gpio_set_af(hw->port_tx, GPIO_AF7, hw->pin_tx);
    gpio_mode_setup(hw->port_rx, GPIO_MODE_AF, GPIO_PUPD_PULLUP, hw->pin_rx);
    gpio_set_af(hw->port_rx, GPIO_AF7, hw->pin_rx);

#endif

   usart_set_baudrate(usart, USART_BAUDRATE);
   usart_set_databits(usart, 8);
   usart_set_stopbits(usart, USART_STOPBITS_2);
   usart_set_mode(usart, USART_MODE_TX_RX);
   usart_set_parity(usart, USART_PARITY_NONE);
   usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
   usart_enable_rx_dma(usart);
   usart_enable_tx_dma(usart);

#ifdef STM32F1

   dma_channel_reset(hw->dmactl, hw->dmatx);
   dma_set_read_from_memory(hw->dmactl, hw->dmatx);
   dma_set_peripheral_address(hw->dmactl, hw->dmatx, (uint32_t)&USART_DR(usart));
   dma_set_peripheral_size(hw->dmactl, hw->dmatx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dmactl, hw->dmatx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dmactl, hw->dmatx);

   dma_channel_reset(hw->dmactl, hw->dmarx);
   dma_set_peripheral_address(hw->dmactl, hw->dmarx, (uint32_t)&USART_DR(usart));
   dma_set_peripheral_size(hw->dmactl, hw->dmarx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dmactl, hw->dmarx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dmactl, hw->dmarx);
   dma_enable_channel(hw->dmactl, hw->dmarx);

#else

    ResetTxDMA();
    ResetRxDMA();

#endif

   usart_enable(usart);
}

// /** Run the terminal */
void Terminal::Run()
{
    // Get the number of bytes remaining in the DMA transfer.
    // In circular mode, NDTR reloads automatically; thus,
    // the "write pointer" (i.e. how many bytes have been received) is:
    int ndtr = dma_get_number_of_data(hw->dmactl, hw->dmarx);
    int currentIdx = (bufSize - ndtr) % bufSize; // new hardware write index

    // Process all newly received characters from lastIdx up to currentIdx
    while (lastIdx != currentIdx)
    {
        char c = inBuf[lastIdx];
        lastIdx = (lastIdx + 1) % bufSize;
        if (echo)
            usart_send_blocking(usart, c);
    }

    // Check if we have received any new data
    if (currentIdx != 0)
    {
        // Compute the index of the last received character in the ring buffer.
        int lastCharIdx = (currentIdx + bufSize - 1) % bufSize;

        // If the most recently received character is a newline or carriage return,
        // consider the command complete.
        if (inBuf[lastCharIdx] == '\n' || inBuf[lastCharIdx] == '\r')
        {
            // Temporarily disable the DMA RX stream to avoid new writes while processing.
#ifdef STM32F1
            dma_disable_channel(hw->dmactl, hw->dmarx);
#else
            dma_disable_stream(hw->dmactl, hw->dmarx);
#endif

            // Since the DMA is circular, we assume that the command starts at index 0.
            // To simplify, we null-terminate the command at the current index.
            inBuf[currentIdx] = '\0';

            // Reset our software read pointer to 0.
            lastIdx = 0;

            // Process the command stored in inBuf.
            char *space = (char*)my_strchr(inBuf, ' ');
            if (space == NULL || *space == 0)
            {
                // No space found: look for newline (or carriage return) and set args empty.
                space = (char*)my_strchr(inBuf, '\n');
                if (space == NULL)
                    space = (char*)my_strchr(inBuf, '\r');
                args[0] = '\0';
            }
            else
            {
                my_strcpy(args, space + 1);
            }
            if (space != NULL)
                *space = '\0';  // Null-terminate the command

            pCurCmd = NULL;
            if (my_strcmp(inBuf, "enableuart") == 0)
            {
                EnableUart(args);
            }
            else if (my_strcmp(inBuf, "fastuart") == 0)
            {
                FastUart(args);
            }
            else if (my_strcmp(inBuf, "echo") == 0)
            {
                Echo(args);
            }
            else
            {
                pCurCmd = CmdLookup(inBuf);
            }
            if (pCurCmd != NULL)
            {
                usart_wait_send_ready(usart);
                pCurCmd->CmdFunc(this, args);
            }
            else if (my_strlen(inBuf) > 1 && enabled)
            {
                Send("Unknown command sequence\r\n");
            }
            // Reset the RX DMA ready for new command
            ResetRxDMA();
        }
        else if (inBuf[0] == '!' && pCurCmd != NULL)
        {
            // Special handling for '!' commands.
            ResetRxDMA();
            lastIdx = 0;
            pCurCmd->CmdFunc(this, args);
        }
    }
}


/*
 * Revision 1 hardware can only use synchronous sending as the DMA channel is
 * occupied by the encoder timer (TIM3, channel 3).
 * All other hardware can use DMA for seamless sending of data. We use double
 * buffering, so while one buffer is sent by DMA we can prepare the other
 * buffer to go next.
*/
void Terminal::PutChar(char c)
{
   if (!txDmaEnabled)
   {
      usart_send_blocking(usart, c);
   }
   else if (c == '\n' || curIdx == (bufSize - 1))
   {
      outBuf[curBuf][curIdx] = c;
      SendCurrentBuffer(curIdx + 1);
      curIdx = 0;
   }
   else
   {
      outBuf[curBuf][curIdx] = c;
      curIdx++;
   }
}

void Terminal::SendBinary(const uint8_t* data, uint32_t len)
{
   uint32_t limitedLen = len < bufSize ? len : bufSize;

   for (uint32_t i = 0; i < limitedLen; i++)
      outBuf[curBuf][i] = data[i];
   SendCurrentBuffer(limitedLen);
}

void Terminal::SendBinary(const uint32_t* data, uint32_t len)
{
   uint32_t limitedLen = len < (bufSize / sizeof(uint32_t)) ? len : bufSize / sizeof(uint32_t);
   memcpy32((int*)outBuf[curBuf], (int*)data, limitedLen);
   SendCurrentBuffer(limitedLen * sizeof(uint32_t));
}

bool Terminal::KeyPressed()
{
   return usart_get_flag(usart, USART_SR_RXNE);
}

void Terminal::FlushInput()
{
   usart_recv(usart);
}

void Terminal::DisableTxDMA()
{
   txDmaEnabled = false;
#ifdef STM32F1
   dma_disable_channel(hw->dmactl, hw->dmatx);
#else
   dma_disable_stream(hw->dmactl, hw->dmatx);
#endif 
   usart_disable_tx_dma(usart);
}


#ifndef STM32F1

void Terminal::ResetTxDMA()
{
    dma_disable_stream(hw->dmactl, hw->dmatx);  // Ensure DMA disabled first
    while (DMA2_S7CR & DMA_SxCR_EN); // Wait until fully disabled
    dma_stream_reset(hw->dmactl, hw->dmatx);
    dma_channel_select(hw->dmactl, hw->dmatx, hw->dmaChannel); 
    dma_set_transfer_mode(hw->dmactl, hw->dmatx, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_peripheral_address(hw->dmactl, hw->dmatx, (uint32_t)&USART_DR(usart));
    dma_set_memory_address(hw->dmactl, hw->dmatx, (uint32_t)outBuf);
    dma_set_number_of_data(hw->dmactl, hw->dmatx, bufSize);
    dma_set_memory_size(hw->dmactl, hw->dmatx, DMA_SxCR_MSIZE_8BIT);
    dma_enable_memory_increment_mode(hw->dmactl, hw->dmatx);
    dma_enable_stream(hw->dmactl, hw->dmatx); 
}

#endif

void Terminal::ResetRxDMA()
{

#ifdef STM32F1

   dma_disable_channel(hw->dmactl, hw->dmarx);
   dma_set_memory_address(hw->dmactl, hw->dmarx, (uint32_t)inBuf);
   dma_set_number_of_data(hw->dmactl, hw->dmarx, bufSize);
   dma_enable_channel(hw->dmactl, hw->dmarx);

# else

    dma_disable_stream(hw->dmactl, hw->dmarx);      // Disable DMA to prevent conflicts
    while (DMA2_S2CR & DMA_SxCR_EN);;  // Wait until fully disabled
    
    dma_clear_interrupt_flags(hw->dmactl, hw->dmarx, DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_DMEIF | DMA_FEIF);
    dma_stream_reset(hw->dmactl, hw->dmarx);

    dma_channel_select(hw->dmactl, hw->dmarx, hw->dmaChannel); 
    dma_set_transfer_mode(hw->dmactl, hw->dmarx, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_size(hw->dmactl, hw->dmarx, DMA_SxCR_PSIZE_8BIT);
    dma_set_memory_size(hw->dmactl, hw->dmarx, DMA_SxCR_MSIZE_8BIT);
    dma_enable_memory_increment_mode(hw->dmactl, hw->dmarx);
    dma_enable_circular_mode(hw->dmactl, hw->dmarx); 
    dma_set_peripheral_address(hw->dmactl, hw->dmarx, (uint32_t)&USART_DR(usart));
    dma_set_memory_address(hw->dmactl, hw->dmarx, (uint32_t)inBuf);
    dma_set_number_of_data(hw->dmactl, hw->dmarx, bufSize);

    dma_enable_stream(hw->dmactl, hw->dmarx);

#endif

}

void Terminal::EnableUart(char* arg)
{
   arg = my_trim(arg);
   int val = my_atoi(arg);

   if (val == nodeId)
   {
      enabled = true;
#ifdef STM32F1
    // STM32F1: Configure GPIO mode and alternate function for output
    gpio_set_mode(remap ? hw->port_re : hw->port, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, remap ? hw->pin_re : hw->pin);
#else
    // STM32F4: Configure GPIO mode and set alternate function
    gpio_mode_setup(hw->port_tx, GPIO_MODE_AF, GPIO_PUPD_NONE, hw->pin_tx);
    gpio_set_af(hw->port_tx, GPIO_AF7, hw->pin_tx);
#endif
      Send("OK\r\n");
   }
   else
   {
      enabled = false;
#ifdef STM32F1
      gpio_set_mode(remap ? hw->port_re : hw->port, GPIO_MODE_INPUT,
                    GPIO_CNF_INPUT_FLOAT, remap ? hw->pin_re : hw->pin);
#else
    gpio_mode_setup(hw->port_tx, GPIO_MODE_INPUT, GPIO_PUPD_NONE, hw->pin_tx);
#endif
   }
}

void Terminal::FastUart(char *arg)
{
   arg = my_trim(arg);
   int baud = arg[0] == '0' ? USART_BAUDRATE : (arg[0] == '2' ? 2250000 : 921600);
   if (enabled)
   {
      char buf[10];
      my_ltoa(buf, baud, 10);
      Send("\nOK\r\n");
      Send("Baud rate now ");
      Send(buf);
      Send("\r\n");
   }
   usart_set_baudrate(usart, baud);
   usart_set_stopbits(usart, USART_STOPBITS_1);
}

void Terminal::Echo(char* arg)
{
   arg = my_trim(arg);

   if (arg[0] != 0)
      echo = arg[0] == '0' ? false : true;

   if (echo)
      Send("Echo on\r\n");
   else
      Send("Echo off\r\n");
}

const TERM_CMD* Terminal::CmdLookup(char *buf)
{
   const TERM_CMD *pCmd = termCmds;

   if (!enabled) return NULL;

   for (; NULL != pCmd->cmd; pCmd++)
   {
      if (0 == my_strcmp(buf, pCmd->cmd))
      {
         break;
      }
   }
   if (NULL == pCmd->cmd)
   {
      pCmd = NULL;
   }
   return pCmd;
}

void Terminal::Send(const char *str)
{
   SendBinary((const uint8_t*)str, my_strlen(str));
}

void Terminal::SendCurrentBuffer(uint32_t len)
{
   while (!dma_get_interrupt_flag(hw->dmactl, hw->dmatx, DMA_TCIF) && !firstSend);

#ifdef STM32F1
   dma_disable_channel(hw->dmactl, hw->dmatx);
#else
   dma_disable_stream(hw->dmactl, hw->dmatx);
#endif
   dma_set_number_of_data(hw->dmactl, hw->dmatx, len);
   dma_set_memory_address(hw->dmactl, hw->dmatx, (uint32_t)outBuf[curBuf]);
   dma_clear_interrupt_flags(hw->dmactl, hw->dmatx, DMA_TCIF);
#ifdef STM32F1
   dma_enable_channel(hw->dmactl, hw->dmatx);
#else
   dma_enable_stream(hw->dmactl, hw->dmatx);
#endif
   curBuf = !curBuf; //switch buffers
   firstSend = false; //only needed once so we don't get stuck in the while loop above
}

//Backward compatibility for printf
extern "C" void putchar(int c)
{
   Terminal::defaultTerminal->PutChar(c);
}
