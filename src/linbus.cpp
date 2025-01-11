/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include "linbus.h"

#define HWINFO_ENTRIES (sizeof(hwInfo) / sizeof(struct HwInfo))

#ifdef STM32F1
const LinBus::HwInfo LinBus::hwInfo[] = {
    {USART1, DMA_CHANNEL4, DMA_CHANNEL5, GPIOA, GPIO_USART1_TX},
    {USART2, DMA_CHANNEL7, DMA_CHANNEL6, GPIOA, GPIO_USART2_TX},
    {USART3, DMA_CHANNEL2, DMA_CHANNEL3, GPIOB, GPIO_USART3_TX},
};
#else
const LinBus::HwInfo LinBus::hwInfo[] = {
   // chosen from F405 reference manual
    {(uint32_t)USART1, (uint32_t)DMA2, DMA_STREAM7, DMA_STREAM2, 4, (uint32_t)GPIOA, GPIO9},  
    {(uint32_t)USART2, (uint32_t)DMA1, DMA_STREAM6, DMA_STREAM5, 4, (uint32_t)GPIOA, GPIO2},  
    {(uint32_t)USART3, (uint32_t)DMA1, DMA_STREAM3, DMA_STREAM1, 4, (uint32_t)GPIOB, GPIO10}  
};
#endif

/** \brief Create a new LIN bus object and initialize USART, GPIO and DMA
 * \pre According USART, GPIO and DMA clocks must be enabled
 * \param usart USART base address
 * \param baudrate 9600 or 19200
 *
 */
LinBus::LinBus(uint32_t usart_port, int baudrate)
   : usart(usart_port)
{
   hw = hwInfo;

   for (uint32_t i = 0; i < HWINFO_ENTRIES; i++)
   {
      if (hw->usart == usart) break;
      hw++;
   }
#ifdef STM32F1
   gpio_set_mode(hw->port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, hw->pin);
#else
   gpio_mode_setup(hw->port, GPIO_MODE_AF, GPIO_PUPD_NONE, hw->pin);
   gpio_set_af(hw->port, GPIO_AF7, hw->pin);
#endif

   usart_set_baudrate(usart, baudrate);
   usart_set_databits(usart, 8);
   usart_set_stopbits(usart, USART_STOPBITS_1);
   usart_set_mode(usart, USART_MODE_TX_RX);
   usart_set_parity(usart, USART_PARITY_NONE);
   usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
   USART_CR2(usart) |= USART_CR2_LINEN;
   usart_enable_tx_dma(usart);
   usart_enable_rx_dma(usart);

#ifdef STM32F1
   dma_channel_reset(DMA1, hw->dmatx);
   dma_set_read_from_memory(DMA1, hw->dmatx);
   dma_set_peripheral_address(DMA1, hw->dmatx, (uint32_t)&USART_DR(usart));
   dma_set_memory_address(DMA1, hw->dmatx, (uint32_t)sendBuffer);
   dma_set_peripheral_size(DMA1, hw->dmatx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, hw->dmatx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, hw->dmatx);

   dma_channel_reset(DMA1, hw->dmarx);
   dma_set_peripheral_address(DMA1, hw->dmarx, (uint32_t)&USART_DR(usart));
   dma_set_peripheral_size(DMA1, hw->dmarx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(DMA1, hw->dmarx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(DMA1, hw->dmarx);
#else
   // Configure TX DMA
   dma_stream_reset(hw->dma_controller, hw->dmatx);
   dma_set_priority(hw->dma_controller, hw->dmatx, DMA_SxCR_PL_HIGH);
   dma_set_transfer_mode(hw->dma_controller, hw->dmatx, DMA_SxCR_DIR_MEM_TO_PERIPHERAL); // TX: Memory to Peripheral
   dma_set_peripheral_address(hw->dma_controller, hw->dmatx, (uint32_t)&USART_DR(usart));
   dma_set_memory_address(hw->dma_controller, hw->dmatx, (uint32_t)sendBuffer);
   dma_set_number_of_data(hw->dma_controller, hw->dmatx, sizeof(sendBuffer));
   dma_set_peripheral_size(hw->dma_controller, hw->dmatx, DMA_SxCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dma_controller, hw->dmatx, DMA_SxCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dma_controller, hw->dmatx);
   dma_set_priority(hw->dma_controller, hw->dmatx, DMA_SxCR_PL_HIGH);
   // Select the channel for TX
   dma_channel_select(hw->dma_controller, hw->dmatx, DMA_SxCR_CHSEL_4);

   // Configure RX DMA
   dma_stream_reset(hw->dma_controller, hw->dmarx);
   dma_set_priority(hw->dma_controller, hw->dmarx, DMA_SxCR_PL_HIGH);
   dma_set_transfer_mode(hw->dma_controller, hw->dmarx, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);  // RX: Peripheral to Memory
   dma_set_peripheral_address(hw->dma_controller, hw->dmarx,(uint32_t)&USART_DR(usart));
   dma_set_memory_address(hw->dma_controller, hw->dmarx, (uint32_t)recvBuffer);
   dma_set_number_of_data(hw->dma_controller, hw->dmarx, sizeof(recvBuffer));
   dma_set_peripheral_size(hw->dma_controller, hw->dmarx, DMA_SxCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dma_controller, hw->dmarx, DMA_SxCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dma_controller, hw->dmarx);
   // Select the channel for RX
   dma_channel_select(hw->dma_controller, hw->dmarx, DMA_SxCR_CHSEL_4);
#endif
   usart_enable(usart);
}

/** \brief Send data on LIN bus
 *
 * \param id feature ID
 * \param data payload data, if any
 * \param len length of payload, if any
 *
 */
void LinBus::Request(uint8_t id, uint8_t* data, uint8_t len)
{
   int sendLen = len == 0 ? 2 : len + 3;

   if (len > 8) return;
#ifdef STM32F1
   dma_disable_channel(DMA1, hw->dmatx);
   dma_disable_channel(DMA1, hw->dmarx);
#else
   dma_disable_stream(hw->dma_controller, hw->dmatx);
   dma_disable_stream(hw->dma_controller, hw->dmarx);
#endif
   dma_set_number_of_data(DMA1, hw->dmatx, sendLen);
   dma_set_memory_address(DMA1, hw->dmarx, (uint32_t)recvBuffer);
   dma_set_number_of_data(DMA1, hw->dmarx, sizeof(recvBuffer));

   sendBuffer[0] = 0x55; //Sync
   sendBuffer[1] = Parity(id);

   for (uint8_t i = 0; i < len; i++)
      sendBuffer[i + 2] = data[i];

   sendBuffer[len + 2] = Checksum(sendBuffer[1], data, len);

   dma_clear_interrupt_flags(DMA1, hw->dmatx, DMA_TCIF);

   USART_CR1(usart) |= USART_CR1_SBK;
#ifdef STM32F1
   dma_enable_channel(DMA1, hw->dmatx);
   dma_enable_channel(DMA1, hw->dmarx);
#else
   dma_enable_stream(hw->dma_controller, hw->dmatx);
   dma_enable_stream(hw->dma_controller, hw->dmarx);
#endif
}

/** \brief Check whether we received valid data with given PID and length
 *
 * \param pid Feature ID to check for
 * \param requiredLen Length of data we expect
 * \return true if data with given properties was received
 *
 */
bool LinBus::HasReceived(uint8_t id, uint8_t requiredLen)
{
   int numRcvd = dma_get_number_of_data(DMA1, hw->dmarx);
   int receiveIdx = sizeof(recvBuffer) - numRcvd;

   if (requiredLen > 8) return false;

   uint8_t pid = Parity(id);

   if (receiveIdx == (requiredLen + payloadIndex + 1) && recvBuffer[pidIndex] == pid)
   {
      uint8_t checksum = Checksum(recvBuffer[pidIndex], &recvBuffer[payloadIndex], requiredLen);

      return checksum == recvBuffer[requiredLen + payloadIndex];
   }

   return false;
}

/** \brief Calculate LIN checksum
 *
 * \param pid ID with parity
 * \param data uint8_t*
 * \param len int
 * \return checksum
 *
 */
uint8_t LinBus::Checksum(uint8_t pid, uint8_t* data, int len)
{
   uint8_t checksum = pid;

   for (int i = 0; i < len; i++)
   {
      uint16_t tmp = (uint16_t)checksum + (uint16_t)data[i];
      if (tmp > 256) tmp -= 255;
      checksum = tmp;
   }
   return checksum ^ 0xff;
}

uint8_t LinBus::Parity(uint8_t id)
{
   bool p1 = !(((id & 0x2) > 0) ^ ((id & 0x8) > 0) ^ ((id & 0x10) > 0) ^ ((id & 0x20) > 0));
   bool p0 = ((id & 0x1) > 0) ^ ((id & 0x2) > 0) ^ ((id & 0x4) > 0) ^ ((id & 0x10) > 0);

   return id | p1 << 7 | p0 << 6;
}
