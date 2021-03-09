/**
  ******************************************************************************
  * @file spi.c
  * @brief This file contains the main function for the BLDC motor control
  * @author Neidermeier
  * @version
  * @date March-2021
  ******************************************************************************
  */
#ifndef SPI_H
#define SPI_H

/* Includes ------------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/


/*
 *  timer1 free running, time between periodic task is 0x2080
 */
#define TIME_OUT_0 0x1000

#define RX_BUF_SZ 16


/* Declarations --------------------------------------------------------------*/

extern uint8_t spi_rx_buf[RX_BUF_SZ]; // tmp test


/* Function prototypes -------------------------------------------------------*/

int SPI_read_write_b(uint8_t * chbuf, uint8_t data, uint16_t time_out);
void SPI_controld(void);


#endif
