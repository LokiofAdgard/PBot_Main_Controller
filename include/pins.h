#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// LED
#define LED_RGB GPIO_NUM_48
#define LED_STR GPIO_NUM_21

// UART
#define TX_1 GPIO_NUM_17
#define RX_1 GPIO_NUM_18

#define TX_2 GPIO_NUM_40
#define RX_2 GPIO_NUM_39

// I2C
#define SDA1 GPIO_NUM_6
#define SCL1 GPIO_NUM_7

#define SCL2 GPIO_NUM_3
#define SDA2 GPIO_NUM_8

// SPI
#define CSN  GPIO_NUM_9
#define MOSI GPIO_NUM_10
#define MISO GPIO_NUM_11
#define SCK  GPIO_NUM_12
#define CE   GPIO_NUM_13

// CAN
#define CAN_TX GPIO_NUM_15
#define CAN_RX GPIO_NUM_16

#endif