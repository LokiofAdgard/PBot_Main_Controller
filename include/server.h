#ifndef SERVER_H
#define SERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

#include "structs.h"

#define SPEED     400
#define SPEED_LOW 350

void server_init(void);

#endif