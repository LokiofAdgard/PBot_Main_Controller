#ifndef SERVER_H
#define SERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

#include "structs.h"

#define SPEED     30
#define SPEED_LOW 20

void server_init(void);

#endif