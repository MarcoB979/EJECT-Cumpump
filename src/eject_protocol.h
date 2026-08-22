// Shared command/message definitions used by both main.cpp (BLE) and xtoys-webhook.cpp
#pragma once

#include <Arduino.h>

#define OSSM_ID  1  // OSSM_ID Default can be changed with M5 Remote in the Future will be Saved in EPROOM
#define EJECT_ID 2  // M5_ID Default can be changed with M5 Remote in the Future will be Saved in EPROOM
#define M5_ID    99 // M5_ID Default can be changed with M5 Remote in the Future will be Saved in EPROOM
#define XTOYS_ID 77 // Sender ID used for commands received via the XToys webhook

// Command States
#define CUMSPEED  20
#define CUMTIME   21
#define CUMSIZE   22
#define CUMACCEL  23
#define OFF 10
#define ON  11

#define CONNECT 88
#define HEARTBEAT 99

typedef struct struct_message {
  float speed;
  float depth;
  float stroke;
  float sensation;
  float pattern;
  bool rstate;
  bool connected;
  bool heartbeat;
  int command;
  float value;
  int target;
  int sender;
} struct_message;

// Defined in main.cpp; shared so xtoys-webhook.cpp can deliver/observe commands.
extern struct_message Sent;
extern struct_message received;
extern volatile bool g_bleMessagePending;
