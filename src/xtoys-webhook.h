// Public interface for the XToys webhook integration (WiFi + websocket).
// Kept fully separate from the BLE control path in main.cpp.
#pragma once

#include "eject_protocol.h"

void setupXtoysWebhook();
void xtoysWebhookLoop();

// Mirrors an outbound BLE status/heartbeat message to the XToys webhook, if connected.
void xtoysSendMessage(const struct_message &msg);
