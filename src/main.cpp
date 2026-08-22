#include <Arduino.h>          // Basic Needs

// Include the Stepper library:
#include <ESP_FlexyStepper.h>
#include <NimBLEDevice.h>
#include "eject_protocol.h"
#include "xtoys-webhook.h"

// IO pin assignments esp32
//const int MOTOR_STEP_PIN = 17;
//const int MOTOR_DIRECTION_PIN = 4;
//const int MOTOR_ENA_PIN = 16;


// IO pin assignments C3 (super mini) and C3 Pro (mini)
//const int MOTOR_ENA_PIN = 10;
//const int MOTOR_DIRECTION_PIN = 1;
//const int MOTOR_STEP_PIN = 0;



// IO pin assignments C3 (super mini) and C3 Pro (mini) in surringe 
const int MOTOR_ENA_PIN = 2;
const int MOTOR_DIRECTION_PIN = 3;
const int MOTOR_STEP_PIN = 4;

///////////////////////////////////////////
////
////  To Debug or not to Debug
////
///////////////////////////////////////////

// Uncomment the following line if you wish to print DEBUG info
#define DEBUG 

#ifdef DEBUG
#define LogDebug(...) Serial.println(__VA_ARGS__)
#define LogDebugFormatted(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebug(...) ((void)0)
#define LogDebugFormatted(...) ((void)0)
#endif


#define DEBUGPRIO 

#ifdef DEBUGPRIO
#define LogDebugPRIO(...) Serial.println(__VA_ARGS__)
#define LogDebugFormattedPRIO(...) Serial.printf(__VA_ARGS__)
#else
#define LogDebugPRIO(...) ((void)0)
#define LogDebugFormattedPRIO(...) ((void)0)
#endif


#define OSSM 1
#define CUM 2

// create the stepper motor object
ESP_FlexyStepper stepper;

// Command States, IDs and struct_message are shared via eject_protocol.h

unsigned long previousMillis = 0;
long interval = 1000;

constexpr bool USE_STEPPER_SERVICE = (portNUM_PROCESSORS > 1);



int squirts = 0;

float cum_time = 0.0;
float cum_speed = 1000.0;
float cum_size = 0.0;
float cum_accel = 50.0; // 0..100, where lower values mean longer ramp-up time.
float cum_on = 0;

// Positive/negative controls the physical direction of a squirt stroke.
// Flip the sign if the hardware is wired/mechanically reversed.
const float CUM_SIZE_SCALE = 26.5f;
const bool INVERT_CUM_DIRECTION = true;
// Divides commanded speed. 10.0 means run at one-tenth speed.
const float CUM_SPEED_RATIO = 20.0f;

static float computeStrokeAccelSps2(float speedSps, float strokeSteps, float accelSetting)
{
  // accelSetting behavior:
  // 100 => near-instant ramp, 50 => ramp for half the squirt, 10 => ramp for 90% of squirt.
  float clampedSetting = constrain(accelSetting, 0.0f, 100.0f);
  float rampFraction = (100.0f - clampedSetting) / 100.0f;

  if (speedSps < 1.0f) {
    speedSps = 1.0f;
  }

  float absStrokeSteps = fabsf(strokeSteps);
  if (absStrokeSteps < 1.0f) {
    return 10.0f;
  }

  if (rampFraction <= 0.0f) {
    // Instant-ramp request: use a high acceleration.
    return 100000.0f;
  }

  float squirtDurationSec = absStrokeSteps / speedSps;
  float rampTimeSec = squirtDurationSec * rampFraction;
  if (rampTimeSec < 0.02f) {
    rampTimeSec = 0.02f;
  }

  float accelSps2 = speedSps / rampTimeSec;
  if (accelSps2 < 10.0f) {
    accelSps2 = 10.0f;
  }
  return accelSps2;
}

// Track remaining steps for current squirt
long stepsRemainingInSquirt = 0;
const long CHUNK_SIZE = 100; // Small moves to allow parameter updates

bool g_squirtActive = false;
float g_squirtAbsSteps = 0.0f;

static float computeProfiledSpeedSps(float baseSpeedSps, float accelSetting, float progress)
{
  // accelSetting behavior:
  // 100 => almost full-time cruise (minimal ramp)
  // 50  => 50% of stroke spent in ramp (accel+decel), 50% at cruise
  // 10  => 90% of stroke spent in ramp (accel+decel), 10% at cruise
  float clampedAccel = constrain(accelSetting, 0.0f, 100.0f);
  float rampTotalFraction = (100.0f - clampedAccel) / 100.0f;
  float rampSideFraction = rampTotalFraction * 0.5f;

  float p = constrain(progress, 0.0f, 1.0f);
  float factor = 1.0f;

  if (rampSideFraction > 0.0001f) {
    if (p < rampSideFraction) {
      factor = p / rampSideFraction;
    } else if (p > (1.0f - rampSideFraction)) {
      factor = (1.0f - p) / rampSideFraction;
    } else {
      factor = 1.0f;
    }
  }

  // Keep movement alive at beginning/end of the profile.
  if (factor < 0.08f) {
    factor = 0.08f;
  }

  float speed = baseSpeedSps * factor;
  if (speed < 1.0f) {
    speed = 1.0f;
  }
  return speed;
}

// Variable to store if sending data was successful
float sent_speed;
float sent_depth;
float sent_stroke;
float sent_sensation;
float sent_pattern;
bool sent_rstate;
bool sent_connected;
int sent_command;
float sent_value;
int sent_target;
int sent_sender;

float received_speed;
float received_depth;
float received_stroke;
float received_sensation;
float received_pattern;
bool received_rstate;
bool received_connected;
bool received_heartbeat;
int received_target;
int received_sender;

bool M5_paired = false;

struct_message Sent;
struct_message received;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 5000UL;
static const char *EJECT_BLE_DEVICE_NAME = "Eject";
static const char *EJECT_BLE_SERVICE_UUID = "5f8bb7f0-9f17-4aa8-9c42-3d8b8b4d9001";
static const char *EJECT_BLE_RX_UUID = "5f8bb7f1-9f17-4aa8-9c42-3d8b8b4d9001";
static const char *EJECT_BLE_TX_UUID = "5f8bb7f2-9f17-4aa8-9c42-3d8b8b4d9001";

NimBLEServer *g_bleServer = nullptr;
NimBLECharacteristic *g_bleRxChar = nullptr;
NimBLECharacteristic *g_bleTxChar = nullptr;
volatile bool g_bleClientConnected = false;
volatile bool g_bleMessagePending = false;
unsigned long g_lastHeartbeatMs = 0;

static bool sendBleMessage(const struct_message &msg) {
  xtoysSendMessage(msg);

  if (!g_bleClientConnected || g_bleTxChar == nullptr) {
    LogDebugFormatted("BLE TX SKIP cmd=%d val=%.2f target=%d sender=%d hb=%d\n",
                      msg.command,
                      msg.value,
                      msg.target,
                      msg.sender,
                      msg.heartbeat ? 1 : 0);
    return false;
  }

  LogDebugFormatted("BLE TX cmd=%d val=%.2f target=%d sender=%d hb=%d\n",
                    msg.command,
                    msg.value,
                    msg.target,
                    msg.sender,
                    msg.heartbeat ? 1 : 0);
  g_bleTxChar->setValue((uint8_t *)&msg, sizeof(msg));
  g_bleTxChar->notify();
  return true;
}

class EjectBleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
    (void)pServer;
    (void)connInfo;
    g_bleClientConnected = true;
    LogDebugPRIO("BLE client connected");
  }

  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
    (void)pServer;
    (void)connInfo;
    (void)reason;
    g_bleClientConnected = false;
    M5_paired = false;
    NimBLEDevice::startAdvertising();
    LogDebugPRIO("BLE client disconnected");
  }
};

class EjectBleRxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
    (void)connInfo;
    std::string value = pCharacteristic->getValue();
    if (value.size() != sizeof(struct_message)) {
      LogDebugFormatted("BLE RX invalid size: %d (expected %d)\n", (int)value.size(), (int)sizeof(struct_message));
      return;
    }

    memcpy(&received, value.data(), sizeof(received));
    g_bleMessagePending = true;
  }
};

// Callback when data is received
void OnDataRecv() {
  LogDebug("Received step 1");

  LogDebug("Received Command: ");
  LogDebug(received.command);
  LogDebug("Received value: ");
  LogDebug(received.value);
  LogDebug("Received to target ID: ");
  LogDebug(received.target);
  LogDebug("Received from sender: ");
  LogDebug(received.sender);
  if(!M5_paired){
    LogDebug("M5 is not paired");}
  if (received.sender == M5_ID && !M5_paired) {
    M5_paired = true;
    LogDebug("M5 remote Connected");
  }

  if(received.target != EJECT_ID && received.target != 0) {
    LogDebug("Received BLE data not intended for EJECT, ignoring.");
    return;
  }

    LogDebug("Received command ");
    LogDebug(received.command);
    switch(received.command)
    {
      case CUMSPEED:
      {
      cum_speed = (received.value * 500) / CUM_SPEED_RATIO;  //150 for suringe pump  / 500 for cum pump  
      }
      break;
      case CUMTIME:
      {
      interval = (long)received.value;
      if (interval < 0) {
        interval = 0;
      }
      
      }
      break;
      case CUMSIZE:
      {
      const float directionSign = INVERT_CUM_DIRECTION ? -1.0f : 1.0f;
      cum_size = received.value * CUM_SIZE_SCALE * directionSign;
      }
      break;
      case CUMACCEL:
      {
      cum_accel = received.value;
      }
      break;
      case ON:
      {
//        stepper.releaseEmergencyStop();
        cum_on=1;
        squirts=0;
        g_squirtActive = false;
      }
      break;
      case OFF:
      {
      cum_on=0;
      g_squirtActive = false;
//      stepper.emergencyStop(true);
      }
      break;
      case CONNECT or HEARTBEAT:
      {

        Sent.target = M5_ID;
        Sent.sender = EJECT_ID;
        Sent.command = HEARTBEAT;
        Sent.heartbeat = true;
        bool result = sendBleMessage(Sent);
        LogDebug(result ? 1 : 0);
        
        if (result) {
          M5_paired = true;
          LogDebug("M5 remote Connected");
        }
    
      }
      break;
    }
    LogDebugFormattedPRIO("ONDATARECV:  CUMSPEED: %.2f, CUMTIME: %ld, CUMSIZE: %.2f, CUMACCEL: %.2f\n", cum_speed, interval, cum_size, cum_accel);
}

void setupBLEComm() {
    NimBLEDevice::init(EJECT_BLE_DEVICE_NAME);
    g_bleServer = NimBLEDevice::createServer();
    g_bleServer->setCallbacks(new EjectBleServerCallbacks());

    NimBLEService *service = g_bleServer->createService(EJECT_BLE_SERVICE_UUID);
    g_bleRxChar = service->createCharacteristic(
        EJECT_BLE_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    g_bleTxChar = service->createCharacteristic(
        EJECT_BLE_TX_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    g_bleRxChar->setCallbacks(new EjectBleRxCallbacks());

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(EJECT_BLE_SERVICE_UUID);
    advertising->enableScanResponse(true);
    advertising->start();

    LogDebugPRIO("BLE advertising started for Eject");
}

void setup()
{  
    Serial.begin(115200);
    LogDebug("EJECT Cumpump starting up");
    setupBLEComm();
    setupXtoysWebhook();

    // connect and configure the stepper motor to its IO pins
    pinMode(MOTOR_ENA_PIN, OUTPUT);
    digitalWrite(MOTOR_ENA_PIN, LOW);
    pinMode(MOTOR_DIRECTION_PIN, OUTPUT);
    // After connectToPins, FlexyStepper drives MOTOR_DIRECTION_PIN per move command sign.
    digitalWrite(MOTOR_DIRECTION_PIN, LOW);
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

  if (USE_STEPPER_SERVICE)
  {
    stepper.startAsService(1);
    LogDebug("Stepper service enabled on secondary core");
  }
  else
  {
    LogDebug("Stepper service disabled on single-core target");
  }

  stepper.setSpeedInStepsPerSecond(500);
  stepper.setAccelerationInStepsPerSecondPerSecond(200);
  stepper.setDecelerationInStepsPerSecondPerSecond(100);
  stepper.setTargetPositionRelativeInSteps(-500);
  
  cum_on=0;
  squirts=0;
  LogDebug("Status = ");
  LogDebug (cum_on);
}

// ============ SERVICE MODE LOOP (REPLACE WITH OLD CODE TO REVERT) ============
void loop()
{
  xtoysWebhookLoop();

  if (g_bleMessagePending) {
    g_bleMessagePending = false;
    OnDataRecv();
  }

  if (cum_on==1)
  {
    // Update speed/accel - applies immediately to ongoing motion.
    if (cum_speed < 1.0f) {cum_speed = 1.0f;}
    if (cum_accel < 0.0f) {cum_accel = 0.0f;}
    if (cum_accel > 100.0f) {cum_accel = 100.0f;}
    stepper.setSpeedInStepsPerSecond(cum_speed);
    float accelToApply = computeStrokeAccelSps2(cum_speed, cum_size, cum_accel);
    stepper.setAccelerationInStepsPerSecondPerSecond(accelToApply);
    stepper.setDecelerationInStepsPerSecondPerSecond(accelToApply);
    
    if (interval == 0)
    {
      // Continuous mode (time=0): keep setting new target.
      if (stepper.getDistanceToTargetSigned() == 0)
      {
        digitalWrite(MOTOR_ENA_PIN, LOW);
        stepper.setTargetPositionRelativeInSteps(-100000); // Large move
        LogDebugPRIO("Continuous running - speed:" + String(cum_speed));
      }
    }
    else
    {
      // Normal squirt mode
      if (!g_squirtActive && squirts < interval && stepper.getDistanceToTargetSigned() == 0)
      {
        digitalWrite(MOTOR_ENA_PIN, LOW);
        float strokeAccel = computeStrokeAccelSps2(cum_speed, cum_size, cum_accel);
        stepper.setAccelerationInStepsPerSecondPerSecond(strokeAccel);
        stepper.setDecelerationInStepsPerSecondPerSecond(strokeAccel);
        g_squirtAbsSteps = fabsf(cum_size);
        if (g_squirtAbsSteps < 1.0f) {
          g_squirtAbsSteps = 1.0f;
        }
        g_squirtActive = true;
        stepper.setTargetPositionRelativeInSteps(cum_size);
        LogDebugPRIO("Starting squirt " + String(squirts + 1) + "/" + String(interval) + " size:" + String(cum_size) + " speed:" + String(cum_speed) + " accel%:" + String(cum_accel) + " accelSps2:" + String(strokeAccel));
      }

      if (g_squirtActive)
      {
        float remainingSteps = fabsf((float)stepper.getDistanceToTargetSigned());
        float traveledSteps = g_squirtAbsSteps - remainingSteps;
        if (traveledSteps < 0.0f) {
          traveledSteps = 0.0f;
        }
        float progress = traveledSteps / g_squirtAbsSteps;
        float profiledSpeed = computeProfiledSpeedSps(cum_speed, cum_accel, progress);
        stepper.setSpeedInStepsPerSecond(profiledSpeed);

        if (stepper.getDistanceToTargetSigned() == 0)
        {
          g_squirtActive = false;
          squirts = squirts + 1;
        }
      }
      
      // All squirts done?
      if (!g_squirtActive && squirts >= interval && stepper.getDistanceToTargetSigned() == 0)
      {
        cum_on = 0;
        squirts = 0;
        digitalWrite(MOTOR_ENA_PIN, HIGH);
        LogDebug("All squirts done");
        Sent.command = OFF;
        Sent.sender = EJECT_ID;
        Sent.target = M5_ID;
        if (!sendBleMessage(Sent)) {
          LogDebug("Sending to M5 'squirts done' failed");
        }
      }
    }
  }
  else  // cum_on == 0 (stopped)
  {
    squirts = 0;
    g_squirtActive = false;
    stepper.setTargetPositionToStop(); // Decelerate to stop
    digitalWrite(MOTOR_ENA_PIN, HIGH);
  }
  if (!USE_STEPPER_SERVICE)
  {
    stepper.processMovement();
  }

  if (g_bleClientConnected && (millis() - g_lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
    Sent.command = HEARTBEAT;
    Sent.sender = EJECT_ID;
    Sent.heartbeat = true;
    Sent.target = M5_ID;
    if (!sendBleMessage(Sent)) {
      LogDebug("failed to send heartbeat");
    }
    g_lastHeartbeatMs = millis();
  }
}
