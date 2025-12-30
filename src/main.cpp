#include <Arduino.h>          // Basic Needs

// Include the Stepper library:
#include <ESP_FlexyStepper.h>
#include <esp_now.h>
#include <WiFi.h>

// IO pin assignments
const int MOTOR_STEP_PIN = 17;
const int MOTOR_DIRECTION_PIN = 4;
const int MOTOR_ENA_PIN = 16;


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

#define OSSM 1
#define CUM 2

#define OSSM_ID  1 //OSSM_ID Default can be changed with M5 Remote in the Future will be Saved in EPROOM
#define EJECT_ID 2 //M5_ID Default can be changed with M5 Remote in the Future will be Saved in EPROOM
#define M5_ID 99 //M5_ID Default can be changed with M5 Remote in the Future will be Saved in EPROOM

uint8_t M5_Remote_Address[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// create the stepper motor object
ESP_FlexyStepper stepper;

// Command States

#define CUMSPEED  20
#define CUMTIME   21
#define CUMSIZE   22
#define CUMACCEL  23
#define OFF 10
#define ON  11

#define CONNECT 88
#define HEARTBEAT 99


unsigned long previousMillis = 0;
long interval = 1000;



int squirts = 0;

float cum_time = 0.0;
float cum_speed = 1000.0;
float cum_size = 0.0;
float cum_accel = 0.0;
float cum_on = 0;

// Variable to store if sending data was successful
String success;

float out_esp_speed;
float out_esp_depth;
float out_esp_stroke;
float out_esp_sensation;
float out_esp_pattern;
bool out_esp_rstate;
bool out_esp_connected;
int out_esp_command;
float out_esp_value;
int out_esp_target;
int out_esp_sender;

float incoming_esp_speed;
float incoming_esp_depth;
float incoming_esp_stroke;
float incoming_esp_sensation;
float incoming_esp_pattern;
bool incoming_esp_rstate;
bool incoming_esp_connected;
bool incoming_esp_heartbeat;
int incoming_esp_target;
int incoming_esp_sender;

typedef struct struct_message {
  float esp_speed;
  float esp_depth;
  float esp_stroke;
  float esp_sensation;
  float esp_pattern;
  bool esp_rstate;
  bool esp_connected;
  bool esp_heartbeat;
  int esp_command;
  float esp_value;
  int esp_target;
  int esp_sender;
} struct_message;

bool M5_paired = false;

struct_message outgoingcontrol;
struct_message incomingcontrol;

esp_now_peer_info_t peerInfo;


TaskHandle_t eRemote_t  = nullptr;  // Esp Now Remote Task
//TaskHandle_t eRemote_t_EJECT  = nullptr;  // Esp Now Remote Task

void espNowRemoteTask(void *pvParameters); // Handels the EspNow Remote
#define HEARTBEAT_INTERVAL 15000/portTICK_PERIOD_MS	// 5 seconds

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
Serial.println("Received step 1");
  memcpy(&incomingcontrol, incomingData, sizeof(incomingcontrol));

  Serial.print("Received Command: ");
  LogDebug(incomingcontrol.esp_command);
  Serial.print("Received value: ");
  LogDebug(incomingcontrol.esp_value);
  Serial.print("Received to target ID: ");
  LogDebug(incomingcontrol.esp_target);
  Serial.print("Received from sender: ");
  LogDebug(incomingcontrol.esp_sender);
  LogDebugFormatted("from MAC addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if(!M5_paired){
    LogDebug("M5 is not paired");}
  else{
    LogDebugFormatted("M5 paired and ready. M5 Remote addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", M5_Remote_Address[0], M5_Remote_Address[1], M5_Remote_Address[2], M5_Remote_Address[3], M5_Remote_Address[4], M5_Remote_Address[5]);
  }
  




  if(incomingcontrol.esp_target == EJECT_ID  && 
     M5_paired == false){

    // Remove the existing peer (0xFF:0xFF:0xFF:0xFF:0xFF:0xFF)
    esp_err_t result = esp_now_del_peer(peerInfo.peer_addr);

    if (result == ESP_OK) {

      memcpy(M5_Remote_Address, mac, 6); //get the mac address of the sender
      
      // Add the new peer
      memcpy(peerInfo.peer_addr, M5_Remote_Address, 6);
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        LogDebugFormatted("New peer added successfully, M5 Remote addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", M5_Remote_Address[0], M5_Remote_Address[1], M5_Remote_Address[2], M5_Remote_Address[3], M5_Remote_Address[4], M5_Remote_Address[5]);
        M5_paired = true;
      }
      else {
        LogDebug("Failed to add new peer");
      }
    }
    else {
      LogDebug("Failed to remove peer");
    }

    outgoingcontrol.esp_target = M5_ID;
    outgoingcontrol.esp_sender = EJECT_ID;
    outgoingcontrol.esp_command = HEARTBEAT;


    result = esp_now_send(M5_Remote_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
    LogDebug(result);
    
    if (result == ESP_OK) {
      M5_paired = true;
      LogDebug("M5 remote Connected");
    }
  }

  //if(incomingcontrol.esp_target == EJECT_ID){
    Serial.print("Received command ");
    LogDebug(incomingcontrol.esp_command);
    switch(incomingcontrol.esp_command)
    {
      case CUMSPEED:
      {
      cum_speed = incomingcontrol.esp_value * 20;
      }
      break;
      case CUMTIME:
      {
      interval = incomingcontrol.esp_value ;
      }
      break;
      case CUMSIZE:
      {
      cum_size = incomingcontrol.esp_value * 1500;
      }
      break;
      case CUMACCEL:
      {
      cum_accel = incomingcontrol.esp_value * 100;
      }
      break;
      case ON:
      {
      cum_on=1;
      squirts=0;
      }
      break;
      case OFF:
      {
      cum_on=0;
      }
      break;
      case CONNECT or HEARTBEAT:
      {

        outgoingcontrol.esp_target = M5_ID;
        outgoingcontrol.esp_sender = EJECT_ID;
        outgoingcontrol.esp_command = HEARTBEAT;
        esp_err_t result = esp_now_send(M5_Remote_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
        LogDebug(result);
        
        if (result == ESP_OK) {
          M5_paired = true;
          LogDebug("M5 remote Connected");
        }
    
      }
      break;
    }
    Serial.print("CUMSPEED: ");
    LogDebug(cum_speed);

  //}
}

void setup()
{  
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Serial.print("MAC Address Eject: ");
    Serial.println(WiFi.macAddress());
    LogDebug(WiFi.macAddress());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
    }
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

      
    xTaskCreatePinnedToCore(espNowRemoteTask,      /* Task function. */
      "espNowRemoteTask",  /* name of task. */
      3096,               /* Stack size of task */
      NULL,               /* parameter of the task */
      5,                  /* priority of the task */
      &eRemote_t,         /* Task handle to keep track of created task */
      0);                 /* pin task to core 0 */
    
      // Register peer
    memcpy(peerInfo.peer_addr, M5_Remote_Address, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
  
      // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
    }
    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);


    // connect and configure the stepper motor to its IO pins
    pinMode(MOTOR_ENA_PIN, OUTPUT);
    digitalWrite(MOTOR_ENA_PIN, LOW);
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

    //Send first heartbeat to M5 remote

    if(!M5_paired){
      LogDebug("Sending first heartbeat");
      LogDebugFormatted("to M5 remote addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", M5_Remote_Address[0], M5_Remote_Address[1], M5_Remote_Address[2], M5_Remote_Address[3], M5_Remote_Address[4], M5_Remote_Address[5]);

      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = M5_ID;
      outgoingcontrol.esp_sender = EJECT_ID;
      esp_err_t result = esp_now_send(M5_Remote_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      if (result == !ESP_OK) {
        LogDebug("Sending to M5 failed (not paired)");
      }

    }

    //test to show board works 

  stepper.setSpeedInStepsPerSecond(5000);
  stepper.setAccelerationInStepsPerSecondPerSecond(2000);
  stepper.setDecelerationInStepsPerSecondPerSecond(1000);
  stepper.moveToPositionInSteps(5000);
  cum_on=0;
  squirts=0;
  Serial.print("Status = ");
  LogDebug (cum_on);
}

void loop()
{
  if (cum_on==1)
  {
  stepper.setSpeedInStepsPerSecond(cum_speed);
  stepper.setAccelerationInStepsPerSecondPerSecond(cum_accel);
  stepper.setDecelerationInStepsPerSecondPerSecond(cum_accel);

  unsigned long currentMillis = millis();
  if (squirts < interval)
  {
    digitalWrite(MOTOR_ENA_PIN, LOW);
    previousMillis = currentMillis;
    //if(cum_size>=20.00){
    //  Serial.println("Indefinite pumping");
    //  stepper.moveRelativeInSteps(100000000);}
    //if(!cum_size!=20.00){stepper.moveRelativeInSteps(cum_size);}
    stepper.moveRelativeInSteps(cum_size);
    delay(1000/cum_accel/2000);
  }
  if (stepper.getDistanceToTargetSigned() == 0){
     if (squirts <= interval) {
       squirts=squirts+1;}
       digitalWrite(MOTOR_ENA_PIN, HIGH);
       Serial.print("finished squirt ");
       LogDebug (squirts);}
     if (squirts > interval){
       cum_on=0;
       float squirts=0;
       Serial.print("Squirts done ");
       outgoingcontrol.esp_command = OFF;
       outgoingcontrol.esp_sender = EJECT_ID;
       outgoingcontrol.esp_target = M5_ID;
       esp_err_t result = esp_now_send(M5_Remote_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
       if (result == !ESP_OK) {
        LogDebug("Sending to M5 'squirts done' failed");
      }

            //ADD SEND TO M5 TO RESET CUM BUTTON
       //LogDebug (squirts);
       Serial.print("Squirts: ");
       LogDebug (squirts);
       Serial.print("Status: ");
       LogDebug (cum_on);
     }
     
  }
if (cum_on==0){
  squirts=0;

}
  
}

void espNowRemoteTask(void *pvParameters)
{
  for(;;){
      if(M5_paired){
              LogDebug("Heartbeat to M5");
//      LogDebugFormatted("OSSM addresss : %02X:%02X:%02X:%02X:%02X:%02X\n", OSSM_Address[0], OSSM_Address[1], OSSM_Address[2], OSSM_Address[3], OSSM_Address[4], OSSM_Address[5]);
      outgoingcontrol.esp_command = HEARTBEAT;
      outgoingcontrol.esp_sender = EJECT_ID;
      outgoingcontrol.esp_heartbeat = true;
      outgoingcontrol.esp_target = M5_ID;
      esp_err_t result = esp_now_send(M5_Remote_Address, (uint8_t *) &outgoingcontrol, sizeof(outgoingcontrol));
      if (result == ESP_OK) {
//        LogDebug("Sent ok");
      } 
      else {
        LogDebug("failed to send heartbeat");
      }

    }
    delay(200);
    //problem when adding if eject not paired
    vTaskDelay(HEARTBEAT_INTERVAL);
  }
}
