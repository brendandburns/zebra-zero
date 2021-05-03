#include <Arduino.h>
#include <Encoder.h>
#include <Motor.h>
#include <BTS7960.h>
#include <PacketSerial.h>
#include "packets.h"

Encoder encoder(21, 20);
Encoder encoder2(19, 18);
Encoder encoder3(3, 2);
BTS7960 controller(22, 23, 4, 5, true);
BTS7960 controller2(24, 25, 6, 7, true);
BTS7960 controller3(26, 27, 8, 9, false);
Motor motor(&controller, &encoder, "joint-1", 255);
Motor motor2(&controller2, &encoder2, "joint-2", 255);
Motor motor3(&controller3, &encoder3, "joint-3", 255);

#define MOTOR_COUNT 3
Motor* motors[MOTOR_COUNT] = { &motor, &motor2, &motor3 };

PacketSerial packetSerial;
Packets packets(&packetSerial, motors, MOTOR_COUNT);

long last = millis();

void packetReceived(const uint8_t *buffer, size_t size)
{
  Packets::receivePacket(buffer, size, &packets);
}

void setup()
{
    packetSerial.begin(115200);
    packetSerial.setPacketHandler(&packetReceived);
    
    for(int i = 0; i < MOTOR_COUNT; i++) {
      motors[i]->stop();
    }
}

int count = 0;
void loop()
{
  packetSerial.update();
  for(int i = 0; i < MOTOR_COUNT; i++) {
    motors[i]->loop();
  }
  if ((count++ % 1000) == 0)
  {
    packets.maybePanic();
  }
}
