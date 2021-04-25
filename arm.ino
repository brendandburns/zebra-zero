#include <Arduino.h>
#include <Encoder.h>
#include <Motor.h>
#include <BTS7960.h>
#include <PacketSerial.h>
#include "packets.h"

Encoder encoder(20, 21);
BTS7960 controller(22, 23, 2, 3);
Motor motor(&controller, &encoder, "joint-2", 255);
Motor* motors[1] = { &motor };

PacketSerial packetSerial;
Packets packets(&packetSerial, motors, 1);

long last = millis();

void packetReceived(const uint8_t *buffer, size_t size)
{
  Packets::receivePacket(buffer, size, &packets);
}

void setup()
{
    packetSerial.begin(115200);
    packetSerial.setPacketHandler(&packetReceived);
    
    motor.stop();
}

int count = 0;
void loop()
{
  packetSerial.update();
  motor.loop();
  if ((count++ % 1000) == 0)
  {
    packets.maybePanic();
  }
}
