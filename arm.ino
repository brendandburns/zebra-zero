#include <Arduino.h>
#include <Encoder.h>
#include <EncoderLibEncoder.h>
#include <ESP32Encoder.h>
#include <ESP32LibEncoder.h>

#include <Motor.h>
#include <BTS7960.h>
#include <PacketSerial.h>
#include "packets.h"

#ifdef ESP32
ESP32Encoder enc;
ESP32LibEncoder encoder(&enc);

ESP32Encoder enc2;
ESP32LibEncoder encoder2(&enc2);

ESP32Encoder enc3;
ESP32LibEncoder encoder3(&enc3);

//Encoder encoder4(39, 36);
BTS7960 controller(27, 27, 1, 1, true);
BTS7960 controller2(27, 27, 1, 1, true);
BTS7960 controller3(27, 27, 1, 1, false);

#else
Encoder enc(21, 20);
EncoderLibEncoder encoder(&enc);
Encoder enc2(19, 18);
EncoderLibEncoder encoder2(&enc2);
Encoder enc3(3, 2);
EncoderLibEncoder encoder3(&enc3);
BTS7960 controller(22, 23, 4, 5, true);
BTS7960 controller2(24, 25, 6, 7, true);
BTS7960 controller3(26, 27, 8, 9, false);

foo
#endif

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
    enc.attachFullQuad(21, 20);
    enc2.attachFullQuad(19, 18);
    enc3.attachFullQuad(3, 2);

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
