#ifndef __PACKETS_H__
#define __PACKETS_H__

#include <PacketSerial.h>
#include <Motor.h>

class Packets
{
private:
    PacketSerial *serial;
    String panicMsg;
    bool isPanic;
    Motor** motors;
    size_t count;

public:
    enum Command {
        Debug = 0,
        Status,
        Stop,
        Raw,
        Speed,
        Position,
    };

    Packets(PacketSerial *serial, Motor** motors, size_t motor_count);

    void panic(const char *msg);
    void sendDebug();
    void sendStatus();

    void handleStop();
    void handleRaw(const uint8_t *buffer, size_t size);
    void handleSpeed(const uint8_t *buffer, size_t size);
    void handlePosition(const uint8_t *buffer, size_t size);

    void maybePanic();

    static void receivePacket(const uint8_t *buffer, size_t size, Packets* packets);
private:
    void sendMessage(const char *msg);
};
#endif // __PACKETS_H__