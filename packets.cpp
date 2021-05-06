#include "packets.h"

Packets::Packets(PacketSerial *serial, Motor** motors, size_t count)
    : serial(serial), panicMsg(""), isPanic(false), motors(motors), count(count) {}

void Packets::panic(const char *msg)
{
  this->panicMsg = String(msg);
  this->isPanic = true;
}

void Packets::maybePanic()
{
  if (this->isPanic)
  {
    this->sendMessage(this->panicMsg.c_str());
  }
}

void Packets::sendMessage(const char *msg)
{
  uint8_t buffer[256];
  size_t len = strlen(msg);
  if (len == 0)
  {
    return;
  }
  if (len > 254)
  {
    len = 255;
  }
  buffer[0] = Debug;
  memcpy(buffer + 1, msg, len + 1);
  this->serial->send(buffer, len + 2);
}

uint8_t *writeLong(uint8_t *addr, long val)
{
  memcpy(addr, &val, sizeof(long));
  return addr + sizeof(long);
}

uint8_t *writeInt(uint8_t *addr, int val)
{
  memcpy(addr, &val, sizeof(int));
  return addr + sizeof(int);
}

uint8_t *writeBool(uint8_t *addr, bool val)
{
  *addr = val ? 1 : 0;
  return addr + 1;
}

uint8_t *writeByte(uint8_t *addr, uint8_t val)
{
  *addr = val;
  return addr + 1;
}

// Format is 0x01 (header) | N | pos-1 | speed-1 | ... | pos-N | speed-N
void Packets::sendStatus()
{
  size_t msg_size = 1 /* header */ + 1 /* count */ + (4 /* long pos */ + 2 /* int speed */) * this->count; 
  uint8_t buffer[msg_size];
  buffer[0] = Status;
  buffer[1] = this->count;
  uint8_t *addr = buffer + 2;
  for (int i = 0; i < this->count; i++) {
    addr = writeLong(addr, this->motors[i]->position());
    addr = writeInt(addr, this->motors[i]->raw_speed());
  }
  serial->send(buffer, msg_size);
}

#define BUFFER_SIZE 512
// Format is 0x00 (header) | String
void Packets::sendDebug()
{
  uint8_t buffer[BUFFER_SIZE * this->count];
  char *out = (char *)buffer + 1;
  buffer[0] = Debug;
  size_t total_size = 1;
  for (int i = 0; i < this->count; i++) {
    size_t size = this->motors[i]->print(out, BUFFER_SIZE - 1);
    out += size;
    *(out++) = '\n';
    total_size += size + 1;
  }

  serial->send(buffer, total_size);
}

// Set the raw power of the motors
// Expected format is 0x03 | index | power
void Packets::handleRaw(const uint8_t *buffer, size_t size)
{
  if (size != 3)
  {
    panic("Wrong sized raw message!");
    return;
  }
  int index = buffer[1];
  if (index >= this->count) {
    panic("Motor index too large!");
    return;
  }
  this->motors[index]->setMode(Motor::Raw);
  this->motors[index]->setRawSpeed((int8_t) buffer[2]);
  this->sendStatus();
}

// Set the desired speed of a motor (PID controlled)
// Expected format is 0x04 | index | speed
void Packets::handleSpeed(const uint8_t *buffer, size_t size)
{
  // count * 3 (speeds) + 1 (header)
  if ((size % 3) != 1)
  {
    panic("wrong size speed!");
    return;
  }

  int count = (size - 1) / 3;
  for (int i = 0; i < count; i++) {
    const uint8_t *ptr = buffer + i * 3 + 1;
    if (*ptr >= this->count) {
      panic("Motor index too large!");
      return;
    }

    int speed;
    memcpy(&speed, ptr + 1, 2);

    this->motors[*ptr]->setDesiredSpeed(speed);
  }
  this->sendStatus();
}

// Set the desired position for the wheels
// Format is 0x05 | index-0 | position-0 | ... | index-N | position-N
void Packets::handlePosition(const uint8_t *buffer, size_t size)
{
  // Length should be # of positions * 5 bytes + 1 for the header
  if ((size % 5) != 1)
  {
    panic("wrong size position!");
    return;
  }

  int count = (size - 1) / 5;
  for (int i = 0; i < count; i++) {
    long pos;
    const uint8_t *ptr = buffer + i * 5 + 1;

    if (*ptr >= this->count) {
      panic("Motor index too large!");
      return;
    }


    memcpy(&pos, ptr + 1, 4);
    this->motors[*ptr]->setPosition(pos);
  }
  this->sendStatus();
}

void Packets::handleStop()
{
  for (int i = 0; i < this->count; i++) {
    this->motors[i]->stop();
  }
  this->sendStatus();
}

void Packets::receivePacket(const uint8_t *buffer, size_t size, Packets *packets)
{
  if (size == 0)
  {
    packets->panic("Size zero message!");
    return;
  }
  switch (buffer[0])
  {
  case Packets::Debug:
    packets->sendDebug();
  case Packets::Status:
    packets->sendStatus();
    break;
  case Packets::Stop:
    packets->handleStop();
    break;
  case Packets::Raw:
    packets->handleRaw(buffer, size);
    break;
  case Packets::Speed:
    packets->handleSpeed(buffer, size);
    break;
  case Packets::Position:
    packets->handlePosition(buffer, size);
    break;
  default:
    packets->panic("Unknown message!");
    break;
  }
}
