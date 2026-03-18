#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <Arduino.h>

// Packet format: [0xAA] [len] [data_hi] [data_lo] ... [checksum]
// checksum = XOR of all bytes after start byte (len + data bytes)
// Values are int16_t, transmitted big-endian.

#define SERIAL_COMM_START_BYTE 0xAA
#define SERIAL_COMM_MAX_VALUES 32

class SerialComm {
public:
    SerialComm(Stream& serial) : _serial(serial) {}

    // Write a list of int16_t values
    void write(const int16_t* data, uint8_t len);

    // Read a packet into data[]. Returns number of values read, or -1 if no
    // complete packet is available yet. Call repeatedly in loop().
    int read(int16_t* data, uint8_t maxLen);

private:
    Stream& _serial;

    // Read-state machine
    enum State { WAIT_START, WAIT_LEN, READ_DATA, WAIT_CHECKSUM };
    State _state = WAIT_START;
    uint8_t _expectedLen = 0;
    uint8_t _bytesRead = 0;
    uint8_t _buf[SERIAL_COMM_MAX_VALUES * 2];
    uint8_t _checksum = 0;
};

#endif // SERIAL_COMM_H
