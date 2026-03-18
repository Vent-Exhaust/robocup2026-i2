#include "serial_comm.h"

void SerialComm::write(const int16_t* data, uint8_t len) {
    uint8_t checksum = len;

    _serial.write(SERIAL_COMM_START_BYTE);
    _serial.write(len);

    for (uint8_t i = 0; i < len; i++) {
        uint8_t hi = (data[i] >> 8) & 0xFF;
        uint8_t lo = data[i] & 0xFF;
        _serial.write(hi);
        _serial.write(lo);
        checksum ^= hi ^ lo;
    }

    _serial.write(checksum);
}

int SerialComm::read(int16_t* data, uint8_t maxLen) {
    while (_serial.available()) {
        uint8_t byte = _serial.read();

        switch (_state) {
            case WAIT_START:
                if (byte == SERIAL_COMM_START_BYTE) {
                    _state = WAIT_LEN;
                }
                break;

            case WAIT_LEN:
                if (byte == 0 || byte > SERIAL_COMM_MAX_VALUES) {
                    _state = WAIT_START;  // invalid length, resync
                    break;
                }
                _expectedLen = byte;
                _bytesRead = 0;
                _checksum = byte;  // start checksum with len byte
                _state = READ_DATA;
                break;

            case READ_DATA:
                _buf[_bytesRead++] = byte;
                _checksum ^= byte;
                if (_bytesRead == _expectedLen * 2) {
                    _state = WAIT_CHECKSUM;
                }
                break;

            case WAIT_CHECKSUM:
                _state = WAIT_START;
                if (byte != _checksum) {
                    break;  // checksum mismatch, discard packet
                }
                // Decode int16_t values
                uint8_t count = min(_expectedLen, maxLen);
                for (uint8_t i = 0; i < count; i++) {
                    data[i] = (int16_t)((_buf[i * 2] << 8) | _buf[i * 2 + 1]);
                }
                return count;
        }
    }

    return -1;  // no complete packet yet
}
