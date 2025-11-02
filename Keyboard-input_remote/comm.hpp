#pragma once
#include <asio/io_context.hpp>
#include <asio/serial_port.hpp>
#include <string>
#include <vector>

extern uint8_t mac_index_dest;

constexpr size_t MAC_ADDRESS_TOTAL = 12;
constexpr size_t MAC_LENGTH = 6;

constexpr uint8_t mac_addresses[MAC_ADDRESS_TOTAL][MAC_LENGTH] = {
    {0x24, 0x0A, 0xC4, 0x0A, 0x1A, 0x11},
    {0x24, 0x0A, 0xC4, 0x0A, 0x1B, 0x11},
    {0x24, 0x0A, 0xC4, 0x0A, 0x2A, 0x11},
    {0x24, 0x0A, 0xC4, 0x0A, 0x2B, 0x11},
    {0x24, 0x0A, 0xC4, 0x0A, 0x3A, 0x11},
    {0x24, 0x0A, 0xC4, 0x0A, 0x3A, 0x22},
    {0x24, 0x0A, 0xC4, 0x0A, 0x3B, 0x11},
    {0x24, 0x0A, 0xC4, 0x0A, 0x3B, 0x22},
    {0x24, 0x0A, 0xC4, 0x0A, 0xFF, 0xF1},
    {0x24, 0x0A, 0xC4, 0x0A, 0xFF, 0xF2},
    {0x24, 0x0A, 0xC4, 0x0A, 0xFF, 0xF3},
    {0x24, 0x0A, 0xC4, 0x0A, 0xFF, 0xF4},
};

constexpr const char* mac_names[MAC_ADDRESS_TOTAL] = {
    "BRIDGE-ZERO",
    "BRIDGE-ONE",
    "BRIDGE-TWO",
    "BRIDGE-THREE",
    "BRIDGE-FOUR",
    "BRIDGE-FIVE",
    "BRIDGE-SIX",
    "BRIDGE-SEVEN",
    "BRIDGE-EIGHT",
    "BRIDGE-NINE",
    "BRIDGE-TEN",
    "BRIDGE-ELEVEN"};

class SerialComm {
   public:
    SerialComm();
    ~SerialComm();

    // enumerate and detect esp32 port
    std::vector<std::string> enumerateSerialPorts();

    // select port / auto-select if esp32 detected
    std::string selectPort();

    // open serial connection
    bool open(const std::string& portName, unsigned int baudRate = 115200);

    // send key
    bool send(char key);

    // close connection
    void close();

    // check if port is open
    bool isOpen() const;

   private:
    asio::io_context io_;
    asio::serial_port* serial_;
};
