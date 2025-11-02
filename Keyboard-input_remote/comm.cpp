#include "comm.hpp"

#include <asio/write.hpp>
#include <iostream>

uint8_t mac_index_dest = 0;  // default

#ifdef _WIN32
#include <devguid.h>
#include <setupapi.h>
#include <windows.h>
#pragma comment(lib, "setupapi.lib")
#endif

SerialComm::SerialComm() : serial_(nullptr) {}

SerialComm::~SerialComm() {
    close();
}

std::vector<std::string> SerialComm::enumerateSerialPorts() {
    std::vector<std::string> ports;

#ifdef _WIN32
    HDEVINFO hDevInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);

    if (hDevInfo == INVALID_HANDLE_VALUE) return ports;

    SP_DEVINFO_DATA devInfoData;
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); i++) {
        char friendlyName[256];
        if (SetupDiGetDeviceRegistryPropertyA(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME,
                                              NULL, (PBYTE)friendlyName, sizeof(friendlyName), NULL)) {
            std::string name(friendlyName);

            if (name.find("CP210") != std::string::npos ||
                name.find("CH340") != std::string::npos ||
                name.find("10C4") != std::string::npos ||
                name.find("1A86") != std::string::npos ||
                name.find("6970") != std::string::npos ||
                name.find("303A") != std::string::npos ||
                name.find("Silicon Labs") != std::string::npos ||
                name.find("USB-SERIAL CH340") != std::string::npos) {
                size_t pos = name.find("COM");
                if (pos != std::string::npos) {
                    size_t end = name.find(")", pos);
                    if (end != std::string::npos) {
                        std::string port = name.substr(pos, end - pos);
                        ports.push_back(port);
                        std::cout << "Found potential ESP32: " << name << std::endl;
                    }
                }
            }
        }
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);
#else
    // Linux/macOS port enumeration
#endif

    return ports;
}

std::string SerialComm::selectPort() {
    std::string portName;

#ifdef _WIN32
    auto ports = enumerateSerialPorts();

    if (ports.empty()) {
        std::cout << "No ESP32 detected. Enter COM port manually: ";
        std::cin >> portName;
    } else if (ports.size() == 1) {
        portName = ports[0];
        std::cout << "Auto-detected ESP32 on: " << portName << std::endl;
    } else {
        std::cout << "Multiple ESP32 devices found:" << std::endl;
        for (size_t i = 0; i < ports.size(); i++) {
            std::cout << i + 1 << ". " << ports[i] << std::endl;
        }
        std::cout << "Select port (1-" << ports.size() << "): ";
        int choice;
        std::cin >> choice;
        if (choice > 0 && choice <= (int)ports.size()) {
            portName = ports[choice - 1];
        } else {
            std::cerr << "Invalid selection." << std::endl;
            return "";
        }
    }
#else
    std::cout << "Enter the COM port your ESP32 is on (e.g., /dev/ttyUSB0): ";
    std::cin >> portName;
#endif

    return portName;
}

bool SerialComm::open(const std::string& portName, unsigned int baudRate) {
    try {
        serial_ = new asio::serial_port(io_);
        serial_->open(portName);
        serial_->set_option(asio::serial_port_base::baud_rate(baudRate));
        serial_->set_option(asio::serial_port_base::character_size(8));
        serial_->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        serial_->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

        std::cout << "\nSerial connection successful on " << portName << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error: Failed to open serial port " << portName << std::endl;
        std::cerr << e.what() << std::endl;
        delete serial_;
        serial_ = nullptr;
        return false;
    }
}

bool SerialComm::send(char key) {
    if (!serial_ || !serial_->is_open()) {
        std::cerr << "Error: Serial port not open" << std::endl;
        return false;
    }

    uint8_t data[] = {
        0xFD,
        0xFD,
        0xFF,
        0xFF,
        mac_index_dest,
        1,
        key};

    try {
        asio::write(*serial_, asio::buffer(&data, sizeof(data)));
        std::cout << "Sending to " << mac_names[mac_index_dest] << ": '" << key << "'" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
        return false;
    }
}

void SerialComm::close() {
    if (serial_) {
        if (serial_->is_open()) {
            serial_->close();
        }
        delete serial_;
        serial_ = nullptr;
    }
}

bool SerialComm::isOpen() const {
    return serial_ && serial_->is_open();
}