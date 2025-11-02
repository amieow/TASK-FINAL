// Template FP-1
// Referensi model robot: https://webots.cloud/run?version=R2025a&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwebots%2Fblob%2Freleased%2Fprojects%2Frobots%2Frobotis%2Fdarwin-op%2Fprotos%2FRobotisOp2.proto
// Referensi Api Function: https://cyberbotics.com/doc/reference/nodes-and-api-functions

#include <iostream>
#include <map>
#include <thread>
#include <chrono>
#include <fstream>
#include <unordered_map>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/istreamwrapper.h>
#include <asio/read.hpp>
#include <asio/serial_port.hpp>
#include <asio/io_context.hpp>
#include <asio/read.hpp>
namespace wb = webots;
namespace rj = rapidjson;
#define keys_input_interval 0.300
#define INCREMENT 0.1
#define SERIAL_PORT "\\\\.\\COM3"

rj::Document loadPoseFromFile(const std::string &filename)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open())
    {
        std::cerr << "Gagal membuka file pose: " << filename << std::endl;
        return rj::Document();
    }

    rj::IStreamWrapper isw(ifs);
    rj::Document document;
    document.ParseStream(isw);
    if (document.HasParseError())
    {
        std::cerr << "Gagal mengurai file pose: " << filename << std::endl;
    }
    return document;
}

int main(int argc, char **argv)
{
    wb::Robot *robot = new wb::Robot();
    const int timeStep = static_cast<int>(robot->getBasicTimeStep());
    asio::io_context io;
    asio::serial_port serialReader(io);

    std::cout << "Connecting to serial port " << SERIAL_PORT << " at 115200 baud..." << std::endl;

    try
    {
        serialReader.open(SERIAL_PORT);
        serialReader.set_option(asio::serial_port_base::baud_rate(115200));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error opening serial port: " << e.what() << std::endl;
        delete robot;
        return 1;
    }
    std::cout << "[SUCCESS] Connected to COM3 at 115200 baud" << std::endl;

    // Map to store all motors
    std::map<std::string, wb::Motor *> motors;
    std::map<std::string, wb::PositionSensor *> positionSensors;
    std::unordered_map<char, std::string> keyToPoses = {
        {'W', "pose-jalan-maju"},
        {'A', "pose-geser-kiri"},
        {'S', "pose-jalan-mundur"},
        {'D', "pose-geser-kanan"},
        {'E', "pose-belok-kanan"},
        {'Q', "pose-belok-kiri"},
        {'R', "pose-jongkok"},
        {'F', "pose-berdiri"},
    };
    const char *motorNames[] = {
        // Head motors
        "Head", "Neck",

        "ShoulderL", "ShoulderR",
        "ArmUpperL", "ArmUpperR",
        "ArmLowerL", "ArmLowerR",

        // Paha Z(Belok)
        "PelvYL", "PelvYR",

        // Paha X(Miring)
        "PelvL", "PelvR",

        // Paha atas Y(gerak depan / belakang)
        "LegUpperL", "LegUpperR",

        // Lutut
        "LegLowerL", "LegLowerR",

        // Tumit
        "AnkleL", "AnkleR",

        // Tumit X(miring)
        "FootL", "FootR"};
    // Initialize all motors and position sensors
    int numMotors = sizeof(motorNames) / sizeof(motorNames[0]);
    std::cout << "Initializing " << numMotors << " motors..." << std::endl;

    for (int i = 0; i < numMotors; i++)
    {
        // Get each motor
        wb::Motor *motor = robot->getMotor(motorNames[i]);
        if (motor)
        {
            motors[motorNames[i]] = motor;
            std::cout << "[SUCCESS] Motor found: " << motorNames[i] << std::endl;

            // Get corresponding position sensor
            std::string sensorName = std::string(motorNames[i]) + "S";
            wb::PositionSensor *sensor = robot->getPositionSensor(sensorName);
            if (sensor)
            {
                sensor->enable(timeStep);
                positionSensors[motorNames[i]] = sensor;
                std::cout << "[SUCCESS] Sensor enabled! : " << sensorName << std::endl;
            }
        }
        else
        {
            std::cerr << "[FAILED] Motor not found! : " << motorNames[i] << std::endl;
        }
    }

    // launch the option for each key
    std::cout << "Available key commands and poses:" << std::endl;
    for (const auto &[name, val] : keyToPoses)
    {
        std::cout << "Key '" << name << "' for: " << val << ".json" << std::endl;
    }
    // std::cout << "input option(choose the number between 1 or 2) :" << std::endl;
    // std::cout << "1. Serial Input from COM3" << std::endl;
    // std::cout << "2. Keyboard Input" << std::endl;
    // int inputOption;
    // bool serialInputMode = false;
    // std::cin >> inputOption;
    // if (inputOption == 1)
    // {
    //     serialInputMode = true;
    //     std::cout << "Serial Input Mode selected." << std::endl;
    // }
    // else
    // {
    //     std::cout << "Keyboard Input Mode selected." << std::endl;
    // }
    // Main control loop
    while (robot->step(timeStep) != -1)
    {
        // Read serial data
        std::string serialData;
        asio::error_code ec;
        robot->step(timeStep);
        if (true)
        {
            char buf[256];
            size_t bytesRead = serialReader.read_some(asio::buffer(buf), ec);
            if (ec)
            {
                std::cerr << "Error reading from serial port: " << ec.message() << std::endl;
                continue;
            }
            serialData.append(buf, bytesRead);
            if (bytesRead <= 0)
                continue;
            std::cout << "Received serial data (" << bytesRead << " bytes): " << serialData << std::endl;
            if (bytesRead > 1)
                continue;
        }
        else
        {
            char inputChar;
            std::cin >> inputChar;
            serialData += inputChar;
        }

        char command = static_cast<char>(serialData[0]);
        if (keyToPoses.find(command) == keyToPoses.end())
            continue;
        std::cout << "Command received: " << command << " -> " << keyToPoses.at(command) << std::endl;

        rj::Document poseDoc = loadPoseFromFile("../../poses/" + keyToPoses.at(command) + ".json");
        if (poseDoc.IsNull() || !poseDoc.IsObject())
        {
            std::cerr << "Invalid poses data for command: " << command << std::endl;
            continue;
        }
        if (!poseDoc.HasMember("name") || !poseDoc["name"].IsString() || !poseDoc.HasMember("pose_group") || !poseDoc["pose_group"].IsArray())
        {
            std::cerr << "No 'pose_group' or 'name' object in poses data for command: " << command << std::endl;
            continue;
        }

        const rj::Value &poseGroup = poseDoc["pose_group"];
        for (rj::SizeType i = 0; i < poseGroup.Size(); i++)
        {
            const rj::Value &pose = poseGroup[i];
            if (!pose.IsObject() || !pose.HasMember("nama") || !pose["nama"].IsString() || !pose.HasMember("pose") || !pose["pose"].IsArray())
            {
                std::cerr << "Invalid pose format in pose_group at index " << i << std::endl;
                continue;
            }
            std::cout << "Setting pose_group" << "[" << i + 1 << "] :" << pose["nama"].GetString() << std::endl;
            const rj::Value &poseArray = pose["pose"];
            for (rj::SizeType j = 0; j < poseArray.Size(); j++)
            {
                
                if (!poseArray[j].IsObject() || !poseArray[j].HasMember("posisi") || !poseArray[j]["posisi"].IsArray() || !poseArray[j].HasMember("nama"))
                {
                    std::cerr << "Invalid pose format in pose at index i=" << i << " j=" << j << std::endl;
                    continue;
                }
                std::string poseName = poseArray[j]["nama"].GetString();
                std::cout << "  Setting pose: " << poseName << std::endl;
                const rj::Value &posisiArray = poseArray[j]["posisi"];
                for (rj::SizeType k = 0; k < posisiArray.Size(); k++)
                {
                    if (!posisiArray[k].IsDouble())
                    {
                        std::cerr << "Invalid position value at index k=" << k << " in pose: " << poseName << std::endl;
                        continue;
                    }
                    double targetPosition = posisiArray[k].GetDouble();
                    motors[motorNames[k]]->setPosition(targetPosition);
                    robot->step(timeStep);
                    // Wait until the motor reaches the target position
                                }
            }
        }
    }
    delete robot;
    return 0;
}