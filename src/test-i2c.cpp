/*
 * Copyright (c) 2024 by Bert Laverman. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <cstdint>
#include <iostream>
#include <format>
#include <map>

#include <algorithm>

#include <memory>
#include <thread>
#include <chrono>

#include <zero2w.hpp>
#include <interfaces/pigpiod-i2c.hpp>
#include <interfaces/i2cdev-i2c.hpp>
#include <protocols/i2c-protocol-driver.hpp>
#include <protocols/i2c-device-handler.hpp>
#include <util/message-queue.hpp>

#include "i2c-ini.hpp"

using namespace nl::rakis::raspberrypi;
using namespace nl::rakis::raspberrypi::protocols;
using nl::rakis::i2c::I2CState;


#if !defined(HAVE_I2C)
#error "This example needs I2C enabled"
#endif


static I2CState config;


static std::map<uint64_t, uint8_t> addressById;
static std::map<uint8_t, uint64_t> idByAddress;
static constexpr uint8_t firstPico{ 0x61 };

static bool parseHex(uint8_t& byte, char h1, char h2) {
    uint8_t b;
    if ((h1 >= '0') && (h1 <= '9')) {
        b = static_cast<uint8_t>((h1 - '0') << 4);
    } else if ((h1 >= 'a') && (h1 <= 'f')) {
        b = static_cast<uint8_t>((h1 - 'a' + 10) << 4);
    } else if ((h1 >= 'A') && (h1 <= 'F')) {
        b = static_cast<uint8_t>((h1 - 'A' + 10) << 4);
    } else {
        return false;
    }
    if ((h2 >= '0') && (h2 <= '9')) {
        b |= static_cast<uint8_t>(h2 - '0');
    } else if ((h2 >= 'a') && (h2 <= 'f')) {
        b |= static_cast<uint8_t>(h2 - 'a' + 10);
    } else if ((h2 >= 'A') && (h2 <= 'F')) {
        b |= static_cast<uint8_t>(h2 - 'A' + 10);
    } else {
        return false;
    }
    byte = b;
    return true;
}

static bool parseBoardId(uint8_t bytes[8], const std::string& s) {
    if ((s.size() != 17) || (s[8] != '-')) {
        std::cerr << "Invalid Board ID in config '" << s << "'.\n";
        return false;
    }
    return parseHex(bytes [0], s [0], s [1])
        && parseHex(bytes [1], s [2], s [3])
        && parseHex(bytes [2], s [4], s [5])
        && parseHex(bytes [3], s [6], s [7])
        && parseHex(bytes [4], s [9], s [10])
        && parseHex(bytes [5], s [11], s [12])
        && parseHex(bytes [6], s [13], s [14])
        && parseHex(bytes [7], s [15], s [16]);
}

static void loadBoardsFromConfig()
{
    for (auto key : config.boardIds()) {
        std::cerr << "Checking board '" << key << "' for I2C address info.\n";

        if (config.hasBoardValue(key, "boardId") && config.hasBoardValue(key, "address")) {
            auto configId = config.boardValue(key, "boardId");
            auto address = atoi(config.boardValue(key, "address").c_str());

            BoardId id;
            if (!parseBoardId(id.bytes, configId)) {
                std::cerr << "Ignoring unparsable board Id of '" << key << "'.\n";
            } else if ((address < firstPico) || (address >= 0b10000000000)) {
                std::cerr << std::format("Ignoring bad address 0x{:02x} for board '{}'.\n", address, key);
            } else {
                std::cerr << std::format("Board '{}', id '{}', address 0x{:02x}.\n", key, configId, address);
                addressById[id.id] = address;
                idByAddress[address] = id.id;
            }
        } else {
            std::cerr << "- Nothing for '" << key << "'.\n";
        }
    }
}


template <typename HandlerType>
static void processHello(HandlerType& handler, uint8_t sender, const MsgHello& msg)
{
    std::cerr << std::format("Received Hello message from 0x{:02x}, board with Id {:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}\n",
                            sender,
                             msg.boardId.bytes[0], msg.boardId.bytes[1], msg.boardId.bytes[2], msg.boardId.bytes[3],
                             msg.boardId.bytes[4], msg.boardId.bytes[5], msg.boardId.bytes[6], msg.boardId.bytes[7]);

    if (sender == 0x00) {
        // A Pico is asking for an address
        uint8_t picoAddress{ 0x00 };
        auto it = addressById.find(msg.boardId.id);
        if (it == addressById.end()) {
            for (auto i = firstPico; i < 0x7f; i++){
                if (idByAddress.find(i) == idByAddress.end()) {
                    std::cerr << std::format("- We'll give this board address 0x{:02x}.\n", i);
                    picoAddress = i;
                    break;
                }
            }
        } else {
            picoAddress = addressById [msg.boardId.id];
            std::cerr << std::format("- We know this one: It has address 0x{:02x}.\n", picoAddress);
        }
        if (picoAddress != 0x00) {
            if (!handler.sendSetAddress(msg.boardId, picoAddress)) {
                std::cerr << std::format("* Failed to send address 0x{:02x}.\n", picoAddress);
            } else {
                idByAddress [picoAddress] = msg.boardId.id;
                addressById [msg.boardId.id] = picoAddress;
            }
        }
    }
}


static BoardId controllerId{ .id = ControllerId };


int main([[maybe_unused]] int argc, [[maybe_unused]] char*argv[])
{
    config.load();
    loadBoardsFromConfig();
    std::cerr << std::format("Loaded {} boards.\n", config.countBoardIds());

    unsigned count{30};
    if (argc == 2) {
        count = atoi(argv[1]);
    }
    auto pi2picoBus = std::make_shared<interfaces::I2CDevI2C>("/dev/i2c-1");
    auto pico2piBus = std::make_shared<interfaces::PigpiodBSCI2C>();
    pi2picoBus->verbose(true);
    pico2piBus->verbose(true);

    I2CProtocolDriver<util::MessageQueue> driver;
    driver.addInterface(pi2picoBus);
    driver.addInterface(pico2piBus);
    I2CDeviceHandler<I2CProtocolDriver<util::MessageQueue>> deviceHandler(driver, controllerId);

    driver.registerHandler(Command::Hello, "Hello handler", [&driver,&deviceHandler]([[maybe_unused]] Command command, uint8_t sender, const std::vector<uint8_t>& data) -> void {
        if (data.size() == sizeof(MsgHello)) {
            const MsgHello* msg = reinterpret_cast<const MsgHello*>(data.data());
            processHello(deviceHandler, sender, *msg);
        }
    });

    std::cerr << "Starting test\n";

    const uint8_t controllerAddress = 0x0a;

    driver.listenAddress(controllerAddress);
    driver.startListening();

    std::cerr << "Starting to wait for someone to talk to us.\n";

    for (unsigned outer = 0; outer < count; outer++) {
        deviceHandler.sendHello(controllerId);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        driver.processIncoming();
    }

    std::cerr << "Shutting down.\n";
    driver.stopListening();
    driver.close();
}

