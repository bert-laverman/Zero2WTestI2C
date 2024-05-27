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

#include <zero2w.hpp>
#include <interfaces/zero2w-i2c.hpp>
#include <protocols/i2c-protocol-driver.hpp>
#include <devices/remote-max7219.hpp>

#include "i2c-ini.hpp"

using namespace nl::rakis::raspberrypi;
using namespace nl::rakis::raspberrypi::protocols;
using namespace nl::rakis::raspberrypi::devices;
using nl::rakis::i2c::I2CState;


#if !defined(HAVE_I2C)
#error "This example needs I2C enabled"
#endif
#if !defined(HAVE_MAX7219)
#error "This example needs MAX7219 enabled"
#endif


static I2CState config;


static std::map<uint64_t, uint8_t> addressById;
static std::map<uint8_t, uint64_t> idByAddress;
static constexpr uint8_t firstPico{ 0x61 };

static void loadBoardsFromConfig()
{

}


template <typename DriverType>
static void processHello(DriverType& driver, uint8_t sender, MsgHello& msg)
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
                    std::cerr << std::format("We'll give this board address 0x{:02x}.\n", i);
                    picoAddress = i;
                    break;
                }
            }
        } else {
            std::cerr << std::format("We know this one: It has address 0x{:02x}.\n", addressById [msg.boardId.id]);
        }
        if (picoAddress != 0x00) {
            if (!driver.sendSetAddress(msg.boardId, picoAddress)) {
                std::cerr << std::format("Failed to send address 0x{:02x}.\n", picoAddress);
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
    std::cerr << std::format("Loaded {} boards.\n", config.countBoardIds());

    unsigned count{30};
    if (argc == 2) {
        count = atoi(argv[1]);
    }
    [[maybe_unused]]
    Zero2W& berry(Zero2W::instance(true));
    interfaces::Zero2WI2C_i2cdev pi2picoBus("/dev/i2c-1");
    interfaces::Zero2WI2C_pigpio pico2piBus;
    pi2picoBus.verbose(true);
    pico2piBus.verbose(true);

    I2CProtocolDriver driver(pi2picoBus, pico2piBus);

    std::cerr << "Starting test\n";

    const uint8_t controllerAddress = 0x0a;

    driver.enableControllerMode();
    driver.registerHandler(Command::Hello, "Hello handler", [&driver](Command command, uint8_t sender, const std::span<uint8_t> data) -> void {
        std::cerr << std::format("Received command 0x{:02x} from 0x{:02x} with {} bytes of data.\n", toInt(command), sender, data.size());
        switch (command) {
            case Command::Hello:
                if (data.size() == sizeof(MsgHello)) {
                    MsgHello msg;
                    std::memcpy(&msg, data.data(), sizeof(MsgHello));
                    processHello(driver, sender, msg);
                }
                break;
            default:
                std::cerr << std::format("Unknown command 0x{:02x} from 0x{:02x} with {} bytes of data.\n", toInt(command), sender, data.size());
                break;
        }
    });
    driver.enableResponderMode(controllerAddress);

    RemoteMAX7219 max(driver, 0x61);

    std::cerr << "Starting to wait for someone to talk to us.\n";

    for (unsigned outer = 0; outer < count; outer++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        std::cerr << std::format("Announcing we are at address 0x{:02x}.\n", controllerAddress);
        driver.sendHello(controllerId);

        max.setNumber(1, outer);
        max.sendData();
    }

    std::cerr << "Shutting down.\n";
    pi2picoBus.close();
    pico2piBus.close();
}

