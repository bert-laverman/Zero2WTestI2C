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

#include <cstdlib>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

#include <zero2w.hpp>
#include <interfaces/pigpiod-i2c.hpp>
#include <interfaces/i2cdev-i2c.hpp>
#include <protocols/i2c-protocol-driver.hpp>
#include <protocols/i2c-bus-controller.hpp>
#include <util/message-queue.hpp>
#include <devices/remote-max7219.hpp>

#include "i2c-ini.hpp"

using namespace nl::rakis::raspberrypi;
using namespace nl::rakis::raspberrypi::protocols;
using nl::rakis::i2c::I2CState;


#if !defined(HAVE_I2C)
#error "This example needs I2C enabled"
#endif


static I2CState config;


static constexpr uint8_t firstPico{ 0x61 };      // the first address we hand out; saved addresses below this are ignored

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

static void loadBoardsFromConfig(I2CBusController<I2CProtocolDriver<util::MessageQueue>>& controller)
{
    for (auto key : config.boardIds()) {
        std::cerr << "Checking board '" << key << "' for I2C address info.\n";

        if (config.hasBoardValue(key, "boardid") && config.hasBoardValue(key, "address")) {
            auto configId = config.boardValue(key, "boardid");
            auto address = atoi(config.boardValue(key, "address").c_str());

            BoardId id;
            if (!parseBoardId(id.bytes, configId)) {
                std::cerr << "Ignoring unparsable board Id of '" << key << "'.\n";
            } else if ((address < firstPico) || (address >= 0b10000000000)) {
                std::cerr << std::format("Ignoring bad address 0x{:02x} for board '{}'.\n", address, key);
            } else {
                std::cerr << std::format("Board '{}', id '{}', address 0x{:02x}.\n", key, configId, address);
                controller.addKnown(id, static_cast<uint8_t>(address));
            }
        } else {
            std::cerr << "- Nothing for '" << key << "'.\n";
        }
    }
}


static std::string boardIdString(const BoardId& id)
{
    return std::format("{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}",
                       id.bytes[0], id.bytes[1], id.bytes[2], id.bytes[3], id.bytes[4], id.bytes[5], id.bytes[6], id.bytes[7]);
}

/**
 * Save the address of a board that confirmed it, so the board gets the same address after we have been restarted. A
 * board that is already in the state file keeps its section (and name), a new one gets its board id as name.
 */
static void rememberBoard(const BoardId& id, uint8_t address)
{
    const std::string boardId{ boardIdString(id) };
    std::string name{ boardId };
    for (auto key : config.boardIds()) {
        if (config.hasBoardValue(key, "boardid") && (config.boardValue(key, "boardid") == boardId)) {
            name = key;
            break;
        }
    }
    if (config.hasBoardValue(name, "address") && (std::atoi(config.boardValue(name, "address").c_str()) == address)) {
        return;     // already saved
    }
    config.setBoardValue(name, "boardid", boardId);
    config.setBoardValue(name, "address", std::to_string(address));
    config.save();
}


/**
 * Set MAX_DEMO=brightness to compare displays instead of counting: every 4 seconds they go through all segments on full
 * strength (the test mode of the chip, whatever the brightness), and then 88888888 at brightness 0, 3, 7 and 15.
 */
template <typename Display>
static void brightnessStep(Display& max, int32_t seconds, bool say)
{
    static constexpr uint8_t levels[] = { 0, 3, 7, 15 };
    const unsigned step = (static_cast<unsigned>(seconds) / 4) % 5;
    if (static_cast<unsigned>(seconds) % 4 != 0) {
        return;
    }
    if (step == 0) {
        if (say) { std::cerr << "Brightness: all segments on, full strength (test mode)\n"; }
        max.displayTest(1);
    } else {
        if (step == 1) { max.displayTest(0); }
        if (say) { std::cerr << std::format("Brightness: 88888888 at {}\n", levels[step - 1]); }
        max.setBrightness(levels[step - 1]);
        max.setNumber(0, 88888888);
    }
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char*argv[])
{
    config.load();

    unsigned count{30};
    if (argc >= 2) {
        count = atoi(argv[1]);
    }
    // The other arguments are the addresses of boards with a MAX7219 8-digit display. The first one counts the seconds up, the
    // second counts down, and so on.
    const bool brightnessDemo = (std::getenv("MAX_DEMO") != nullptr) && (std::string(std::getenv("MAX_DEMO")) == "brightness");
    std::vector<uint8_t> maxAddresses;
    for (int i = 2; i < argc; i++) {
        maxAddresses.push_back(static_cast<uint8_t>(std::strtoul(argv[i], nullptr, 0)));
    }
    auto pi2picoBus = std::make_shared<interfaces::I2CDevI2C>("/dev/i2c-1");
    auto pico2piBus = std::make_shared<interfaces::PigpiodBSCI2C>();
    pi2picoBus->verbose(true);
    pico2piBus->verbose(true);

    I2CProtocolDriver<util::MessageQueue> driver;
    driver.addInterface(pi2picoBus);
    driver.addInterface(pico2piBus);

    // The bus controller hands out the addresses. We keep them in the state file: the ones we know go in, and the ones that
    // boards confirm are saved.
    I2CBusController controller(driver);
    controller.addressRange(firstPico, 0x77);
    loadBoardsFromConfig(controller);
    std::cerr << std::format("Loaded {} boards.\n", config.countBoardIds());
    controller.onConfirmed([](const BoardId& id, uint8_t address) { rememberBoard(id, address); });
    controller.registerHandlers();

    std::cerr << "Starting test\n";

    const uint8_t controllerAddress = 0x0a;

    driver.listenAddress(controllerAddress);
    driver.startListening();

    using RemoteMax = devices::RemoteMAX7219<I2CProtocolDriver<util::MessageQueue>>;
    struct Display {
        std::unique_ptr<RemoteMax> max;
        bool started{ false };
    };
    std::vector<Display> displays;
    for (auto address : maxAddresses) {
        displays.push_back(Display{ std::make_unique<RemoteMax>(driver, address) });
        displays.back().max->numDevices(1);
        std::cerr << std::format("The MAX7219 on the board with address 0x{:02x} will count {}.\n", address,
                                 (displays.size() % 2 == 1) ? "the seconds" : "down");
    }

    std::cerr << "Starting to wait for someone to talk to us.\n";

    // One tick is 10 ms. The controller says Hello once per second, and repeats the addresses that were not confirmed.
    for (unsigned tick = 0; tick < count * 100; tick++) {
        controller.tick();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        driver.processIncoming();

        // Once a board with a display is there: set it up, and count, or compare brightness.
        if (tick % 100 == 50) {
            const int32_t seconds = static_cast<int32_t>(tick / 100);
            for (size_t i = 0; i < displays.size(); i++) {
                auto& display = displays[i];
                if (!controller.isOnline(maxAddresses[i])) {
                    continue;
                }
                if (!display.started) {
                    display.max->reset();
                    display.max->setBrightness(3);
                    display.started = true;
                }
                if (brightnessDemo) {
                    brightnessStep(*display.max, seconds, i == 0);
                } else {
                    display.max->setNumber(0, (i % 2 == 0) ? seconds : static_cast<int32_t>(count) - seconds);
                }
            }
        }
    }

    for (auto& display : displays) {
        if (display.started) {
            display.max->clear();
        }
    }

    std::cerr << "Shutting down.\n";
    driver.stopListening();
    driver.close();
}

