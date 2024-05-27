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


#include <cctype>
#include <iostream>
#include <fstream>
#include <string>
#include <ranges>

#include "i2c-ini.hpp"


using namespace nl::rakis::i2c;


/**
 * @brief Load the configuration from the INI file.
 */
void I2CState::load()
{
    std::string iniFile(config_file_);
    std::ifstream iniStream(iniFile);

    if (!iniStream) {
        std::cerr << "Cannot open INI file: " << iniFile << "\n";
        return;
    }
    std::string line;
    std::string section{ "general" };
    while (std::getline(iniStream, line)) {
        line = trim(line);

        if (line.empty() || line[0] == '#') {
            continue;
        }
        if ((line.front() == '[') && (line.back() == ']')) {
            section = line.substr(1, line.size() - 2);
            continue;
        }
        auto eqPos = line.find('=');
        if (eqPos != std::string::npos) {
            auto key = trim(line.substr(0, eqPos));
            for (auto& c: key) {
                c = std::tolower(c);
            }
            auto value = trim(line.substr(eqPos + 1));
            config_[section][key] = value;
        }
    }
}


/**
 * @brief Save the configuration to the INI file.
 */
void I2CState::save()
{
    if (!dirty()) {
        return;
    }
    {
        std::string iniFile(config_file_);
        std::ofstream iniStream(iniFile);
        if (!iniStream) {
            std::cerr << "Cannot open INI file: " << iniFile << "\n";
            return;
        }
        for (auto& section : config_) {
            iniStream << '[' << section.first << "]\n";
            for (auto& keyValue : section.second) {
                iniStream << keyValue.first << " = " << keyValue.second << "\n";
            }
        }
        markClean();
    }
    std::cerr << "Saved configuration.\n";
}