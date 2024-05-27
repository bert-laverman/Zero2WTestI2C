#pragma once
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


#include <ranges>
#include <tuple>
#include <string>
#include <map>


namespace nl::rakis::i2c {

/**
 * @brief This class maintains the mapping of BoardIds to addresses, and what devices are available.
 */
class I2CState {
    std::string config_file_{ "i2c-state.ini" };
    std::map<std::string, std::map<std::string,std::string>> config_;
    bool dirty_{ false };

    static inline std::string trim(std::string str) {
        auto view = str
                | std::views::drop_while(isspace)
                | std::views::reverse
                | std::views::drop_while(isspace)
                | std::views::reverse;
        return std::string(view.begin(), view.end());
    }

    static inline constexpr const char* boardHeader{ "board:" };
    static inline constexpr const char* interfaceHeader{ "interface:" };
    static inline constexpr const char* deviceHeader{ "device:" };

public:
    void load();
    void save();

    inline bool dirty() const { return dirty_; }
    inline void markClean() { dirty_ = false; }
    inline void markDirty() { dirty_ = true; }

    inline bool has(std::string section) const{ return config_.find(section) != config_.end(); }
    inline std::map<std::string,std::string>& operator[](const std::string& section) { return config_[section]; }
    inline const std::map<std::string,std::string>& operator[](const std::string& section) const { return config_.at(section); }

    inline bool has(std::string section, std::string key) const {
        return has(section) && config_.at(section).find(key) != config_.at(section).end();
    }

    // Boards
    inline auto boardIds() const -> auto {
        return std::views::keys(config_)
            | std::views::filter([](auto& key) { return key.starts_with(boardHeader); })
            | std::views::transform([](auto& key) { return key.substr(6); });
    }
    inline bool hasBoardId(std::string board_id) const { return has(boardHeader + board_id); }
    inline unsigned countBoardIds() const {
        unsigned count{ 0 };
        for ([[maybe_unused]] auto key : boardIds()) { count++; }
        return count;
    }
    inline std::map<std::string,std::string>& board(std::string board_id) { return config_.at(boardHeader + board_id); }
    inline const std::map<std::string,std::string>& board(std::string board_id) const { return config_.at(boardHeader + board_id); }

    inline std::map<std::string,std::string>& addBoard(std::string board_id) {
        if (!hasBoardId(board_id)) { markDirty(); }
        return config_[boardHeader + board_id];
    }
    inline bool hasBoardValue(std::string board_id, std::string key) const {
        return hasBoardId(board_id) && board(board_id).find(key) != board(board_id).end();
    }
    inline auto boardKeys(std::string board_id) const { return std::views::keys(board(board_id)); }
    inline std::string boardValue(std::string board_id, std::string key) const { return trim(board(board_id).at(key)); }
    inline void setBoardValue(std::string board_id, std::string key, std::string value) { addBoard(board_id) [key] = value; markDirty(); }
};

} // namespace nl::rakis::i2c