/*
 * Copyright (C) 2025-2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the license, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ipc-iox-private.h"

#include <iox2/iceoryx2.hpp>

#include <cstdint>
#include <string_view>

namespace fs = std::filesystem;

namespace Syntalos::ipc
{

namespace
{

uint64_t stablePathHash(std::string_view value)
{
    uint64_t hash = 14695981039346656037ULL;

    for (const auto ch : value) {
        hash ^= static_cast<unsigned char>(ch);
        hash *= 1099511628211ULL;
    }

    return hash;
}

iox2::bb::FileName makeIoxPrefixForRootPath(std::string_view rootPath)
{
    constexpr char hexChars[] = "0123456789abcdef";
    const auto hash = stablePathHash(rootPath);

    std::string prefix = "sy_";
    for (int shift = 60; shift >= 0; shift -= 4)
        prefix.push_back(hexChars[(hash >> shift) & 0x0f]);
    prefix.push_back('_');

    const auto staticPrefix =
        iox2::bb::StaticString<iox2::bb::platform::IOX2_MAX_FILENAME_LENGTH>::from_utf8_null_terminated_unchecked(
            prefix.c_str())
            .value();
    return iox2::bb::FileName::create(staticPrefix).value();
}

} // namespace

std::string makeModuleServiceName(const std::string &instanceId, const std::string &channelName)
{
    // the total resulting length of this string must not be longer than 255 characters, because
    // that is the length set for IDs in SY_IOX_ID_MAX_LEN
    std::string svcId = "Sy/" + instanceId.substr(0, 120) + "/" + channelName.substr(0, 128);
    assert(svcId.length() <= SY_IOX_ID_MAX_LEN);
    return svcId;
}

void findAndCleanupDeadNodes()
{
    iox2::Node<iox2::ServiceType::Ipc>::list(ioxDefaultConfig().global_config(), [](auto node_state) -> auto {
        node_state.dead([](auto view) -> auto {
            std::cout << "ipc: Detected dead node: ";
            if (view.details().has_value()) {
                std::cout << view.details().value().name().to_string().unchecked_access().c_str();
            }
            std::cout << std::endl;
            IOX2_DISCARD_RESULT(view.try_remove_stale_resources());
        });
        return iox2::CallbackProgression::Continue;
    }).value();
}

const iox2::Config &ioxDefaultConfig()
{
    static const auto config = [] {
        auto cfg = iox2::Config();

        fs::path runtimeDir;
        if (const char *env_p = std::getenv("XDG_RUNTIME_DIR")) {
            runtimeDir = fs::path(env_p);
            if (!fs::is_directory(runtimeDir))
                runtimeDir = fs::path("/tmp");
        } else {
            runtimeDir = fs::path("/tmp");
        }

        const auto rootPathFs = runtimeDir / "syntalos-iox";
        const auto rootPathString = rootPathFs.string();
        const auto rootPath =
            iox2::bb::StaticString<iox2::bb::platform::IOX2_MAX_PATH_LENGTH>::from_utf8_null_terminated_unchecked(
                rootPathString.c_str())
                .value();

        cfg.global().set_root_path(iox2::bb::Path::create(rootPath).value());
        // Work around POSIX shm objects ignoring root_path by namespacing the prefix per runtime directory.
        cfg.global().set_prefix(makeIoxPrefixForRootPath(rootPathString));

        cfg.global().node().set_cleanup_dead_nodes_on_creation(true);
        cfg.global().node().set_cleanup_dead_nodes_on_destruction(true);

        cfg.defaults().publish_subscribe().set_backpressure_strategy(iox2::BackpressureStrategy::RetryUntilDelivered);
        cfg.defaults().request_response().set_client_backpressure_strategy(
            iox2::BackpressureStrategy::RetryUntilDelivered);
        cfg.defaults().request_response().set_server_backpressure_strategy(
            iox2::BackpressureStrategy::RetryUntilDelivered);

        return cfg;
    }();

    return config;
}

} // namespace Syntalos::ipc
