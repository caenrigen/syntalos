/*
 * Copyright (C) 2026 Syntalos Project
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "modules/camera-arv/qarv/decoders/monounpacked.h"
#include "modules/camera-arv/qarv/decoders/swscaledecoder.h"
#include "symemopt.h"

#include <mimalloc.h>
#include <opencv2/core/utility.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;

struct Options {
    std::string format = "all";
    int width = 1920;
    int height = 1080;
    int iterations = 500;
    int repetitions = 5;
    int retained = 32;
    std::string freeThread = "both";
};

struct Measurement {
    double seconds;
    std::uint64_t checksum;
};

enum class SubmissionMode {
    LegacyClone,
    DirectOwnership,
};

enum class FreeThread {
    Producer,
    Consumer,
};

[[noreturn]] void usage(const char *program, int exitCode)
{
    std::ostream &out = exitCode == EXIT_SUCCESS ? std::cout : std::cerr;
    out << "Usage: " << program << " [OPTIONS]\n"
        << "  --format all|rgb8|mono8|mono12\n"
        << "  --width N\n"
        << "  --height N\n"
        << "  --iterations N\n"
        << "  --repetitions N\n"
        << "  --retained N      Number of simulated in-flight frames\n"
        << "  --free-thread producer|consumer|both\n"
        << "  --help\n";
    std::exit(exitCode);
}

int parsePositive(std::string_view value, std::string_view option)
{
    int parsed = 0;
    const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || parsed <= 0) {
        std::cerr << "Invalid value for " << option << ": " << value << '\n';
        std::exit(EXIT_FAILURE);
    }
    return parsed;
}

Options parseOptions(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--help")
            usage(argv[0], EXIT_SUCCESS);
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << arg << '\n';
            usage(argv[0], EXIT_FAILURE);
        }

        const std::string_view value(argv[++i]);
        if (arg == "--format")
            options.format = value;
        else if (arg == "--width")
            options.width = parsePositive(value, arg);
        else if (arg == "--height")
            options.height = parsePositive(value, arg);
        else if (arg == "--iterations")
            options.iterations = parsePositive(value, arg);
        else if (arg == "--repetitions")
            options.repetitions = parsePositive(value, arg);
        else if (arg == "--retained")
            options.retained = parsePositive(value, arg);
        else if (arg == "--free-thread")
            options.freeThread = value;
        else {
            std::cerr << "Unknown option: " << arg << '\n';
            usage(argv[0], EXIT_FAILURE);
        }
    }

    if (options.format != "all" && options.format != "rgb8"
        && options.format != "mono8" && options.format != "mono12") {
        std::cerr << "Unknown format: " << options.format << '\n';
        usage(argv[0], EXIT_FAILURE);
    }
    if (options.freeThread != "producer" && options.freeThread != "consumer"
        && options.freeThread != "both") {
        std::cerr << "Unknown free thread: " << options.freeThread << '\n';
        usage(argv[0], EXIT_FAILURE);
    }
    return options;
}

std::unique_ptr<QArvDecoder> makeDecoder(std::string_view format, QSize size)
{
    if (format == "rgb8") {
        return std::make_unique<QArv::SwScaleDecoder>(
            size,
            AV_PIX_FMT_RGB24,
            ARV_PIXEL_FORMAT_RGB_8_PACKED,
            SWS_FAST_BILINEAR);
    }
    if (format == "mono8") {
        return std::make_unique<
            QArv::MonoUnpackedDecoder<uint8_t, 8, ARV_PIXEL_FORMAT_MONO_8>>(size);
    }
    if (format == "mono12") {
        return std::make_unique<
            QArv::MonoUnpackedDecoder<uint16_t, 12, ARV_PIXEL_FORMAT_MONO_12>>(size);
    }

    std::cerr << "Unsupported benchmark format: " << format << '\n';
    std::exit(EXIT_FAILURE);
}

std::vector<char> makeInput(std::string_view format, const Options &options)
{
    const size_t bytesPerPixel = format == "rgb8" ? 3 : format == "mono12" ? 2 : 1;
    std::vector<char> input(
        static_cast<size_t>(options.width) * static_cast<size_t>(options.height) * bytesPerPixel);
    for (size_t i = 0; i < input.size(); ++i)
        input[i] = static_cast<char>((i * 131U + 17U) & 0xffU);
    return input;
}

cv::Mat decodeFrame(
    QArvDecoder &decoder,
    QByteArrayView input,
    SubmissionMode mode,
    cv::Mat &reusableDecoderOutput)
{
    if (mode == SubmissionMode::LegacyClone) {
        decoder.decodeInto(input, reusableDecoderOutput);
        return reusableDecoderOutput.clone();
    }

    cv::Mat output;
    decoder.decodeInto(input, output);
    return output;
}

void addToChecksum(const cv::Mat &output, int iteration, std::uint64_t &checksum)
{
    const size_t byteCount = output.total() * output.elemSize();
    checksum += output.data[(static_cast<size_t>(iteration) * 131U) % byteCount];
}

Measurement measure(
    QArvDecoder &decoder,
    QByteArrayView input,
    const Options &options,
    SubmissionMode mode,
    FreeThread freeThread)
{
    cv::Mat reusableDecoderOutput;
    std::uint64_t checksum = 0;

    if (freeThread == FreeThread::Producer) {
        std::vector<cv::Mat> retained(static_cast<size_t>(options.retained));
        const auto start = Clock::now();
        for (int i = 0; i < options.iterations; ++i) {
            const size_t slot = static_cast<size_t>(i) % retained.size();
            retained[slot] = decodeFrame(decoder, input, mode, reusableDecoderOutput);
            addToChecksum(retained[slot], i, checksum);
        }
        const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
        return {seconds, checksum};
    }

    std::vector<cv::Mat> queue(static_cast<size_t>(options.retained));
    alignas(64) std::atomic<size_t> written = 0;
    alignas(64) std::atomic<size_t> read = 0;
    std::atomic<bool> startConsumer = false;

    std::thread consumer([&] {
        while (!startConsumer.load(std::memory_order_acquire))
            std::this_thread::yield();

        for (int i = 0; i < options.iterations; ++i) {
            const size_t position = read.load(std::memory_order_relaxed);
            while (written.load(std::memory_order_acquire) == position)
                std::this_thread::yield();

            cv::Mat output = std::move(queue[position % queue.size()]);
            addToChecksum(output, i, checksum);
            read.store(position + 1, std::memory_order_release);
        }
    });

    const auto start = Clock::now();
    startConsumer.store(true, std::memory_order_release);
    for (int i = 0; i < options.iterations; ++i) {
        const size_t position = written.load(std::memory_order_relaxed);
        while (position - read.load(std::memory_order_acquire) >= queue.size())
            std::this_thread::yield();

        queue[position % queue.size()] = decodeFrame(decoder, input, mode, reusableDecoderOutput);
        written.store(position + 1, std::memory_order_release);
    }
    consumer.join();
    const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
    return {seconds, checksum};
}

double median(std::vector<double> values)
{
    std::ranges::sort(values);
    const size_t middle = values.size() / 2;
    if ((values.size() % 2U) == 0)
        return (values[middle - 1] + values[middle]) / 2.0;
    return values[middle];
}

void benchmarkFormat(
    std::string_view format,
    const Options &options,
    FreeThread freeThread)
{
    const QSize size(options.width, options.height);
    auto decoder = makeDecoder(format, size);
    const auto input = makeInput(format, options);
    const QByteArrayView inputView(input.data(), static_cast<qsizetype>(input.size()));

    cv::Mat warmupOutput;
    for (int i = 0; i < 20; ++i)
        decoder->decodeInto(inputView, warmupOutput);

    std::vector<double> legacyTimes;
    std::vector<double> directTimes;
    legacyTimes.reserve(static_cast<size_t>(options.repetitions));
    directTimes.reserve(static_cast<size_t>(options.repetitions));
    std::uint64_t legacyChecksum = 0;
    std::uint64_t directChecksum = 0;

    // Alternate the order to reduce systematic cache and thermal bias.
    for (int repetition = 0; repetition < options.repetitions; ++repetition) {
        const bool directFirst = (repetition % 2) != 0;
        const Measurement first = measure(
            *decoder,
            inputView,
            options,
            directFirst ? SubmissionMode::DirectOwnership : SubmissionMode::LegacyClone,
            freeThread);
        const Measurement second = measure(
            *decoder,
            inputView,
            options,
            directFirst ? SubmissionMode::LegacyClone : SubmissionMode::DirectOwnership,
            freeThread);

        (directFirst ? directTimes : legacyTimes).push_back(first.seconds);
        (directFirst ? legacyTimes : directTimes).push_back(second.seconds);
        (directFirst ? directChecksum : legacyChecksum) += first.checksum;
        (directFirst ? legacyChecksum : directChecksum) += second.checksum;
    }

    if (legacyChecksum != directChecksum) {
        std::cerr << "Output mismatch for " << format << '\n';
        std::exit(EXIT_FAILURE);
    }

    const double legacySeconds = median(std::move(legacyTimes));
    const double directSeconds = median(std::move(directTimes));
    const size_t outputBytes = static_cast<size_t>(options.width)
        * static_cast<size_t>(options.height)
        * CV_ELEM_SIZE(decoder->cvType());
    const double frameMiB = static_cast<double>(outputBytes) / (1024.0 * 1024.0);

    std::cout << std::left << std::setw(8) << format
              << " free=" << std::setw(8)
              << (freeThread == FreeThread::Producer ? "producer" : "consumer")
              << " frame=" << std::fixed << std::setprecision(2) << frameMiB << " MiB"
              << " legacy-clone=" << std::setprecision(4) << legacySeconds << " s"
              << " direct-owning=" << directSeconds << " s"
              << " speedup=" << std::setprecision(3) << legacySeconds / directSeconds << "x"
              << " avoided-copy=" << std::setprecision(2) << frameMiB << " MiB/frame"
              << " checksum=" << legacyChecksum << '\n';
}
} // namespace

int main(int argc, char **argv)
{
    const Options options = parseOptions(argc, argv);
    Syntalos::configureMimallocDefaultAllocator();

    std::cout << "camera-arv decodeInto benchmark\n"
              << "OpenCV: " << cv::getVersionString() << '\n'
              << "OpenCV CPU features: " << cv::getCPUFeaturesLine() << '\n'
              << "mimalloc: " << MI_MALLOC_VERSION << '\n'
#ifdef NDEBUG
              << "assertions: disabled\n"
#else
              << "assertions: enabled\n"
#endif
              << "resolution: " << options.width << 'x' << options.height
              << ", iterations: " << options.iterations
              << ", repetitions: " << options.repetitions
              << ", retained frames: " << options.retained << "\n\n";

    constexpr std::array<std::string_view, 3> allFormats = {"rgb8", "mono8", "mono12"};
    constexpr std::array<FreeThread, 2> allFreeThreads = {
        FreeThread::Producer,
        FreeThread::Consumer,
    };
    const auto benchmarkSelectedFormat = [&](FreeThread freeThread) {
        if (options.format == "all") {
            for (const std::string_view format : allFormats)
                benchmarkFormat(format, options, freeThread);
        } else {
            benchmarkFormat(options.format, options, freeThread);
        }
    };

    if (options.freeThread == "both") {
        for (const FreeThread freeThread : allFreeThreads)
            benchmarkSelectedFormat(freeThread);
    } else {
        benchmarkSelectedFormat(
            options.freeThread == "producer" ? FreeThread::Producer : FreeThread::Consumer);
    }
    return EXIT_SUCCESS;
}
