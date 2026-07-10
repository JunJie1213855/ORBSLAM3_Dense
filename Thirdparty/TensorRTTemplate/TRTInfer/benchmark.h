#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <chrono>
#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iomanip>
#include <functional>
#include "TRTinfer.h"

/**
 * @brief Simple inference timing statistics
 */
struct TimingStats
{
    float min_ms;
    float max_ms;
    float mean_ms;
    float median_ms;
    float p95_ms;
    float p99_ms;
    float fps;

    void print() const
    {
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Min:      " << min_ms << " ms" << std::endl;
        std::cout << "  Max:      " << max_ms << " ms" << std::endl;
        std::cout << "  Mean:     " << mean_ms << " ms" << std::endl;
        std::cout << "  Median:   " << median_ms << " ms" << std::endl;
        std::cout << "  95th:     " << p95_ms << " ms" << std::endl;
        std::cout << "  99th:     " << p99_ms << " ms" << std::endl;
        std::cout << "  FPS:      " << fps << std::endl;
    }
};

/**
 * @brief Benchmark utility for TensorRT inference performance testing
 */
class Benchmark
{
public:
    /**
     * @brief Run benchmark with warmup
     * @param infer_func Function to execute for each iteration
     * @param warmup_iterations Number of warmup iterations (default: 10)
     * @param benchmark_iterations Number of benchmark iterations (default: 100)
     * @return Timing statistics
     */
    static TimingStats run(std::function<void()> infer_func,
                           int warmup_iterations = 10,
                           int benchmark_iterations = 100)
    {
        std::cout << "\n========== Benchmark ==========" << std::endl;
        std::cout << "Warmup iterations: " << warmup_iterations << std::endl;
        std::cout << "Benchmark iterations: " << benchmark_iterations << std::endl;
        std::cout << "==============================\n" << std::endl;

        // Warmup phase
        std::cout << "Warming up..." << std::endl;
        for (int i = 0; i < warmup_iterations; ++i)
        {
            infer_func();
        }
        std::cout << "Warmup completed.\n" << std::endl;

        // Benchmark phase
        std::cout << "Running benchmark..." << std::endl;
        std::vector<float> times_ms;
        times_ms.reserve(benchmark_iterations);

        for (int i = 0; i < benchmark_iterations; ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();
            infer_func();
            auto end = std::chrono::high_resolution_clock::now();

            float time_ms = std::chrono::duration<float, std::milli>(end - start).count();
            times_ms.push_back(time_ms);
        }

        // Calculate statistics
        std::sort(times_ms.begin(), times_ms.end());

        TimingStats stats;
        stats.min_ms = times_ms.front();
        stats.max_ms = times_ms.back();
        stats.mean_ms = std::accumulate(times_ms.begin(), times_ms.end(), 0.0f) / times_ms.size();
        stats.median_ms = times_ms[times_ms.size() / 2];
        stats.p95_ms = times_ms[static_cast<size_t>(times_ms.size() * 0.95)];
        stats.p99_ms = times_ms[static_cast<size_t>(times_ms.size() * 0.99)];
        stats.fps = 1000.0f / stats.mean_ms;

        // Print results
        std::cout << "\n========== Benchmark Results ==========" << std::endl;
        stats.print();
        std::cout << "=====================================" << std::endl;

        return stats;
    }

    /**
     * @brief Run benchmark for TRTInfer model with warmup
     * @tparam InputType Input type (void* or cv::Mat)
     * @param model TRTInfer model instance
     * @param input_blob Input data
     * @param warmup_iterations Number of warmup iterations
     * @param benchmark_iterations Number of benchmark iterations
     * @return Timing statistics
     */
    template<typename InputType>
    static TimingStats runModel(TRTInfer& model,
                                const std::unordered_map<std::string, InputType>& input_blob,
                                int warmup_iterations = 10,
                                int benchmark_iterations = 100)
    {
        return run(
            [&model, &input_blob]() {
                model(input_blob);
            },
            warmup_iterations,
            benchmark_iterations
        );
    }
};

#endif  // BENCHMARK_H
