#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/fft_filter_ccf.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/top_block.h>
#include <gnuradio/analog/sig_source.h>
#include <gnuradio/uhd/usrp_sink.h>

#include <gnuradio/digital/glfsr_source_b.h>
#include <gnuradio/digital/constellation.h>
#include <gnuradio/digital/chunks_to_symbols.h>

#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/streams_to_vector.h>
#include <gnuradio/blocks/throttle.h>

#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <random>
#include <queue>
#include <mutex>
#include <iostream>

#include "matplotlibcpp.h"
#include "send_noise.h"
#include "scan_enemy.h"

namespace plt = matplotlibcpp;


// 第一次扫描算法
std::vector<std::pair<double, float>> adaptive_scan(
    uhd::usrp::multi_usrp::sptr usrp, double start_freq, double end_freq,
    double rate_rx, double gain_rx, double bandwidth_rx, int min_scans = 1) {

    std::vector<std::pair<double, float>> target_freq;  // 存储找到的频率及其幅值
    double center_freq = start_freq;  // 起点
    int scan_count = 0;  // 记录扫描次数

    // 需要改变的参数
    while (scan_count < min_scans) {
        usrp->set_rx_freq(center_freq);
        usrp->set_rx_rate(bandwidth_rx);

        // 执行单次扫描，获取频率和对应的幅值
        std::vector<std::pair<double, float>> current_freqs = scan_enemy(usrp, center_freq, rate_rx, gain_rx, 1024);

        std::cout << "I'm OK" << std::endl;

        // 如果找到频率，将它们加入 target_freq 中
        if (!current_freqs.empty()) {
            target_freq.insert(target_freq.end(), current_freqs.begin(), current_freqs.end());
        }

        // 调整频率，继续扫描
        center_freq += bandwidth_rx;

        // 如果超过了结束频率，重新从 start_freq 开始
        if (center_freq >= end_freq) {
            center_freq = start_freq;
            scan_count++;  // 增加扫描计数
        }
    }

    // 对频率和幅值对按照幅值从大到小进行排序
    std::sort(target_freq.begin(), target_freq.end(), 
        [](const std::pair<double, float>& a, const std::pair<double, float>& b) {
            return a.second > b.second;  // 按幅值从大到小排序
        });

    // 只返回前三个幅值最大的频率，如果数量不足 3 个，则返回全部
    if (target_freq.size() > 3) {
        target_freq.resize(3);  // 保留前三个
    }

    return target_freq;
}



// 排序
std::vector<double> sort_freq(std::vector<double> target_freq)
{
    std::sort(target_freq.begin(), target_freq.end());
    return target_freq;   // 将这些数据从小到大排序，决定下一次扫频的开始
}

std::vector<double> calcu_params(std::vector<double> target_freq){
    /* 计算下一次的开始、结尾、中心频率、采样率 */
    double start_freq = target_freq.front();
    double end_freq = target_freq.back();

    double center_freq = (start_freq + end_freq) / 2;
    double sample_rate = int((end_freq - start_freq)) + 1.0;

    std::vector<double> params = {start_freq, end_freq, center_freq, sample_rate};

    return params;  // 下次采样的时候就在这些区间里采样
}


// 发射信号
void tx_signal(uhd::usrp::multi_usrp::sptr usrp, uhd::stream_args_t stream_args_tx, uhd::device_addr_t usrp_addr, double freq_tx, double rate_tx, double gain_tx, double bandwidth_tx) {
    while (true) {
        usrp->set_tx_freq(freq_tx);
        send_noise(stream_args_tx, usrp_addr, usrp, freq_tx, rate_tx, gain_tx, 1.0, 5.0, bandwidth_tx);
        std::this_thread::sleep_for(std::chrono::seconds(2)); // 每2秒发射一次信号
    }
}


// 应对敌方跳频的方法

// 方法一
/* 应对敌方跳频，一旦发现信号立马启动干扰 */

void Noise_instant_loop(uhd::usrp::multi_usrp::sptr usrp, uhd::device_addr_t usrp_addr, double start_freq, double end_freq, double sample_rate, double gain, double bandwidth_tx){
    // 创建发射数据流
    uhd::stream_args_t stream_args_tx("fc32", "sc16");
    std::vector<size_t> channels_tx = {0, 1};  // 使用第0和第1通道
    stream_args_tx.channels = channels_tx;

    double temp = start_freq;
    double step = sample_rate;

    // 立马启动干扰
    while(true){
        if(temp >= end_freq){
            temp = start_freq;
        }
        // 单次扫描
        std::pair<double, float> instance_signal = scan_enemy_sudden(usrp, temp, sample_rate, gain, 1024);

        if(instance_signal.first != 0){
            std::cout << instance_signal.first << std::endl;
            send_noise(stream_args_tx, usrp_addr, usrp, instance_signal.first, sample_rate, gain, 1.0, 10.0, bandwidth_tx);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 加一个防止爆炸

            temp = instance_signal.first + sample_rate / 2;
            
            continue;
        }
        else {
            temp += sample_rate;
        }
    }
}


// 方法二   智能扫描与预测  我们先扫描一遍，预测出start_freq-end_freq中可能最可能出现信号的频段
struct FrequencyBlock {
    double start_freq;
    double end_freq;
    int priority;
};

// 简单卡尔曼滤波器
double kalman_predict(double current_freq, double previous_freq, double& estimate, double& estimate_error, double process_noise, double measurement_noise) {
    double prediction = estimate;
    estimate_error += process_noise;
    double kalman_gain = estimate_error / (estimate_error + measurement_noise);
    estimate = prediction + kalman_gain * (current_freq - prediction);
    estimate_error = (1 - kalman_gain) * estimate_error;
    return estimate;
}

// 扫描线程
void scan_thread(uhd::usrp::multi_usrp::sptr usrp, std::vector<FrequencyBlock>& freq_blocks, double sample_rate, double gain, std::queue<std::pair<double, float>>& signal_queue, std::mutex& queue_mutex) {
    while (true) {
        for (auto& block : freq_blocks) {
            if (block.priority > 0) { // 只扫描优先级高于0的频段
                std::pair<double, float> detected_signal = scan_enemy_sudden(usrp, block.start_freq, sample_rate, gain, 1024);
                if (detected_signal.first != 0) {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    signal_queue.push(detected_signal);
                    block.priority++; // 提高优先级
                } else {
                    block.priority = std::max(block.priority - 1, 1); // 降低优先级，但不低于1
                }
            }
        }
    }
}

// 检测线程
void detection_thread(std::queue<std::pair<double, float>>& signal_queue, std::vector<double>& freq_history, double& kalman_estimate, double& kalman_error, std::mutex& queue_mutex) {
    while (true) {
        if (!signal_queue.empty()) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            auto detected_signal = signal_queue.front();
            signal_queue.pop();

            freq_history.push_back(detected_signal.first);
            if (freq_history.size() > 10) {
                freq_history.erase(freq_history.begin());
            }

            // 使用卡尔曼滤波器预测下一个频率
            double predicted_freq = kalman_predict(detected_signal.first, freq_history.back(), kalman_estimate, kalman_error, 0.1, 1.0);
            std::cout << "Predicted next frequency: " << predicted_freq << " Hz" << std::endl;
        }
    }
}

// 到时候就直接调用这个就OK
void run_interference_system(uhd::device_addr_t usrp_addr, uhd::usrp::multi_usrp::sptr usrp, double start_freq, double end_freq, double sample_rate, double gain) {
    std::queue<std::pair<double, float>> signal_queue;
    std::vector<double> freq_history;
    std::mutex queue_mutex;

    double kalman_estimate = 0;
    double kalman_error = 1;

    // 初始化频段块
    std::vector<FrequencyBlock> freq_blocks;
    for (double freq = start_freq; freq < end_freq; freq += sample_rate) {
        freq_blocks.push_back({freq, freq + sample_rate, 1});
    }

    // 启动扫描线程
    std::thread scan_t(scan_thread, usrp, std::ref(freq_blocks), sample_rate, gain, std::ref(signal_queue), std::ref(queue_mutex));

    // 启动检测线程
    std::thread detection_t(detection_thread, std::ref(signal_queue), std::ref(freq_history), std::ref(kalman_estimate), std::ref(kalman_error), std::ref(queue_mutex));

    // 等待线程结束
    scan_t.join();
    detection_t.join();
}



// 执行函数
int main() {
    uhd::device_addr_t usrp_addr("type=b200");
    auto usrp = uhd::usrp::multi_usrp::make(usrp_addr);

    // initialize USRP
    double rate_tx = 2e6;
    double freq_tx = 1000e6;
    double gain_tx = 70.0;
    double bandwidth_tx = 1e6;

    usrp->set_tx_rate(rate_tx);
    usrp->set_tx_freq(freq_tx, 0);
    usrp->set_tx_freq(freq_tx, 1);
    usrp->set_tx_gain(gain_tx, 0);
    usrp->set_tx_gain(gain_tx, 1);
    usrp->set_tx_bandwidth(bandwidth_tx, 0);
    usrp->set_tx_bandwidth(bandwidth_tx, 1);

    double rate_rx = 2e6;
    double freq_rx = 1000e6;
    double gain_rx = 70.0;
    double bandwidth_rx = 1e6;

    usrp->set_rx_rate(rate_rx);
    usrp->set_rx_freq(freq_rx, 0);
    usrp->set_rx_freq(freq_rx, 1);
    usrp->set_rx_gain(gain_rx, 0);
    usrp->set_rx_gain(gain_rx, 1);
    usrp->set_rx_bandwidth(bandwidth_rx, 0);
    usrp->set_rx_bandwidth(bandwidth_rx, 1);

    // 接受逻辑
    uhd::stream_args_t stream_args_rx("fc32", "sc16");
    std::vector<size_t> channels_rx = {0};  // 使用第0和第1通道
    stream_args_rx.channels = channels_rx;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args_rx);

    // 发送逻辑
    uhd::stream_args_t stream_args_tx("fc32", "sc16");
    std::vector<size_t> channels_tx = {0, 1};  // 使用第0和第1通道
    stream_args_tx.channels = channels_tx;

    // adaptive_scan
    double start_freq_rx = 890e6;
    double end_freq_rx = 1110e6;

    // 立即干扰策略
    Noise_instant_loop(usrp, usrp_addr, start_freq_rx, end_freq_rx, rate_rx, 70.0, bandwidth_rx);

    // 并行启动tx与rx
    // std::thread tx_thread(tx_signal, usrp, stream_args_tx, usrp_addr, freq_tx, rate_tx, gain_tx, bandwidth_tx);

    std::vector<std::pair<double, float>> detected_freqs = adaptive_scan(usrp, start_freq_rx, end_freq_rx, rate_rx, gain_rx, bandwidth_rx);


    if(!detected_freqs.empty()){
        std::cout << "Detected Main Freq:" << std::endl;
        for(auto& target : detected_freqs){
            std::cout << target.first << "Hz，" << "Alum is " << target.second << std::endl;

            usrp->set_tx_freq(target.first);
            std::cout << target.first << std::endl;
            send_noise(stream_args_tx, usrp_addr, usrp, target.first, rate_tx, gain_tx, 1.0, 60.0, bandwidth_tx);
        }
    } else {
        std::cout << "No Main Freq Detected" << std::endl;
    }
    
    return 0;
}
