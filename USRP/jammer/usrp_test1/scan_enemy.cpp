#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <fftw3.h>
#include <iostream>
#include <vector>
#include <complex>
#include <thread>

#include <gnuradio/top_block.h>
#include <gnuradio/uhd/usrp_source.h>
#include <gnuradio/fft/fft_vcc.h>
#include <gnuradio/blocks/stream_to_vector.h>
#include <gnuradio/blocks/complex_to_mag_squared.h>
#include <gnuradio/blocks/vector_sink.h>
#include <gnuradio/blocks/throttle.h>
#include <gnuradio/filter/firdes.h>
#include <iostream>

#include "send_noise.h"

// #include "xlnt/xlnt.hpp"

// 重新组合

void split_freq_magnitude_pairs(
    const std::vector<std::pair<double, float>>& freq_magnitude_pairs,
    std::vector<double>& frequencies,
    std::vector<float>& magnitudes)
{
    frequencies.clear();
    magnitudes.clear();

    for (const auto& pair : freq_magnitude_pairs) {
        frequencies.push_back(pair.first);
        magnitudes.push_back(pair.second);
    }
}


/* 写入xslx表 */
/*
void save_frequencies_to_excel(const std::vector<double>& frequencies, const std::vector<float>& magnitudes, const std::string& filename){
    if (frequencies.size() != magnitudes.size()) {
        throw std::runtime_error("Frequencies and magnitudes vectors must be of the same size.");
    }

    xlnt::workbook wb;
    xlnt::worksheet ws = wb.active_sheet();
    ws.title("Detected Frequencies");

    ws.cell("A1").value("Freq(Hz)");
    ws.cell("B1").value("Magnitudes");

    for(size_t i = 0; i < frequencies.size(); ++i){
        ws.cell("A" + std::to_string(i + 2)).value(frequencies[i]);
        ws.cell("B" + std::to_string(i + 2)).value(magnitudes[i]);
    }

    wb.save(filename);
}
*/

// 使用流图来解决FFT问题
std::vector<std::pair<double, float>> scan_enemy(
    uhd::usrp::multi_usrp::sptr usrp, double center_freq, double sample_rate, double gain, int fft_size = 1024) {

    // 创建流图
    auto tb = gr::make_top_block("scan_enemy");

    // 创建 USRP 源块，用于接收信号
    auto usrp_source = gr::uhd::usrp_source::make(uhd::device_addr_t("type=b200"), uhd::stream_args_t("fc32"));
    usrp_source->set_samp_rate(sample_rate);
    usrp_source->set_center_freq(center_freq);
    usrp_source->set_gain(gain);

    // 将接收到的流信号转为向量，准备做 FFT
    auto stream_to_vector = gr::blocks::stream_to_vector::make(sizeof(gr_complex), fft_size);

    // FFT块：执行FFT变换
    // 创建 Hamming 窗口
    std::vector<float> window = gr::fft::window::hamming(fft_size);

    // FFT块：执行FFT变换
    auto fft_block = gr::fft::fft_vcc::make(fft_size, true, window, true, 1);

    // 将 FFT 输出的复数转为幅值平方
    auto complex_to_mag_squared = gr::blocks::complex_to_mag_squared::make(fft_size);

    // 用于收集 FFT 结果的 vector_sink
    auto vector_sink = gr::blocks::vector_sink_f::make(fft_size);

    // 创建流图连接
    tb->connect(usrp_source, 0, stream_to_vector, 0);
    tb->connect(stream_to_vector, 0, fft_block, 0);
    tb->connect(fft_block, 0, complex_to_mag_squared, 0);
    tb->connect(complex_to_mag_squared, 0, vector_sink, 0);

    // 运行流图
    tb->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 等待信号处理
    tb->stop();
    tb->wait();

    // 从 vector_sink 中获取数据
    std::vector<float> magnitudes = vector_sink->data();

    // 扫描阈值检测并打印
    float threshold = 3;  // 设定阈值
    std::vector<std::pair<double, float>> freq_magnitude_pairs;
    
    std::cout << "Analyzing FFT output..." << std::endl;
    for (int i = 0; i < fft_size; i++) {
        // 计算每个频率 bin 对应的频率
        double freq = center_freq + (i - fft_size / 2) * sample_rate / fft_size;
        float magnitude = magnitudes[i];

        std::cout << "Frequency: " << freq << " Hz, Magnitude: " << magnitude << std::endl;

        // 根据阈值筛选出显著的频率
        if (magnitude > threshold) {
            freq_magnitude_pairs.push_back(std::make_pair(freq, magnitude));
        }
    }

    return freq_magnitude_pairs;
}




/* 应对敌方的跳频 */

// 方法一：实时干扰响应：在扫描到敌人的信号时，立马启动干扰
std::pair<double, float> scan_enemy_sudden(
    uhd::usrp::multi_usrp::sptr usrp, double center_freq, double sample_rate, double gain, int fft_size = 1024) {

    auto tb = gr::make_top_block("scan_enemy");

    auto usrp_source = gr::uhd::usrp_source::make(uhd::device_addr_t("type=b200"), uhd::stream_args_t("fc32"));
    usrp_source->set_samp_rate(sample_rate);
    usrp_source->set_center_freq(center_freq);
    usrp_source->set_gain(gain);

    auto stream_to_vector = gr::blocks::stream_to_vector::make(sizeof(gr_complex), fft_size);
    std::vector<float> window = gr::fft::window::hamming(fft_size);   // 启动FFT变换
    auto fft_block = gr::fft::fft_vcc::make(fft_size, true, window, true, 1);

    // 将 FFT 输出的复数转为幅值平方
    auto complex_to_mag_squared = gr::blocks::complex_to_mag_squared::make(fft_size);
    auto vector_sink = gr::blocks::vector_sink_f::make(fft_size);

    tb->connect(usrp_source, 0, stream_to_vector, 0);
    tb->connect(stream_to_vector, 0, fft_block, 0);
    tb->connect(fft_block, 0, complex_to_mag_squared, 0);
    tb->connect(complex_to_mag_squared, 0, vector_sink, 0);

    tb->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 等待信号处理
    tb->stop();
    tb->wait();


    // 从 vector_sink 中获取数据
    std::vector<float> magnitudes = vector_sink->data();

    // 扫描阈值检测并打印
    float threshold = 20000;  // 设定阈值
    std::vector<std::pair<double, float>> freq_magnitude_pairs;
    
    std::cout << "Analyzing FFT output..." << std::endl;
    for (int i = 0; i < fft_size; i++) {
        // 计算每个频率 bin 对应的频率
        double freq = center_freq + (i - fft_size / 2) * sample_rate / fft_size;
        float magnitude = magnitudes[i];

        // std::cout << "Frequency: " << freq << " Hz, Magnitude: " << magnitude << std::endl;

        // 一旦发现，立马启动，开始干扰
        if (magnitude > threshold) {
            freq_magnitude_pairs.push_back(std::make_pair(freq, magnitude));
            return std::make_pair(freq, magnitude);
        }
    }
    return std::make_pair(0, 0);
}
