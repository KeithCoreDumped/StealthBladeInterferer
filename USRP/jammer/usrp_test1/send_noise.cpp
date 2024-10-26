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
#include <fftw3.h>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// 绘制频谱图的函数
void plot_spectrum(const std::vector<std::complex<float>>& data, double rate) {
    namespace plt = matplotlibcpp;
    
    size_t N = data.size();
    
    // 准备 FFT 输入输出
    fftwf_complex* in = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * N);
    fftwf_complex* out = (fftwf_complex*) fftwf_malloc(sizeof(fftwf_complex) * N);
    fftwf_plan p = fftwf_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    // 填充输入数据
    for (size_t i = 0; i < N; ++i) {
        in[i][0] = data[i].real();
        in[i][1] = data[i].imag();
    }

    // 执行 FFT
    fftwf_execute(p);

    // 计算频率轴
    std::vector<double> frequencies(N);
    std::vector<double> magnitudes(N);

    for (size_t i = 0; i < N; ++i) {
        double freq = i * rate / N;  // 计算每个 bin 对应的频率
        double magnitude = std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]);

        frequencies[i] = freq;
        magnitudes[i] = magnitude;
    }

    // 绘制频谱图
    plt::figure();
    plt::plot(frequencies, magnitudes);
    plt::xlabel("Frequency (Hz)");
    plt::ylabel("Magnitude");
    plt::title("Signal Spectrum");
    plt::show();

    // 清理 FFTW 资源
    fftwf_destroy_plan(p);
    fftwf_free(in);
    fftwf_free(out);
}

void plot_time_domain(const std::vector<std::complex<float>>& data, double samp_rate) {
    std::vector<double> time(data.size());
    std::vector<double> real_part(data.size());

    // 计算时间轴和实部
    for (size_t i = 0; i < data.size(); ++i) {
        time[i] = i / samp_rate;  // 时间 = 样本索引 / 采样率
        real_part[i] = std::real(data[i]);
    }

    // 绘制时域图像
    plt::plot(time, real_part);
    plt::title("Time Domain Signal");
    plt::xlabel("Time (s)");
    plt::ylabel("Amplitude");
    plt::grid(true);
    plt::show();
}

// create GLFSR
gr::digital::glfsr_source_b::sptr create_glfsr(int degree=10, int mask=0x3FF, int seed=1, int length = 1023, bool flag=true){
    return gr::digital::glfsr_source_b::make(degree, mask, seed, true);
}

// create bpsk
gr::digital::chunks_to_symbols_bc::sptr create_bpsk(){
    const std::vector<std::complex<float>> bpsk_map = {
        std::complex<float>(-1.0, 0.0),  // mirror 0 -> -1.0
        std::complex<float>(1.0, 0.0)    // mirror 1 -> 1.0
    };
    return gr::digital::chunks_to_symbols_bc::make(bpsk_map);
}

// create low_pass
gr::filter::fft_filter_ccf::sptr create_lowpass(double samp_rate=600e3, double cutoff_freq=300e3, double transition_width=40e3){
    std::vector<float> lpf_taps = gr::filter::firdes::low_pass(
        1.0,             // gain
        samp_rate,       // sample
        cutoff_freq,     // cutoff_freq
        transition_width, // transition
        gr::filter::firdes::WIN_HAMMING // Hamming
    );

    return gr::filter::fft_filter_ccf::make(1, lpf_taps);
}

// 干扰信号
void send_noise(uhd::stream_args_t stream_args, uhd::device_addr_t usrp_addr, uhd::usrp::multi_usrp::sptr usrp, double freq, double rate, double gain, double amplitude, double duration, double bandwidth) {
    usrp->set_tx_freq(freq, 0);
    usrp->set_tx_freq(freq, 1);

    std::cout << "the freq of usrp is:" << freq << std::endl;

    usrp->set_tx_bandwidth(bandwidth, 0);
    usrp->set_tx_bandwidth(bandwidth, 1);

    usrp->set_tx_gain(70.0, 0);
    usrp->set_tx_gain(70.0, 1);

    auto tb = gr::make_top_block("send_data");
    tb->set_max_output_buffer(30000);

    gr::digital::glfsr_source_b::sptr glfsr = create_glfsr();
    gr::digital::chunks_to_symbols_bc::sptr bpsk = create_bpsk();
    gr::filter::fft_filter_ccf::sptr low_pass_filter = create_lowpass(2e6, 1e6, 200e3);
    
    auto vector_sink = gr::blocks::vector_sink_c::make();

    auto usrp_sink = gr::uhd::usrp_sink::make(usrp_addr, stream_args);

    auto throttle = gr::blocks::throttle::make(sizeof(std::complex<float>), rate);

    tb->connect(glfsr, 0, bpsk, 0);
    tb->connect(bpsk, 0, low_pass_filter, 0);
    tb->connect(low_pass_filter, 0, throttle, 0);
    tb->connect(throttle, 0, usrp_sink, 0);
    tb->connect(throttle, 0, usrp_sink, 1);

    tb->connect(throttle, 0, vector_sink, 0);

    // run stream
    std::thread tb_thread([tb]() {
        tb->run();
    });

    // run time
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));

    tb->stop();
    tb->wait();
    tb_thread.join();

    // run calculate time
    const std::vector<std::complex<float>>& data = vector_sink->data();
    // plot_time_domain(data, rate);
    // plot_spectrum(data, rate);
}