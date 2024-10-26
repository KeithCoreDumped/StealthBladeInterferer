#ifndef SIGNAL_PROCESSING_H
#define SIGNAL_PROCESSING_H

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
#include <complex>

#include "matplotlibcpp.h"

// Namespace alias for matplotlibcpp
namespace plt = matplotlibcpp;

// Function to plot time domain signal
void plot_time_domain(const std::vector<std::complex<float>>& data, double samp_rate);

// Function to create a GLFSR source
gr::digital::glfsr_source_b::sptr create_glfsr(int degree = 10, int mask = 1, int seed = 0x400, int length = 1023, bool flag = true);

// Function to create a BPSK modulator
gr::digital::chunks_to_symbols_bc::sptr create_bpsk();

// Function to create a low-pass filter
gr::filter::fft_filter_ccf::sptr create_lowpass(double samp_rate = 2e6, double cutoff_freq = 1e6, double transition_width = 100e3);

// Function to send noise through a USRP
void send_noise(uhd::stream_args_t stream_args, uhd::device_addr_t usrp_addr, uhd::usrp::multi_usrp::sptr usrp, double freq, double rate, double gain, double amplitude, double duration, double bandwidth);

#endif // SIGNAL_PROCESSING_H
