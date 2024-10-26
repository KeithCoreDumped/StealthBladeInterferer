#ifndef SCAN_ENEMY_HPP
#define SCAN_ENEMY_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include <fftw3.h>

#ifdef __cplusplus
}
#endif

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <fftw3.h>
#include <iostream>
#include <vector>
#include <complex>

// 函数声明
// void save_frequencies_to_excel(const std::vector<double>& frequencies, const std::string& filename);

std::vector<std::pair<double, float>> scan_enemy(
    uhd::usrp::multi_usrp::sptr usrp, double center_freq, double sample_rate, double gain, int fft_size = 1024);

std::pair<double, float> scan_enemy_sudden(
    uhd::usrp::multi_usrp::sptr usrp, double center_freq, double sample_rate, double gain, int fft_size = 1024);

#endif // SCAN_ENEMY_HPP
