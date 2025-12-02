#include "sysid/signal/factory.hpp"
#include <cmath>

namespace sysid::signal {

Signal::Signal(Type type, double amplitude, double frequency,
               double phase, double offset)
    : type_(type), A_(amplitude), f_(frequency), phi_(phase), offset_(offset)
{}

double Signal::value(double t) const {
    if (f_ == 0.0) return 0.0;

    double T = 1.0 / f_;
    double t_shifted = t + phi_ / (2.0 * M_PI * f_);  // phase as time shift
    double phi = std::fmod(t_shifted, T);
    if (phi < 0) phi += T;

    double waveform_value = 0.0;

    switch (type_) {
    case Type::Sine:
        waveform_value = std::sin(2.0 * M_PI * f_ * t + phi_);
        break;

    case Type::Saw:
        waveform_value = 2.0 * (phi / T - std::floor(phi / T + 0.5));
        break;

    case Type::Trap: {
        double r = T / 6.0;
        double p = T / 6.0;
        double up_end   = r;
        double high_end = r + p;
        double down_end = r + p + r;

        if (phi < up_end)
            waveform_value = -1.0 + 2.0 * (phi / r);
        else if (phi < high_end)
            waveform_value = 1.0;
        else if (phi < down_end)
            waveform_value = 1.0 - 2.0 * ((phi - high_end) / r);
        else
            waveform_value = -1.0;
        break;
    }

    case Type::Step:
        waveform_value = (phi < T / 2.0) ? 1.0 : -1.0;
        break;
    }

    return waveform_value * A_ + offset_;
}

} // namespace sysid::signal
