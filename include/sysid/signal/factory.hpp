#pragma once
#include <cmath>

namespace sysid::signal {

enum class Type { Sine, Saw, Trap, Step };

class Signal {
public:
    // Constructor: amplitude, frequency, phase, optional vertical shift
    Signal(Type type, double amplitude, double frequency,
           double phase = 0.0, double offset = 0.0);

    double value(double t) const;

private:
    Type type_;
    double A_;      // amplitude
    double f_;      // frequency
    double phi_;    // phase
    double offset_; // vertical shift

    double r_ = 0.0; // ramp time for Trap / Saw
    double p_ = 0.0; // plateau time for Trap
};

} // namespace sysid::signal
