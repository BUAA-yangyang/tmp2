#ifndef SAFE_STAND_GYRO_FILTER_H
#define SAFE_STAND_GYRO_FILTER_H

#include <array>
#include <cmath>
#include <limits>

struct SafeStandGyroFilterResult {
    bool valid;
    bool discontinuity;
    double norm;
};

class SafeStandGyroFilter {
public:
    SafeStandGyroFilter(double timeConstantS, double maximumStepS)
        : _timeConstantS(timeConstantS), _maximumStepS(maximumStepS) {
        reset();
    }

    void reset() {
        _initialized = false;
        _stampS = 0.0;
        _value.fill(0.0);
    }

    SafeStandGyroFilterResult update(
        double stampS, const std::array<double, 3>& sample) {
        if(!std::isfinite(stampS)
           || !std::isfinite(_timeConstantS)
           || !std::isfinite(_maximumStepS)
           || _timeConstantS <= 0.0
           || _maximumStepS <= 0.0
           || !finite(sample)) {
            reset();
            return {false, true, std::numeric_limits<double>::infinity()};
        }

        if(!_initialized) {
            _value = sample;
            _stampS = stampS;
            _initialized = true;
            return {true, false, vectorNorm(_value)};
        }

        const double stepS = stampS - _stampS;
        if(stepS < 0.0 || stepS > _maximumStepS) {
            _value = sample;
            _stampS = stampS;
            return {true, true, vectorNorm(_value)};
        }
        if(stepS == 0.0) {
            return {true, false, vectorNorm(_value)};
        }

        const double alpha = -std::expm1(-stepS / _timeConstantS);
        for(std::size_t index = 0; index < _value.size(); ++index) {
            _value[index] += alpha * (sample[index] - _value[index]);
        }
        _stampS = stampS;
        return {true, false, vectorNorm(_value)};
    }

private:
    static bool finite(const std::array<double, 3>& value) {
        return std::isfinite(value[0])
            && std::isfinite(value[1])
            && std::isfinite(value[2]);
    }

    static double vectorNorm(const std::array<double, 3>& value) {
        return std::sqrt(
            value[0] * value[0]
            + value[1] * value[1]
            + value[2] * value[2]);
    }

    double _timeConstantS;
    double _maximumStepS;
    bool _initialized;
    double _stampS;
    std::array<double, 3> _value;
};

#endif  // SAFE_STAND_GYRO_FILTER_H
