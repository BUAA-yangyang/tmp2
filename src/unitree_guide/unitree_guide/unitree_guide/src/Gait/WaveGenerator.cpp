/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/WaveGenerator.h"
#include <cmath>
#include <iostream>

WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias, double controlPeriod)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias),
      _controlPeriod(controlPeriod), _passT(0.0)
{

    if ((_period <= 0) || !std::isfinite(_period))
    {
        std::cout << "[ERROR] The period of WaveGenerator should be finite and greater than 0" << std::endl;
        exit(-1);
    }

    if ((_stRatio >= 1) || (_stRatio <= 0))
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0))
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    if ((_controlPeriod <= 0) || !std::isfinite(_controlPeriod))
    {
        std::cout << "[ERROR] The controlPeriod of WaveGenerator should be finite and greater than 0" << std::endl;
        exit(-1);
    }

    _contactPast.setZero();
    _phasePast << 0.5, 0.5, 0.5, 0.5;
    _switchStatus.setZero();
    _statusPast = WaveStatus::SWING_ALL;
}

WaveGenerator::~WaveGenerator()
{
}

void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status)
{
    // Keep gait phase on the same discrete controller clock used by the
    // estimator and trajectory generators.  Wall-clock phase advancement
    // makes the gait run 1 / RTF times too fast when Gazebo is slower than
    // real time (for example under Livox load).
    _passT = fmod(_passT + _controlPeriod, _period);
    calcWave(_phase, _contact, status);

    if (status != _statusPast)
    {
        if (_switchStatus.sum() == 0)
        {
            _switchStatus.setOnes();
        }
        calcWave(_phasePast, _contactPast, _statusPast);
        // two special case
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL))
        {
            _contactPast.setOnes();
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL))
        {
            _contactPast.setZero();
        }
    }

    if (_switchStatus.sum() != 0)
    {
        for (int i(0); i < 4; ++i)
        {
            if (_contact(i) == _contactPast(i))
            {
                _switchStatus(i) = 0;
            }
            else
            {
                _contact(i) = _contactPast(i);
                _phase(i) = _phasePast(i);
            }
        }
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status;
        }
    }

    phaseResult = _phase;
    contactResult = _contact;
}

float WaveGenerator::getTstance()
{
    return _period * _stRatio;
}

float WaveGenerator::getTswing()
{
    return _period * (1 - _stRatio);
}

float WaveGenerator::getT()
{
    return _period;
}

void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL)
    {
        for (int i(0); i < 4; ++i)
        {
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period;
            if (_normalT(i) < _stRatio)
            {
                contact(i) = 1;
                phase(i) = _normalT(i) / _stRatio;
            }
            else
            {
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    }
    else if (status == WaveStatus::SWING_ALL)
    {
        contact.setZero();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
    else if (status == WaveStatus::STANCE_ALL)
    {
        contact.setOnes();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
}
