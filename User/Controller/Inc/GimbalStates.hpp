//
// Created by cosmosmount on 2025/9/10.
//

#ifndef RM26_H7_GIMBALSTATES_HPP
#define RM26_H7_GIMBALSTATES_HPP

#include "state.hpp"
#include "filter.hpp"
#include "GM6020.hpp"

using namespace Filter;
using namespace Math;

namespace GimbalStates
{
    inline GM6020* YawMotor;
    inline GM6020* PitchMotor;

    class Relax : public State {
    public:
        void init() override;
        void enter() override;
        void execute() override;
        void exit() override;
    };

    class Run : public State {
    public:
        void init() override;
        void enter() override;
        void execute() override;
        void exit() override;
    private:
        float YawSet = 0.0f;
        KalmanFilter YawPosFilter;

        float PitchSet = 0.0f;
        float PitchLowerLimit = 12*DegreeToRad;
        float PitchUpperLimit = -27*DegreeToRad;
        KalmanFilter PitchPosFilter;
    };
}

#endif //RM26_H7_GIMBALSTATES_HPP