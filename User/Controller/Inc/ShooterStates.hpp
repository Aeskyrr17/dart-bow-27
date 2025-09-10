//
// Created by cosmosmount on 2025/9/10.
//

#ifndef RM26_H7_SHOOTERSTATES_HPP
#define RM26_H7_SHOOTERSTATES_HPP

#include "state.hpp"

namespace ShooterStates
{
    class Relax : public State {
    public:
        void init() override;
        void enter() override;
        void execute() override;
        void exit() override;
    };

    class Warm : public State {
    public:
        void init() override;
        void enter() override;
        void execute() override;
        void exit() override;
    };

    class Fire : public State {
    public:
        void init() override;
        void enter() override;
        void execute() override;
        void exit() override;
    };
}

#endif //RM26_H7_SHOOTERSTATES_HPP