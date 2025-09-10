//
// Created by cosmosmount on 2025/9/10.
//

#ifndef RM26_H7_CHASSISSTATES_HPP
#define RM26_H7_CHASSISSTATES_HPP

#include "state.hpp"

namespace ChassisStates {
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
    };
}

#endif //RM26_H7_CHASSISSTATES_HPP