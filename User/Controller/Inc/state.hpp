//
// Created by cosmosmount on 2025/9/10.
//

#ifndef RM26_H7_STATE_HPP
#define RM26_H7_STATE_HPP

class State {
public:
    virtual ~State() = default;

    virtual void init();
    virtual void enter();
    virtual void execute();
    virtual void exit();
};

#endif //RM26_H7_STATE_HPP