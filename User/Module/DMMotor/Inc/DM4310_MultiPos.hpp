#ifndef RM26_DM4310_MULTIPOS_HPP
#define RM26_DM4310_MULTIPOS_HPP

#include <cstdint>

#include "DMMotor.hpp"

class DM4310_MultiPos final : public DMMotor
{
private:
    float P_MIN;
    float P_MAX;

    float V_MIN;
    float V_MAX;

    float T_MIN;
    float T_MAX;

public:
    float KP;
    float KD;

    float LowerPosLimit;
    float UpperPosLimit;
    float offset;

    float rawPositionFdb;
    float lastRawPositionFdb;
    int32_t positionRoundCount;
    bool positionInited;

    DM4310_MultiPos();
    virtual ~DM4310_MultiPos() = default;

    [[nodiscard]] inline float Get_P_MAX() const override { return P_MAX; }
    [[nodiscard]] inline float Get_P_MIN() const override { return P_MIN; }

    [[nodiscard]] inline float Get_V_MAX() const override { return V_MAX; }
    [[nodiscard]] inline float Get_V_MIN() const override { return V_MIN; }

    [[nodiscard]] inline float Get_T_MAX() const override { return T_MAX; }
    [[nodiscard]] inline float Get_T_MIN() const override { return T_MIN; }

    MotorStateTypeDef AliveCheck() override;
    void SetOutput() override;
    void ReceiveData(uint8_t *buffer) override;
};

#endif // RM26_DM4310_MULTIPOS_HPP
