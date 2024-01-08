#ifndef MOTOR_VELOCITY_FEEDBACK2_H
#define MOTOR_VELOCITY_FEEDBACK2_H

#include "can_wrapper/Wheels.h"
#include "can_wrapper/RosCanConstants.hpp"
#include "CM/CM.h"
#include "can_wrapper/FeedbackHandler.hpp"

class MotorVelocityFeedback2:
    public FeedbackHandler<can_wrapper::Wheels>
{
    public:
    MotorVelocityFeedback2(
        CM_AddressFamily addressFamily,
        std::vector<CM_Address> stms,
        std::string rosTopic_Processed
    ) : FeedbackHandler(addressFamily,stms,rosTopic_Processed)
    {}

    virtual void handleCanMessage(CM_CanMessage cm) override
    {
        mProcessedData = can_wrapper::Wheels();
        tryPublish();
    }
};

#endif //MOTOR_VELOCITY_FEEDBACK2_H