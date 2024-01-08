#ifndef FeedbackHandler_hpp_
#define FeedbackHandler_hpp_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <linux/can.h>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include "can_wrapper/Wheels.h"
#include "can_wrapper/RosCanConstants.hpp"
#include "CM/CM.h"

template <class Pub_T>
class FeedbackHandler
{
public:
    FeedbackHandler(
        CM_AddressFamily addressFamily,
        std::vector<CM_Address> stms,
        std::string rosTopic_Processed,
        std::string rosTopic_CanRx = RosCanConstants::RosTopics::can_raw_RX,
        std::string rosTopic_CanTx = RosCanConstants::RosTopics::can_raw_TX
    );

    void sendRequest();

    void handleRequestTimerCallback(const ros::TimerEvent &);

    
    
protected:

    bool tryPublish();

    virtual void handleCanMessage(CM_CanMessage cm) = 0;

        Pub_T mProcessedData;

private:
    void handleCanRx(const can_msgs::Frame::ConstPtr &message);
    
    ros::NodeHandle mNh;

    ros::Subscriber mCanRxSub;
    ros::Publisher mCanTxPub;

    ros::Publisher mProcessedPub;

    const CM_AddressFamily mAdrFamily;
    const std::vector<CM_Address> mStms;
    std::unordered_map<CM_Address,ros::Time> mRecivedStms;    
};

#include "can_wrapper/FeedbackHandler.inl"

#endif //FeedbackHandler_hpp_