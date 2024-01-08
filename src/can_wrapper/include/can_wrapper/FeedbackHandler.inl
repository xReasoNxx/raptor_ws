#ifndef FeedbackHandler_inl_
#define FeedbackHandler_inl_

template <class Pub_T>
FeedbackHandler<Pub_T>::FeedbackHandler(
    CM_AddressFamily addressFamily,
    std::vector<CM_Address> stms,
    std::string rosTopic_Processed,
    std::string rosTopic_CanRx,
    std::string rosTopic_CanTx
):
mAdrFamily(addressFamily),
mStms(stms)
{
    mCanRxSub = mNh.subscribe(rosTopic_CanRx, 256, &FeedbackHandler::handleCanRx, this);
    mCanTxPub = mNh.advertise<can_msgs::Frame>(rosTopic_CanTx, 128);
    mProcessedPub = mNh.advertise<Pub_T>(rosTopic_Processed, 128);

    mRecivedStms.reserve(mStms.size());
    for(auto target : mStms)
    {
        mRecivedStms.emplace(std::pair<CM_Address,ros::Time>(target,ros::Time()));
    }
}


template <class Pub_T>
void FeedbackHandler<Pub_T>::sendRequest()
{
    CM_CanMessage request;
    request.dataLen = 0;
    for(auto target : mStms)
    {
        request.address = (CM_Address_t)target | (CM_Address_t)mAdrFamily | CAN_RTR_FLAG;
        mCanTxPub.publish(CM_convert_CanMessage_mtof(request));
    }
}


template <class Pub_T>
void FeedbackHandler<Pub_T>::handleRequestTimerCallback(const ros::TimerEvent &)
{
	sendRequest();
}


template <class Pub_T>
void FeedbackHandler<Pub_T>::handleCanRx(const can_msgs::Frame::ConstPtr &message)
{
    if ((message->id & CM_ADDRESS_FAMILY_MASK) != (CM_Address_t)mAdrFamily)
		return;
    handleCanMessage(CM_convert_CanMessage_ftom(message.get()));
}

template <class Pub_T>
bool FeedbackHandler<Pub_T>::tryPublish()
{
    if(!std::all_of(mRecivedStms.begin(), mRecivedStms.end(), [](std::pair<CM_Address,ros::Time> pair){return pair.second >= ros::Time::now() - RosCanConstants::max_stm_sync_time;}))
        return false;
    mProcessedPub.publish(mProcessedData);
    return true;
}

#endif //FeedbackHandler_inl_