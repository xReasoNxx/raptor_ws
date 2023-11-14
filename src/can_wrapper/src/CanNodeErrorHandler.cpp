#include "can_wrapper/CanNodeErrorHandler.hpp"

bool CanNodeErrorHandler::sIsInitialized = false;
ros::NodeHandle CanNodeErrorHandler::sNh;
ros::Subscriber CanNodeErrorHandler::sRawCanSub;
ros::Publisher CanNodeErrorHandler::sCanRawPub;

void CanNodeErrorHandler::init()
{
	if (sIsInitialized)
		return;
	sIsInitialized = true;
	sRawCanSub = sNh.subscribe(RosCanConstants::RosTopics::can_raw_RX, 256, &CanNodeErrorHandler::handleRosCallback);
	sCanRawPub = sNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
}

void CanNodeErrorHandler::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	if ((msg->id & CanMessage::Masks::Error_Mask) != CanMessage::Masks::Error_Mask)
		return;
	CanMessage cm(msg.get());
	handleErrorFrame(cm);
}

void CanNodeErrorHandler::handleErrorFrame(CanMessage cmErr)
{
	if (cmErr.data.node_errors.select_err & 0b0001)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes_Mask,
			cmErr.data.node_errors.rpm_scale_err,
			CanNodeSettingsProvider::TypeGroups::Rpm_Scale_Group,
			MaxNumberOfParams::Rpm_Scale_Params);

	if (cmErr.data.node_errors.select_err & 0b0010)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes_Mask,
			cmErr.data.node_errors.motor_a_reg_err,
			CanNodeSettingsProvider::TypeGroups::Motor_A_Reg_Group,
			MaxNumberOfParams::Motor_Reg_Params);

	if (cmErr.data.node_errors.select_err & 0b0100)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes_Mask,
			cmErr.data.node_errors.motor_b_reg_err,
			CanNodeSettingsProvider::TypeGroups::Motor_B_Reg_Group,
			MaxNumberOfParams::Motor_Reg_Params);

	if (cmErr.data.node_errors.select_err & 0b1000)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes_Mask,
			cmErr.data.node_errors.motor_c_reg_err,
			CanNodeSettingsProvider::TypeGroups::Motor_C_Reg_Group,
			MaxNumberOfParams::Motor_Reg_Params);
}

void CanNodeErrorHandler::handleError(uint8_t dev_id, uint8_t err, CanNodeSettingsProvider::TypeGroups err_group, MaxNumberOfParams iter_count)
{
	if (err != 0xF)
	{
		sCanRawPub.publish(createResponseFrame(dev_id, err | err_group));
		ros::spinOnce();
		return;
	}

	for (uint8_t i = 0; i < iter_count; i++)
	{
		sCanRawPub.publish(createResponseFrame(dev_id, i | err_group));
	}

	ros::spinOnce();
}

can_msgs::Frame CanNodeErrorHandler::createResponseFrame(uint8_t dev_id, uint8_t type_id)
{
	CanMessage cm;
	cm.address = dev_id | CanMessage::Masks::Error_Mask;
	cm.data.stm_init.type_id = type_id;
	cm.data.stm_init.var = CanNodeSettingsProvider::getSetting(dev_id, cm.data.stm_init.type_id);
	return (can_msgs::Frame)cm;
}