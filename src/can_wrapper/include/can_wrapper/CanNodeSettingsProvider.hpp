#ifndef CAN_WRAPPER_CAN_NODE_SETTINGS_PROVIDER_HPP
#define CAN_WRAPPER_CAN_NODE_SETTINGS_PROVIDER_HPP
#define INIT_MAX_TYPE_ID 0x50

#include <cstdint>
#include <memory>
#include <linux/can.h>
#include "can_wrapper/CanMessage.hpp"

/**
 * @brief Provides access to CAN node settings.
 */
class CanNodeSettingsProvider
{
public:
	enum TypeGroups : uint8_t
	{
		Rpm_Scale_Group = 0x10,
		Motor_A_Reg_Group = 0x20,
		Motor_B_Reg_Group = 0x30,
		Motor_C_Reg_Group = 0x40
	};

	enum RpmScaleAdresses : uint8_t
	{
		Motor_Control = 0x10,
		Encoder_Feedback = 0x11
	};

	/**
	 * @brief Initializes the settings provider.
	 */
	static void init();

	/**
	 * @brief Gets a setting value for a specific frame ID and type group.
	 * @param frame_id The frame ID that gets properly masked to device_id.
	 * @param typeGroup The type group family.
	 * @param setting_id The setting ID.
	 * @return The setting value.
	 */
	static float getSetting(canid_t frame_id, TypeGroups typeGroup, uint8_t setting_id);

	/**
	 * @brief Gets a setting value for a specific type ID and setting ID.
	 * @param type_id The type ID.
	 * @param setting_id The setting ID.
	 * @return The setting value.
	 */
	static float getSetting(uint8_t type_id, uint8_t setting_id);

	/**
	 * @brief Sets a setting value for a specific type ID and setting ID.
	 * @param type_id The type ID.
	 * @param setting_id The setting ID.
	 * @param value The value to set.
	 * @return 0 if successful, -1 otherwise.
	 */
	static int8_t setSetting(uint8_t type_id, uint8_t setting_id, float value);

	/**
	 * @brief Sets a setting value for all devices for a specific setting ID.
	 * @param setting_id The setting ID.
	 * @param value The value to set.
	 * @return 0 if successful, -1 otherwise.
	 */
	static int8_t setSettingForAllDevices(uint8_t setting_id, float value);

private:
	static bool sIsInitialized;
	/**
	 * @brief The node settings array.
	 */
	static float mNodeSettings[0xF][INIT_MAX_TYPE_ID];

	CanNodeSettingsProvider() = delete; // Prevent instantiation
};

#endif // CAN_WRAPPER_CAN_NODE_SETTINGS_PROVIDER_HPP