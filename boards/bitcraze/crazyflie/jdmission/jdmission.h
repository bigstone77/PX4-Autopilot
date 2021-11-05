
#pragma once

#include <stdint.h>

#include <battery/battery.h>

#include <drivers/device/device.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/log_message.h>

typedef struct __attribute__((__packed__)){
	uint32_t goldenkey;
	uint8_t length;
	uint8_t checksum;
	uint16_t option;
	union{
		int8_t data8[80];
		int16_t data16[40];
		int32_t data32[20];
	};
}packet_t;


class JDMission : public ModuleBase<JDMission>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	JDMission();
	~JDMission() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	bool init();

private:

	uORB::SubscriptionCallbackWorkItem _sensor_gyro_sub{this, ORB_ID(sensor_gyro)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _optical_flow_sub{ORB_ID(optical_flow)};
	uORB::Subscription _log_message_sub{ORB_ID(log_message)};
	void Checksum(packet_t *pkt);
	void task_main();
	void Run() override;

	int _fd;
	packet_t packet;
	hrt_abstime last_time;
};


