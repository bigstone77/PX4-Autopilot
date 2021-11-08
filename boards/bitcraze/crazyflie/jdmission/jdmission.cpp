/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file syslink_main.cpp
 * Entry point for syslink module used to communicate with the NRF module on a Crazyflie
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mqueue.h>

#include <drivers/drv_board_led.h>

#include <systemlib/err.h>

#include <board_config.h>

#include "jdmission.h"


extern float motor_output[4];
extern "C" int getPrintBuf(char *buf);
static char printBuf[100];
JDMission::JDMission():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	packet.goldenkey = 0x6114A826;
	packet.length = 30;
	packet.option = 1;
}


JDMission::~JDMission()
{
}


void JDMission::Checksum(packet_t *pkt)
{
	int sum=pkt->option&0xFF;

	sum += (pkt->option>>8)&0xFF;
	for(int n=0; n<pkt->length-8; n++)
		sum += pkt->data8[n];
	pkt->checksum = (sum&0xFF);
}

void JDMission::Run()		//2ms
{
	hrt_abstime now_us = hrt_absolute_time();

	static int cnt;
	if((++cnt%20)!=0)
		return;

	int _len = getPrintBuf(printBuf);
	if(_len != 0){
		packet.option = 2;
		packet.length = _len+8;
		memcpy(packet.data8, printBuf, _len);
		packet.data8[_len] = 0;
		Checksum(&packet);
		write(_fd, (const void *)&packet, packet.length);
		last_time = now_us;
		return;

	}
	packet.option = 1;
	packet.length = 28;
	if(_sensor_gyro_sub.updated()){
		sensor_gyro_s gyro;
		if (_sensor_gyro_sub.update(&gyro)) {
			packet.data8[0] = (int8_t)(gyro.x*10);
			packet.data8[1] = (int8_t)(gyro.y*10);
			packet.data8[2] = (int8_t)(gyro.z*10);
			packet.data8[3] = 0;
		}
	}
	if(_sensor_accel_sub.updated()){
		sensor_accel_s accel;
		if (_sensor_accel_sub.update(&accel)) {
			packet.data8[4] = (int8_t)(accel.x*5);
			packet.data8[5] = (int8_t)(accel.y*5);
			packet.data8[6] = (int8_t)(accel.z*5);
			packet.data8[7] = 0;
		}
	}
	if(_optical_flow_sub.updated()){
		optical_flow_s optical_flow;
		if(_optical_flow_sub.update(&optical_flow)){
			packet.data8[12] = (int8_t)(optical_flow.pixel_flow_x_integral*50);
			packet.data8[13] = (int8_t)(optical_flow.pixel_flow_y_integral*50);
		}
	}

	if(_distance_sensor_sub.updated()){
		distance_sensor_s distance_sensor;
		if(_distance_sensor_sub.update(&distance_sensor)){
			packet.data8[14] = (int8_t)(distance_sensor.current_distance*20);
		}
	}

	if (_magnetometer_sub.updated()) {
		vehicle_magnetometer_s magnetometer;
		_magnetometer_sub.update(&magnetometer);
		packet.data8[16] = (int8_t)(magnetometer.magnetometer_ga[0]*100);
		packet.data8[17] = (int8_t)(magnetometer.magnetometer_ga[1]*100);
		packet.data8[18] = (int8_t)(magnetometer.magnetometer_ga[2]*100);
		packet.data8[19] = 0;
	}

	if((now_us-last_time)>20000){
		for(int n=0;n <4 ;n++)
			packet.data8[8+n] = (int8_t)(motor_output[n]*100);
		Checksum(&packet);
		write(_fd, (const void *)&packet, packet.length);

		last_time = now_us;
	}
}

bool JDMission::init()
{
	_fd = px4_open("/dev/ttyS2", O_RDWR | O_NOCTTY);
	if(_fd < 0)
		return false;

	if (!_sensor_gyro_sub.registerCallback())
		return false;

	ScheduleNow();
	return true;
}

int JDMission::task_spawn(int argc, char *argv[])
{
	JDMission *instance = new JDMission();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int JDMission::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int JDMission::print_usage(const char *reason)
{
//	printf("fd:%d\r\n", _fd);
	return 0;
}

extern "C" __EXPORT int jdmission_main(int argc, char *argv[])
{
	return JDMission::main(argc, argv);
}


static char _printBuf[64][80];
static int rx_cnt, tx_cnt;
extern "C" __EXPORT void putPrintBuf(char *buf)
{
	if(_printBuf[rx_cnt][0] == 0){
		int len = strlen(buf);
		len = len<50? len : 50;
		memcpy(_printBuf[rx_cnt], buf, len);
		_printBuf[rx_cnt][len] = 0;
		rx_cnt = (rx_cnt+1)&0x3F;
	}
}


extern "C" __EXPORT int getPrintBuf(char *buf)
{
	if(_printBuf[tx_cnt][0] == 0)
		return 0;

	int len = strlen(_printBuf[tx_cnt]);
	memcpy(buf, _printBuf[tx_cnt], len);
	buf[len] = 0;
	memset(_printBuf[tx_cnt], 0, 80);
	tx_cnt = (tx_cnt+1)&0x3F;
	return len;
}

extern "C" __EXPORT int _print(int n, FAR const IPTR char *fmt, ...)
{
  va_list ap;
  int     ret;
  char buf[80];
  static int cnt;

  if((++cnt%n)!=0)
	  return 0;

  va_start(ap, fmt);
#ifdef CONFIG_FILE_STREAM
  ret = vfprintf(stdout, fmt, ap);
#else
  ret = vdprintf(STDOUT_FILENO, fmt, ap);
#endif
  vsnprintf(buf, 70, fmt, ap);
  va_end(ap);
  putPrintBuf(buf);
  return ret;
}


