/* neuv_defs.hpp */
/*
 * Software License Agreement (BSD License)
 *
 *  Neuvition SDK Library - www.neuvition.com
 *  Copyright (c) 2016-2019, Neuvition, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived 
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS  
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#ifndef __NEUVDEFS_H__
#define __NEUVDEFS_H__

#ifdef _WINDOWS
#define _DLL_EXPORT
#else
#define _LINUX
#endif
#ifdef _LINUX
#define DECLSPEC_EXPORT  
#define CALLBACK __attribute__((stdcall))
#endif

#ifdef _DLL_EXPORT
#define CALLBACK __stdcall
#ifdef  NEUVWINSDK_EXPORTS
#define DECLSPEC_EXPORT _declspec(dllexport)
#else
#define DECLSPEC_EXPORT _declspec(dllimport)
#endif 
#endif

#include <vector>
#include <cstdlib>
#include <cstdint>
#include <opencv2/imgcodecs.hpp>

namespace neuvition {

	#define MAX_LASERS   4
	#define TOF_DIM      3
	#define TOF_WIDTH 1800
	#define TOF_HEIGHT 720
	#define QP(n) (1.0f/(1<<n))
	#ifdef _DLL_EXPORT
	#ifdef __cplusplus
	extern "C" {
	#endif 
	#endif

	typedef uint32_t nvid_t;
	typedef struct NEUV_UNIT { 
		int x; 
		int y; 
		int z; 
		uint8_t r; 
		uint8_t g; 
		uint8_t b; 
		uint8_t lid; 
		uint8_t apd_id; 
		uint16_t row; // line
		uint16_t col; // pixel
		uint32_t tof; 
		uint8_t intensity; 
		// uint32_t timestamp;
		uint32_t time_sec;
		uint32_t time_usec;
		uint8_t level; 
		uint8_t tofts; 
		uint8_t intensityts; 

		NEUV_UNIT& operator = (const NEUV_UNIT& val)
		{
			x = val.x; 
			y = val.y; 
			z = val.z; 
			r = val.r; 
			g = val.g; 
			b = val.b; 
			lid = val.lid; 
			apd_id = val.apd_id; 
			row= val.row; // line
			col = val.col; // pixel
			tof = val.tof; 
			intensity = val.intensity; 
			// timestamp = val.timestamp;
			time_sec = val.time_sec;
			time_usec = val.time_usec;
			level = val.level; 
			tofts = val.tofts; 
			intensityts = val.intensityts; 
			
			return *this;
		}
	} NeuvUnit;

	typedef struct NEUV_POINT { 
		int x; 
		int y; 
		int z; 
		uint8_t r; 
		uint8_t g; 
		uint8_t b; 
		uint32_t tof; 
	} NeuvPoint; 
	
	typedef struct TOF_DATA { 
		uint16_t line_id; 
		uint16_t pixel_id; 
		uint32_t tof_ns; 
		uint8_t laser_id; 
		uint8_t apd_id; 
		uint8_t r; 
		uint8_t g; 
		uint8_t b; 
		uint8_t intensity; 
		uint32_t timestamp; 
	} TofPoint;

	typedef struct IMU_DATA { 
		int32_t sec; 
		int32_t usec; 
		int32_t roll; 
		int32_t yaw; 
		int32_t pitch; 
		int16_t quat_i; 
		int16_t quat_j; 
		int16_t quat_k; 
		int16_t quat_r; 
		int16_t gro_x;
		int16_t gro_y;
		int16_t gro_z;
		int16_t line_accx;
		int16_t line_accy;
		int16_t line_accz;
	} ImuData;

	typedef struct GPRMC { 
		uint8_t status; 
		double utc; 
		char lat; 
		char lon; 
		double latitude; 
		double longitude; 
		double speed; 
		double course; 
		uint32_t ddmmyy; 
		double variation; 
		char vardirection; 
		char checksum; 
	} NeuGPRMC;

	typedef struct NEU_POSCOR { 
		short ratio_x; 
		short ratio_y; 
		short xmove; 
		short ymove; 
		short xcoeff1;  
		short xcoeff2; 
		short ycoeff1; 
		short averz; 
		short zcoeff0; 
	} NeuPosCor;

	typedef struct NEUV_WIREDATA { 
		uint8_t id;
		int angle; 
		int high;
	} NeuvWireData;

	typedef struct NEUV_CT {
		uint8_t detecting; 
		uint8_t has_car;
		uint8_t detected; 
		uint8_t is_open; 
		uint8_t is_alarm; 
		uint8_t is_saved;
		uint32_t width; 
		uint32_t height; 
		uint32_t length;
	} NeuvCt;

	typedef struct LASER_STATUS{
		uint8_t state; 
		float temperature; 
		uint16_t pulserate; 
		uint16_t powervoltage; 
		uint16_t firstcurrent; 
		uint16_t secondcurrent; 
		float pulsewidth; 
		uint16_t laserpower; 
		float boardtemp; 
		float probe1temp; 
		float probe2temp; 
		float fpgatemp; 
		uint8_t evk1status; 
		uint8_t evk2status; 
		uint8_t spistatus; 
		uint8_t imustatus;
		uint32_t launchtimes;
	} NeuvLaserStatus;

typedef struct camera_point_pos 
{
    int x;
    int y;
    int pixel_id;
    int line_id;
    uint8_t r; 
	uint8_t g; 
	uint8_t b; 
	int ladarx;
	int ladary;
	int ladarz;

}CAMERA_POINT_POS;

typedef struct LIDAR_INFO_STATUS
{
	char device_type[32];
	char serial_number[32];
	char manufacturer[32];
	char date_of_manufacture[32];
	char hardware_ver[32];
	char software_ver[32];
	char auth_code[32];
	uint8_t horizontal_fov;
	uint8_t vertical_fov;
	uint32_t max_distance;
	uint16_t accuracy;
	uint16_t wave_length;
	uint32_t curr_time;
	uint32_t power_on_time;
	uint16_t laser_power;
	uint16_t fps;
	uint16_t laser_rate;
	uint8_t cam_status;
	uint8_t lidar_status;
	float lidar_temperature;
} LidarInfoStatus;

   typedef struct _PosCorParams{int xt; int yt;int ctrow; int ctcol; float xcf[24]; float ycf[24];} PosCorParams;
   typedef struct _SLPosCorParams{int xt; int ctcol; float xcf[2];} SLPosCorParams; 
   typedef struct _CameraParams{int width; int height; float fx ; float fy; float cx; float cy; float k1; float k2; float p1; float p2; float k3;} CameraParams;
   typedef struct _CamFusionParams{float rx; float ry; float rz; int tx; int ty; int tz;} CamFusionParams;
   typedef struct _DisCorParams{float dc1; float dc2; float dc3; float dc4;float dc5; float dc6; float dc7;float dc8;}DisCorParams;
   typedef struct _DisZCorParams{float dc1; float dc2; float dc3; float dc4;float dc5; float dc6; float dc7;float dc8;}DisZCorParams;
   typedef struct _CenterCorParams{float ry; float rx;}CenterCorParams;

   typedef struct _ZPulse{int z; float a; float b; float c; float d; float e; float zcf[9];} ZPulse;
   typedef struct _APDPulse{int pixel; int line; float a0; float b0; float a1; float b1;} APDPulse;
   typedef struct _RefCorParams{ZPulse zpulse[2]; APDPulse apdpulse[3];} RefCorParams;
   typedef struct _SurfaceCorParams{float zcf0[36 * 90];} SurfaceCorParams;
 

	typedef std::vector<NeuvUnit> NeuvUnits;
    typedef std::vector<std::vector<NeuvUnit> > NeuvUnits_2D;

	typedef std::vector<NeuvPoint> NeuvPoints;
	typedef std::vector<double> LaserIncidentAngles;
	typedef std::vector<NeuvWireData> NeuvWireDatas;
	typedef std::vector<CAMERA_POINT_POS> NeuvCameraLadarDatas;

	enum neuv_connect_type { ONLY_TCP, TCP_AND_MULTICAST, ONLY_MULTICAST};
	enum neuv_device_type { TITAN_M1, TITAN_M1_PRO, TITAN_M1_PLUS, TITAN_M1_R, TITAN_S1,TITAN_M1_A, TITAN_M1_SL, TITAN_M2, TITAN_M1_M, TITAN_M1_PRO_SL, TITAN_M1_R_SL};
	enum neuv_cmd_code {
		NEUV_CMD_START_SCAN = 1,
		NEUV_CMD_STOP_SCAN = 2,
		NEUV_CMD_GET_DEVICE_INFO = 3,
		NEUV_CMD_START_STREAM = 6,
		NEUV_CMD_STOP_STREAM = 7,
		NEUV_CMD_SHUTDOWN = 8,
		NEUV_CMD_GET_PARAMS = 13,
		NEUV_CMD_SET_RT_PARAM = 14,
		NEUV_CMD_SET_LASER_KHZLV = 15,
		NEUV_CMD_SET_LASER_COUNT = 17,
		NEUV_CMD_SET_FRAME_COUNT = 18,
		NEUV_CMD_SET_TRECT_LINE = 19,
		NEUV_CMD_SET_TRECT_PIXEL = 20,
		NEUV_CMD_SET_PWM_VALUE = 22,
		NEUV_CMD_SET_SCAN_MODE = 23,
		NEUV_CMD_SET_DAC_VOLTAGE = 25,
		NEUV_CMD_SET_NOISE_POINTS = 26,
		NEUV_CMD_SET_VOXTEL_PERIOD = 27,
		NEUV_CMD_SET_CAMERA_CROP_WH = 29,
		NEUV_CMD_SET_CAMERA_CROP_XY = 30,
		NEUV_CMD_SET_SYS_MODE = 31,
		NEUV_CMD_SET_LASER_TASK_MODE = 32,
		NEUV_CMD_SAVE_CONFIG_DATA = 33,
		NEUV_CMD_SET_PIXEL_OFFSET = 34,
		NEUV_CMD_SET_DENSITY = 35,
		NEUV_CMD_UPDATE_FPGA_VERSION = 38,
		NEUV_CMD_UPDATE_FPGA_STATUS = 39,
		NEUV_CMD_GET_DEVICE_STATUS = 40,
		NEUV_CMD_CAM_IMAGE_UPDATED = 41,
		NEUV_CMD_SET_CAM_X_DEVIATE = 42,
		NEUV_CMD_SET_CAM_Y_DEVIATE = 43,
		NEUV_CMD_SET_CAM_Z_DEVIATE = 44,
		NEUV_CMD_STILL_ALIVE = 45,
		NEUV_CMD_GPS_UPDATED = 46,
		NEUV_CMD_SET_VOLTAGE_OFFSET = 50,
		NEUV_CMD_GET_LIDAR_INFO_STATUS = 51,
		NEUV_CMD_NIL = 0xffff,

		NEUV_CMD_TEMP0 = 400,
		NEUV_CMD_TEMP1 = 401,
		NEUV_CMD_TEMP2 = 402,
		NEUV_CMD_TEMP3 = 403,
		NEUV_CMD_TEMP4 = 404,
		NEUV_CMD_TEMP5 = 405,
		NEUV_CMD_TEMP6 = 406,
		NEUV_CMD_TEMP7 = 407,
		NEUV_CMD_TEMP8 = 408,
		NEUV_CMD_TEMP9 = 409,
		NEUV_CMD_TEMP10 = 410,
		NEUV_CMD_TEMP80 = 480,
		NEUV_CMD_TEMP81 = 481,
		NEUV_CMD_TEMP_RSP = 499,

		NEUV_CMD_LIVE_PUB = 0x10001, // publish
		NEUV_CMD_LIVE_SUB = 0x10002, // subscribe
		NEUV_CMD_LIVE_FRH = 0x10003, // frame head
		NEUV_CMD_LIVE_FRT = 0x10004, // frame tail
		NEUV_CMD_LIVE_PCZ = 0x10005, // frame payload
		NEUV_CMD_LIVE_CLP = 0x10006, // client-side ping-pong
		NEUV_CMD_LIVE_SLP = 0x10007, // server-side ping-pong
		NEUV_CMD_LIVE_PS1 = 0x10008, // start push stream
		NEUV_CMD_LIVE_PS2 = 0x10009, // stop push stream
		NEUV_CMD_LIVE_LS0 = 0x1000a, // get stream info
		NEUV_CMD_LIVE_LS1 = 0x1000b, // start pull stream
		NEUV_CMD_LIVE_LS2 = 0x1000c, // stop pull stream
		NEUV_CMD_LIVE_NIL = 0x1ffff
	};

	class INeuvEvent {
	public:
		virtual ~INeuvEvent() {}
		virtual void on_connect(int, const char*) = 0;
		virtual void on_disconnect(int) = 0;
		virtual void on_response(int, enum neuv_cmd_code) = 0;
		virtual void on_framedata(int, int64_t, const NeuvUnits&, const nvid_t&) = 0;

		virtual void on_imudata(int, int64_t, const NeuvUnits&, const ImuData&) = 0;
		virtual void on_mjpgdata(int, int64_t, cv::Mat) = 0;
		virtual void on_pczdata(bool) = 0;
		virtual void on_Ladar_Camera(neuvition::NeuvCameraLadarDatas * neuvcameraladardatas) = 0;
		virtual void on_lidar_info_status(neuvition::LidarInfoStatus * lidarInfoStatus) = 0;

	};
	
	class  NeuvEventPCZCallBack {
		public:
	virtual void on_framedata(int, int64_t, const NeuvUnits&, const nvid_t&) = 0;
		
	};
	

	DECLSPEC_EXPORT PosCorParams * Get_Pcz_PosCorParams();

	DECLSPEC_EXPORT void Set_Pcz_PosCorParams(PosCorParams * params);

	DECLSPEC_EXPORT void set_pczevent_callback(NeuvEventPCZCallBack  * pczevent_cb_t);

	DECLSPEC_EXPORT void open_pczdata(char * absolutefilepath);



	

	DECLSPEC_EXPORT int compute_tof2xyz_table(const double angle_x, const double angle_y, const double bias_y, const int device_type, const NeuPosCor& pos_cor);
	DECLSPEC_EXPORT void compute_tof2xyz_table_with_multi_lasers(const LaserIncidentAngles& angles, const int device_type, const NeuPosCor& pos_cor);
	DECLSPEC_EXPORT uint64_t ntohll(uint64_t val);
	DECLSPEC_EXPORT uint64_t htonll(uint64_t val);

	DECLSPEC_EXPORT int setup_client(const char* host, const int port, INeuvEvent* handler, const bool flag);
	DECLSPEC_EXPORT int setup_client2(const char *host_addr, const int host_port, const char *listen_addr, const char *multi_addr, const int multi_port, INeuvEvent *handler, const bool flag, neuv_connect_type connect_type);
	DECLSPEC_EXPORT int teardown_client();
	DECLSPEC_EXPORT int start_scan();
	DECLSPEC_EXPORT int stop_scan();
	DECLSPEC_EXPORT int start_stream();
	DECLSPEC_EXPORT int stop_stream();
	DECLSPEC_EXPORT int shutdown_device();
	DECLSPEC_EXPORT int query_device_status();
	DECLSPEC_EXPORT int query_device_params();
	DECLSPEC_EXPORT int save_device_params();

	DECLSPEC_EXPORT int set_event_handler(INeuvEvent* handler);
	DECLSPEC_EXPORT int set_reconnect_params(const bool enabled, const size_t seconds);
	DECLSPEC_EXPORT int set_flip_axis(const bool flip_x, const bool flip_y);
	DECLSPEC_EXPORT int set_laser_power(const int percent);
	DECLSPEC_EXPORT int set_laser_interval(const int index);
	DECLSPEC_EXPORT int set_scan_mode(const int index);
	DECLSPEC_EXPORT int set_frame_frequency(const int fps);
	DECLSPEC_EXPORT int set_frame_line_quantity(const int frameline);
	DECLSPEC_EXPORT int set_camera_status(const bool enabled);
	DECLSPEC_EXPORT int set_imu_status(const bool enabled);
	DECLSPEC_EXPORT int set_gps_status(const bool enabled);
	DECLSPEC_EXPORT int get_laser_power();
	DECLSPEC_EXPORT int get_laser_interval();
	DECLSPEC_EXPORT LaserIncidentAngles get_laser_angles();
	DECLSPEC_EXPORT NeuGPRMC get_gps_details();
	DECLSPEC_EXPORT NeuPosCor get_poscor_params();
	DECLSPEC_EXPORT NeuvLaserStatus get_laser_status();
	DECLSPEC_EXPORT int get_lidar_info_status();
	DECLSPEC_EXPORT int get_frame_frequency();
	DECLSPEC_EXPORT int get_frame_line_quantity();
	DECLSPEC_EXPORT int get_scan_mode();
	DECLSPEC_EXPORT int get_device_type();
	DECLSPEC_EXPORT int get_tof_one_length();
	DECLSPEC_EXPORT int get_tof_with_timestamp();
	DECLSPEC_EXPORT bool is_camera_on();
	DECLSPEC_EXPORT bool is_imu_on();
	DECLSPEC_EXPORT int set_data_save(const bool enabled);
	DECLSPEC_EXPORT int set_mjpg_curl(const bool enabled);
	DECLSPEC_EXPORT double get_hfov();
	DECLSPEC_EXPORT double get_vfov();

	DECLSPEC_EXPORT bool is_connected();
	DECLSPEC_EXPORT bool is_scanning();
	DECLSPEC_EXPORT bool is_streaming();
	DECLSPEC_EXPORT size_t get_lidar_frame_count();
	DECLSPEC_EXPORT size_t get_lidar_transferred_bytes();


	//// DEBUG
	DECLSPEC_EXPORT int set_evk_offset(uint8_t id, short offset);
	DECLSPEC_EXPORT int set_slm_mode(uint8_t slmvalue);
	DECLSPEC_EXPORT int laser_angles_clear();
	DECLSPEC_EXPORT int laser_angles_add(double angle);
	DECLSPEC_EXPORT void set_tlog_time(const nvid_t frameid, const int index, const int64_t microsec);
	DECLSPEC_EXPORT void set_tlog_size(const nvid_t frameid, const int index, const size_t cloudsize);
	DECLSPEC_EXPORT void reset_tlog(const nvid_t frameid);
	DECLSPEC_EXPORT void print_tlog(const nvid_t frameid);
	DECLSPEC_EXPORT int laser_status_update();
	DECLSPEC_EXPORT int set_debug_temp_data(enum neuv_cmd_code, int data);
	DECLSPEC_EXPORT int get_debug_temp_data(enum neuv_cmd_code);
	DECLSPEC_EXPORT int set_frank_filter_enabled(bool enabled);
	DECLSPEC_EXPORT int set_g_filter_enabled(bool enabled);
	DECLSPEC_EXPORT int set_g_gaus_fit_enabled(bool enabled);
	DECLSPEC_EXPORT int set_g_linear_fit_enabled(bool enabled);
	DECLSPEC_EXPORT int set_g_remove_ground_enabled(bool enabled);
	DECLSPEC_EXPORT int set_jason_filter_enabled(bool enabled);
	DECLSPEC_EXPORT int set_jason_tof_value(int value);
	DECLSPEC_EXPORT int set_jason_process_c_fusion(NeuvUnits& point, const cv::Mat &mjpgMat);

	DECLSPEC_EXPORT int jason_cameartopoint_pos(int x,int y,int & pixel_id,int & line_id,int & cloudpointx,int & cloudpointy,CAMERA_POINT_POS * temppos,int threadid);
	DECLSPEC_EXPORT int jason_cameartopointlist_pos(int x,int y,int & pixel_id,int & line_id,int & cloudpointx,int & cloudpointy,int threadid);
	DECLSPEC_EXPORT cv::Mat get_jason_camearmat(int threadid);
	DECLSPEC_EXPORT cv::Mat get_jasonlist_camearmat(int threadid);
	DECLSPEC_EXPORT void jason_camearinfo_clear(int threadid);
	DECLSPEC_EXPORT void jason_camearinfolist_clear(int threadid);
	DECLSPEC_EXPORT void jason_camearinfo_Add(int threadid);
	
	DECLSPEC_EXPORT	void jason_get_c_flip_axis(bool flip_x, bool flip_y);
	DECLSPEC_EXPORT void JasonSet_config_params(char * configstr);
	DECLSPEC_EXPORT	std::string JasonGet_config_params();

	DECLSPEC_EXPORT void jason_pcz_correct1(NeuvUnits* points);
	DECLSPEC_EXPORT void jason_pcz_correct2(NeuvUnits* points,int device_type);
	DECLSPEC_EXPORT void jason_pcz_correct_fun(NeuvUnits* points,int device_type);



	DECLSPEC_EXPORT void lidarHardware_json(const char *ipadress,char * datajson);
	DECLSPEC_EXPORT void lidarCalibrtions_json(const char *ipadress,char * datajson);
	DECLSPEC_EXPORT void lidarVerifications_json(const char *ipadress,char * datajson);

	DECLSPEC_EXPORT int set_log_output_enabled(bool enabled);
	DECLSPEC_EXPORT bool is_task_mode_on();
	DECLSPEC_EXPORT int set_npvt_value(int value);
	DECLSPEC_EXPORT int set_task_mode(bool enabled);
	DECLSPEC_EXPORT int get_scan_line_start();
	DECLSPEC_EXPORT int get_scan_line_end();
	DECLSPEC_EXPORT int get_scan_pixel_start();
	DECLSPEC_EXPORT int get_scan_pixel_end();
	DECLSPEC_EXPORT int set_tof_cache(const bool enabled, const std::vector<float> level);
	DECLSPEC_EXPORT int set_scan_line_pixel(int startLine, int endLine, int startPixel, int endPixel);
	DECLSPEC_EXPORT double get_matrix_data(unsigned int i0, unsigned int i1, unsigned int i2, unsigned int i3);

	DECLSPEC_EXPORT int set_c_pulse_tof_correct_enabled(bool enabled);
	DECLSPEC_EXPORT int set_c_reflectance_enabled(bool enabled);
	DECLSPEC_EXPORT int set_c_surface_collect_enabled(bool enabled);
	DECLSPEC_EXPORT int set_c_surface_correct_enabled(bool enabled);	
	DECLSPEC_EXPORT int set_c_mos_filter_enabled(bool enabled);
    DECLSPEC_EXPORT int set_c_cor_binary_options(uint8_t flag);

	DECLSPEC_EXPORT int set_camera_fps(int video_fps);
	DECLSPEC_EXPORT int set_pcz_path(const char* pczPath);
	DECLSPEC_EXPORT void get_pcz_path_id(char * pczId);
	DECLSPEC_EXPORT int set_c_filter_enabled(bool enabled);

	//wire 
	
	DECLSPEC_EXPORT int set_c_wire_distance_limit_low(int low);
	DECLSPEC_EXPORT int set_c_wire_distance_limit_high(int high);
	DECLSPEC_EXPORT int set_c_wire_detection_enabled(bool enabled);
    
	//container 
   
	DECLSPEC_EXPORT int set_c_container_detection_enabled(bool enabled);
    DECLSPEC_EXPORT int set_c_container_manual_enabled(bool enabled);
    DECLSPEC_EXPORT int set_c_container_dispresion_size(int size);
    DECLSPEC_EXPORT int set_c_container_dispresion_depth(int depth);
    DECLSPEC_EXPORT int set_c_container_manual_handle(int is_handle);


	//ship bridge
	
	DECLSPEC_EXPORT int set_c_ship_bridge_enabled(bool enabled);
	DECLSPEC_EXPORT int set_c_ship_bridge_h1(int h);
	DECLSPEC_EXPORT int set_c_ship_bridge_h2(int h);
	DECLSPEC_EXPORT int set_c_ship_bridge_h3(int h);
	DECLSPEC_EXPORT int set_c_ship_bridge_h4(int h);


    //salt pile
    DECLSPEC_EXPORT int set_c_salt_pile_params(int status, int pos_id);
#ifdef _DLL_EXPORT
#ifdef __cplusplus
	}
#endif 
#endif
}

#endif /* __NEUVDEFS_H__ */

