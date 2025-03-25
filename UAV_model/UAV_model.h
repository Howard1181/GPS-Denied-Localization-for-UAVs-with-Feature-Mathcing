#pragma once
#include <fstream>
#include <iostream>
#include <vector>
#include <filesystem>
#include <sstream>
#include <cmath>
#include <string>
#include "UAV_model_socket.h"

#define M_PI 3.1415926
namespace fs = std::filesystem;
using namespace std;

/* 礚诀把计/瓜把计 */
int t_end = 10000000;
double uav_v = 10;  // 讽琌m/s
double uav_vx = 0;
double uav_vy = 0;
double uav_px = 0;
double uav_py = 0;
double Flight_psi = 0;
double switch_flag;
double duration_btShot = 0;
int j;
int global_sendState = 0;  // socket篈办跑计
bool actAL_Flag = false;

double Map_Width = 4103; // 伦
double Map_Height = 3894;
double AerialP_Lon[104] = { 0 };
double AerialP_Lat[104] = { 0 };
double Aerial_pixX = 0;
double Aerial_pixY = 0;
double Flight_Yaw[104] = { 0 };
double Flight_Pitch[104] = { 0 };
double Flight_Roll[104] = { 0 };
double Aerial_pix0X[104];
double Aerial_pix0Y[104];
double TL_coor[2] = { 120.7158570, 24.2630837 };
double TR_coor[2] = { 120.7223298, 24.2630837 };
double BL_coor[2] = { 120.7158570, 24.2573723 };
double BR_coor[2] = { 120.7223298, 24.2573723 };
double Lon_per_pix = (TR_coor[0] - TL_coor[0]) / Map_Width;
double Lat_per_pix = (TL_coor[1] - BL_coor[1]) / Map_Height;
vector<string> log_FirstColumn;

// ﹍て肚癳socketsturct
void initSendStruct(uavModel_sendMsg& sendMsg_struct)
{
	sendMsg_struct.shotImg_size = 0;
	sendMsg_struct.cameraTrig = false;
	sendMsg_struct.switch_flag = false;
	sendMsg_struct.actAL_flag = false;
	sendMsg_struct.k = 0;
	sendMsg_struct.uavPsi = 0;
	sendMsg_struct.uavRoll = 0;
	sendMsg_struct.uavVelocity = 0;
	sendMsg_struct.duration = 0;
	sendMsg_struct.tk_droneData = 0;
	sendMsg_struct.tk_img = 0;
	sendMsg_struct.uavPos = { 0,0 };
}
