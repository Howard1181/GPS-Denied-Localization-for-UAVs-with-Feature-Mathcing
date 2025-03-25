// Server端
#pragma once
#pragma comment(lib, "Ws2_32.lib")    

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <Windows.h>
#include <string>
#include <cassert>
#include <opencv2/opencv.hpp>

// 多執行緒
#include <thread>
#include <chrono>
#include <condition_variable>
#include <queue>

//using namespace std; // -> 使用 std 會跟 socket 函式庫中的很多名稱定義衝突

// 由傳送與接收者定義
#define PORT_NUM 6600
#define IP_ADDRES "127.0.0.1"

// 定義ICD
struct ImgAL_sendMsg
{
	cv::Point2d estimatedPos;
};
struct ImgAL_receiveMsg
{
	int shotImg_size; //拍攝照片之大小
	std::vector<uchar> shotImg; //拍攝之照片
	bool cameraTrig; //相機快門
	bool switch_flag; //抵達拍攝點判斷
	bool actAL_flag; //演算法開始執行旗標
	int k = 0; //迴圈控制
	double uavPsi; //無人機頭向角，度
	double uavRoll; //無人機滾轉角，度
	double uavVelocity; //無人機速度大小，m/s
	double duration; //兩次拍照時間間隔，毫秒
	double tk_droneData; //無人機資訊對應系統時間 
	double tk_img; //拍照當下的系統時間
	cv::Point2d uavPos; //無人機位置
};
struct ImgAL_droneState
{
	double t_;
	double x, y;
	double phi, theta, psi;
	double uav_v;
};

class ImgAL_socket_
{
private:
	WSAData wsaData;

public:
	SOCKET mysocket;
	SOCKET sConnect;
	SOCKADDR_IN myAddrs;
	SOCKADDR_IN clientAddrs;

	int initSocket(); // 初始化
	int createSocket(); // 建立socket描述
	int setAddress(); // 設定位址資料/綁定port
	int listenSocket(); // 監聽socket
	int confirmConnect(); // 處理連線，確認客戶端是否連線
	ImgAL_receiveMsg receiveMessage(int*); // 接收訊息
	bool receiveAll(SOCKET, char*, int);  // recv offset累加機制
	int* socketState = new int(0); //socket存在判斷機制

	void sendMessage(ImgAL_sendMsg); // 傳送訊息，傳的是 ImgAL_sendMsg 這個結構的資料
	void closeSocket(); // 關閉socket
};



