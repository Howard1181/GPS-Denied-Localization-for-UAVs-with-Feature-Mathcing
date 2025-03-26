// Client端
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


// 由傳送與接收者定義
#define PORT_NUM 6600
#define IP_ADDRES "127.0.0.1"

// 定義ICD
struct uavModel_sendMsg
{
	int shotImg_size; //拍攝照片之大小
	std::vector<uchar> shotImg; //拍攝之照片
	bool cameraTrig; //相機快門
	bool switch_flag; //抵達拍攝點判斷，還沒傳
	bool actAL_flag; //演算法開始執行旗標
	int k; //迴圈控制
	double uavPsi; //無人機頭向角，度
	double uavRoll; // 無人機滾轉角，度，還沒傳
	double uavVelocity; //無人機速度大小，m/s
	double duration; //兩次拍照時間間隔，毫秒
	double tk_droneData; //無人機資訊對應系統時間 
	double tk_img; //拍照當下的系統時間
	cv::Point2d uavPos; //無人機位置
};
struct uavModel_receiveMsg
{
	cv::Point2d estimatedPos;
};

class UAV_model_socket_
{
private:
	WSAData wsaData;

public:
	SOCKET uavSocket;
	SOCKET sConnect;
	SOCKADDR_IN myAddrs;
	SOCKADDR_IN serverAddrs;


	int initSocket(); // 初始化
	int createSocket(); // 建立socket描述
	int setAddress(); // 設定位址資料，在此需server與client互相約定好
	int confirmConnect(); // 處理連線，確認端是否連線
	int recevieMessage(uavModel_receiveMsg); // 接收訊息
	
	int sendMessage(const uavModel_sendMsg&); // 傳送訊息
	void closeSocket();

	bool isConnected = false;

};
