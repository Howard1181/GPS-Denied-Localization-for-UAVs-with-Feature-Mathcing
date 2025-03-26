// Client��
#pragma once
#pragma comment(lib, "Ws2_32.lib")      

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <Windows.h>
#include <string>
#include <cassert>
#include <opencv2/opencv.hpp>

// �h�����
#include <thread>
#include <chrono>


// �Ѷǰe�P�����̩w�q
#define PORT_NUM 6600
#define IP_ADDRES "127.0.0.1"

// �w�qICD
struct uavModel_sendMsg
{
	int shotImg_size; //����Ӥ����j�p
	std::vector<uchar> shotImg; //���ᤧ�Ӥ�
	bool cameraTrig; //�۾��֪�
	bool switch_flag; //��F�����I�P�_�A�٨S��
	bool actAL_flag; //�t��k�}�l����X��
	int k; //�j�鱱��
	double uavPsi; //�L�H���Y�V���A��
	double uavRoll; // �L�H���u�ਤ�A�סA�٨S��
	double uavVelocity; //�L�H���t�פj�p�Am/s
	double duration; //�⦸��Ӯɶ����j�A�@��
	double tk_droneData; //�L�H����T�����t�ήɶ� 
	double tk_img; //��ӷ�U���t�ήɶ�
	cv::Point2d uavPos; //�L�H����m
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


	int initSocket(); // ��l��
	int createSocket(); // �إ�socket�y�z
	int setAddress(); // �]�w��}��ơA�b����server�Pclient���۬��w�n
	int confirmConnect(); // �B�z�s�u�A�T�{�ݬO�_�s�u
	int recevieMessage(uavModel_receiveMsg); // �����T��
	
	int sendMessage(const uavModel_sendMsg&); // �ǰe�T��
	void closeSocket();

	bool isConnected = false;

};
