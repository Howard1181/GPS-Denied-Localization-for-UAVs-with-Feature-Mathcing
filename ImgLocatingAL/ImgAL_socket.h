// Server��
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
#include <condition_variable>
#include <queue>

//using namespace std; // -> �ϥ� std �|�� socket �禡�w�����ܦh�W�٩w�q�Ĭ�

// �Ѷǰe�P�����̩w�q
#define PORT_NUM 6600
#define IP_ADDRES "127.0.0.1"

// �w�qICD
struct ImgAL_sendMsg
{
	cv::Point2d estimatedPos;
};
struct ImgAL_receiveMsg
{
	int shotImg_size; //����Ӥ����j�p
	std::vector<uchar> shotImg; //���ᤧ�Ӥ�
	bool cameraTrig; //�۾��֪�
	bool switch_flag; //��F�����I�P�_
	bool actAL_flag; //�t��k�}�l����X��
	int k = 0; //�j�鱱��
	double uavPsi; //�L�H���Y�V���A��
	double uavRoll; //�L�H���u�ਤ�A��
	double uavVelocity; //�L�H���t�פj�p�Am/s
	double duration; //�⦸��Ӯɶ����j�A�@��
	double tk_droneData; //�L�H����T�����t�ήɶ� 
	double tk_img; //��ӷ�U���t�ήɶ�
	cv::Point2d uavPos; //�L�H����m
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

	int initSocket(); // ��l��
	int createSocket(); // �إ�socket�y�z
	int setAddress(); // �]�w��}���/�j�wport
	int listenSocket(); // ��ťsocket
	int confirmConnect(); // �B�z�s�u�A�T�{�Ȥ�ݬO�_�s�u
	ImgAL_receiveMsg receiveMessage(int*); // �����T��
	bool receiveAll(SOCKET, char*, int);  // recv offset�֥[����
	int* socketState = new int(0); //socket�s�b�P�_����

	void sendMessage(ImgAL_sendMsg); // �ǰe�T���A�Ǫ��O ImgAL_sendMsg �o�ӵ��c�����
	void closeSocket(); // ����socket
};



