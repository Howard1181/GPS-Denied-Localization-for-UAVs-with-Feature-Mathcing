#include "UAV_model_socket.h"

int UAV_model_socket_::initSocket()
{
    WORD version = MAKEWORD(2, 2); // ����
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData); // ���\�^�� 0�A���禡�i�T�O���ε{���X�ݤ��PWindows���x�����h�����\��

    if (iResult != 0)
    {
        printf("WSAStartup ����: %d\n", iResult);
        return -1;
    }
}

int UAV_model_socket_::createSocket()
{
    uavSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (uavSocket == INVALID_SOCKET)
    {
        printf("Socket�إߥ���\n");
        return -1;
    }

}

int UAV_model_socket_::setAddress()
{
    memset(&serverAddrs, 0, sizeof(serverAddrs)); // �M��,�N��Ƴ]��0
    serverAddrs.sin_addr.s_addr = inet_addr(IP_ADDRES); // �]�wIP,���������ݡA���禡�NUP�ରunsigned long
    serverAddrs.sin_family = AF_INET;  // IPv4
    serverAddrs.sin_port = htons(PORT_NUM);  // �B�z�����줸���ǻP�D���줸���Ǥ��P���禡
    if (connect(uavSocket, (SOCKADDR*)&serverAddrs, sizeof(serverAddrs)) == SOCKET_ERROR)  // �T�w���s��
    {
        std::cout << "Connection failed: " << WSAGetLastError() << std::endl;
        closesocket(uavSocket);
        WSACleanup();
        return -1;
    }
    std::cout << "Connected to server! " << std::endl;
    return 0;
}

int UAV_model_socket_::confirmConnect()
{
    struct sockaddr_in serverAddr; // client�ݦ�}��T
    int serverAddrLen = sizeof(serverAddr);
    sConnect = accept(uavSocket, (SOCKADDR*)&serverAddr, &serverAddrLen);
    if (sConnect != INVALID_SOCKET)
    {
        printf("Client got connection from %s \n", inet_ntoa(serverAddr.sin_addr));
        return 0;
    }
    else
    {
        std::cout << "Accept Failed: " << WSAGetLastError() << "\n" << std::endl;
        closesocket(uavSocket);
        WSACleanup();
        return -1;
    }
}

// �ǰe�T��(���ˬd�O�e���\�α������\)
int UAV_model_socket_::sendMessage(const uavModel_sendMsg& sendMsg) // �ΰѦҷ�Ѽ�
{
    // ���N����`�j�p�ǹL�h
    int totalSent_Size = sizeof(sendMsg.shotImg_size) + sendMsg.shotImg_size + sizeof(sendMsg.cameraTrig) + sizeof(sendMsg.actAL_flag)
                    + sizeof(sendMsg.k) + sizeof(sendMsg.uavPsi) + sizeof(sendMsg.uavVelocity) + sizeof(sendMsg.duration)
                    + sizeof(sendMsg.uavPos);
    //int bytesSent = send(uavSocket, (char*)&totalSent_Size, sizeof(totalSent_Size), 0);
    //if (bytesSent == SOCKET_ERROR)
    //{
    //    int error = WSAGetLastError();
    //    if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
    //        std::cout << "Connection lost: " << error << std::endl;
    //        /*isConnected = false;*/
    //    }
    //    else {
    //        std::cout << "Send failed: " << error << std::endl;
    //    }
    //    return -1;
    //}
 
    // �۾��֪����A(���e�۾��֪��A�������ݨM�w�n�����ƻ���)
    // ��int�o�ث��O����ƭY����&�|����@�ӼƭȡA���O����
    int bytesSent = send(uavSocket, (const char*)&sendMsg.cameraTrig, sizeof(sendMsg.cameraTrig), 0);
    if (bytesSent == SOCKET_ERROR)
    {
        int error = WSAGetLastError();
        if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
            std::cout << "Connection lost: " << error << std::endl;
            /*isConnected = false;*/
        }
        else {
            std::cout << "Send failed: " << error << std::endl;
        }
        return -1;
    }

    /* �b�{���B��L�{��������Ǫ���� */
    // �L�H����m
    bytesSent = send(uavSocket, (const char*)&sendMsg.uavPos, sizeof(sendMsg.uavPos), 0);
    if (bytesSent == SOCKET_ERROR)
    {
        int error = WSAGetLastError();
        if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
            std::cout << "Connection lost: " << error << std::endl;
            /*isConnected = false;*/
        }
        else {
            std::cout << "Send failed: " << error << std::endl;
        }
        return -1;
    }
    // �L�H���Y�V��
    bytesSent = send(uavSocket, (const char*)&sendMsg.uavPsi, sizeof(sendMsg.uavPsi), 0);
    if (bytesSent == SOCKET_ERROR)
    {
        int error = WSAGetLastError();
        if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
            std::cout << "Connection lost: " << error << std::endl;
            /*isConnected = false;*/
        }
        else {
            std::cout << "Send failed: " << error << std::endl;
        }
        return -1;
    }
    // �L�H���t��
    bytesSent = send(uavSocket, (const char*)&sendMsg.uavVelocity, sizeof(sendMsg.uavVelocity), 0);
    if (bytesSent == SOCKET_ERROR)
    {
        int error = WSAGetLastError();
        if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
            std::cout << "Connection lost: " << error << std::endl;
            /*isConnected = false;*/
        }
        else {
            std::cout << "Send failed: " << error << std::endl;
        }
        return -1;
    }
    // �L�H�����Atimestamp
    bytesSent = send(uavSocket, (const char*)&sendMsg.tk_droneData, sizeof(sendMsg.tk_droneData), 0);
    //std::cout << "���A�ɶ�: " << sendMsg.tk_droneData << std::endl;
    if (bytesSent == SOCKET_ERROR)
    {
        int error = WSAGetLastError();
        if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
            std::cout << "Connection lost: " << error << std::endl;
            /*isConnected = false;*/
        }
        else {
            std::cout << "Send failed: " << error << std::endl;
        }
        return -1;
    }

    /* �b camera trigger = true ���ɫJ�~�Ǫ���� */
    if (sendMsg.cameraTrig) 
    {
        // �L�H���Ӥ�timestamp
        bytesSent = send(uavSocket, (const char*)&sendMsg.tk_img, sizeof(sendMsg.tk_img), 0);
        //std::cout << "���A�ɶ�: " << sendMsg.tk_img << std::endl;
        if (bytesSent == SOCKET_ERROR)
        {
            int error = WSAGetLastError();
            if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
                std::cout << "Connection lost: " << error << std::endl;
                /*isConnected = false;*/
            }
            else {
                std::cout << "Send failed: " << error << std::endl;
            }
            return -1;
        }
        // �⦸�֪����j
        bytesSent = send(uavSocket, (char*)&sendMsg.duration, sizeof(sendMsg.duration), 0);
        if (bytesSent == SOCKET_ERROR)
        {
            int error = WSAGetLastError();
            if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
                std::cout << "Connection lost: " << error << std::endl;
                /*isConnected = false;*/
            }
            else {
                std::cout << "Send failed: " << error << std::endl;
            }
            return -1;
        }
        // ����Ӥ����j�p
        bytesSent = send(uavSocket, (char*)&sendMsg.shotImg_size, sizeof(sendMsg.shotImg_size), 0);
        if (bytesSent == SOCKET_ERROR)
        {
            int error = WSAGetLastError();
            if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
                std::cout << "Connection lost: " << error << std::endl;
                /*isConnected = false;*/
            }
            else {
                std::cout << "Send failed: " << error << std::endl;
            }
            return -1;
        }
        // ���ᤧ�Ӥ�
        // ��� vector �o�˪������A�����ǻ��ܼƥ����ä��O�ǻ��䩳�h�ƾڪ��a�}�C�ݭn�X�ݩ��h�ƾڡ]�p.data()�^���㦡���a�}�C
        bytesSent = send(uavSocket, (char*)sendMsg.shotImg.data(), sendMsg.shotImg_size, 0);
        if (bytesSent == SOCKET_ERROR)
        {
            int error = WSAGetLastError();
            if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
                std::cout << "Connection lost: " << error << std::endl;
                /*isConnected = false;*/
            }
            else {
                std::cout << "Send failed: " << error << std::endl;
            }
            return -1;
        }
        // �j�鱱��k
        bytesSent = send(uavSocket, (const char*)&sendMsg.k, sizeof(sendMsg.k), 0);
        if (bytesSent == SOCKET_ERROR)
        {
            int error = WSAGetLastError();
            if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
                std::cout << "Connection lost: " << error << std::endl;
                /*isConnected = false;*/
            }
            else {
                std::cout << "Send failed: " << error << std::endl;
            }
            return -1;
        }
        // �q������t��k�X��
        bytesSent = send(uavSocket, (const char*)&sendMsg.actAL_flag, sizeof(sendMsg.actAL_flag), 0);
        if (bytesSent == SOCKET_ERROR)
        {
            int error = WSAGetLastError();
            if (error == WSAECONNRESET || error == WSAETIMEDOUT) {
                std::cout << "Connection lost: " << error << std::endl;
                /*isConnected = false;*/
            }
            else {
                std::cout << "Send failed: " << error << std::endl;
            }
            return -1;
        }
    }
    return 0;
}

// �����T��
int UAV_model_socket_::recevieMessage(uavModel_receiveMsg receiveMsg)
{
    ZeroMemory(&receiveMsg, sizeof(receiveMsg));
    int bytesReceived = recv(uavSocket, (char*)&receiveMsg, sizeof(&receiveMsg), 0);
    if (bytesReceived > 0) {
        std::cout << "Received from server: " << (char*)&receiveMsg << std::endl;
        return 0;
    }
    else {
        std::cout << "Received failed: " << WSAGetLastError() << std::endl;
        return -1;
    }
}

void UAV_model_socket_::closeSocket()
{
    closesocket(uavSocket);
    WSACleanup();
}
