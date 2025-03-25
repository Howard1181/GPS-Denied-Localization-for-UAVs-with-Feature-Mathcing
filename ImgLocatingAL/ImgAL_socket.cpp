#include "ImgAL_socket.h" 

// ��l��socket
int ImgAL_socket_::initSocket()
{
    // ��l��
    WORD version = MAKEWORD(2, 2); // ����
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData); // ���\�^��0�A���禡�i�T�O���ε{���X�ݤ��PWindows���x�����h�����\��

    if (iResult != 0)
    {
        printf("WSAStartup ����: %d\n", iResult);
        return -1;
    }
}

// �إ�socket�y�z
int ImgAL_socket_::createSocket()
{
    mysocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (mysocket == INVALID_SOCKET)
    {
        printf("Socket�إߥ���\n");
        return -1;
    }
    return 0;
}

// �]�w��}���/�j�wport
int ImgAL_socket_::setAddress()
{
    memset(&myAddrs, 0, sizeof(myAddrs)); // �M��,�N��Ƴ]��0
    myAddrs.sin_addr.s_addr = inet_addr(IP_ADDRES); // �]�wIP,���������ݡA���禡�NUP�ରunsigned long
    myAddrs.sin_family = AF_INET;  // IPv4
    myAddrs.sin_port = htons(PORT_NUM);  // �B�z�����줸���ǻP�D���줸���Ǥ��P���禡

    // �o�̥[ :: ����]�O�]�� namespace std ���]���@�Ө禡�s�� bind�A��::�����w�����ܼ�
    int r = bind(mysocket, (SOCKADDR*)&myAddrs , sizeof(myAddrs));  // (SOCKADDR*)���j���૬; �Naddrs�Pmysocket�j�b�@�_?
    if (r == SOCKET_ERROR)
    {
        std::cout << "Bind Failed: " << WSAGetLastError() << "\n" << std::endl;
        closesocket(mysocket);
        WSACleanup();
        return -1;
    }
    
    return 0;
}

// ��ťsocket
int ImgAL_socket_::listenSocket()
{
    int l = listen(mysocket, SOMAXCONN); // backlog���̤j��ť�h�ֳs�u�ASOMAXCONN ��ܨt�γ̤j��
    //assert(l != SOCKET_ERROR);
    if (l == SOCKET_ERROR)
    {
        std::cout << "Listen Failed: " << WSAGetLastError() << "\n" << std::endl;
        closesocket(mysocket);
        WSACleanup();
        return -1;
    }
    else
    {
        std::cout << "Server is listening port " << PORT_NUM << "\n" << std::endl;
        return 0;
    }
}

// �B�z�s�u�A�T�{�Ȥ�ݬO�_�s�u
int ImgAL_socket_::confirmConnect()
{
    struct sockaddr_in clientAddr; // client�ݦ�}��T
    int clientAddrLen = sizeof(clientAddr);
    sConnect = accept(mysocket, (SOCKADDR*)&clientAddr, &clientAddrLen);
    if (sConnect != INVALID_SOCKET)
    {
        printf("Server got connection from %s \n", inet_ntoa(clientAddr.sin_addr));
        return 0;
    }
    else
    {
        std::cout << "Accept Failed: " << WSAGetLastError() << "\n" << std::endl;
        closesocket(mysocket);
        WSACleanup();
        return -1;
    }
}

// �ǰe�T��
void ImgAL_socket_::sendMessage(ImgAL_sendMsg sendMsg)
{
    send(sConnect, (const char*)&sendMsg, sizeof(sendMsg), 0);
}

// �����T���AReturn ImgAL_receiveMsg �o�ӵ��c���ܼ�
ImgAL_receiveMsg ImgAL_socket_::receiveMessage(int* socketState)
{
    //bytesReceived = recv(sConnect, (char*)&totalSent_Size, sizeof(totalSent_Size), 0);
    //if (bytesReceived <= 0)
    //{
    //    // �o���ٻݧP�_����Aclose socket/break
    //    std::cout << "Failed to receive data total length. " << WSAGetLastError() << std::endl;
    //    *socketState = -1;
    //}
    ImgAL_receiveMsg receivedMsg_socket;
    //��l�Ʊ����ϰ�A�o�����totalSent_Size�ӷ�@�j�p�ոլ�
    //ZeroMemory(&receivedMsg_socket, sizeof(receivedMsg_socket)); // �ϥ�sizeof(ptr)��^���O���Ъ��j�p�A�O8bytes�A�o�ر��p����sizeof(*ptr)
    receivedMsg_socket.shotImg_size = 0;
    receivedMsg_socket.cameraTrig = false;
    //receivedMsg_socket.switch_flag = false;
    receivedMsg_socket.actAL_flag = false;
    receivedMsg_socket.k = 0;
    receivedMsg_socket.uavPsi = 0.0;
    //receivedMsg_socket.uavRoll = 0.0;
    receivedMsg_socket.uavVelocity = 0.0;
    receivedMsg_socket.duration = 0.0;
    receivedMsg_socket.tk_droneData = 0.0;
    receivedMsg_socket.tk_img = 0.0;
    receivedMsg_socket.uavPos = { 0.0, 0.0 };
    // �ʺA���������ե� clear
    receivedMsg_socket.shotImg.clear();

    // 4. �����۾��֪����A(�o��令�������֪����A�A�P�_�U���n�����Ǹ��)
    if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.cameraTrig), sizeof(receivedMsg_socket.cameraTrig)))
    {
        //std::cout << "receive cam trig. " << std::endl;
    }
    else
    {
        std::cout << "Failed to receive camera trigger. " << WSAGetLastError() << std::endl;
        *socketState = -1;
        return receivedMsg_socket;
    }

    /* �{���B�椤������n������� */
    // 6. �����L�H����m
    if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.uavPos), sizeof(receivedMsg_socket.uavPos)))
    {
        //std::cout << "receive uavPos. " << std::endl;
    }
    else
    {
        std::cout << "Failed to receive uav position failed. " << WSAGetLastError() << std::endl;
        *socketState = -1;
        return receivedMsg_socket;
    }

    // 7. �����L�H���Y�V��
    if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.uavPsi), sizeof(receivedMsg_socket.uavPsi)))
    {
        //std::cout << "receive uavPsi. " << std::endl;
    }
    else
    {
        std::cout << "Failed to receive uav psi. " << WSAGetLastError() << std::endl;
        *socketState = -1;
        return receivedMsg_socket;
    }

    // 8. �����L�H���t��
    if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.uavVelocity), sizeof(receivedMsg_socket.uavVelocity)))
    {
        //std::cout << "receive uav velocity. " << std::endl;
    }
    else
    {
        std::cout << "Failed to receive uav velocity. " << WSAGetLastError() << std::endl;
        *socketState = -1;
        return receivedMsg_socket;
    }
    // 9. �����L�H�����Atimestamp
    if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.tk_droneData), sizeof(receivedMsg_socket.tk_droneData)))
    {
        //std::cout << "receive drone state timestamp: " << receivedMsg_socket.tk_droneData << std::endl;
    }
    else
    {
        std::cout << "Failed to receive drone state timestamp. " << WSAGetLastError() << std::endl;
        *socketState = -1;
        return receivedMsg_socket;
    }

    /* �ھ� camera trigger �ӨM�w�O�_����*/
    if (receivedMsg_socket.cameraTrig)
    {
        // 10. �����ũ�Ӥ�timestamp
        if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.tk_img), sizeof(receivedMsg_socket.tk_img)))
        {
            //std::cout << "receive img timestamp: " << receivedMsg_socket.tk_img << std::endl;
        }
        else
        {
            std::cout << "Failed to receive img timestamp. " << WSAGetLastError() << std::endl;
            *socketState = -1;
            return receivedMsg_socket;
        }
        // 1. �����⦸�֪����j
        if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.duration), sizeof(receivedMsg_socket.duration)))
        {
            //std::cout << "receive duration. " << std::endl;
        }
        else
        {
            std::cout << "Failed to receive time duration between two shot. " << WSAGetLastError() << std::endl;
            *socketState = -1;
            return receivedMsg_socket;
        }

        // 2. ��������Ӥ����j�p
        if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.shotImg_size), sizeof(receivedMsg_socket.shotImg_size)))
        {
            //std::cout << "receive img size. " << std::endl;
        }
        else
        {
            std::cout << "Failed to receive Img data size. " << WSAGetLastError() << std::endl;
            *socketState = -1;
            return receivedMsg_socket;
        }

        // 3. �������ᤧ�Ӥ�
        // ����Ӥ��j�p����A��l�ƪŦV�qshotImg
        receivedMsg_socket.shotImg.resize(receivedMsg_socket.shotImg_size);
        if (receiveAll(sConnect, reinterpret_cast<char*>(receivedMsg_socket.shotImg.data()), receivedMsg_socket.shotImg_size))
        {
            //std::cout << "receive img. " << std::endl;
        }
        else
        {
            std::cout << "Failed to receive Img data. " << WSAGetLastError() << std::endl;
            *socketState = -1;
            return receivedMsg_socket;
        }

        // 5. �����j�鱱��k
        if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.k), sizeof(receivedMsg_socket.k)))
        {
            //std::cout << "receive k. " << std::endl;
        }
        else
        {
            std::cout << "Failed to receive loop control 'k'. " << WSAGetLastError() << std::endl;
            *socketState = -1;
            return receivedMsg_socket;
        }

        // 9. �����t��k�}�l����X��
        if (receiveAll(sConnect, reinterpret_cast<char*>(&receivedMsg_socket.actAL_flag), sizeof(receivedMsg_socket.actAL_flag)))
        {
            //std::cout << "receive actAL_flag. " << std::endl;
        }
        else
        {
            std::cout << "Failed to receive algorithm activation signal. " << WSAGetLastError() << std::endl;
            *socketState = -1;
            return receivedMsg_socket;
        }
    }
    
    return receivedMsg_socket;
}

// recv offset�֥[����
bool ImgAL_socket_::receiveAll(SOCKET sock, char* buffer, int totalSize)
{
    int bytesReadTotal = 0;
    while (bytesReadTotal < totalSize)
    {
        // �ݳo���ٻݭn�h��byte
        int byteToRead = totalSize - bytesReadTotal;

        // �q buffer + bytesReadTotal�A�}�lŪ byteToRead �j�p
        int ret = recv(sock, buffer + bytesReadTotal, byteToRead, 0);
        if (ret <= 0)
        {
            return false;
        }

        bytesReadTotal += ret;
    }
    return true;
}

// ����socket
void ImgAL_socket_::closeSocket()
{
    closesocket(sConnect);
    closesocket(mysocket);
    WSACleanup();
}


