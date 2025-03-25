#include "ImgAL_socket.h" 

// 初始化socket
int ImgAL_socket_::initSocket()
{
    // 初始化
    WORD version = MAKEWORD(2, 2); // 版本
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData); // 成功回傳0，此函式可確保應用程式訪問不同Windows平台的底層網路功能

    if (iResult != 0)
    {
        printf("WSAStartup 失敗: %d\n", iResult);
        return -1;
    }
}

// 建立socket描述
int ImgAL_socket_::createSocket()
{
    mysocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (mysocket == INVALID_SOCKET)
    {
        printf("Socket建立失敗\n");
        return -1;
    }
    return 0;
}

// 設定位址資料/綁定port
int ImgAL_socket_::setAddress()
{
    memset(&myAddrs, 0, sizeof(myAddrs)); // 清空,將資料設為0
    myAddrs.sin_addr.s_addr = inet_addr(IP_ADDRES); // 設定IP,此為本機端，此函式將UP轉為unsigned long
    myAddrs.sin_family = AF_INET;  // IPv4
    myAddrs.sin_port = htons(PORT_NUM);  // 處理網路位元順序與主機位元順序不同的函式

    // 這裡加 :: 的原因是因為 namespace std 中也有一個函式叫做 bind，而::為指定全局變數
    int r = bind(mysocket, (SOCKADDR*)&myAddrs , sizeof(myAddrs));  // (SOCKADDR*)為強制轉型; 將addrs與mysocket綁在一起?
    if (r == SOCKET_ERROR)
    {
        std::cout << "Bind Failed: " << WSAGetLastError() << "\n" << std::endl;
        closesocket(mysocket);
        WSACleanup();
        return -1;
    }
    
    return 0;
}

// 監聽socket
int ImgAL_socket_::listenSocket()
{
    int l = listen(mysocket, SOMAXCONN); // backlog為最大監聽多少連線，SOMAXCONN 表示系統最大值
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

// 處理連線，確認客戶端是否連線
int ImgAL_socket_::confirmConnect()
{
    struct sockaddr_in clientAddr; // client端位址資訊
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

// 傳送訊息
void ImgAL_socket_::sendMessage(ImgAL_sendMsg sendMsg)
{
    send(sConnect, (const char*)&sendMsg, sizeof(sendMsg), 0);
}

// 接收訊息，Return ImgAL_receiveMsg 這個結構的變數
ImgAL_receiveMsg ImgAL_socket_::receiveMessage(int* socketState)
{
    //bytesReceived = recv(sConnect, (char*)&totalSent_Size, sizeof(totalSent_Size), 0);
    //if (bytesReceived <= 0)
    //{
    //    // 這裡還需判斷機制，close socket/break
    //    std::cout << "Failed to receive data total length. " << WSAGetLastError() << std::endl;
    //    *socketState = -1;
    //}
    ImgAL_receiveMsg receivedMsg_socket;
    //初始化接收區域，這邊先用totalSent_Size來當作大小試試看
    //ZeroMemory(&receivedMsg_socket, sizeof(receivedMsg_socket)); // 使用sizeof(ptr)返回的是指標的大小，是8bytes，這種情況應用sizeof(*ptr)
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
    // 動態成員直接調用 clear
    receivedMsg_socket.shotImg.clear();

    // 4. 接收相機快門狀態(這邊改成先接收快門狀態，判斷下面要收哪些資料)
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

    /* 程式運行中都持續要收的資料 */
    // 6. 接收無人機位置
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

    // 7. 接收無人機頭向角
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

    // 8. 接收無人機速度
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
    // 9. 接收無人機狀態timestamp
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

    /* 根據 camera trigger 來決定是否接收*/
    if (receivedMsg_socket.cameraTrig)
    {
        // 10. 接收空拍照片timestamp
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
        // 1. 接收兩次快門間隔
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

        // 2. 接收拍攝照片之大小
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

        // 3. 接收拍攝之照片
        // 接到照片大小之後，初始化空向量shotImg
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

        // 5. 接收迴圈控制k
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

        // 9. 接收演算法開始執行旗標
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

// recv offset累加機制
bool ImgAL_socket_::receiveAll(SOCKET sock, char* buffer, int totalSize)
{
    int bytesReadTotal = 0;
    while (bytesReadTotal < totalSize)
    {
        // 看這次還需要多少byte
        int byteToRead = totalSize - bytesReadTotal;

        // 從 buffer + bytesReadTotal再開始讀 byteToRead 大小
        int ret = recv(sock, buffer + bytesReadTotal, byteToRead, 0);
        if (ret <= 0)
        {
            return false;
        }

        bytesReadTotal += ret;
    }
    return true;
}

// 關閉socket
void ImgAL_socket_::closeSocket()
{
    closesocket(sConnect);
    closesocket(mysocket);
    WSACleanup();
}


