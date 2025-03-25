#include "UAV_model_socket.h"

int UAV_model_socket_::initSocket()
{
    WORD version = MAKEWORD(2, 2); // 版本
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData); // 成功回傳 0，此函式可確保應用程式訪問不同Windows平台的底層網路功能

    if (iResult != 0)
    {
        printf("WSAStartup 失敗: %d\n", iResult);
        return -1;
    }
}

int UAV_model_socket_::createSocket()
{
    uavSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (uavSocket == INVALID_SOCKET)
    {
        printf("Socket建立失敗\n");
        return -1;
    }

}

int UAV_model_socket_::setAddress()
{
    memset(&serverAddrs, 0, sizeof(serverAddrs)); // 清空,將資料設為0
    serverAddrs.sin_addr.s_addr = inet_addr(IP_ADDRES); // 設定IP,此為本機端，此函式將UP轉為unsigned long
    serverAddrs.sin_family = AF_INET;  // IPv4
    serverAddrs.sin_port = htons(PORT_NUM);  // 處理網路位元順序與主機位元順序不同的函式
    if (connect(uavSocket, (SOCKADDR*)&serverAddrs, sizeof(serverAddrs)) == SOCKET_ERROR)  // 確定有連到
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
    struct sockaddr_in serverAddr; // client端位址資訊
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

// 傳送訊息(需檢查是送成功及接收成功)
int UAV_model_socket_::sendMessage(const uavModel_sendMsg& sendMsg) // 用參考當參數
{
    // 先將資料總大小傳過去
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
 
    // 相機快門狀態(先送相機快門，讓接收端決定要接收甚麼資料)
    // 像int這種型別的資料若不用&會等於一個數值，不是指標
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

    /* 在程式運行過程中都持續傳的資料 */
    // 無人機位置
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
    // 無人機頭向角
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
    // 無人機速度
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
    // 無人機狀態timestamp
    bytesSent = send(uavSocket, (const char*)&sendMsg.tk_droneData, sizeof(sendMsg.tk_droneData), 0);
    //std::cout << "狀態時間: " << sendMsg.tk_droneData << std::endl;
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

    /* 在 camera trigger = true 的時侯才傳的資料 */
    if (sendMsg.cameraTrig) 
    {
        // 無人機照片timestamp
        bytesSent = send(uavSocket, (const char*)&sendMsg.tk_img, sizeof(sendMsg.tk_img), 0);
        //std::cout << "狀態時間: " << sendMsg.tk_img << std::endl;
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
        // 兩次快門間隔
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
        // 拍攝照片之大小
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
        // 拍攝之照片
        // 對於像 vector 這樣的類型，直接傳遞變數本身並不是傳遞其底層數據的地址。需要訪問底層數據（如.data()）或顯式取地址。
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
        // 迴圈控制k
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
        // 通知執行演算法旗標
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

// 接收訊息
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
