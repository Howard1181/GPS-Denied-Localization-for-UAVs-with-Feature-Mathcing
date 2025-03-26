/* 由本支程式call影像定位演算法(使用多執行緒或保持通訊) */

#include "UAV_model.h"
UAV_model_socket_ setSocket;
uavModel_sendMsg sendMsg_struct;
mutex mtx; //互斥鎖
condition_variable bufferCV; // 用於通知主執行緒


using namespace std::chrono;

// socketSender子執行緒
void socketSender()
{
    while (true)
    {
        {
            unique_lock<mutex> lock(mtx);
            bufferCV.wait(lock);
            //if (sendMsg_struct.cameraTrig == true)
            //{
                int sendState = setSocket.sendMessage(sendMsg_struct);
                //cout << "透過Socket傳送資料中...." << endl;
                if (sendState == -1)
                {
                    cout << "傳送狀態異常，關閉Socket。" << endl;
                    global_sendState = sendState;
                    setSocket.closeSocket();
                    break;
                }

            //    sendMsg_struct.cameraTrig = false;
            //}
        } 
        // 調整更新率
        this_thread::sleep_for(std::chrono::milliseconds(100)); // 暫停100毫秒，10hz
    }
}

int main()
{
    // 讀取空拍照片
    string folderPath_camera = "../photo";
    vector<string> imageFiles_camera;
    if (fs::exists(folderPath_camera) && fs::is_directory(folderPath_camera))
    {
        for (const auto& entry : fs::directory_iterator(folderPath_camera))
        {
            imageFiles_camera.push_back(entry.path().string());
        }
    }
    else
    {
        cerr << "資料夾不存在或不是一個有效的資料夾" << folderPath_camera << endl;
    }
    
    // 讀取空拍資料(flight psi)
    string PhoInfo_excelFile = "../log/FongYuanLog2.csv";
    ifstream file(PhoInfo_excelFile);
    string line;
    vector<vector<string>> data;
    if (!file.is_open())
    {
        cerr << "無法打開檔案" << PhoInfo_excelFile << endl;
    }
    // 逐行讀取csv
    while (getline(file, line))
    {
        stringstream ss(line);
        string cell;
        vector<string> row; // 橫的

        while (getline(ss, cell, ','))
        {
            row.push_back(cell);
        }

        data.push_back(row);
    }
    file.close();
    
    // 存取照片log資料
    for (int i = 1; i < data.size(); i++)
    {
        AerialP_Lon[i - 1] = stod(data[i][17]);
        AerialP_Lat[i - 1] = stod(data[i][16]);
        Flight_Yaw[i - 1] = stod(data[i][8]);
        Flight_Pitch[i - 1] = stod(data[i][9]);
        Flight_Roll[i - 1] = stod(data[i][10]); // 不確定編號對不對
        log_FirstColumn.push_back(data[i][0]);
        Aerial_pix0X[i - 1] = abs(AerialP_Lon[i - 1] - BL_coor[0]) / Lon_per_pix;
        Aerial_pix0Y[i - 1] = Map_Height - ((AerialP_Lat[i - 1] - BL_coor[1]) / Lat_per_pix);
    }

    // 初始化
    Flight_psi = atan2(Aerial_pix0Y[0] - 0, Aerial_pix0X[0] - 0);
    switch_flag = 0;
    j = 0;
    //double prev_timestamp_drone = -1;
    //double current_timestamp_drone = 0;
    //double dt = 0.0; // 兩次收到資料的時間差

    // 初始化傳送資料struct
    initSendStruct(sendMsg_struct);
    

    // Socket
    int socketInit_stat = setSocket.initSocket();
    int socketCreate_stat = setSocket.createSocket();
    int socketConnect_stat = setSocket.setAddress();

    // 啟動sendSocket子執行緒
    thread socketThread(socketSender); //建立子執行緒
    socketThread.detach(); //分離子執行緒

    // 無人機模型
    while (sendMsg_struct.k < 104)
    {
        uav_vx = uav_v * cos(Flight_psi);
        uav_vy = uav_v * sin(Flight_psi);

        uav_px = uav_px + uav_vx;
        uav_py = uav_py + uav_vy;
        cout << "x: " << uav_px << " y: " << uav_py << endl;

        //每次更新位置時，抓取"現在"的時間戳
        //auto now_droneDdata = std::chrono::system_clock::now();
        auto now_droneDdata = std::chrono::steady_clock::now();
        // 將自 1970/1/1 以來的時間轉為毫秒
        auto now_ms_droneData = std::chrono::duration_cast<std::chrono::milliseconds>(now_droneDdata.time_since_epoch()).count();

        // 需要一直傳的資料放這
        cv::Point2d uavPos(uav_px, uav_py);

        static time_point<system_clock> last_time; // 宣告快門間隔時間參數，用static宣告只會執行一次

        // 抵達拍攝點相當於在某個位置按下快門
        if ( pow(Aerial_pix0X[j] - uav_px, 2) + pow(Aerial_pix0Y[j] - uav_py, 2) <= 500 )
        {
            actAL_Flag = true;
            // 這裡要計算兩次按快門之間的時間並傳給演算法，用來預測無人機位置
            auto now_time = system_clock::now();
            if (j != 0 && last_time.time_since_epoch().count() > 0) // 確保 last_time 已初始化
            {
                duration_btShot = duration_cast<milliseconds>(now_time - last_time).count();
                cout << "兩次按快門間隔: " << duration_btShot << " 毫秒" << endl;
            }
            // 更新 last_time 為本次觸發的時間點
            last_time = now_time;

            cout << "抵達拍攝點: " << j + 1 << endl;
            switch_flag = 1;
        }

        if (switch_flag == 1)
        {
            if (j >= data.size() - 1) { break; }
            // 讀取img2(當作無人機空拍照片拍攝)
            cv::Mat img2 = cv::imread(imageFiles_camera[j]);

            // 每次要拍照時，抓取"現在"的時間戳，這邊讀取照片的動作當作拍照
            //auto now_img = std::chrono::system_clock::now();
            auto now_img = std::chrono::steady_clock::now();
            // 將自 1970/1/1 以來的時間轉為毫秒
            auto now_ms_img = std::chrono::duration_cast<std::chrono::milliseconds>(now_img.time_since_epoch()).count();

            printf("目前照片: %d.png\n", j + 1);
            // 試試照片壓縮成字節流格式再傳送
            vector<uchar> imgbuffer;
            cv::imencode(".jpg", img2, imgbuffer); // !!
            Flight_psi = atan2(Aerial_pix0Y[j + 1] - Aerial_pix0Y[j], Aerial_pix0X[j + 1] - Aerial_pix0X[j]);

            // 判斷socket狀態
            if ( global_sendState == -1 ) 
            {
                cout << "Socket狀態異常，跳出無人機模型迴圈。" << endl;
                break;
            }
            else
            {
                cout << "傳送訊息~~~~ " << endl;
            }

            // 將資料填入struct中
            {
                lock_guard<mutex> lock(mtx);
                sendMsg_struct.shotImg_size = imgbuffer.size();
                sendMsg_struct.shotImg = imgbuffer;
                sendMsg_struct.k = j;
                sendMsg_struct.uavPos = uavPos;
                sendMsg_struct.duration = duration_btShot;
                //sendMsg_struct.uavPsi = (Flight_psi * 180 / 3.1415926) + 90; //離北方之頭向角
                sendMsg_struct.uavPsi = Flight_psi; //離北方之頭向角
                sendMsg_struct.uavVelocity = uav_v;
                sendMsg_struct.tk_droneData = now_ms_droneData; //每次更新位置的時間點，自1970/1/1之後的毫秒
                sendMsg_struct.tk_img = now_ms_img; //每次拍照的時間點
                sendMsg_struct.actAL_flag = actAL_Flag;
                sendMsg_struct.cameraTrig = true;
                bufferCV.notify_one();
            }

            //sendMsg_struct.cameraTrig = false;
            switch_flag = 0;
            j++;
        }
        else // 若還沒到拍攝點，則將必要資訊儲存進struct，如果想要畫無人機實時動態的話
        {
            lock_guard<mutex> lock(mtx);
            sendMsg_struct.uavPos = uavPos;
            //sendMsg_struct.shotImg_size = 0; //拍攝照片之大小
            //sendMsg_struct.shotImg = { 0 }; //拍攝之照片
            //sendMsg_struct.k = 0; //迴圈控制
            //sendMsg_struct.uavPsi = (Flight_psi * 180 / 3.1415926) + 90; //無人機頭向角，度
            sendMsg_struct.uavPsi = Flight_psi; //無人機頭向角，度
            //sendMsg_struct.uavRoll = 0; // 無人機滾轉角，度
            sendMsg_struct.uavVelocity = uav_v; //無人機速度大小，m/s
            sendMsg_struct.tk_droneData = now_ms_droneData; //每次更新位置的時間點
            //sendMsg_struct.actAL_flag = actAL_Flag;
            //sendMsg_struct.duration = duration_btShot; // 兩次拍照時間間隔，毫秒
            sendMsg_struct.cameraTrig = false; //相機快門
            bufferCV.notify_one();
        }

        this_thread::sleep_for(std::chrono::milliseconds(100)); // 暫停100毫秒，10hz
        //this_thread::sleep_for(std::chrono::seconds(1));

    }
    setSocket.closeSocket();

    return 0;
}


