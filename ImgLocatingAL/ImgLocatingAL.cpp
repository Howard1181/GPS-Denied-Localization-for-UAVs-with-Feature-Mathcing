
#include "Header.h"
ImgLocate IL;
ImgAL_socket_ setSocket;
ImgAL_receiveMsg receivedMsg_receiver, receivedMsg_main;  // 一個將Receiver socket收到的塞進buffer，一個用來當作主程式儲存buffer的值
mutex mtx; //互斥鎖
bool activate_alg_flag = false;
bool bias_corret = false;

// 預測位置子執行緒，這裡應該用實時收到的receivedMsg_receiver
void posPredicter()
{
    cout << "無人機預測位置子執行緒待命中 ----- " << endl;
    double v = 0, psi = 0, prdcx = 0, prdcy = 0, prdcx0 = 0, prdcy0 = 0, timestamp_drone = 0;
    cv::Point2d currentActual, currentPredicted;
    static double corrected_v = 0;
    static double corrected_psi = 0;
    static bool firstCorrectionDone = false;
    int i = 0;
    double prev_timestamp_drone = -1.0;
    double dt = 0.0; // 兩次收到資料的時間差 (秒)
    ImgAL_droneState drone_state;

    cv::namedWindow("Map", cv::WINDOW_NORMAL); // 可自己調整大小的視窗

    while (true) 
    { 
        // 判斷socket狀態
        if (*setSocket.socketState == -1)
        {
            cout << "Socket連線狀態異常，預測位置子執行緒退出" << endl;
            break;
        }
        // 先獲取失去GPS後第一點在圖上的Pixel
        // 肯定是要從socket取得無人機現在資料

        if (activate_alg_flag == true) // 當演算法執行旗標 = true (抵達失去GPS的那個點才觸發預測機制)
        {
            {
                unique_lock<mutex> lock(mtx);
                IL.bufferCV.wait(lock); // 這裡不wait就會出錯
                if (prdcx == 0 && prdcy == 0)  // 確保只初始化一次，先儲存失去GPS的第一點
                {
                    prdcx = receivedMsg_receiver.uavPos.x;
                    prdcy = receivedMsg_receiver.uavPos.y;
                    psi = receivedMsg_receiver.uavPsi;
                    v = receivedMsg_receiver.uavVelocity;
                    prev_timestamp_drone = receivedMsg_receiver.tk_droneData; //毫秒
                }
                else
                {
                    // 誤差回溯修正，修改成演算法計算的位置
                    if (bias_corret == true) // 理論上是要從第二點拍照點開始回溯
                    {
                        prdcx = IL.EstimatedP_pixel.x;
                        prdcy = IL.EstimatedP_pixel.y;

                        // 將照片 timestamp 去 predicterBuffer 中做尋找，找對應的idx
                        int throwbackIdx = IL.getPredictBuffer_idx(IL.img_timestamp);
                        i = 1;

                        // 刪除throwbackIdx前predicterBuffer的值
                        IL.predicterBuffer.erase(IL.predicterBuffer.begin(), IL.predicterBuffer.begin() + throwbackIdx);

                        bias_corret = false;
                        firstCorrectionDone = true;
                    }

                    // 在第一次回溯後，使用保存的 corrected_v 與 corrected_psi，預測位置
                    if (firstCorrectionDone)
                    {
                        dt = (IL.predicterBuffer[i].t_ - IL.predicterBuffer[i-1].t_);
                        if (dt > 300) { dt = 300; }
                        corrected_v = IL.predicterBuffer[i].uav_v;
                        corrected_psi = IL.predicterBuffer[i].psi;
                        cout << "dt-" << dt << endl;
                        // 持續儲存無人機狀態真實值進buffer，必須要知道在某時刻正確的資訊，才可以做回溯
                        v = receivedMsg_receiver.uavVelocity;
                        //psi = (receivedMsg_receiver.uavPsi - 90) * 3.1415926 / 180; // rad
                        psi = receivedMsg_receiver.uavPsi; // rad


                        // 根據預測位置buffer內容預測位置
                        prdcx = prdcx + corrected_v * cos(corrected_psi) * dt / 100; // 模型理論上是100毫秒更新一次，這邊看是100毫秒的多少倍
                        prdcy = prdcy + corrected_v * sin(corrected_psi) * dt / 100;

                        // 要重新累積buffer中計算的位置，不然選照片會選到錯的
                        IL.predicterBuffer[i].x = prdcx;
                        IL.predicterBuffer[i].y = prdcy;

                        //prev_timestamp_drone = current_timestamp;
                        i++;
                    }
                    else
                    {
                        double current_timestamp = receivedMsg_receiver.tk_droneData;
                        dt = (current_timestamp - prev_timestamp_drone);
                        if (dt > 300) { dt = 300; }
                        cout << "dt-" << dt << endl;

                        // 如果還未做過回溯，則使用實時資料
                        v = receivedMsg_receiver.uavVelocity;
                        //psi = (receivedMsg_receiver.uavPsi - 90) * 3.1415926 / 180; // rad
                        psi = receivedMsg_receiver.uavPsi; // rad

                        // 根據無人機提供之姿態訊息預測位置
                        prdcx = prdcx + v * cos(psi) * dt / 100;
                        prdcy = prdcy + v * sin(psi) * dt / 100;

                        prev_timestamp_drone = current_timestamp;
                    }

                    // 開發階段計算與無人機實際點的誤差
                    /*double d = sqrt(pow(prdcx - receivedMsg_receiver.uavPos.x, 2) + pow(prdcy - receivedMsg_receiver.uavPos.y, 2));
                    cout << "預測距離與實際誤差: " << d << endl;*/
                }
            } 
        }    

        // 無人機預測訊息存進buffer (需設條件清理)
        {
            unique_lock<mutex> lock(mtx);
            drone_state.t_ = receivedMsg_receiver.tk_droneData;
            drone_state.psi = psi;
            drone_state.uav_v = v;

            drone_state.x = prdcx; // 這裡有錯，在回溯後prdcx/prdcy要取代原來位置的值，不是一直往後塞
            drone_state.y = prdcy;
            IL.predicterBuffer.push_back(drone_state);
        }

        // 畫出預測與實際位置
        currentActual = cv::Point2d(receivedMsg_receiver.uavPos.x, receivedMsg_receiver.uavPos.y);
        currentPredicted = cv::Point2d(prdcx, prdcy);
        circle(IL.img1, currentActual, 10, cv::Scalar(255, 0, 0), -1);
        circle(IL.img1, currentPredicted, 10, cv::Scalar(0, 255, 0), -1);
        cv::imshow("Map", IL.img1);
        cv::waitKey(1);

        //this_thread::sleep_for(std::chrono::milliseconds(100)); // 暫停100毫秒，10hz
    }
}

// 接收子執行緒
void socketReceiver()
{
    while (true)
    {
        //bufferCV.wait(lock, [] { return messageBuffer.size() < MAX_QUEUE_SIZE; });
        IL.initRecvStruct(receivedMsg_receiver);
        receivedMsg_receiver = setSocket.receiveMessage(setSocket.socketState); //使用接收函式

        if (*setSocket.socketState == -1)
        {
            cout << "Socket連線狀態異常，接收子執行緒退出" << endl;
            break;
        }
        // 鎖保護
        {
            unique_lock<mutex> lock(mtx);
            //IL.initRecvStruct(receivedMsg_receiver);
            //receivedMsg_receiver = setSocket.receiveMessage(setSocket.socketState); //使用接收函式
            if (receivedMsg_receiver.cameraTrig == true)
            {
                IL.messageBuffer.push(receivedMsg_receiver); //將receivedMsg_receiver資料推送到緩衝區
                cout << "buffer size: " << IL.messageBuffer.size() << endl;
            } 
            lock.unlock();
        }
        IL.bufferCV.notify_all(); //通知主執行緒處理
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    IL.bufferCV.notify_all(); // 確保退出時不會阻塞主執行緒
}
    
int main()
{
    // 儲存資料到記憶體
    vector<string> ImgName = IL.getCamImgFiles();
    vector<vector<string>> ImgLog = IL.getCamImgLog();
    
    // 演算法參數初始化
    double AerialP_Lon[104] = { 0 };
    double AerialP_Lat[104] = { 0 };
    double Aerial_pixX = 0;
    double Aerial_pixY = 0;
    double Flight_Yaw[104] = { 0 };
    double Flight_Pitch[104] = { 0 };
    int activate_flag = 1; // 執行演算法旗標，目前為判斷無人機到拍攝點 d < 30
    int j = 0; // 演算法迴圈
    int idx = 0; // 讀取空拍照片用
    int block_X = 0;
    int block_Y = 0;
    int picture_num = 0;
    vector<string> log_FirstColumn;
    string cropped_name;
    cv::Point2d Center_points, Estimated_Points;
    cv::Point2d Aerial_pix0[104];
    cv::Mat img2;

    // 提取空拍照片log內容
    size_t LengthLog = ImgLog.size();
    for (int i = 1; i < LengthLog; i++)
    {
        AerialP_Lon[i-1] =stod(ImgLog[i][17]);
        AerialP_Lat[i-1] = stod(ImgLog[i][16]);
        Flight_Yaw[i-1] = stod(ImgLog[i][8]);
        Flight_Pitch[i-1] = stod(ImgLog[i][9]);
        log_FirstColumn.push_back(ImgLog[i][0]);
        Aerial_pix0[i-1].x = abs(AerialP_Lon[i-1] - IL.BL_coor[0]) / IL.Lon_per_pix;
        Aerial_pix0[i-1].y = IL.Map_Height - (abs(AerialP_Lat[i - 1] - IL.BL_coor[1]) / IL.Lat_per_pix);
    }

    // 讀取地圖
    IL.img1 = cv::imread(IL.folderPath_map);
    if (IL.img1.empty())
    {
        cerr << "無法讀取地圖" << endl;
        return -1;
    }

    // 在地圖上標註實際拍攝點位置
    for (int i = 0; i < LengthLog-1; i++)
    {
        cv::Point2d drawp(Aerial_pix0[i].x, Aerial_pix0[i].y);
        cv::circle(IL.img1, drawp, 22, cv::Scalar(0, 0, 255), 10);
        if (i < (LengthLog - 2)) { cv::line(IL.img1, Aerial_pix0[i], Aerial_pix0[i + 1], cv::Scalar(0, 0, 255), 10); }
    }

    //cv::namedWindow("Map", cv::WINDOW_NORMAL); // 可自己調整大小的視窗
    //cv::imshow("Map", IL.img1);
    //cv::waitKey(100); // 若()為0則是無限等待使用者輸入，1000則為1秒以此類推

    // 初始化struct
    IL.initRecvStruct(receivedMsg_main); // 這裡初始化的是主程式的物件，跟recv函式裡面宣告沒關係，所以才應該用指標，之後要改
    IL.initRecvStruct(receivedMsg_receiver);

    // 建立socket
    int socketInit_stat = setSocket.initSocket();
    int socketCreate_stat = setSocket.createSocket();
    int socketConnect_stat = setSocket.setAddress();
    int socketListen_stat = setSocket.listenSocket();
    int sokcetClient_stat = setSocket.confirmConnect();

    // 啟動子執行緒
    thread socketThread(socketReceiver); //建立子執行緒
    socketThread.detach(); //分離子執行緒
    thread predictionThread(posPredicter);
    predictionThread.detach();

    // 演算法執行，要想要用甚麼觸發演算法執行，傳照片過來? flag要打開(=1)多久?
    while (j < 104)
    {
        {
            unique_lock<mutex> lock(mtx); //這裡要用unique_lock底下才能用wait()
            IL.initRecvStruct(receivedMsg_main);
            //接收子執行緒通知主程式有資料
            //IL.bufferCV.wait_for(lock, std::chrono::milliseconds(1000), [] { return !IL.messageBuffer.empty(); });
            IL.bufferCV.wait(lock, [] { return !IL.messageBuffer.empty(); });
            if (*setSocket.socketState == -1)
            {
                cout << "Socket連線狀態異常，主程式退出" << endl;
                lock.unlock();
                break;
            }

            //將buffer資料存入主程式struct
            receivedMsg_main = IL.messageBuffer.front();
            //清除buffer第一筆資料
            IL.messageBuffer.pop();
            //解鎖
            //lock.unlock();
            activate_alg_flag = receivedMsg_main.actAL_flag;
        }

        
        cout << "receivedMsg.k: " << receivedMsg_main.k << endl; // 檢查傳過來的k
        if (receivedMsg_main.k < 0 || receivedMsg_main.k > 1000)
        {
            cout << "Socket傳輸內容異常, receivedMsg.k = " << receivedMsg_main.k << endl;
            break;
        }

        while (receivedMsg_main.cameraTrig == true)
        {
            img2 = cv::imdecode(receivedMsg_main.shotImg, cv::IMREAD_COLOR);
            j = receivedMsg_main.k;
            //cout << receivedMsg_socket->uavPos << endl;
            printf("目前照片: %d.png\n", j + 1);
            // 讀取img2(上機要換成空拍照片)
            //cv::Mat img2 = cv::imread(ImgName[j]);
            fs::path full_path = ImgName[j];
            fs::path photoName = full_path.filename();

            // 根據空拍照片檔案名字找到log檔對應位子
            string s1 = photoName.string();
            auto it = find(log_FirstColumn.begin(), log_FirstColumn.end(), s1);
            if (it != log_FirstColumn.end())
            {
                idx = distance(log_FirstColumn.begin(), it);
            }
            else
            {
                cout << "Target: " << s1 << " not found." << endl;
            }

            //依照子執行緒預測的位置選取圖片
            if (receivedMsg_main.k == 0 ) // 第一筆uav位置丟失的GPS資料，後面就要用演算法預測之位置
            {
                Aerial_pixX = abs(AerialP_Lon[0] - IL.BL_coor[0]) / IL.Lon_per_pix;
                Aerial_pixY = IL.Map_Height - (abs(AerialP_Lat[0] - IL.BL_coor[1]) / IL.Lat_per_pix);
                block_X = ceil(Aerial_pixX / IL.CropSize); 
                block_Y = ceil(Aerial_pixY / IL.CropSize);
                picture_num = (IL.numcol / IL.CropSize) * (block_X - 1) + block_Y;
            }
            else
            {
                // 演算法預測之位置
                lock_guard<mutex> lock(mtx);
                int mapCenterIdx = IL.getPredictBuffer_idx(receivedMsg_main.tk_img);
                block_X = ceil(IL.predicterBuffer[mapCenterIdx].x / IL.CropSize);
                block_Y = ceil(IL.predicterBuffer[mapCenterIdx].y / IL.CropSize);
                picture_num = (IL.numcol / IL.CropSize) * (block_X - 1) + block_Y;

            }

            //Aerial_pixX = abs(AerialP_Lon[idx] - IL.BL_coor[0]) / IL.Lon_per_pix;
            //Aerial_pixY = IL.Map_Height - (abs(AerialP_Lat[idx] - IL.BL_coor[1]) / IL.Lat_per_pix);
            //block_X = ceil(Aerial_pixX / IL.CropSize);  // 這邊到時候要改成uav預位置算法之位置
            //block_Y = ceil(Aerial_pixY / IL.CropSize);
            //picture_num = (IL.numcol / IL.CropSize) * (block_X - 1) + block_Y;

            // 選取離線地圖對應小圖
            cropped_name = to_string(picture_num) + ".png";
            fs::path fullCropPath = IL.CroppedMapPath + "/" + cropped_name;

            // 拼接對應 3*3 小地圖並存入資料夾
            cv::Mat Puzzle33 = IL.puzzleMap(picture_num);
            if (Puzzle33.empty() || img2.empty())
            {
                cout << "照片為空" << endl;
                break;
            }
             
            // 執行影像定位演算法(包含照片處理)，回傳疊圖中心點
            Center_points = IL.locatingAlgorithm(Puzzle33, img2, Flight_Yaw[j]);

            // 估算演算法計算之經緯度
            Estimated_Points = IL.PositionCalculation(block_X, block_Y, j, Flight_Pitch[j], Flight_Yaw[j], Center_points);
            cout << "Estimated lat: " << Estimated_Points.y << " ,Estimated lon: " << Estimated_Points.x << endl;
            cout << "Actual lat: " << AerialP_Lat[j] << " ,Actual lon: " << AerialP_Lon[j] << endl;

            //計算距離誤差
            cv::Point2d deltaPosition(Estimated_Points.x - AerialP_Lon[j], Estimated_Points.y - AerialP_Lat[j]);
            double a = pow(sin((deltaPosition.y / 2) * M_PI / 180), 2) + cos(Estimated_Points.y * M_PI / 180) * cos(AerialP_Lat[j] * M_PI / 180) * pow(sin((deltaPosition.x / 2) * M_PI / 180), 2);
            double c = 2 * atan2(sqrt(a), sqrt(1 - a));
            int R = 6371; // km, 地球半徑
            double d = 1000 * R * c; // m
            cout << "距離誤差: " << d << " 公尺" << endl;

            // 畫出估計經緯度與實際經緯度
            cv::Point2d Estimated_Points2pix((Estimated_Points.x - IL.BL_coor[0]) / IL.Lon_per_pix,
                IL.Map_Height - (Estimated_Points.y - IL.BL_coor[1]) / IL.Lat_per_pix);

            // 畫出演算法估計點
            int size = 20;
            cv::Point2d TL(Estimated_Points2pix.x - size, Estimated_Points2pix.y - size);
            cv::Point2d TR(Estimated_Points2pix.x + size, Estimated_Points2pix.y - size);
            cv::Point2d BL(Estimated_Points2pix.x - size, Estimated_Points2pix.y + size);
            cv::Point2d BR(Estimated_Points2pix.x + size, Estimated_Points2pix.y + size);  

            {
                lock_guard<mutex> lock(mtx);
                cv::line(IL.img1, TL, BR, cv::Scalar(0, 255, 255), 10);
                cv::line(IL.img1, TR, BL, cv::Scalar(0, 255, 255), 10);
                // 計算完估計位置後使位置預測子執行緒回溯到演算法計算之位置重新預測
                IL.EstimatedP_pixel = Estimated_Points2pix; //估測位置(像素)存進全域變數
                IL.img_timestamp = receivedMsg_main.tk_img; //拍照時間存進全域變數
                if (receivedMsg_main.k >= 1) { bias_corret = true; }
            }
            

            // 清理內存/迴圈控制
            receivedMsg_main.cameraTrig = false;
            //Puzzle33.release();
            //img2.release();
        }
    }
    //cv::imwrite("C:/Ncsist_how/Repos/ImgLocation/影像定位matlab實作/Results/cpp/FY_result.png", img1);
    // 清除指標
    delete setSocket.socketState;
    // 清除buffer
    while (!IL.messageBuffer.empty()) 
    {
        IL.messageBuffer.pop(); // 刪除隊首的元素
    }
    // 關閉socket
    setSocket.closeSocket(); 
    cout << "關閉socket..." << endl;
    return 0;
}

