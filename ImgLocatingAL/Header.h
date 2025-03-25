#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <filesystem>
#include "ImgAL_socket.h"  // 要include這個，namespace std才可以用。why?
#include <opencv2/opencv.hpp>

#define M_PI 3.1415926

using namespace std; // -> 使用std會跟socket函式庫中的很多名稱定義衝突  
namespace fs = std::filesystem;

class ImgLocate
{
	// 照片讀取資訊
	string folderPath_camera = "E:/無人機影像定位/CY影像定位實作/豐原空拍_2";
	string PhoInfo_excelFile = "../../../FongYuanLog2.csv";

	// 演算法參數
	int xStepNum = 0;
	int yStepNum = 0;
	int xyStepNum = 0;
	int discard_count = 0; // 捨棄次數
	

	// 運算函式
	cv::Mat im2double(const cv::Mat& img);
	cv::Mat histMatch(cv::Mat&, cv::Mat&);

	
public:
	vector<string> getCamImgFiles();
	vector<vector<string>> getCamImgLog();
	string CroppedMapPath = "E:/無人機影像定位/CY影像定位實作/Cropped_Map_FU"; // 中文路徑也可以
	string folderPath_map = "E:/無人機影像定位/CY影像定位實作/離線地圖/豐原/8KUHD_FongYuan2.png";
	void initRecvStruct(ImgAL_receiveMsg&);
	cv::Mat puzzleMap(int picture_num);
	void setResolution(cv::Mat&, double);
	void setAngle(cv::Mat&, double);
	void grayImg(cv::Mat&);
	void adjustBright_Contrast(cv::Mat&, cv::Mat&, int);
	void PresetPhoto(cv::Mat&, cv::Mat&, double, double);
	cv::Point2d locatingAlgorithm(cv::Mat&, cv::Mat&, double);
	cv::Point2d PositionCalculation(int, int, int, double, double, cv::Point2d);
	

	// 宣告地圖/空拍基本資訊(如何封裝?)(可用建構子?或是寫一函式)
	double Map_Width = 4103; // 豐原
	double Map_Height = 3894;
	double CropSize = 300;
	int numrow = ceil(Map_Width / CropSize) * CropSize; // 注意Map_Width/Height, CropSize要設double結果才會對
	int numcol = ceil(Map_Height / CropSize) * CropSize;

	//double TL_coor[2] = { 120.7157564, 24.2630274 };
	//double TR_coor[2] = { 120.7223255, 24.2630537 };
	//double BL_coor[2] = { 120.7158070, 24.2573680 };
	//double BR_coor[2] = { 120.7223298, 24.2573723 };

	// 地圖四角經緯度
	double TL_coor[2] = { 120.7158570, 24.2630837 };
	double TR_coor[2] = { 120.7223298, 24.2630837 };
	double BL_coor[2] = { 120.7158570, 24.2573723 };
	double BR_coor[2] = { 120.7223298, 24.2573723 };
	double Lon_per_pix = (TR_coor[0] - TL_coor[0]) / Map_Width;
	double Lat_per_pix = (TL_coor[1] - BL_coor[1]) / Map_Height;

	// 演算法參數
	cv::Mat img1;
	bool k = false; // 演算法重跑旗標
	cv::Point2d EstimatedP_pixel; // 演算法計算之位置(像素位置)
	double img_timestamp;
	// 定義應用層緩衝區為ImgAL_receiveMsg的struct
	queue<ImgAL_receiveMsg> messageBuffer; // 拍攝照片緩衝區
	deque<ImgAL_droneState> predicterBuffer; // 預測位置緩衝區
	condition_variable bufferCV; // 用於通知主執行緒
	bool stopReceiver = false; // 用於停止子執行緒
	const int MAX_QUEUE_SIZE = 10;
	int getPredictBuffer_idx(double);
};

// 初始化socket接收物件
void ImgLocate::initRecvStruct(ImgAL_receiveMsg& receivedMsg)
{
	receivedMsg.shotImg_size = 0;
	receivedMsg.cameraTrig = false;
	receivedMsg.switch_flag = false;
	receivedMsg.actAL_flag = false;
	receivedMsg.k = 0;
	receivedMsg.uavPsi = 0;
	receivedMsg.uavRoll = 0;
	receivedMsg.uavVelocity = 0;
	receivedMsg.duration = 0;
	receivedMsg.tk_droneData = 0;
	receivedMsg.tk_img = 0;
	receivedMsg.uavPos = { 0,0 };
}


// 將照片轉換成double的函式
cv::Mat ImgLocate::im2double(const cv::Mat& img)
{
	cv::Mat img_double;
	img.convertTo(img_double, CV_64F, 1.0 / 255.0);
	return img_double;
}

// 取得空拍照片路徑string
vector<string> ImgLocate::getCamImgFiles()
{
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
	return imageFiles_camera;
}

// 取得空拍照片log檔
vector<vector<string>> ImgLocate::getCamImgLog()
{
	ifstream file(PhoInfo_excelFile);
	string line;
	vector<vector<string>> data;
	if (!file.is_open())
	{
		cerr << "無法打開檔案" << PhoInfo_excelFile << endl;
		return data;
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
	return data;
}

// 拼接當前座標照片
cv::Mat ImgLocate::puzzleMap(int picture_num)
{
	cv::Mat MapNum1 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum2 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum3 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum4 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum5 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum6 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum7 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum8 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3),
			MapNum9 = cv::Mat::zeros(CropSize, CropSize, CV_8UC3);
	cv::Mat Puzzle_map33(CropSize * 3, CropSize * 3, CV_8UC3);
	xStepNum = floor((numrow - CropSize) / CropSize + 1);
	yStepNum = floor((numcol - CropSize) / CropSize + 1);
	xyStepNum = xStepNum * yStepNum;

	if ( (picture_num - yStepNum -1) < 1 || (picture_num - yStepNum - 1) > xyStepNum )
	{
		MapNum1.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num - yStepNum - 1) + ".png";
		MapNum1 = cv::imread(full_path);
	}

	if ((picture_num - yStepNum) < 1 || (picture_num - yStepNum) > xyStepNum)
	{
		MapNum2.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num - yStepNum) + ".png";
		MapNum2 = cv::imread(full_path);
	}

	if ((picture_num - yStepNum + 1) < 1 || (picture_num - yStepNum + 1) > xyStepNum)
	{
		MapNum3.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num - yStepNum + 1) + ".png";
		MapNum3 = cv::imread(full_path);
	}

	if ((picture_num - 1) < 1 || (picture_num - 1) > xyStepNum)
	{
		MapNum4.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num - 1) + ".png";
		MapNum4 = cv::imread(full_path);
	}

	if ((picture_num) < 1 || (picture_num) > xyStepNum)
	{
		MapNum5.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num) + ".png";
		MapNum5 = cv::imread(full_path);
	}

	if ((picture_num + 1) < 1 || (picture_num + 1) > xyStepNum)
	{
		MapNum6.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num + 1) + ".png";
		MapNum6 = cv::imread(full_path);
	}

	if ((picture_num + yStepNum - 1) < 1 || (picture_num + yStepNum - 1) > xyStepNum)
	{
		MapNum7.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num + yStepNum - 1) + ".png";
		MapNum7 = cv::imread(full_path);
	}

	if ((picture_num + yStepNum) < 1 || (picture_num + yStepNum) > xyStepNum)
	{
		MapNum8.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num + yStepNum) + ".png";
		MapNum8 = cv::imread(full_path);
	}

	if ((picture_num + yStepNum + 1) < 1 || (picture_num + yStepNum + 1) > xyStepNum)
	{
		MapNum9.setTo(cv::Scalar(0, 0, 0)); // 黑色
	}
	else {
		string full_path = CroppedMapPath + "/" + to_string(picture_num + yStepNum + 1) + ".png";
		MapNum9 = cv::imread(full_path);
	}

	// 避免上下邊界誤判
	for (int count = 0; count < xStepNum; count++)
	{
		if (picture_num == (count - 1) * yStepNum + 1)
		{
			MapNum1.setTo(cv::Scalar(0, 0, 0));
			MapNum4.setTo(cv::Scalar(0, 0, 0)); 
			MapNum7.setTo(cv::Scalar(0, 0, 0)); 
		}
		if (picture_num == count * xyStepNum)
		{
			MapNum3.setTo(cv::Scalar(0, 0, 0));
			MapNum6.setTo(cv::Scalar(0, 0, 0));
			MapNum9.setTo(cv::Scalar(0, 0, 0));
		}
	}

	// 拼接成 3*3 的小地圖
	cv::Mat row1, row2, row3;
	cv::hconcat(vector<cv::Mat>{MapNum1, MapNum4, MapNum7}, row1); // 還可以這樣用vector?
	cv::hconcat(vector<cv::Mat>{MapNum2, MapNum5, MapNum8}, row2);
	cv::hconcat(vector<cv::Mat>{MapNum3, MapNum6, MapNum9}, row3);

	cv::vconcat(vector<cv::Mat>{row1, row2, row3}, Puzzle_map33);

	cv::imshow("FinalImg", Puzzle_map33); 
	cv::waitKey(200);
	cv::destroyWindow("FinalImg");

	// 將拼接好的照片寫入資料夾
	string puzzlePath = "E:/無人機影像定位/CY影像定位實作/小地圖拼接/Puzzle33_cpp.png";
	cv::imwrite(puzzlePath, Puzzle_map33);
	return Puzzle_map33;
}

// 調整照片解析度函數
void ImgLocate::setResolution(cv::Mat& img2, double sFactor)
{
	cv::resize(img2, img2, cv::Size(), sFactor, sFactor);
}

// 調整照片角度
void ImgLocate::setAngle(cv::Mat& img2, double angle)
{
	cv::Point2f center(img2.cols / 2.0F, img2.rows / 2.0F); // 計算圖像中心
	cv::Mat rotateMatrix = cv::getRotationMatrix2D(center, angle, 1.0); // 創建旋轉矩陣
	cv::Rect boundingbox = cv::RotatedRect(center, img2.size(), angle).boundingRect(); // 計算旋轉後圖像大小
	rotateMatrix.at<double>(0, 2) += boundingbox.width / 2.0 - center.x; // 平移以符合邊界
	rotateMatrix.at<double>(1, 2) += boundingbox.height / 2.0 - center.y;

	cv::warpAffine(img2, img2, rotateMatrix, boundingbox.size());
	//cv::namedWindow("rotated_img", cv::WINDOW_NORMAL);
	//cv::imshow("rotated_img", rotated_img);
	//cv::waitKey(0);
}

// 灰階化
void ImgLocate::grayImg(cv::Mat& img)
{
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
	//cv::namedWindow("rotated_img", cv::WINDOW_NORMAL);
	//cv::imshow("rotated_img", grayImg);
	//cv::waitKey(0);
}

// 判斷並調整亮度對比
void ImgLocate::adjustBright_Contrast(cv::Mat& img1, cv::Mat& img2, int dicard_cn)
{
	cv::Mat adjusted_img2;
	cv::Mat temp_img1, temp_img2;
	cv::Scalar mean1, mean2, std1, std2; // 四個double直得型別，分別為BGR與透明度

	temp_img1 = img1;
	temp_img2 = img2;
	temp_img1.convertTo(temp_img1, CV_32F); // 將圖像轉成浮點數型別
	temp_img2.convertTo(temp_img2, CV_32F);
	cv::normalize(temp_img1, temp_img1, 0, 255, cv::NORM_MINMAX); // 正規化到[0 255]之間
	cv::normalize(temp_img2, temp_img2, 0, 255, cv::NORM_MINMAX);
	
	cv::meanStdDev(temp_img1, mean1, std1);
	cv::meanStdDev(temp_img2, mean2, std2);

	cout << "平均值:" << mean1 << "," << mean2 << endl;
	cout << "標準差:" << std1 << "," << std2 << endl;

	// 依據平均值標準差判斷是否要光度均一化
	if (abs(mean1[0] - mean2[0]) >= 10 && abs(mean1[0] - mean2[0] <= 25) || abs(std1[0] - std2[0]) >= 10)
	{
		if (discard_count <= 1)
		{
			cout << "執行光度均一化" << endl;
			adjusted_img2 = histMatch(img1, img2);
		}
		else if (discard_count > 1)
		{
			cout << "錯誤達2次，不執行光度均一化" << endl;
			adjusted_img2 = img2;
		}
		temp_img2 = adjusted_img2;
		cv::normalize(temp_img2, temp_img2, 0, 255, cv::NORM_MINMAX);
		cv::meanStdDev(temp_img2, mean2, std2);
		cout << "調整後平均值:" << mean1 << "," << mean2 << endl;
		cout << "調整後標準差:" << std1 << "," << std2 << endl;
	}
	else
	{
		if (discard_count <= 1)
		{
			cout << "兩張照片差異小或過大，不執行光度均一化" << endl;
			adjusted_img2 = img2;
		}
		else if (discard_count > 1)
		{
			adjusted_img2 = histMatch(img1, img2);
			cout << "錯誤達2次，執行光度均一化" << endl;
			temp_img2 = adjusted_img2;
			cv::normalize(temp_img2, temp_img2, 0, 255, cv::NORM_MINMAX);
			cv::meanStdDev(temp_img2, mean2, std2);
			cout << "調整後平均值:" << mean1 << "," << mean2 << endl;
			cout << "調整後標準差:" << std1 << "," << std2 << endl;
		}

	}
}

// 光度均一化
cv::Mat ImgLocate::histMatch(cv::Mat& map, cv::Mat& photo)
{
	if (photo.empty() || map.empty())
	{
		cerr << "Error: one or both img empty" << endl;
	}

	cv::Mat mapHist, photoHist;
	int histSize = 256;
	float range[] = { 0, 256 };
	const float* histRange = { range };

	// 計算直方圖
	// calcHist(輸入圖像, 輸入圖像數量, 使用的通道索引, 不使用掩碼(可針對部分區域計算值方圖), 輸出直方圖, 直方圖維度, 每個維度的取值範圍)
	cv::calcHist(&photo, 1, 0, cv::Mat(), photoHist, 1, &histSize, &histRange);
	cv::calcHist(&map, 1, 0, cv::Mat(), mapHist, 1, &histSize, &histRange);

	// 計算CDF(累積分布函數) - 像素值的累積概率分布
	photoHist /= photo.total();
	mapHist /= map.total();
	for (int i = 1; i < histSize; i++)
	{
		photoHist.at<float>(i) += photoHist.at<float>(i - 1); // ??
		mapHist.at<float>(i) += mapHist.at<float>(i - 1);
	}

	// 查找表(LUT) - 1D數組，用於儲存每個像素值的映射關係
	cv::Mat lut(1, 256, CV_8U); // 一定要CV_8U?
	int refidx = 0;
	for (int srcidx = 0; srcidx < histSize; ++srcidx)
	{
		while (refidx < histSize && photoHist.at<float>(srcidx) > mapHist.at<float>(refidx))
		{
			refidx ++;
		}
		lut.at<uchar>(srcidx) = refidx;
	}

	cv::Mat matched;
	cv::LUT(photo, lut, matched); // 傳過來的照片要式cv::Mat型式，不能先經過convert_to

	return matched;
}

// 照片前處理
void ImgLocate::PresetPhoto(cv::Mat& Map, cv::Mat& photo, double Flight_yaw, double s_factor)
{
	// 調整照片解析度
	setResolution(photo, s_factor);

	// 無人機空拍照片/地圖轉灰階
	grayImg(Map);
	grayImg(photo);
	//cv::equalizeHist(photo, photo); // 增強空拍照片亮度對比，與matlab計算稍不同

	// 調整亮度對比
	adjustBright_Contrast(Map, photo, discard_count);

	// 調整照片角度符合無人機Yaw(要先處理照片再轉，不然轉完的黑邊會影響平均值/標準差)
	setAngle(photo, -Flight_yaw);
}

// 特徵檢測與描述符
cv::Point2d ImgLocate::locatingAlgorithm(cv::Mat& Map, cv::Mat& photo, double yaw)
{
	// 創建偵測器參數
	bool extended_descriptor = true;	      // 擴展描述子
	bool rotate = true;					   	  // 考慮旋轉
	float detect_threshold = 0.002f;		  // 檢測門檻
	int nOctaves = 3;					      // 金字塔層數
	int nOctaveLayers = 4;				      // 每層的金字塔層數，影響檢測細緻程度

	do
	{
		cv::Mat temp_Map = Map;
		cv::Mat temp_photo = photo;

		if (discard_count == 0)
		{
			PresetPhoto(temp_Map, temp_photo, yaw, 0.2);
			printf("空拍照片解析度調整為'0.2'倍 考慮旋轉\n");
		}
		else if (discard_count == 1)
		{
			PresetPhoto(temp_Map, temp_photo, yaw, 0.2);
			detect_threshold = 0.001f;
			rotate = true;
			printf("空拍照片解析度調整為'0.2'倍 檢測門檻降低 考慮旋轉\n");
		}
		else if (discard_count == 2)
		{
			PresetPhoto(temp_Map, temp_photo, yaw, 0.3);
			detect_threshold = 0.001f;
			rotate = true;
			nOctaves = 4;
			printf("空拍照片解析度調整為'0.3'倍 檢測門檻降低 考慮旋轉 金字塔層數增加\n");
		}

		// KAZE特徵檢測器
		cv::Ptr<cv::KAZE> kaze = cv::KAZE::create(
			extended_descriptor,		 // 是否使用擴展描述子
			rotate,						 // 是否考慮旋轉
			detect_threshold,			 // 檢測門檻，越大越少特徵點
			nOctaves,					 // 金字塔層數，多尺度分析，值越大檢測範圍越廣
			nOctaveLayers,				 // 每層的金字塔層數，影響檢測細緻程度
			cv::KAZE::DIFF_CHARBONNIER	 // 擴散模型(DIFF_PM_G1: 高噪聲圖像，需檢測微弱特徵)(DIFF_PM_G2: 圖像質量高或對性能精度要求較平衡)
								         // (DIFF_WEICKERT: 結構化場景，如建築街道地形)(DIFF_CHARBONNIER: 穩定性要求高的場景，大範圍匹配)
		);

		// 特徵點/描述符
		vector<cv::KeyPoint> keyPoints1, keyPoints2;
		cv::Mat descriptor1, descriptor2;
		kaze->detect(temp_Map, keyPoints1);
		kaze->detect(temp_photo, keyPoints2);

		// 按距離排序，愈小匹配愈好，可減少計算量與減少錯誤
		sort(keyPoints1.begin(), keyPoints1.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {return a.response > b.response; });
		sort(keyPoints2.begin(), keyPoints2.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {return a.response > b.response; });
		if (keyPoints1.size() > 2000) { keyPoints1.resize(2000); }
		if (keyPoints2.size() > 2000) { keyPoints2.resize(2000); } // 這樣是否就不會出現 Assertion Failed 錯誤

		//cv::KeyPointsFilter::retainBest(keyPoints1, 2000);
		//cv::KeyPointsFilter::retainBest(keyPoints2, 2000);

		// 描述符
		kaze->compute(temp_Map, keyPoints1, descriptor1); // 需限制特徵點數量，不然會因為占用大量內存導致記憶體出錯
		kaze->compute(temp_photo, keyPoints2, descriptor2);

		// 匹配
		cv::BFMatcher matcher(cv::NORM_L2, true);
		vector<cv::DMatch> match_pts;
		matcher.match(descriptor1, descriptor2, match_pts);


		cv::Mat inliers, tform, H;
		vector<cv::Point2f> points1, points2;
		double averageDistance = 0.0;
		// 提取匹配點
		for (const auto& matcher : match_pts)
		{
			points1.push_back(keyPoints1[matcher.queryIdx].pt);
			points2.push_back(keyPoints2[matcher.trainIdx].pt);
			averageDistance += matcher.distance;
		}

		// 使用RANSAC演算法
		//tform = cv::findHomography(points2, points1, cv::RANSAC, 5.0, inliers);  // 3.0為閥值，較大的閥值可容許更多的內點。tform為透視變換矩陣
																			     // (srcPoints, desPoints, method, ransacReprojThreshold, mask, maxIters, confidence)
		H = cv::estimateAffine2D(points2, points1, inliers, cv::RANSAC, 5.0); // 2x3的矩陣，此為相似變換
		tform = cv::Mat::eye(3, 3, CV_64F);
		H.copyTo(tform(cv::Rect(0, 0, 3, 2))); // 將相似變換矩陣補成3x3

		double tform_min, tform_max;
		cv::minMaxLoc(tform, &tform_min, &tform_max);
		cout << "最大值: " << tform_max << " 行列式: " << cv::determinant(tform) << endl;

		// 將keypoints篩選為RANSAC演算法計算出有效的匹配點
		vector<cv::DMatch> inlier_matche_pts;
		for (size_t i = 0; i < match_pts.size(); i++)
		{
			if (inliers.at<uchar>(i))  // 遮罩(0為outliers; 1為inliers)
			{
				inlier_matche_pts.push_back(match_pts[i]);
			}
		}
		 
		// 判斷匹配結果(這邊耗太多資源)
		averageDistance /= match_pts.size();
		cout << "平均距離" << averageDistance << endl;

		int matchedThreshold = 8;
		if (inlier_matche_pts.size() < matchedThreshold && discard_count < 3) // 效能消耗較小
		{
			k = true;
			discard_count++;
			if (discard_count == 3)
			{
				cout << "錯誤達三次，捨棄" << endl;
				k = false;
				discard_count = 0;
				//tform = cv::Mat::zeros(3, 3, CV_8U);
				cv::Point CenterPoints(0,0);
				// 清理內存
				temp_Map.release();
				temp_photo.release();
				descriptor1.release();
				descriptor2.release();
				keyPoints1.clear();
				keyPoints2.clear();
				points1.clear();
				points2.clear();
				inliers.release();
				return CenterPoints;
			}
			cout << "特徵點數量過少重新匹配" << endl;
			continue;
		}
		else if (tform.empty() || tform_max > 1500 || cv::determinant(tform) < 1e-4) // 效能消耗較大
		{
			k = true;
			discard_count++;
			if (discard_count == 3)
			{
				cout << "錯誤達三次，捨棄" << endl;
				k = false;
				discard_count = 0;
				//tform = cv::Mat::zeros(3, 3, CV_8U);
				cv::Point CenterPoints(0, 0);
				// 清理內存
				temp_Map.release();
				temp_photo.release();
				descriptor1.release();
				descriptor2.release();
				keyPoints1.clear();
				keyPoints2.clear();
				points1.clear();
				points2.clear();
				inliers.release();
				return CenterPoints;
			}
			cout << "特視變換矩陣為空或條件數過大，匹配不穩定" << endl;
			continue;
		}
		else
		{
			// 匹配結果
			cv::cvtColor(temp_Map, temp_Map, cv::COLOR_GRAY2BGR); // 轉成3通道
			cv::cvtColor(temp_photo, temp_photo, cv::COLOR_GRAY2BGR);
			cv::Mat img_matches;
			cv::drawMatches(temp_Map, keyPoints1, temp_photo, keyPoints2, match_pts, img_matches);
			cv::namedWindow("Kaze Matches", cv::WINDOW_NORMAL);
			cv::imshow("Kaze Matches", img_matches);
			cv::waitKey(200);

			cv::drawMatches(temp_Map, keyPoints1, temp_photo, keyPoints2, inlier_matche_pts, img_matches);
			cv::namedWindow("Inlier Kaze Matches", cv::WINDOW_NORMAL);
			cv::imshow("Inlier Kaze Matches", img_matches);
			cv::waitKey(200);

			// 疊圖
			// 1.計算變換後的圖像邊界
			vector<cv::Point2d> newphotoCorners = { cv::Point2d(0,0), cv::Point2d(temp_photo.cols,0),
													cv::Point2d(temp_photo.cols, temp_photo.rows), cv::Point2d(0, temp_photo.rows) };
			vector<cv::Point2d> transformedCorner;
			cv::perspectiveTransform(newphotoCorners, transformedCorner, tform); // 透視變換後圖像四個角點座標，表示在map上的新座標
			// 2.計算新的邊界框
			// 透視變換後的photo可能超出map邊界框的大小，需檢查新photo四個角點的邊界，min檢查是否超出負方向，max檢查是否超出正方向
			double x_min = min({ transformedCorner[0].x, transformedCorner[1].x, transformedCorner[2].x, transformedCorner[3].x, 0.0 });
			double x_max = max({ transformedCorner[0].x, transformedCorner[1].x, transformedCorner[2].x, transformedCorner[3].x, (double)temp_Map.cols });
			double y_min = min({ transformedCorner[0].y, transformedCorner[1].y, transformedCorner[2].y, transformedCorner[3].y, 0.0 });
			double y_max = max({ transformedCorner[0].y, transformedCorner[1].y, transformedCorner[2].y, transformedCorner[3].y, (double)temp_Map.rows });
			// 3.計算新的視窗尺寸
			int newWidth = static_cast<int>(x_max - x_min);
			int newHeight = static_cast<int>(y_max - y_min);
			// 4.平移變換
			// 當視窗被擴展時，原點(0,0)可能需要跟著偏移
			cv::Mat translation = (cv::Mat_<double>(3, 3) << 1, 0, -x_min,
															 0, 1, -y_min,
														     0, 0, 1);
			tform = translation * tform;
			// 5.透視變換
			cv::Mat warped_img;
			cv::warpPerspective(temp_photo, warped_img, tform, cv::Size(newWidth, newHeight));
			// 6.將目標圖像貼到新圖像中
			cv::Mat stitchedImg = cv::Mat::zeros(newHeight, newWidth, temp_photo.type());
			cv::Mat roi = stitchedImg(cv::Rect(-x_min, -y_min, temp_Map.cols, temp_Map.rows));
			temp_Map.copyTo(roi);
			// 7.疊加
			double alpha = 0.5; // 透明度
			cv::addWeighted(stitchedImg, alpha, warped_img, 1 - alpha, 0.0, stitchedImg);

			// 計算疊圖中心點
			double Xmdn = ((transformedCorner[0].x + transformedCorner[1].x + transformedCorner[2].x + transformedCorner[3].x) / 4.0);
			double Ymdn = ((transformedCorner[0].y + transformedCorner[1].y + transformedCorner[2].y + transformedCorner[3].y) / 4.0); 
			cv::Point2d CenterPoints(Xmdn, Ymdn);
			cv::circle(stitchedImg, CenterPoints, 5, cv::Scalar(0, 255, 255), -1);

			cv::namedWindow("blended", cv::WINDOW_NORMAL);
			cv::imshow("blended", stitchedImg);
			cv::waitKey(200);

			// 清理內存
			cv::destroyWindow("Kaze Matches");
			cv::destroyWindow("Inlier Kaze Matches");
			cv::destroyWindow("blended");
			temp_Map.release();
			temp_photo.release();
			descriptor1.release();
			descriptor2.release();
			keyPoints1.clear();
			keyPoints2.clear();
			points1.clear();
			points2.clear();
			img_matches.release();
			inliers.release();
			warped_img.release();
			stitchedImg.release();

			// 重設判斷參數
			discard_count = 0;
			k = false;

			return CenterPoints; // 若接受匹配結果則返回疊圖中心點
		}

	} while (k == true);
	
}

// 估算經緯度，與實際距離差異
cv::Point2d ImgLocate::PositionCalculation(int blockX, int blockY, int j, double Flight_pitch, double Flight_yaw, cv::Point2d CenterPoints)
{
	cv::Point2d CropMap_TL_Pix(CropSize * (blockX - 2), CropSize * (blockY - 2)); // Puzzle地圖回復到大地圖計算
	double Estimated_Lat, Estimated_Lon;
	Estimated_Lat = TL_coor[1] - (CropMap_TL_Pix.y + CenterPoints.y) * Lat_per_pix;
	Estimated_Lon = TL_coor[0] + (CropMap_TL_Pix.x + CenterPoints.x) * Lon_per_pix;
	
	// 鏡頭角度校正
	double uav_alt = 120; // 豐原
	double gimbal_pitch = 8.5;  // 校正這裡要看實際鏡頭角度而調整
	if (j >= 46) { gimbal_pitch = 0; }

	//double uav_alt = 60; // 逢甲
	//double gimbal_pitch = 0;

	double total_pitch = gimbal_pitch;
	double delta_d = uav_alt * tan(total_pitch * M_PI / 180);
	double delta[2] = { delta_d * sin(Flight_yaw * M_PI / 180), -delta_d * cos(Flight_yaw * M_PI / 180) }; // m

	double deltay2lat = 0.00898 * delta[1] / 1000; // 1000公尺影響 0.00898 緯度
		
	// 1000公尺影響 (1000/(111320*cosd(當地緯度))) 的經度，葫蘆墩公園緯度為24.26
	double deltax2lon = (1000 / (111320 * cos(24.26 * M_PI / 180))) * delta[0] / 1000; 
	//double deltax2lon = (1000 / (111320 * cos(24.17 * M_PI / 180))) * delta[0] / 1000; // 逢甲

	// 計算經校正後的位置
	cv::Point2d Estimated_pos(Estimated_Lon - deltax2lon, Estimated_Lat + deltay2lat);

	return Estimated_pos;
}

// 預測位置子執行緒回溯計算預測位置
int ImgLocate::getPredictBuffer_idx(double img_timestamp)
{
	// 比較函式：比較元素時間戳與目標時間 (如果 buffer.t_<img_timestamp 則傳回true)
	auto comp = [](const ImgAL_droneState& buffer, double img_timestamp) { return buffer.t_ < img_timestamp; };

	// 使用 lower_bound 找到第一個時間戳 >= img_timestamp 的元素位置 (it為迭代器)
	auto it = std::lower_bound(predicterBuffer.begin(), predicterBuffer.end(), img_timestamp, comp);

	if (it == predicterBuffer.begin()) 
	{
		// 若所有元素的時間戳都大於 img_timestamp，則返回第一個元素的索引 0
		return 0;
	}
	if (it == predicterBuffer.end()) 
	{
		// 若所有元素的時間戳都小於 img_timestamp，則返回最後一個元素的索引
		return predicterBuffer.size() - 1;
	}

	// 比較 it 與其前一個元素誰的時間戳更接近目標時間
	auto prev_it = std::prev(it); // 將指標往前一個位置移動
	if (std::abs(it->t_ - img_timestamp) < std::abs(prev_it->t_ - img_timestamp))
	{
		return std::distance(predicterBuffer.begin(), it); // buffer起始位置與迭代器的距離，即為索引值
	}
	else 
	{
		return std::distance(predicterBuffer.begin(), prev_it);
	}

}















