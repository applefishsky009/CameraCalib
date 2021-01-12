#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
#define POINT_X_NUM 6
#define POINT_Y_NUM 8
#define SQUARE_W 3.5
#define SQUARE_H 3.5
#define INPUT_FILE "calibdata_3.5cm_6_8.txt"
#define OUTPUT_FILE "caliberation_result_3.5cm_6_8.txt"

void m_undistort(vector<string> &FilesName, Size image_size, Mat &cameraMatrix, Mat &distCoeffs) {

	Mat mapx = Mat(image_size, CV_32FC1);   //X 坐标重映射参数
	Mat mapy = Mat(image_size, CV_32FC1);   //Y 坐标重映射参数
	Mat R = Mat::eye(3, 3, CV_32F);
	cout << "保存矫正图像" << endl;
	string imageFileName;                  //校正后图像的保存路径
	stringstream StrStm;
	string temp;

	for (int i = 0; i < FilesName.size(); i++) {
		Mat imageSource = imread(FilesName[i]);

		Mat newimage = imageSource.clone();

		//方法一：使用initUndistortRectifyMap和remap两个函数配合实现
		//initUndistortRectifyMap(cameraMatrix,distCoeffs,R, Mat(),image_size,CV_32FC1,mapx,mapy);
		//  remap(imageSource,newimage,mapx, mapy, INTER_LINEAR);

		//方法二：不需要转换矩阵的方式，使用undistort函数实现
		undistort(imageSource, newimage, cameraMatrix, distCoeffs);

		StrStm << i + 1;
		StrStm >> temp;
		imshow("Camera Calibration", newimage);//显示图片
		waitKey(500);//暂停0.5S
		//int nWaitTime = 0;
		//char chKey = cv::waitKey(nWaitTime);
		//if (chKey == 27)
		//	break;
		//if (chKey == ' ')
		//	nWaitTime = !nWaitTime;
		size_t pos = FilesName[i].find_last_of('.');
		imageFileName = FilesName[i].substr(0, pos) + string("_cvt.png");
		//imageFileName = "矫正后图像//" + temp + "_d.jpg";
		imwrite(imageFileName, newimage);

		StrStm.clear();
		imageFileName.clear();
	}
	std::cout << "保存结束" << endl;
}

int main() {
	cout << fixed;
	cout << setprecision(2);
	ifstream fin(INPUT_FILE); /* 标定所用图像文件的路径 */
	ofstream fout(OUTPUT_FILE);  /* 保存标定结果的文件 */
											   //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
	cout << "开始提取角点………………" << endl;
	int image_count = 0;  /* 图像数量 */
	Size image_size;  /* 图像的尺寸 */
	//Size board_size = Size(6, 8);    /* 标定板上每行、列的角点数 */
	Size board_size = Size(POINT_X_NUM, POINT_Y_NUM);    /* 标定板上每行、列的角点数 */
	vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
	string filename;
	vector<string> v_filename;
	int count = -1;//用于存储角点个数。
	while (getline(fin, filename)) {
		image_count++;
		v_filename.push_back(filename);
		/* 输出检验 */
		//cout << "-->count = " << count << endl;
		Mat imageInput = imread(filename);
		if (image_count == 1) { //读入第一张图片时获取图像宽高信息
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}
		// 用于观察检验输出
		cout << "image_count = " << image_count << endl;
		cout << "file_name = " << filename << endl;

		/* 提取角点 */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf)) {
			cout << "can not find chessboard corners!\n"; //找不到角点
			system("pause");
			exit(1);
		} else {
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			/* 亚像素精确化 */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(11, 11)); //对粗提取的角点进行精确化
			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
														   /* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray, board_size, image_points_buf, true); //用于在图片中标记角点
			imshow("Camera Calibration", view_gray);//显示图片
			waitKey(500);//暂停0.5S
			//int nWaitTime = 0;
			//char chKey = cv::waitKey(nWaitTime);
			//if (chKey == 27)
			//	break;
			//if (chKey == ' ')
			//	nWaitTime = !nWaitTime;
		}
	}
	int ii = 0, jj = 0, kk = 0;
	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int CornerNum = board_size.width * board_size.height;  //每张图片上总的角点数
	for (ii = 0; ii < total; ii++) {
		cout << "--> 第 " << ii + 1 << " 图片的数据 --> : " << endl;
		for (jj = board_size.width - 1; jj >= 0; --jj) {
			for (kk = 0; kk < board_size.height; ++kk) {
				cout << "\t";
				cout << " -->" << image_points_seq[ii][jj + kk * board_size.width].x;
				cout << " -->" << image_points_seq[ii][jj + kk * board_size.width].y;
			}
			cout << endl;
		}
		cout << endl;
	}
	cout << "角点提取完成！" << endl;	// 注: 以上可视化过程是列优先，从下到上， 可视化为opencv坐标格式

	// 以下是摄像机标定
	cout << "开始标定………………";
	/* 棋盘三维信息 */
	// 注意用到了棋盘格的实际大小
	Size square_size = Size(SQUARE_W, SQUARE_H);  /* 实际测量得到的标定板上每个棋盘格的大小 */
	vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
										   /* 内外参数 */
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
	vector<int> point_counts;  // 每幅图像中角点的数量
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
	vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
						  /* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t < image_count; t++) {
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++) {
			for (j = 0; j < board_size.width; j++) {
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (i = 0; i < image_count; i++) {
		point_counts.push_back(board_size.width * board_size.height);
	}
	/* 开始标定 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "标定完成！\n";
	//对标定结果进行评价
	cout << "开始评价标定结果………………\n";
	double total_err = 0.0; /* 所有图像的平均误差的总和 */
	double err = 0.0; /* 每幅图像的平均误差 */
	vector<Point2f> image_points2; /* 保存重新计算得到的投影点 */
	cout << "\t每幅图像的标定误差：\n";
	fout << "每幅图像的标定误差：\n";
	for (i = 0; i < image_count; i++) {
		vector<Point3f> tempPointSet = object_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++) {
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	std::cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	std::cout << "评价完成！" << endl;
	//保存定标结果  	
	std::cout << "开始保存定标结果………………" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "相机内参数矩阵：" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "畸变系数：\n";
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++) {
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << tvecsMat[i] << endl;
		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << rvecsMat[i] << endl << endl;
	}
	std::cout << "完成保存" << endl;
	fout << endl;

	// 显示畸变矫正后的图像
	m_undistort(v_filename, image_size, cameraMatrix, distCoeffs);

	system("pause");
	return 0;
}