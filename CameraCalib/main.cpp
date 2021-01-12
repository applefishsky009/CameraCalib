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

	Mat mapx = Mat(image_size, CV_32FC1);   //X ������ӳ�����
	Mat mapy = Mat(image_size, CV_32FC1);   //Y ������ӳ�����
	Mat R = Mat::eye(3, 3, CV_32F);
	cout << "�������ͼ��" << endl;
	string imageFileName;                  //У����ͼ��ı���·��
	stringstream StrStm;
	string temp;

	for (int i = 0; i < FilesName.size(); i++) {
		Mat imageSource = imread(FilesName[i]);

		Mat newimage = imageSource.clone();

		//����һ��ʹ��initUndistortRectifyMap��remap�����������ʵ��
		//initUndistortRectifyMap(cameraMatrix,distCoeffs,R, Mat(),image_size,CV_32FC1,mapx,mapy);
		//  remap(imageSource,newimage,mapx, mapy, INTER_LINEAR);

		//������������Ҫת������ķ�ʽ��ʹ��undistort����ʵ��
		undistort(imageSource, newimage, cameraMatrix, distCoeffs);

		StrStm << i + 1;
		StrStm >> temp;
		imshow("Camera Calibration", newimage);//��ʾͼƬ
		waitKey(500);//��ͣ0.5S
		//int nWaitTime = 0;
		//char chKey = cv::waitKey(nWaitTime);
		//if (chKey == 27)
		//	break;
		//if (chKey == ' ')
		//	nWaitTime = !nWaitTime;
		size_t pos = FilesName[i].find_last_of('.');
		imageFileName = FilesName[i].substr(0, pos) + string("_cvt.png");
		//imageFileName = "������ͼ��//" + temp + "_d.jpg";
		imwrite(imageFileName, newimage);

		StrStm.clear();
		imageFileName.clear();
	}
	std::cout << "�������" << endl;
}

int main() {
	cout << fixed;
	cout << setprecision(2);
	ifstream fin(INPUT_FILE); /* �궨����ͼ���ļ���·�� */
	ofstream fout(OUTPUT_FILE);  /* ����궨������ļ� */
											   //��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	cout << "��ʼ��ȡ�ǵ㡭����������" << endl;
	int image_count = 0;  /* ͼ������ */
	Size image_size;  /* ͼ��ĳߴ� */
	//Size board_size = Size(6, 8);    /* �궨����ÿ�С��еĽǵ��� */
	Size board_size = Size(POINT_X_NUM, POINT_Y_NUM);    /* �궨����ÿ�С��еĽǵ��� */
	vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */
	string filename;
	vector<string> v_filename;
	int count = -1;//���ڴ洢�ǵ������
	while (getline(fin, filename)) {
		image_count++;
		v_filename.push_back(filename);
		/* ������� */
		//cout << "-->count = " << count << endl;
		Mat imageInput = imread(filename);
		if (image_count == 1) { //�����һ��ͼƬʱ��ȡͼ������Ϣ
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}
		// ���ڹ۲�������
		cout << "image_count = " << image_count << endl;
		cout << "file_name = " << filename << endl;

		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf)) {
			cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
			system("pause");
			exit(1);
		} else {
			Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			/* �����ؾ�ȷ�� */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(11, 11)); //�Դ���ȡ�Ľǵ���о�ȷ��
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, true); //������ͼƬ�б�ǽǵ�
			imshow("Camera Calibration", view_gray);//��ʾͼƬ
			waitKey(500);//��ͣ0.5S
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
	int CornerNum = board_size.width * board_size.height;  //ÿ��ͼƬ���ܵĽǵ���
	for (ii = 0; ii < total; ii++) {
		cout << "--> �� " << ii + 1 << " ͼƬ������ --> : " << endl;
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
	cout << "�ǵ���ȡ��ɣ�" << endl;	// ע: ���Ͽ��ӻ������������ȣ����µ��ϣ� ���ӻ�Ϊopencv�����ʽ

	// ������������궨
	cout << "��ʼ�궨������������";
	/* ������ά��Ϣ */
	// ע���õ������̸��ʵ�ʴ�С
	Size square_size = Size(SQUARE_W, SQUARE_H);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
										   /* ������� */
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
	vector<int> point_counts;  // ÿ��ͼ���нǵ������
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
	vector<Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
	vector<Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
						  /* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t < image_count; t++) {
		vector<Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++) {
			for (j = 0; j < board_size.width; j++) {
				Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i < image_count; i++) {
		point_counts.push_back(board_size.width * board_size.height);
	}
	/* ��ʼ�궨 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "�궨��ɣ�\n";
	//�Ա궨�����������
	cout << "��ʼ���۱궨���������������\n";
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	vector<Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	cout << "\tÿ��ͼ��ı궨��\n";
	fout << "ÿ��ͼ��ı궨��\n";
	for (i = 0; i < image_count; i++) {
		vector<Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++) {
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << endl;
	}
	std::cout << "����ƽ����" << total_err / image_count << "����" << endl;
	fout << "����ƽ����" << total_err / image_count << "����" << endl << endl;
	std::cout << "������ɣ�" << endl;
	//���涨����  	
	std::cout << "��ʼ���涨����������������" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
	fout << "����ڲ�������" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "����ϵ����\n";
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i < image_count; i++) {
		fout << "��" << i + 1 << "��ͼ�����ת������" << endl;
		fout << tvecsMat[i] << endl;
		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << endl;
		fout << rotation_matrix << endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << endl;
		fout << rvecsMat[i] << endl << endl;
	}
	std::cout << "��ɱ���" << endl;
	fout << endl;

	// ��ʾ����������ͼ��
	m_undistort(v_filename, image_size, cameraMatrix, distCoeffs);

	system("pause");
	return 0;
}