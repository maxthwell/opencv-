// wave.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "owner.h"

void addWall(IplImage *pImg)
{
	CvScalar scalarWall = cvScalarAll(0); //黑色的反射墙
	CvPoint ptPre = cvPoint(pImg->width/3, pImg->height/2);

	//极坐标形式的回旋墙
	for (double theta = 0; theta < CV_PI * 8.77; theta += 0.05) //绕3周
	{
		double r = 2 * theta;
		CvPoint pt = cvPoint(pImg->width/3+r*cos(theta), pImg->height/2+r*sin(theta));
		cvLine(pImg, ptPre, pt, scalarWall,2);
		ptPre = pt;
	}

	ptPre = cvPoint(pImg->width*2 / 3, pImg->height / 2);
	//极坐标形式的回旋墙
	for (double theta = 0; theta > -CV_PI * 8.77; theta -= 0.05) //绕3周
	{
		double r = 2 * theta;
		CvPoint pt = cvPoint(pImg->width*2/ 3 + r*cos(theta), pImg->height / 2 + r*sin(theta));
		cvLine(pImg, ptPre, pt, scalarWall, 2);
		ptPre = pt;
	}
}

void mouse_callback(int event, int x, int y, int flag, void *pImg)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		CvScalar scalar;
		ZeroMemory(&scalar, sizeof(scalar));
		for (unsigned int i = 0; i < 3; i++)
		{
			scalar.val[i] = rand() % 256;//flag % 2 ? 255 : 0;
			scalar.val[i] /= 4;
			flag /= 2;
		}
		cvSet2D(pImg, y, x, scalar);
	}
}

int main(int argc, char** argv)
{
	HWND hWnd = NULL;
	namedWindow("wave");
	while (1)
	{
		char wk = waitKey(0);
		if (27 == wk) break;
		else if ('a' == wk)
		{
			POINT pt;
			::GetCursorPos(&pt);
			hWnd = WindowFromPoint(pt);
		}
		if (!hWnd) continue;

		float    dt = 0.2;   //最小差分单元
		float    sigma = 0.9995; //衰减因子
		Mat mWnd = ow::wndMat(hWnd);
		if (mWnd.empty()) continue;
		Mat mImg;
		Mat mLaplace;
		Mat mSpeed(mWnd.rows,mWnd.cols,CV_32FC3);
		mSpeed.setTo(0);
		Mat mTemp;
		mWnd.convertTo(mImg, CV_32FC3, 1.0 / 256);

		cvSetMouseCallback("wave",mouse_callback,(void *)&IplImage(mImg));
		while (1)
		{
			char c = waitKey(1);
			if (c == 27)
				break;

			//【1】先对场做拉普拉斯变换，得到其荷子密度
			cv::Laplacian(mImg, mLaplace, mImg.depth());
			//【2】对荷子密度场进行时间积分，得到波动速度
			cv::addWeighted(mSpeed, sigma, mLaplace, dt, 0, mTemp);
			mSpeed = mTemp.clone();
			//【3】对波动速度进行积分，得到波动幅度
			cv::addWeighted(mImg, sigma, mSpeed, dt, 0, mTemp);
			mImg = mTemp.clone();
			//【4】添加反射墙与缝隙
			addWall(&IplImage(mImg));
			static unsigned int i = 0;
			if (!(i++ % 5))
			{
				mouse_callback(CV_EVENT_LBUTTONDOWN, mImg.cols/3+3, mImg.rows/2, 2, &IplImage(mImg)); //添加一个雷达
				mouse_callback(CV_EVENT_LBUTTONDOWN, mImg.cols*2/3-3, mImg.rows/ 2, 2, &IplImage(mImg)); //添加一个雷达
			}

			static unsigned int j = 0;
			if (j++%5 == 1)
				imshow("wave", mImg);
		}
	}
	return 0;
}