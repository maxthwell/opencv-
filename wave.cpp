// wave.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "owner.h"

void addWall(IplImage *pImg)
{
	CvScalar scalarWall = cvScalarAll(0); //��ɫ�ķ���ǽ
	CvPoint ptPre = cvPoint(pImg->width/3, pImg->height/2);

	//��������ʽ�Ļ���ǽ
	for (double theta = 0; theta < CV_PI * 8.77; theta += 0.05) //��3��
	{
		double r = 2 * theta;
		CvPoint pt = cvPoint(pImg->width/3+r*cos(theta), pImg->height/2+r*sin(theta));
		cvLine(pImg, ptPre, pt, scalarWall,2);
		ptPre = pt;
	}

	ptPre = cvPoint(pImg->width*2 / 3, pImg->height / 2);
	//��������ʽ�Ļ���ǽ
	for (double theta = 0; theta > -CV_PI * 8.77; theta -= 0.05) //��3��
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

		float    dt = 0.2;   //��С��ֵ�Ԫ
		float    sigma = 0.9995; //˥������
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

			//��1���ȶԳ���������˹�任���õ�������ܶ�
			cv::Laplacian(mImg, mLaplace, mImg.depth());
			//��2���Ժ����ܶȳ�����ʱ����֣��õ������ٶ�
			cv::addWeighted(mSpeed, sigma, mLaplace, dt, 0, mTemp);
			mSpeed = mTemp.clone();
			//��3���Բ����ٶȽ��л��֣��õ���������
			cv::addWeighted(mImg, sigma, mSpeed, dt, 0, mTemp);
			mImg = mTemp.clone();
			//��4����ӷ���ǽ���϶
			addWall(&IplImage(mImg));
			static unsigned int i = 0;
			if (!(i++ % 5))
			{
				mouse_callback(CV_EVENT_LBUTTONDOWN, mImg.cols/3+3, mImg.rows/2, 2, &IplImage(mImg)); //���һ���״�
				mouse_callback(CV_EVENT_LBUTTONDOWN, mImg.cols*2/3-3, mImg.rows/ 2, 2, &IplImage(mImg)); //���һ���״�
			}

			static unsigned int j = 0;
			if (j++%5 == 1)
				imshow("wave", mImg);
		}
	}
	return 0;
}