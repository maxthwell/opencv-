#pragma once
#ifndef __cplusplus
#define __cplusplus
#endif // !_cplusplus
#include <rpcsal.h>
#include <Windows.h>
#include "opencv_config.h"
#include <assert.h>
#include <vector>
#include <math.h>
#pragma warning(disable:4244)
#pragma warning(disable:4018)
namespace ow
{

	inline Mat wndMat(HWND hWnd)
	{
		HDC hDC = ::GetDC(hWnd);
		if (!hDC) return Mat();

		HDC hMemDC = ::CreateCompatibleDC(hDC);
		if (!hMemDC) return Mat();

		RECT rc;
		::GetWindowRect(hWnd, &rc);

		HBITMAP hBitmap = ::CreateCompatibleBitmap(hDC, (rc.right - rc.left)>>2<<2, (rc.bottom - rc.top));
		if (!hBitmap) return Mat();

		HBITMAP hOldBmp = (HBITMAP)::SelectObject(hMemDC, hBitmap);
		::BitBlt(hMemDC, 0, 0, rc.right, rc.bottom, hDC, 0, 0, SRCCOPY);

		BITMAP bitmap = { 0 };
		if (0 == ::GetObject(hBitmap, sizeof(BITMAP), &bitmap)) return Mat();

		BITMAPINFOHEADER bi = { 0 };
		BITMAPFILEHEADER bf = { 0 };

		CONST int nBitCount = 24;
		bi.biSize = sizeof(BITMAPINFOHEADER);
		bi.biWidth = bitmap.bmWidth;
		bi.biHeight = bitmap.bmHeight;
		bi.biPlanes = 1;
		bi.biBitCount = nBitCount;
		bi.biCompression = BI_RGB;
		DWORD dwSize = ((bitmap.bmWidth * nBitCount + 31) / 32) * 4 * bitmap.bmHeight;

		HANDLE hDib = GlobalAlloc(GHND, dwSize + sizeof(BITMAPINFOHEADER));
		LPBITMAPINFOHEADER lpbi = (LPBITMAPINFOHEADER)GlobalLock(hDib);
		if (lpbi == NULL)
			return Mat();
		*lpbi = bi;
		if (0 == ::GetDIBits(hMemDC, hBitmap, 0, bitmap.bmHeight, (BYTE*)lpbi + sizeof(BITMAPINFOHEADER), (BITMAPINFO*)lpbi, DIB_RGB_COLORS))
			return Mat();
		BYTE *pData = (BYTE*)lpbi + sizeof(BITMAPINFOHEADER);
		int cx = bi.biWidth;
		int cy = bi.biHeight;
		int d = cx * 3;
		Mat mat(cvSize(cx, cy), CV_8UC3);
		if (mat.empty()) return Mat();
		memcpy(&mat.data[0], pData, cy*d);

		GlobalUnlock(hDib);
		GlobalFree(hDib);
		::SelectObject(hMemDC, hOldBmp);
		::DeleteObject(hBitmap);
		::DeleteObject(hMemDC);
		::ReleaseDC(hWnd, hDC);
		return mat;
	}

	// ��ά��ɢС���任����ͨ������ͼ��
	inline void _dwt1(Mat &mat, int nLayer, float threshold)
	{
		// ִ������
		assert(mat.channels() == 1 && mat.depth() == CV_32F);

		Mat matWave(mat.rows, mat.cols, CV_32FC1);
		float *pfSrc = (float *)mat.data;
		float *pfWave = (float *)matWave.data;

		int h = mat.rows;
		int w = mat.cols;
		//С���б任
		while (nLayer--)
		{
			for (int r = 0; r < h; r += 2)
			{
				for (int c = 0; c < w; c++)
				{
					int index0 = r*mat.cols + c;
					int index1 = index0 + mat.cols;
					int index2 = (r / 2)*mat.cols + c;
					int index3 = index2 + mat.cols*h/2;
					pfWave[index2] = (pfSrc[index0] + pfSrc[index1])*0.5f;
					pfWave[index3] = (pfSrc[index1] - pfSrc[index0])*0.5f;
					if (abs(pfWave[index3]) < threshold) pfWave[index3] = 0.0f; //0��ֵ���ƣ�����ѹ�����֣�
				}
			}
			//С���б任

			for (int r = 0; r < h; r++)
			{
				for (int c = 0; c < w; c += 2)
				{
					int index0 = r*mat.cols + c;
					int index1 = index0 + 1;
					int index2 = r*mat.cols + (c / 2);
					int index3 = index2 + w/2;
					pfSrc[index2] = (pfWave[index0] + pfWave[index1])*0.5f;
					pfSrc[index3] = (pfWave[index1] - pfWave[index0])*0.5f;
					if (abs(pfWave[index3]) < threshold) pfWave[index3] = 0.0f; //0��ֵ���ƣ�����ѹ�����֣�
				}
			}
			w/=2; h/=2;
		}
	}

	struct stBGR
	{
		int b;
		int g;
		int r;
		stBGR & operator +=(stBGR &s1)
		{
			b += s1.b;
			g += s1.g;
			r += s1.r;
			return *this;
		}

		stBGR & operator =(stBGR &s)
		{
			b = s.b;
			g = s.g;
			r = s.r;
			return *this;
		}

		stBGR operator +(stBGR &s1)
		{
			stBGR ret;
			ret.b = b+s1.b;
			ret.g = g+s1.g;
			ret.r = r+s1.r;
			return ret;
		}

		stBGR & operator -=(stBGR &s1)
		{
			b -= s1.b;
			g -= s1.g;
			r -= s1.r;
			return *this;
		}

		stBGR operator - (stBGR &s1)
		{
			stBGR ret;
			ret.b = b-s1.b;
			ret.g = g-s1.g;
			ret.r = r-s1.r;
			return ret;
		}

		stBGR & operator *=(int f)
		{
			b *= f;
			g *= f;
			r *= f;
			return *this;
		}

		stBGR operator *(int f)
		{
			stBGR ret;
			ret.b = b*f;
			ret.g = g*f;
			ret.r = r*f;
			return ret;
		}

		stBGR & operator /=(int f)
		{
			b/=f;
			g /= f;
			r /= f;
			return *this;
		}

		stBGR operator /(int f)
		{
			stBGR ret;
			ret.b = b/f;
			ret.g = g/f;
			ret.r = r/f;
			return ret;
		}

		friend float absBGR(stBGR &s)
		{
			return abs(s.b) + abs(s.g) + abs(s.r);
		}
	};

	// ��ά��ɢС���任��3ͨ��intͼ��256x256��ɫ��
	inline void _dwt3(Mat &mat, int nLayer, float threshold)
	{
		// ִ������
		assert(mat.channels() == 3 && mat.depth() == CV_32S);
		stBGR *pfSrc = (stBGR *)mat.data;
		stBGR *pfWave =(stBGR *)malloc(sizeof(stBGR)*mat.rows*mat.cols);

		int h = mat.rows;
		int w = mat.cols;
		//С���б任
		unsigned int index0, index1, index2, index3;
		while (nLayer--)
		{
			for (unsigned int r = 0; r < h; r += 2)
			{
				for (unsigned int c = 0; c < w; c++)
				{
					index0 = r*mat.cols + c;
					index1 = index0 + mat.cols;
					index2 = (r>>1)*mat.cols + c;
					index3 = index2 + mat.cols*(h>>1);
					pfWave[index2]=(pfSrc[index0] + pfSrc[index1])/2;
					pfWave[index3]=(pfSrc[index1] - pfSrc[index0])/2;
				}
			}
			//С���б任
			for (unsigned int r = 0; r < h; r++)
			{
				for (unsigned int c = 0; c < w; c += 2)
				{
					index0 = r*mat.cols + c;
					index1 = index0 + 1;
					index2 = r*mat.cols + (c>>1);
					index3 = index2 + (w>>1);
					pfSrc[index2] = (pfWave[index0] + pfWave[index1])/2;
					pfSrc[index3] = (pfWave[index1] - pfWave[index0])/2;
				}
			}
		//	w=(w>>1); h=(h>>1);
		}
		free(pfWave);
	}

	// ��ά��ɢС���ָ�����ͨ������ͼ��
	inline void _idwt1(Mat & mat, int nLayer)
	{
		// ִ������
		assert(mat.channels() == 1 && mat.depth() == CV_32F);

		Mat matIWave(mat.rows, mat.cols, CV_32FC1);
		float *pfSrc = (float *)mat.data;
		float *pfIWave = (float *)matIWave.data;
		int h = (mat.rows>>(nLayer-1));
		int w = (mat.cols>>(nLayer-1));
		while (nLayer--)
		{
			//С���л�ԭ
			for (unsigned int r = 0; r < h; r += 2)
			{
				for (unsigned int c = 0; c < w; c++)
				{
					unsigned int index0 = r*mat.cols + c;
					unsigned int index1 = index0 + mat.cols;
					unsigned int index2 = (r / 2)*mat.cols + c;
					unsigned int index3 = index2 + mat.cols*h / 2;
					pfIWave[index0] = pfSrc[index2] - pfSrc[index3];
					pfIWave[index1] = pfSrc[index2] + pfSrc[index3];
				}
			}
			//С���л�ԭ
			for (unsigned int r = 0; r < h; r++)
			{
				for (unsigned int c = 0; c < w; c += 2)
				{
					unsigned int index0 = r*mat.cols + c;
					unsigned int index1 = index0 + 1;
					unsigned int index2 = r*mat.cols + (c / 2);
					unsigned int index3 = index2 + w / 2;
					pfSrc[index0] = pfIWave[index2] - pfIWave[index3];
					pfSrc[index1] = pfIWave[index2] + pfIWave[index3];
				}
			}
			h *= 2; w *= 2;
		}
	}

	// ��ά��ɢС���ָ���3ͨ������ͼ��
	inline void _idwt3(Mat & mat, int nLayer)
	{
		// ִ������
		assert(mat.channels() == 3 && mat.depth() == CV_32S);
		Mat matIWave(mat.rows, mat.cols, CV_32SC3);
		stBGR *pfSrc = (stBGR *)mat.data;
		stBGR *pfIWave = (stBGR *)matIWave.data;
		int h = (mat.rows/* >> (nLayer - 1)*/);
		int w = (mat.cols/* >> (nLayer - 1)*/);
		while (nLayer--)
		{
			//С���л�ԭ
			for (int r = 0; r < h; r += 2)
			{
				for (int c = 0; c < w; c++)
				{
					int index0 = r*mat.cols + c;
					int index1 = index0 + mat.cols;
					int index2 = (r / 2)*mat.cols + c;
					int index3 = index2 + mat.cols*h / 2;
					pfIWave[index0] = pfSrc[index2] - pfSrc[index3];
					pfIWave[index1] = pfSrc[index2] + pfSrc[index3];
				}
			}
			//С���л�ԭ
			for (int r = 0; r < h; r++)
			{
				for (int c = 0; c < w; c += 2)
				{
					int index0 = r*mat.cols + c;
					int index1 = index0 + 1;
					int index2 = r*mat.cols + (c / 2);
					int index3 = index2 + w / 2;
					pfSrc[index0] = pfIWave[index2] - pfIWave[index3];
					pfSrc[index1] = pfIWave[index2] + pfIWave[index3];
				}
			}
		//	h *= 2; w *= 2;
		}
	}

	inline Mat DWT(Mat mat, unsigned int nLayer = 1, float threshold = 0.0f)
	{
		Mat wavelet;
		// С��ͼ��ֵ
		CvSize size = cvSize(mat.cols >> nLayer << nLayer, mat.rows >> nLayer << nLayer);
		Mat matR;
		resize(mat, matR, size);
		matR.convertTo(wavelet, CV_32SC3, 256);
		_dwt3(wavelet, nLayer, threshold);
		return wavelet;
	}

	inline Mat IDWT(Mat mat, unsigned int nLayer = 1)
	{
		_idwt3(mat, nLayer);
		return mat;
		Mat iwave;
		mat.convertTo(iwave, CV_8UC3,1.0/256);
		return iwave;
	}

	inline CvSeq* getContours(IplImage *_image, int _thresh)
	{
		IplImage* g_gray = cvCreateImage(cvGetSize(_image), 8, 1);
		CvMemStorage* g_storage = cvCreateMemStorage(0);//Ĭ�ϴ�С
		CvSeq* seq = 0;

		cvCvtColor(_image, g_gray, CV_BGR2GRAY);//ɫ�ʿռ�ת��
		cvThreshold(g_gray, g_gray, _thresh, 255, CV_THRESH_BINARY);//������Ԫ�ؽ��й̶���ֵ���� 
		cvFindContours(g_gray, g_storage, &seq);//�ڶ�ֵͼ����Ѱ������
		cvReleaseImage(&g_gray);
		return seq;
	}
};