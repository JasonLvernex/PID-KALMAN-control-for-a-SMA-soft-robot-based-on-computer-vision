/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE
    #define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "AXonLink.h"
#include <OpenNI.h>
#include "Viewer.h"
#include "Eigen/Dense"
#include <vector>
#include "SerialPort.h" 
#include "stdio.h"
#include <stdlib.h>
using namespace Eigen;
#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/glut.h>
#endif
#include<opencv2/opencv.hpp>
#include "OniSampleUtilities.h"

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_OVERLAY

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))
using namespace cv;
SampleViewer* SampleViewer::ms_self = NULL;

int idlecount = 0;
int test = 0;

//extern float leg_length;
union Idata
{
	float num_d;
	//         String num_s;
	unsigned char num_s[4];
	//         u8        IOArray[512];
};
Idata mynum;
extern CSerialPort mySerialPort;

float leg_length;
float para_real = 1;//  138.28;// 220.14;//mm
float para_x = 21769.2;
float para_y = 18666.3;


typedef unsigned char *byte_pointer;

void show_bytes(byte_pointer start, int len) {
	int i;
	for (i = 0; i < len; i++) {
		printf(" %.2x", start[i]);
	}
	printf("\n");
}

void show_float(float x) {
	show_bytes((byte_pointer)&x, sizeof(float));
}

void SampleViewer::glutIdle()
{
    glutPostRedisplay();
	if (idlecount == 100) {
		if (test == 2)
			test = 0;
		idlecount = 0;
	}
	idlecount++;
}
void SampleViewer::glutDisplay()
{
    SampleViewer::ms_self->display();
}
void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
{
    SampleViewer::ms_self->onKey(key, x, y);
}

SampleViewer::SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color, openni::VideoStream& ir) :
    m_device(device), m_depthStream(depth), m_colorStream(color), m_irStream(ir), m_streams(NULL), m_eViewState(DEFAULT_DISPLAY_MODE), m_pTexMap(NULL)

{
    ms_self = this;
    strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);

	
}
SampleViewer::~SampleViewer()
{
    delete[] m_pTexMap;

    ms_self = NULL;

    if (m_streams != NULL)
    {
        delete []m_streams;
    }
}

openni::Status SampleViewer::init(int argc, char **argv,AXonLinkCamParam* param, int paramindex)
{
    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;
	m_camParam = param;
	m_cparamindex = paramindex;
	for (int i = 0; i < PT_NUM; i++) {
		m_pointw[i] = 0;
		m_ptx[i] = m_pty[i] = 0;
	}
	m_end[0] = 0;
	m_end[1] = 0;

	for (int i = 0; i < PT_NUM*(PT_NUM-1)/2; i++) {
		m_dist[i] = 0;
	}
	 m_ptx1=0;
			m_pty1 = 0;
			m_ptx2 = 0;
			m_pty2 = 0;

			for (int i = 0; i < EMA_NUM; i++) {
				 ema_x1[i]=0;
				 ema_y1[i]=0;
				 ema_z1[i] = 0;
				 ema_x2[i] = 0;
				 ema_y2[i] = 0;
				 ema_z2[i] = 0;
				ema_count = 0;
			}
			Matrix<double, 3, 3>  R;
			Vector3d t;
		
			R(0,0) = param->stExtParam.R_Param[0];
			R(0, 1) = param->stExtParam.R_Param[1];
			R(0, 2) = param->stExtParam.R_Param[2];
			R(1, 0) = param->stExtParam.R_Param[3];
			R(1, 1) = param->stExtParam.R_Param[4];
			R(1, 2) = param->stExtParam.R_Param[5];
			R(2, 0) = param->stExtParam.R_Param[6];
			R(2, 1) = param->stExtParam.R_Param[7];
			R(2, 2) = param->stExtParam.R_Param[8];
/*		R(0, 0) = param->stExtParam.R_Param[0];
			R(1, 0) = param->stExtParam.R_Param[1];
			R(2, 0) = param->stExtParam.R_Param[2];
			R(0, 1) = param->stExtParam.R_Param[3];
			R(1, 1) = param->stExtParam.R_Param[4];
			R(2, 1) = param->stExtParam.R_Param[5];
			R(0, 2) = param->stExtParam.R_Param[6];
			R(1, 2) = param->stExtParam.R_Param[7];
			R(2, 2) = param->stExtParam.R_Param[8];
			*/
			t(0)= param->stExtParam.T_Param[0];
			t(1) = param->stExtParam.T_Param[1];
			t(2) = param->stExtParam.T_Param[2];


	m_camera.setExternalParams(R,t);
	m_depthcamera.setExternalParams(R, t);
	// color 1280*960
	m_camera.setInternalParams(param->astColorParam[0].fx, param->astColorParam[0].cx, param->astColorParam[0].fy, param->astColorParam[0].cy);
   
	m_depthcamera.setInternalParams(param->astDepthParam[0].fx, param->astDepthParam[0].cx, param->astDepthParam[0].fy, param->astDepthParam[0].cy);
	if (m_depthStream.isValid() && m_colorStream.isValid())
    {
        depthVideoMode = m_depthStream.getVideoMode();
        colorVideoMode = m_colorStream.getVideoMode();

        int depthWidth = depthVideoMode.getResolutionX();
        int depthHeight = depthVideoMode.getResolutionY();
        int colorWidth = colorVideoMode.getResolutionX();
        int colorHeight = colorVideoMode.getResolutionY();

        if (depthWidth == colorWidth &&
            depthHeight == colorHeight)
        {
            m_width = depthWidth;
            m_height = depthHeight;
        }
        else
        {
       /*     printf("Error - expect color and depth to be in same resolution: D: %dx%d, C: %dx%d\n",
                depthWidth, depthHeight,
                colorWidth, colorHeight);
            return openni::STATUS_ERROR;
			*/
			m_width = colorWidth;
			m_height = colorHeight;
        }
    }
    else if (m_depthStream.isValid())
    {
        depthVideoMode = m_depthStream.getVideoMode();
        m_width = depthVideoMode.getResolutionX();
        m_height = depthVideoMode.getResolutionY();
    }
    else if (m_colorStream.isValid())
    {
        colorVideoMode = m_colorStream.getVideoMode();
        m_width = colorVideoMode.getResolutionX();
        m_height = colorVideoMode.getResolutionY();
    }
    else
    {
        printf("Error - expects at least one of the streams to be valid...\n");
        return openni::STATUS_ERROR;
    }

    m_streams = new openni::VideoStream*[3];
    m_streams[0] = &m_depthStream;
    m_streams[1] = &m_colorStream;
    m_streams[2] = &m_irStream;

    // Texture map init
    m_nTexMapX = MIN_CHUNKS_SIZE(m_width, TEXTURE_SIZE);
    m_nTexMapY = MIN_CHUNKS_SIZE(m_height, TEXTURE_SIZE);
    m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];

    return initOpenGL(argc, argv);

}
openni::Status SampleViewer::run()	//Does not return
{
    glutMainLoop();

    return openni::STATUS_OK;
}

Vec3b whitecolor; 

void SampleViewer::findEnd() {
	int i;
	int j;
	int count = 0;
	for (i = 0; i < PT_NUM; i++) {

		for (j = i+1; j < PT_NUM; j++) {
			m_dist[count++] = (m_ptx[i] - m_ptx[j])*(m_ptx[i] - m_ptx[j]) + (m_pty[i] - m_pty[j])*(m_pty[i] - m_pty[j]);

		}
	}
	 int maxdist = 0;
	 int end1 = 0;
	 int end2 = 0;
	 count = 0;
	 for (i = 0; i < PT_NUM; i++) {

		 for (j = i + 1; j < PT_NUM; j++) {
			 if (m_dist[count] > maxdist)
			 {
				 maxdist = m_dist[count];
				 end1 = i; end2 = j;
			 }
			 count++;
		 }
	 }
	 printf("select end %d %d \n",end1,end2);
	 m_end[0] = end1;
	 m_end[1] = end2;

}
void SampleViewer::dumpPoint() {

	for (int i = 0; i < PT_NUM; i++) {
		
		printf("Point %d %d %d \n", m_ptx[i], m_pty[i],m_pointw[i]);
	}
}
void SampleViewer::setPoint(int y, int x, int w) {
	int tmpw = 0;
	int tmpy = 0;
	int tmpx = 0;
	int i = 0;
	for ( i = 0; i < PT_NUM; i++) {
		if (m_pointw[i] < w) {
			tmpw = m_pointw[i];
			tmpx = m_ptx[i];
			tmpy = m_pty[i];
		
			m_ptx[i] = x;
			m_pty[i] = y;
			m_pointw[i] = w;
			if (tmpw == 0)return;
			break;
		}
	}
	if (i  <PT_NUM) {

		int tw = 0;
		int ty = 0;
		int tx = 0;
		for (int j = i+1; j < PT_NUM; j++) {
			if (m_pointw[j] == 0)return;
			tw = m_pointw[j];
			tx = m_ptx[j];
			ty = m_pty[j];

			m_ptx[j] = tmpx;
			m_pty[j] = tmpy;
			m_pointw[j] = tmpw;
			tmpx = tx;
			tmpy = ty;
			tmpw = tw;


		}
	
	}

}

void SampleViewer::getPoint(Mat& img) 
{
	int w = img.cols;
	int h = img.rows;
	whitecolor[0] = whitecolor[1] = whitecolor[2] = 255;

	for (int i = 0; i < h; i++)
	{
		bool iswhite = false;
		int start = 0;
		int end = 0;
		for (int j = 0; j < w; j++)
		{

			
			if (img.at<Vec3b>(i, j) == whitecolor) {
				if (!iswhite) {
					start =j;
					iswhite = true;
				}
			}
			else
			{
				if (iswhite)
				{
					iswhite = false;
					end = j;
					if ((end - start) > 0) {
						if((end-start)>10) printf(" %d %d %d\n",i, (int)((start + end) / 2), end - start);
						setPoint(i, (int)((start + end) / 2), end - start);
					}
				
				}
			}

		}//xx
	}//yy
	//printf("getpoint %d %d %d \n",maxx,maxy ,maxwidth);
	dumpPoint();
	findEnd();
}
/*
void crop(Mat&img, Mat&crop_img,std::vector<int> &area) {

	int x1 = std::max(0, area[0]);

	int y1 = std::max(0, area[1]);

	int x2 = std::min(img.cols - 1, area[2]);
	int y2 = std::min(img.rows - 1, area[3]);

	crop_img = img(Range(y1, y2 + 1), Range(x1, x2 + 1));
}
*/
void SampleViewer::display()
{
    int changedIndex;
    openni::Status rc = openni::OpenNI::waitForAnyStream(m_streams, 3, &changedIndex);
    if (rc != openni::STATUS_OK)
    {
        printf("Wait failed\n");
        return;
    }

    switch (changedIndex)
    {
    case 0:
        m_depthStream.readFrame(&m_depthFrame);
		//printf("[%08llu] depth %d\n", (long long)m_depthFrame.getTimestamp(), m_depthFrame.getFrameIndex());
		break;
    case 1:
        m_colorStream.readFrame(&m_colorFrame);
		//printf("[%08llu] color %d\n", (long long)m_colorFrame.getTimestamp(), m_colorFrame.getFrameIndex());
		break;
    case 2:
        m_irStream.readFrame(&m_irFrame);
		//printf("[%08llu] ir %d\n", (long long)m_irFrame.getTimestamp(), m_irFrame.getFrameIndex());
		break;
    default:
        printf("Error in wait\n");
    }
#if 1
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);
#endif
    if (m_depthFrame.isValid())
    {
        calculateHistogram(m_pDepthHist, MAX_DEPTH, m_depthFrame);
    }

    memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

    // check if we need to draw image frame to texture
    if ((m_eViewState == DISPLAY_MODE_OVERLAY ||
        m_eViewState == DISPLAY_MODE_IMAGE) && m_colorFrame.isValid())
    {
		if (test == 0) {
			//printf("color %d %d\n", m_colorFrame.getSensorType(), m_colorFrame.getFrameIndex());
			const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
//			openni::RGB888Pixel* pTexRow = m_pTexMap + m_colorFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);
			int h = m_colorFrame.getHeight();
			int w = m_colorFrame.getWidth();
			Mat I = Mat(h,w,CV_8UC3);
			for (int y = 0; y < h;y++)
			{
				const openni::RGB888Pixel* pImage = pImageRow;
			//	openni::RGB888Pixel* pTex = pTexRow + m_colorFrame.getCropOriginX();
			//	uchar* rowptr = I.ptr<uchar>(y);//redgray也是cv::Mat类型的三通道矩阵，表示的rgb图像
				for (int x = 0; x < w; x++) //++pTex)
				{
					//*pTex = *pImage;
					//I.at<Vec3b>(x, y) = pImage[x];
					I.at<Vec3b>(y,x)[0]= pImage[x].b;
				    I.at<Vec3b>(y,x)[1] = pImage[x].g;
					I.at<Vec3b>(y,x)[2] = pImage[x].r;
					/*
					rowptr[3 * x + 0] = pImage[x].b;
					rowptr[3 * x + 1] = pImage[x].g;
					rowptr[3 * x + 2] = pImage[x].r;*/

				}

				pImageRow += rowSize;
			//	pTexRow += m_nTexMapX;
			}
			//imshow("Canny", I);
			Scalar  l_blue = Scalar(8, 43, 46);
			Scalar  h_blue = Scalar(35, 255, 255);

			Mat hsv = Mat(h, w, CV_8UC3);
			cvtColor(I, hsv, COLOR_BGR2HSV);
			//imshow("hsv", hsv);
			Mat mask = Mat(h, w, CV_8UC3);
			inRange(hsv, l_blue, h_blue, mask);
			//imshow("mask1", mask);
			
			for (int y = 0; y < h; y++)
			{

			if (y <int(h / 5) || y>int( 5* h / 8)) 
				{
					
					for (int x = 0; x < w; x++) //++pTex)
					{				
						mask.at<Vec3b>(y,x)[0] = 0;
						mask.at<Vec3b>(y,x)[1] =0;
						mask.at<Vec3b>(y,x)[2] = 0;
					}
				}
			}
		
			for (int x = 0; x< w; x++)
			{

				if ( x<int(w/10) || x>int(9*w/10))
				{

					for (int y = 0; y < h; y++) //++pTex)
					{
						mask.at<Vec3b>(y, x)[0] = 0;
						mask.at<Vec3b>(y, x)[1] = 0;
						mask.at<Vec3b>(y, x)[2] = 0;
					}
				}
			}
			//imshow("mask2", mask);
			Mat kernel = getStructuringElement(MORPH_RECT, Size(10, 10));
			Mat dst = Mat(h, w, CV_8UC3);
			dilate(mask, dst, kernel);
			imshow("dst", dst);

			std::vector<std::vector<Point>> contours;
			std::vector<Vec4i> hierarchy;
			findContours(dst, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
			Mat imageContours = Mat::zeros(dst.size(), CV_8UC1);
			Mat Contours = Mat::zeros(dst.size(), CV_8UC1);  //绘制  
			std::vector<double> area;
			std::vector<int > center_x;
			std::vector<int > center_y;
			for (int i = 0; i<contours.size(); i++)
			{
				int sumx = 0;
				int sumy = 0;
				area.push_back(contourArea(contours[i]));
				for (int j = 0; j<contours[i].size(); j++)
				{
			
					sumx = sumx + contours[i][j].x;
					sumy = sumy + contours[i][j].y;
				}
		
					center_x.push_back ((int )(sumx / contours[i].size()));
					center_y.push_back((int)(sumy / contours[i].size()));
				

			
				/*//输出hierarchy向量内容  
				char ch[256];
				sprintf(ch, "%d", i);
				std::string str = ch;
				std::cout << "向量hierarchy的第" << str << " 个元素内容为：" << std::endl << hierarchy[i] << std::endl << std::endl;
				*/
				//绘制轮廓  

				drawContours(imageContours, contours, i, Scalar(255), 1, 8, hierarchy);
			}
			imshow("Contours Image", imageContours); //轮廓  
		
			auto v_max =max_element(area.begin(), area.end());
			int v1_max = v_max - area.begin();
			//printf(" find v1max %d %d\n", v1_max,(int) area.size());
			m_ptx1 = center_x[v1_max];
			m_pty1 = center_y[v1_max];
			area.erase(area.begin() + v1_max);
			center_x.erase(center_x.begin() + v1_max);
			center_y.erase(center_y.begin() + v1_max);

			v_max = max_element(area.begin(), area.end());
			 v1_max = v_max - area.begin();
			m_ptx2 = center_x[v1_max];
			m_pty2 = center_y[v1_max];
			printf("get center %d %d %d %d \n", m_ptx1, m_pty1, m_ptx2, m_pty2);
			/*
			whitecolor[0] = whitecolor[1] = whitecolor[2] = 255;
			



			for (int i = 0; i < h; i++)
			{
				bool iswhite = false;
				int start = 0;
				int end = 0;
				for (int j = 0; j < w; j++)
				{

					Vec3b v = dst.at<Vec3b>(i, j);
					if (v[0]==255&& v[1]==255 && v[2]==255) {
						if (!iswhite) {
							start = j;
							iswhite = true;
						}
					}
					else
					{
						if (iswhite)
						{
							iswhite = false;
							end = j;
							if ((end - start) > 0) {
								if ((end - start) > 10)  printf("setpoint %d %d %d %d\n", i, j,(int)((start + end) / 2), end - start);
								setPoint(i, (int)((start + end) / 2), end - start);
							}
							
						}
						start = end = 0;
					}

				}//xx
			}//yy
			 //printf("getpoint %d %d %d \n",maxx,maxy ,maxwidth);
			dumpPoint();
			findEnd();
			*/
			
			//circle(hsv, Point(m_ptx1, m_pty1), 5, Scalar(255, 44, 0),4);
			//circle(hsv, Point(m_ptx2, m_pty2), 5, Scalar(255, 44, 0), 4);
			circle(I,Point(m_ptx1, m_pty1), 5, Scalar(255, 44, 0), 8);
			circle(I, Point(m_ptx2, m_pty2), 5, Scalar(255, 44, 0), 8);

			//printf(" draw c   %d %d \n" , m_ptx[m_end[0]], m_pty[m_end[0]]);
			//printf(" draw c   %d %d \n", m_ptx[m_end[1]], m_pty[m_end[1]]);
			//imshow("hsv", hsv);
			imshow("point", I);
			//imshow("dst,", dst );
			

			test = 1;
		}
    }

    // check if we need to draw IR frame to texture
    if ((m_eViewState == DISPLAY_MODE_OVERLAY ||
        m_eViewState == DISPLAY_MODE_IR) && m_irFrame.isValid())
    {
        // printf("ir %d %d\n", m_irFrame.getSensorType(), m_irFrame.getFrameIndex());
        const OniGrayscale8Pixel* pDepthRow = (const OniGrayscale8Pixel*)m_irFrame.getData();
        openni::RGB888Pixel* pTexRow = m_pTexMap + m_irFrame.getCropOriginY() * m_nTexMapX;
        int rowSize = m_irFrame.getStrideInBytes() / sizeof(OniGrayscale8Pixel);
        for (int y = 0; y < m_irFrame.getHeight(); ++y)
        {
            const OniGrayscale8Pixel* pDepth = pDepthRow;
            openni::RGB888Pixel* pTex = pTexRow + m_irFrame.getCropOriginX();

            for (int x = 0; x < m_irFrame.getWidth(); ++x, ++pDepth, ++pTex)
            {
                if (*pDepth != 0)
                {
                    uint8_t value = *pDepth;
                    pTex->r = value;
                    pTex->g = value;
                    pTex->b = value;
                }
            }

            pDepthRow += rowSize;
            pTexRow += m_nTexMapX;
        }
    }

    // check if we need to draw depth frame to texture
   if ((m_eViewState == DISPLAY_MODE_OVERLAY ||
        m_eViewState == DISPLAY_MODE_DEPTH) && m_depthFrame.isValid())
	//if(m_depthFrame.isValid())
    {
        // printf("depth %d %d\n", m_depthFrame.getSensorType(), m_depthFrame.getFrameIndex());
     // const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
      //  openni::RGB888Pixel* pTexRow = m_pTexMap + m_depthFrame.getCropOriginY() * m_nTexMapX;
     //   int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
		if (test == 1) 
		{
			//printf(" draw c   %d %d  %d %d\n", m_ptx[m_end[0]], m_pty[m_end[0]], m_depthFrame.getHeight(), m_depthFrame.getWidth());
		
			test += 1;
			int x1 =m_ptx1/2;
			int  y1 = m_pty1/2;
			int x2 = m_ptx2 /2;
			int  y2 = m_pty2/2;
			int z1 = 0;
			int z2 = 0;
		//printf(" pixel1 %d %d   pixel2 %d %d \n",x1,y1,x2,y2);
			 const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
			 openni::RGB888Pixel* pTexRow = m_pTexMap + m_depthFrame.getCropOriginY() * m_nTexMapX;
		      int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

			  for (int y = 0; y < m_depthFrame.getHeight(); ++y)
			  {
				  const openni::DepthPixel* pDepth = pDepthRow;
				  openni::RGB888Pixel* pTex = pTexRow + m_depthFrame.getCropOriginX();

				  for (int x = 0; x < m_depthFrame.getWidth(); ++x, ++pDepth, ++pTex)
				  {
					  if (*pDepth != 0 && *pDepth != 0XFFF)
					  {
						  int nHistValue = m_pDepthHist[*pDepth];
						  pTex->r = nHistValue;
						  pTex->g = nHistValue;
						  pTex->b = 0;
						  if (x == x1 && y == y1)
						  {
							  z1 = nHistValue;
							 // printf("cord1 %d %d  %d\n ", x1, y1, nHistValue);
						  }
						  if (x == x2 && y == y2) {
							//printf("cord2 %d %d  %d\n ", x2, y2, nHistValue);
							z2 = nHistValue;
						 }
					  }

				  }

				  pDepthRow += rowSize;
				  pTexRow += m_nTexMapX;
			  }

			  ComputerCoord(x1 * 2, y1 * 2, z1 , x2 * 2, y2 * 2, z2);
			
			  //printf("distance  %f \n", sqrt( pow((x1-x2) ,2) + pow((y1 - y2), 2)+ pow((z1 - z2), 2)));
		}
	/*	float WorldX;
		float WorldY;
		float WorldZ;
		float depthX=0.0;
		float	depthY=0.0;
		openni::DepthPixel* pDepth;
		openni::DepthPixel depthZ;*/
		//int x = m_ptx[m_end[0]];
		//int  y = m_pty[m_end[0]];
		//convertDepthToWorld(m_depthFrame,  depthX,  depthY,  depthZ, &WorldX, &WorldY, &WorldZ);
		/*
        for (int y = 0; y < m_depthFrame.getHeight(); ++y)
        {
            const openni::DepthPixel* pDepth = pDepthRow;
            openni::RGB888Pixel* pTex = pTexRow + m_depthFrame.getCropOriginX();

            for (int x = 0; x < m_depthFrame.getWidth(); ++x, ++pDepth, ++pTex)
            {
                if (*pDepth != 0 && *pDepth != 0XFFF)
                {
                    int nHistValue = m_pDepthHist[*pDepth];
                    pTex->r = nHistValue;
                    pTex->g = nHistValue;
                    pTex->b = 0;
                }
            }

            pDepthRow += rowSize;
            pTexRow += m_nTexMapX;
        }*/
    }
#if 1
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

    // Display the OpenGL texture map
    glColor4f(1,1,1,1);

    glBegin(GL_QUADS);

    int nXRes = m_width;
    int nYRes = m_height;

    // upper left
    glTexCoord2f(0, 0);
    glVertex2f(0, 0);
    // upper right
    glTexCoord2f((float)nXRes/(float)m_nTexMapX, 0);
    glVertex2f(GL_WIN_SIZE_X, 0);
    // bottom right
    glTexCoord2f((float)nXRes/(float)m_nTexMapX, (float)nYRes/(float)m_nTexMapY);
    glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    // bottom left
    glTexCoord2f(0, (float)nYRes/(float)m_nTexMapY);
    glVertex2f(0, GL_WIN_SIZE_Y);

    glEnd();

    // Swap the OpenGL display buffers
    glutSwapBuffers();
#endif
}

void SampleViewer::onKey(unsigned char key, int /*x*/, int /*y*/)
{
    switch (key)
    {
    case 27:
        m_depthStream.stop();
        m_colorStream.stop();
        m_irStream.stop();
        m_depthStream.destroy();
        m_colorStream.destroy();
        m_irStream.destroy();
        m_device.close();
        openni::OpenNI::shutdown();

        exit (1);
    case '1':
		test = 0;
       // m_eViewState = DISPLAY_MODE_OVERLAY;
      //  m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
        break;
    case '2':
        m_eViewState = DISPLAY_MODE_DEPTH;
        m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
        break;
    case '3':
        m_eViewState = DISPLAY_MODE_IMAGE;
        m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
        break;
    case '4':
        m_eViewState = DISPLAY_MODE_IR;
        m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
        break;
    case 'm':
        m_depthStream.setMirroringEnabled(!m_depthStream.getMirroringEnabled());
        m_colorStream.setMirroringEnabled(!m_colorStream.getMirroringEnabled());
        break;
    }

}

openni::Status SampleViewer::initOpenGL(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
    glutCreateWindow (m_strSampleName);
    // 	glutFullScreen();
    glutSetCursor(GLUT_CURSOR_NONE);

    initOpenGLHooks();

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);

    return openni::STATUS_OK;

}
void SampleViewer::initOpenGLHooks()
{
    glutKeyboardFunc(glutKeyboard);
    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);
}
float SampleViewer::emacord(int flag) {
	float sum = 0;
	if (flag == 0)
	{
		for (int i = 0; i < EMA_NUM; i++) 
			sum += ema_x1[i];
		
	}
	else if (flag == 1)
	{
		for (int i = 0; i < EMA_NUM; i++)
			sum += ema_y1[i];

	}
	else  if (flag ==2) {
		for (int i = 0; i < EMA_NUM; i++)
			sum += ema_x2[i];

	}
	else  if (flag == 3) {
		for (int i = 0; i < EMA_NUM; i++)
			sum += ema_y2[i];

	}
	else  if (flag == 4) {
		for (int i = 0; i < EMA_NUM; i++)
			sum += ema_z1[i];

	}
	else  if (flag == 5) {
		for (int i = 0; i < EMA_NUM; i++)
			sum += ema_z2[i];

	}
	else return sum;
	return sum / EMA_NUM;
}

void SampleViewer::ComputerCoord(int  x1, int y1, int z1, int x2, int y2, int z2) {

	Vector2d t1 = {0,0};
	Vector3d wt1 = m_camera.pixel2world(t1);
	printf("0,0        %06d %06d \n", wt1(0), wt1(1));
	t1 = { 1280,960 }; wt1 = m_camera.pixel2world(t1);
	printf("1280,960   %06d %06d \n", wt1(0), wt1(1));
	t1 = { -640,-480 }; wt1 = m_camera.pixel2world(t1);
	printf("-640,-480    %06d %06d \n", wt1(0), wt1(1));
	t1 = { 640,480 }; wt1 = m_camera.pixel2world(t1);
	printf("640,480    %06d %06d \n", wt1(0), wt1(1));
	t1 = { 0,960 }; wt1 = m_camera.pixel2world(t1);
	printf("0,960      %06d %06d \n", wt1(0), wt1(1));

	t1 = { 1280,0 }; wt1 = m_camera.pixel2world(t1);
	printf("1280,0     %06d %06d \n", wt1(0), wt1(1));
	Vector2d p1;
	p1(0) = x1;
	p1(1) = y1;
	Vector3d w1= m_camera.pixel2world(p1);

	Vector2d p2;
	p2(0) = x2;
	p2(1) = y2;
	Vector3d w2 = m_camera.pixel2world(p2);


	
	p1(0) = z1;
	p1(1) = z1;
	Vector3d wz1 = m_depthcamera.pixel2world(p1);


	p2(0) = z2;
	p2(1) = z2;
	Vector3d wz2 = m_depthcamera.pixel2world(p2);
	


	ema_x1[ema_count] = w1[0];
	ema_y1[ema_count] = w1[1];
	ema_z1[ema_count] = wz1[0];

	ema_x2[ema_count] = w2[0];
	ema_y2[ema_count] = w2[1];
	ema_z2[ema_count] = wz2[0];
	/*float world_x2 = x2 * m_camParam->astColorParam[m_cparamindex].fx + m_camParam->astColorParam[m_cparamindex].cx;
	float world_y2 = y2 * m_camParam->astColorParam[m_cparamindex].fy + m_camParam->astColorParam[m_cparamindex].cy;*/
	ema_count++;
	if (emaout)
	{
	float	world_x1 = emacord(0);
	float	world_y1 = emacord(1);
	float	world_x2 = emacord(2);
	float	world_y2 = emacord(3);
	float	world_z1 = emacord(4);
	float	world_z2 = emacord(5);

	/*	float	world_x1 = w1[0];
		float	world_y1 = w1[1];
		float	world_x2 = w2[0];
		float	world_y2 = w2[1];
		float	world_z1 = wz1[0];
		float	world_z2 = wz2[0];
		float	world_z2 = emacord(5); */
	printf("world1: %f %f %f\n", world_x1, world_y1, world_z1);
	printf("world2: %f %f %f\n", world_x2, world_y2, world_z2);
	printf("x : %f  y: %f  z: %f \n", abs(world_x1 - world_x2), abs(world_y1 - world_y2), abs(world_z1 - world_z2));
	leg_length = para_real*sqrt(para_x*(world_x1 - world_x2)*(world_x1 - world_x2) + para_y*(world_y1 - world_y2)*(world_y1 - world_y2)+ (world_z1 - world_z2)*(world_z1 - world_z2));
	
	//if (leg_length < 40) leg_length = 41;
		printf("LEGLENGTH: %f \n", leg_length);
		mynum.num_d = leg_length;
		//mynum.num_d = 3.14;
	show_float(mynum.num_d);
		unsigned char *temp = new unsigned char[8];//动态创建一个数组
	temp[0] = 65;
	temp[1] = 84;
	//temp[2] = mynum.num_s[3];
	//temp[3] = mynum.num_s[2];
	//temp[4] = mynum.num_s[1];
	//temp[5] = mynum.num_s[0];
	temp[2] = mynum.num_s[0];
	temp[3] = mynum.num_s[1];
	temp[4] = mynum.num_s[2];
	temp[5] = mynum.num_s[3];
	temp[6] = 75;
	temp[7] = 76;
	 mySerialPort.WriteData(temp, 8) ;
	 printf("sent: %X \n", mynum.num_s[0]);
	 //cout<< mynum.num_s << endl;
	cout << mySerialPort.GetBytesInCOM() << endl;//这个函数就是显示返回值函数
	}
	else {
		if (ema_count == EMA_NUM)
			emaout = true;
	}
	if (ema_count == EMA_NUM)
		ema_count = 0;
	//float world_z = z * m_camParam->astDepthParam[m_paramindex].fy;

}