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
#ifndef _ONI_SAMPLE_VIEWER_H_
#define _ONI_SAMPLE_VIEWER_H_

#include <OpenNI.h>
#include<opencv2/opencv.hpp>
#include "AXonLink.h"
#include "camera.h"
//#include "camera.h"
#define MAX_DEPTH 10000

enum DisplayModes
{
    DISPLAY_MODE_OVERLAY,
    DISPLAY_MODE_DEPTH,
    DISPLAY_MODE_IMAGE,
    DISPLAY_MODE_IR,
};
#define PT_NUM 10
#define EMA_NUM 5
class SampleViewer
{
public:
    SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color, openni::VideoStream& ir);
    virtual ~SampleViewer();

    virtual openni::Status init(int argc,char **argv,AXonLinkCamParam* param,int paramindex);
    virtual openni::Status run();	//Does not return
	AXonLinkCamParam* m_camParam;
	int m_cparamindex;
	void ComputerCoord(int x1 ,int y1, int z1, int x2, int y2, int z2);
protected:
    virtual void display();
    virtual void displayPostDraw(){};	// Overload to draw over the screen image

    virtual void onKey(unsigned char key, int x, int y);

    virtual openni::Status initOpenGL(int argc, char **argv);
    void initOpenGLHooks();

    openni::VideoFrameRef		m_depthFrame;
    openni::VideoFrameRef		m_colorFrame;
    openni::VideoFrameRef		m_irFrame;

    openni::Device&			m_device;
    openni::VideoStream&			m_depthStream;
    openni::VideoStream&			m_colorStream;
    openni::VideoStream&			m_irStream;
    openni::VideoStream**		m_streams;
	//Camera mcamera;
	int m_ptx[PT_NUM]; int m_pty[PT_NUM];
	int m_pointw[PT_NUM];
	int m_end[2];
	int m_dist[PT_NUM*(PT_NUM - 1) / 2];
	int m_ptx1;
		int	m_pty1;
		int	m_ptx2;
		int	m_pty2;
	int ema_count = 0;
	float ema_x1[EMA_NUM];
	float ema_y1[EMA_NUM];
	float ema_z1[EMA_NUM];
	float ema_x2[EMA_NUM];
	float ema_y2[EMA_NUM];
	float ema_z2[EMA_NUM];
	bool emaout = false;
	void getPoint(cv::Mat&);
	void setPoint(int y, int x, int w);
	void dumpPoint();
	void findEnd();
//	double SampleViewer::calDepth(const Point p_p);
	float emacord(int flag);
	Camera m_camera;
	Camera m_depthcamera;
private:
    SampleViewer(const SampleViewer&);
    SampleViewer& operator=(SampleViewer&);

    static SampleViewer* ms_self;
    static void glutIdle();
    static void glutDisplay();
    static void glutKeyboard(unsigned char key, int x, int y);

    float			m_pDepthHist[MAX_DEPTH];
    char			m_strSampleName[ONI_MAX_STR];
    unsigned int		m_nTexMapX;
    unsigned int		m_nTexMapY;
    DisplayModes		m_eViewState;
    openni::RGB888Pixel*	m_pTexMap;
    int			m_width;
    int			m_height;
};


#endif // _ONI_SAMPLE_VIEWER_H_
