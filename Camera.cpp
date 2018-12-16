#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <atomic>

#include <fcntl.h>
#include "MeeseeksProperties.h"
#include "networktables/NetworkTable.h"

#define VIDEO_WIDTH  640
#define VIDEO_HEIGHT 360

using namespace cv;
using namespace std;

extern MeeseeksProperties g_mp;
extern pthread_mutex_t    g_lock;
extern std::atomic<bool>             g_bStopThreads;
extern std::shared_ptr<NetworkTable> g_targetTable; 

unsigned char g_frameBuffer[VIDEO_WIDTH * VIDEO_HEIGHT * 3];
unsigned int  g_frameBufferSize;
unsigned int  g_frameCount = 0;
double        g_startTime  = 0.0;
bool          g_bNewFrame = false;

int g_targetX = -1;
int g_targetY = -1;

typedef struct {
	char *data;
	struct v4l2_buffer info;
} frameData;

#define BUF_COUNT  8

double GetTimeStamp()
{
	struct timeval tv;

	gettimeofday(&tv, NULL);

	double seconds      = (double)tv.tv_sec;
	double microSeconds = (double)tv.tv_usec;
	return seconds + microSeconds / 1000000.0;
}

class Camera
{
  public    : Camera();
              void Run();
  protected :
              bool StreamOn();
              bool GetBuffer();
              void ProcessBuffer();
              bool ReturnBuffer();
              void StreamOff();
							void CheckParams();

              void SaveFrame(Mat &frame);
              void ProcessFrame(Mat &frame, Mat &source);
							void SetCamParam(int param, int value, const char *pName);

              struct v4l2_buffer m_bufferInfo;
							CameraParams       m_params;
              frameData          m_frameBuffer[BUF_COUNT];
              int                m_type, m_fd;
};

Camera::Camera()
{
	m_fd   = -1;
	m_type = 0;
	
	memset(&m_bufferInfo, 0, sizeof(m_bufferInfo));
	
	for (int i=0; i<BUF_COUNT; i++)
		m_frameBuffer[i].data = NULL;

	m_params.autoExposure             = -10000;
	m_params.exposureAbsolute					= -10000;
	m_params.autoWhiteBalance					= -10000;
	m_params.whiteBalanceTemperature	= -10000;
	m_params.brightness								= -10000;
	m_params.gain											= -10000;
	m_params.autoFocus								= -10000;
}

void Camera::SetCamParam(int param, int value, const char *pName)
{
	v4l2_control params;
	v4l2_queryctrl qc;

	memset(&qc, 0, sizeof(qc));

	qc.id = param;

	int status = ioctl(m_fd, VIDIOC_QUERYCTRL, &qc);

	if (status == -1) {
		printf("Error setting %s. ioctl VIDIOC_QUERYCTRL errno = %d\r\n", pName, errno);
		return;
	}

	if (qc.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf("V4L2_CTRL_FLAG_DISABLED\r\n");
		return;
	}

	params.id = param;
	params.value = value;

	status = ioctl(m_fd, VIDIOC_S_CTRL, &params);

	if (status == -1)
		printf("Error setting %s. ioctl VIDIOC_S_CTRL errno = %d\r\n", pName, errno);
}

bool Camera::StreamOn()
{
  if ((m_fd = open("/dev/video0", O_RDWR)) < 0) {	
    printf("Cannot open /dev/video0\n");	
    return false;
	}

	struct v4l2_format format;

	memset(&format, 0, sizeof(format));

	format.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	format.fmt.pix.width       = VIDEO_WIDTH;
	format.fmt.pix.height      = VIDEO_HEIGHT;

	if (ioctl(m_fd, VIDIOC_S_FMT, &format) < 0) {
    printf("Failure setting video format\n");
    close(m_fd);
    return false;
  }

	struct v4l2_requestbuffers bufrequest;

	memset(&bufrequest, 0, sizeof(bufrequest));
	
  bufrequest.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count  = BUF_COUNT;

	if (ioctl(m_fd, VIDIOC_REQBUFS, &bufrequest) < 0) {
    printf("Failure requesting buffers\n");
    close(m_fd);
    return false;
  }

	for (int i = 0; i<BUF_COUNT; i++) {
		memset(&m_frameBuffer[i], 0, sizeof(frameData));

		m_frameBuffer[i].info.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		m_frameBuffer[i].info.memory = V4L2_MEMORY_MMAP;
		m_frameBuffer[i].info.index  = i;

		if (ioctl(m_fd, VIDIOC_QUERYBUF, &m_frameBuffer[i].info) < 0) {
      printf("Failure querying buffer %d\n", i);
      close(m_fd);
      return false;
		}

		m_frameBuffer[i].data = (char *) mmap(NULL, m_frameBuffer[i].info.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, m_frameBuffer[i].info.m.offset);

		if (m_frameBuffer[i].data == MAP_FAILED) {
      printf("Failure during mmap on buffer %d", i);
	    for (int j = 0; j<i; j++) {
		    if (m_frameBuffer[j].data)
			    munmap(m_frameBuffer[j].data, m_frameBuffer[j].info.length);
      }
      close(m_fd);
      return false;
	  }
		memset(m_frameBuffer[i].data, 0, m_frameBuffer[i].info.length);
	}

	for (int i = 0; i<BUF_COUNT; i++) {
		if (ioctl(m_fd, VIDIOC_QBUF, &m_frameBuffer[i].info) < 0) {
      printf("Error queuing buffer %d\n", i);
	    for (int j = 0; j<BUF_COUNT; j++) {
		    if (m_frameBuffer[j].data)
			    munmap(m_frameBuffer[j].data, m_frameBuffer[j].info.length);
      }
			close(m_fd);
      return false;
		}
	}

	m_type = m_frameBuffer[0].info.type;

	if (ioctl(m_fd, VIDIOC_STREAMON, &m_type) < 0) {
    printf("Error turning on stream\n");
	  for (int i = 0; i<BUF_COUNT; i++) {
		  if (m_frameBuffer[i].data)
			  munmap(m_frameBuffer[i].data, m_frameBuffer[i].info.length);
    }
		close(m_fd);
    return false;
	}

 	g_frameBufferSize = 0;
	
	g_startTime = GetTimeStamp();
}

bool Camera::GetBuffer()
{
	memset(&m_bufferInfo, 0, sizeof(m_bufferInfo));
	
  m_bufferInfo.type = m_type;

	if (ioctl(m_fd, VIDIOC_DQBUF, &m_bufferInfo) < 0) {
    printf("Error dequing buffer\n");
    return false;
  }

  return true;  
}

void Camera::SaveFrame(Mat &frame)
{
	vector<uchar> buf;
	vector<int> params;

	params.push_back(CV_IMWRITE_JPEG_QUALITY);
	params.push_back(g_mp.algorithim.jpgQuality);

	imencode(".jpg", frame, buf, params);

	pthread_mutex_lock(&g_lock);
	memcpy(g_frameBuffer, buf.data(), buf.size());
	g_frameBufferSize = buf.size();
	g_bNewFrame = true;
	pthread_mutex_unlock(&g_lock);
}

void Camera::ProcessFrame(Mat &frame, Mat &source)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(frame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));

	vector<vector<Point> > contours_poly(contours.size());
	
	double maxArea = -1000.0;

	Rect boundRect;

	for (int i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		Moments moment = moments(contours[i]);
		double area = moment.m00;
		if (area > maxArea) {
			maxArea = area;
			boundRect = boundingRect(Mat(contours_poly[i]));
		}
	}
	if (maxArea != -1000.0) {
		g_targetX = boundRect.x + boundRect.width  / 2;
		g_targetY = boundRect.y + boundRect.height / 2;	
   	g_targetTable->PutNumber("X", g_targetX);
   	g_targetTable->PutNumber("Y", g_targetY);
	}

	if (g_mp.algorithim.displayType == 4) {
		Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
		cv::Scalar green(0, 255, 0);
		for (int i = 0; i< contours.size(); i++)
			drawContours(drawing, contours_poly, i, green, 1, 8, vector<Vec4i>(), 0, Point());
		SaveFrame(drawing);
	}
	else if (g_mp.algorithim.displayType == 5) {
		cv::Scalar green(0, 255, 0);
		rectangle(source, boundRect, green, 2, 8, 0);		
		SaveFrame(source);
	}
}

void ErodeDilate(Mat &input, Mat &output)
{
	Mat mat1, mat2;

	if (g_mp.algorithim.erode == 0) {
		if (g_mp.algorithim.dilate == 0) {
			output = input;
			return;
		}
		Mat dilateElement = getStructuringElement(MORPH_RECT, Size(g_mp.algorithim.dilate, g_mp.algorithim.dilate));

		dilate(input, mat1,  dilateElement);
		dilate(mat1, output, dilateElement);
	}
	else {
		if (g_mp.algorithim.dilate == 0) {		
			Mat erodeElement  = getStructuringElement(MORPH_RECT, Size(g_mp.algorithim.erode, g_mp.algorithim.erode));

			erode(input, mat1, erodeElement);
			erode(mat1,  output, erodeElement);		
		}
		else {		
			Mat erodeElement  = getStructuringElement(MORPH_RECT, Size(g_mp.algorithim.erode, g_mp.algorithim.erode));

			erode(input, mat1, erodeElement);
			erode(mat1,  mat2, erodeElement);		

			Mat dilateElement = getStructuringElement(MORPH_RECT, Size(g_mp.algorithim.dilate, g_mp.algorithim.dilate));

			dilate(mat2, mat1,   dilateElement);
			dilate(mat1, output, dilateElement);
		}
	}
}

void Camera::ProcessBuffer()
{
  if (g_mp.algorithim.displayType == 0) {
	  pthread_mutex_lock(&g_lock);
		memcpy(g_frameBuffer, m_frameBuffer[m_bufferInfo.index].data, m_bufferInfo.bytesused);
		g_frameBufferSize = m_bufferInfo.bytesused;
		g_bNewFrame = true;
		pthread_mutex_unlock(&g_lock);
	}

	Mat mjpegData(1, m_bufferInfo.bytesused, CV_8UC1, m_frameBuffer[m_bufferInfo.index].data);
		
  Mat data = imdecode(mjpegData, -1);
		
	if (data.data != NULL) {
		Mat grayFrame, hsvFrame, blurData, erodeDilateData;

		int kernelSzie = g_mp.algorithim.blur * 2 + 1;

		GaussianBlur(data, blurData, Size(kernelSzie, kernelSzie), 0, 0 );	
		
		if (g_mp.algorithim.displayType == 1)
			SaveFrame(blurData);

		ErodeDilate(blurData, erodeDilateData);

		if (g_mp.algorithim.displayType == 2)
			SaveFrame(erodeDilateData);
			
		if (g_mp.algorithim.colorSpace == 1) {
			cvtColor(erodeDilateData, hsvFrame, COLOR_BGR2HSV);	
	   	inRange(hsvFrame, Scalar(g_mp.algorithim.minRed, g_mp.algorithim.minGreen, g_mp.algorithim.minBlue), Scalar(g_mp.algorithim.maxRed, g_mp.algorithim.maxGreen, g_mp.algorithim.maxBlue), grayFrame);
		}
		else
			inRange(erodeDilateData, Scalar(g_mp.algorithim.minRed, g_mp.algorithim.minGreen, g_mp.algorithim.minBlue), Scalar(g_mp.algorithim.maxRed, g_mp.algorithim.maxGreen, g_mp.algorithim.maxBlue), grayFrame);

		if (g_mp.algorithim.displayType == 3)
			SaveFrame(grayFrame);

		ProcessFrame(grayFrame, data);
  }

	g_frameCount++;
}

bool Camera::ReturnBuffer()
{
		if (ioctl(m_fd, VIDIOC_QBUF, &m_bufferInfo) < 0)
      return false;

    return true;
}

void Camera::StreamOff()
{
	if (ioctl(m_fd, VIDIOC_STREAMOFF, &m_type) < 0)
		printf("Error turning off streaming\n");

	for (int i = 0; i<BUF_COUNT; i++) {
		if (m_frameBuffer[i].data) {
			munmap(m_frameBuffer[i].data, m_frameBuffer[i].info.length);
			m_frameBuffer[i].data = NULL;
		}
	}

  if (m_fd) {
    close(m_fd);
		m_fd = 0;
	}
}

void Camera::CheckParams()
{
	printf("Checking params ...\n");

	if (g_mp.camera.autoExposure != m_params.autoExposure) {
		printf("Setting auto exposure to %d\n", g_mp.camera.autoExposure);
		SetCamParam(V4L2_CID_EXPOSURE_AUTO, g_mp.camera.autoExposure, "Auto Exposure");
	}

	if (g_mp.camera.exposureAbsolute != m_params.exposureAbsolute) {		
		printf("Setting absolute exposure to %d\n", g_mp.camera.exposureAbsolute);
		SetCamParam(V4L2_CID_EXPOSURE_ABSOLUTE, g_mp.camera.exposureAbsolute, "Absolute Exposure");
	}

	if (g_mp.camera.autoWhiteBalance != m_params.autoWhiteBalance) {		
		printf("Setting auto whtie balance to %d\n", g_mp.camera.autoWhiteBalance);
		SetCamParam(V4L2_CID_AUTO_WHITE_BALANCE, g_mp.camera.autoWhiteBalance, "Auto White Balance");
	}

	if (g_mp.camera.whiteBalanceTemperature != m_params.whiteBalanceTemperature) {	
		printf("Setting white balance temperature to %d\n", g_mp.camera.whiteBalanceTemperature);
		SetCamParam(V4L2_CID_WHITE_BALANCE_TEMPERATURE, g_mp.camera.whiteBalanceTemperature, "White Balance Temperature");
	}

	if (g_mp.camera.brightness != m_params.brightness) {		
		printf("Setting brightness to %d\n", g_mp.camera.brightness);
		SetCamParam(V4L2_CID_BRIGHTNESS, g_mp.camera.brightness, "Brightness");
	}

	if (g_mp.camera.gain != m_params.gain) {		
		printf("Setting gain to %d\n", g_mp.camera.gain);
		SetCamParam(V4L2_CID_GAIN, g_mp.camera.gain, "Gain");
	}

	if (g_mp.camera.autoFocus != m_params.autoFocus) {	
		printf("Setting auto focus to %d\n", g_mp.camera.autoFocus);
		SetCamParam(V4L2_CID_FOCUS_AUTO, g_mp.camera.autoFocus, "Auto Focus");
	}	

	if (g_mp.camera.zoom != m_params.zoom) {	
		printf("Setting zoom to %d\n", g_mp.camera.zoom);
		SetCamParam(V4L2_CID_ZOOM_ABSOLUTE, g_mp.camera.zoom, "Zoom");
	}

	if (g_mp.camera.contrast != m_params.contrast) {	
		printf("Setting contrast to %d\n", g_mp.camera.contrast);
		SetCamParam(V4L2_CID_CONTRAST, g_mp.camera.contrast, "Contrast");
	}

	if (g_mp.camera.sharpness != m_params.sharpness) {	
		printf("Setting sharpness to %d\n", g_mp.camera.sharpness);
		SetCamParam(V4L2_CID_SHARPNESS, g_mp.camera.sharpness, "Sharpness");
	}

	if (g_mp.camera.saturation != m_params.saturation) {	
		printf("Setting saturation to %d\n", g_mp.camera.saturation);
		SetCamParam(V4L2_CID_SATURATION, g_mp.camera.saturation, "Saturation");
	}	

	if (g_mp.camera.focus != m_params.focus) {	
		printf("Setting focus to %d\n", g_mp.camera.focus);
		SetCamParam(V4L2_CID_FOCUS_ABSOLUTE, g_mp.camera.focus, "Focus");
	}						

	m_params = g_mp.camera;
}

void Camera::Run()
{
	int loopCount = 0;

  StreamOn();

	do	{
    if (!GetBuffer())
      break;

    ProcessBuffer();

    if (!ReturnBuffer()) 
      break;

		if (g_mp.NewProperties())
			CheckParams();

	} while (!g_bStopThreads);

  StreamOff();

	printf("Camera thread terminated\n");
}

void CameraThread()
{
  Camera camera;

  camera.Run();
}
