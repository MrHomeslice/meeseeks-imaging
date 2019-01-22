#include <libv4l2.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <algorithm>

#include <fcntl.h>
#include "MeeseeksProperties.h"
#include "networktables/NetworkTable.h"

#define VIDEO_WIDTH  640
#define VIDEO_HEIGHT 360

using namespace cv;
using namespace std;

extern MeeseeksProperties            g_mp;
extern pthread_mutex_t               g_lock;
extern std::atomic<bool>             g_bStopThreads, g_bSimActive, g_bSimFrameReady;
extern std::shared_ptr<NetworkTable> g_targetTable; 

unsigned char g_frameBuffer[VIDEO_WIDTH * VIDEO_HEIGHT * 3];
unsigned int  g_frameBufferSize;
unsigned int  g_frameCount = 0;
double        g_startTime  = 0.0;
bool          g_bNewFrame = false;

const float c_tapeWidth   = 14.5;
const float c_screenWidth = 640.0;			
const float c_tan35       = tan(CV_PI * (35.0 / 180.0));

int g_targetX = -1;
int g_targetY = -1;

typedef struct {
  char *data;
  struct v4l2_buffer info;
} frameData;

typedef struct
{
  RotatedRect rRect;
  Rect        bRect;
  Point2f     rRectPoints[4]; 	
  float       angle;
} TapeData;

typedef struct
{
  TapeData left;
  TapeData right;
  float    heightRatio;
  float    distance;
  float    angle;
} TargetData;

typedef struct {
  Rect        bRect;
  Point2f     rRectPoints[4];   
  float       angle;  
} FloorMarker;

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
              void RunSim();
  protected :
              bool StreamOn();
              bool GetBuffer();
              bool GetSimBuffer();
              void ProcessBuffer();
              bool ReturnBuffer();
              void StreamOff();
              void CheckParams();

              void SaveFrame(Mat &frame);
              void ProcessTape(Mat &frame, Mat &source);
              void ProcessFloor(Mat &frame, Mat &source);              
              void SetCamParam(int param, int value, const char *pName);

              struct v4l2_buffer  m_bufferInfo;
              CameraParams        m_params;
              Scalar 						  m_yellow, m_blue, m_red, m_green;							
              frameData           m_frameBuffer[BUF_COUNT];
              vector<TargetData>  m_tapeTargets;
              vector<FloorMarker> m_floorMarkers;
              int                 m_type, m_fd, m_simFrameLength;
              char      				  m_simFrame[640 * 360 * 4];
};

Camera::Camera()
      : m_yellow(0, 255, 255), m_blue(255, 0, 0), m_red(0, 0, 255), m_green(0, 255, 0)
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

bool XPositionSort(TapeData a, TapeData b) 
{ 
  return (a.rRect.center.x < b.rRect.center.x);
}

void Camera::ProcessTape(Mat &frame, Mat &source)
{
  vector<vector<Point>> contours;
  vector<Vec4i> 				hierarchy;

  static double s_test = 0;

  g_targetTable->PutNumber("X", s_test++);

  findContours(frame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));

  vector<TapeData> tapeVector;
    
  for (int i=0; i<contours.size(); i++) {

    TapeData tapeData;

    tapeData.rRect = minAreaRect(Mat(contours[i]));

    if (tapeData.rRect.size.width * tapeData.rRect.size.height > g_mp.algorithim.minTapeArea) {
          
      tapeData.rRect.points(tapeData.rRectPoints);
        
      Point2f edge1 = Vec2f(tapeData.rRectPoints[1].x, tapeData.rRectPoints[1].y) - Vec2f(tapeData.rRectPoints[0].x, tapeData.rRectPoints[0].y);
      Point2f edge2 = Vec2f(tapeData.rRectPoints[2].x, tapeData.rRectPoints[2].y) - Vec2f(tapeData.rRectPoints[1].x, tapeData.rRectPoints[1].y);

      Point2f usedEdge = (norm(edge2) > norm(edge1)) ? edge2 : edge1;
      Point2f reference = Vec2f(1,0);

      tapeData.angle = 180.0f/CV_PI * acos((reference.x * usedEdge.x + reference.y * usedEdge.y) / (norm(reference) * norm(usedEdge)));	

      if (tapeData.angle >= g_mp.algorithim.minTapeAngle && tapeData.angle <= g_mp.algorithim.maxTapeAngle) {

        tapeData.bRect = boundingRect(Mat(contours[i]));		

        tapeVector.push_back(tapeData);

        if (g_mp.algorithim.displayType == 4) {

          for (unsigned int j=0; j<4; ++j)
            line(source, tapeData.rRectPoints[j], tapeData.rRectPoints[(j + 1) % 4], m_red);

          circle(source, tapeData.rRect.center, 2, m_red);        
        }        
      }
    }	
  }

  std::sort(tapeVector.begin(), tapeVector.end(), XPositionSort);

  int tapeSize = tapeVector.size();

  for (int l=0; l<tapeSize-1; l++) {
    
    if (tapeVector[l].angle < 90.0) {
      
      for (int r = l+1; r<tapeSize; r++) {
        
        if (tapeVector[r].angle > 90.0) {

          float leftHeight  = tapeVector[l].bRect.height;
          float rightHeight = tapeVector[r].bRect.height;		
          float heightRatio = rightHeight / leftHeight;

          if (heightRatio >= g_mp.algorithim.minTapeHeightRatio && heightRatio <= g_mp.algorithim.maxTapeHeightRatio) {
            float yDistance = fabs(tapeVector[l].rRect.center.y - tapeVector[r].rRect.center.y);
            float yDistanceFactor = yDistance / leftHeight;

            if (yDistanceFactor >= g_mp.algorithim.minTapeYDistRatio && yDistanceFactor <= g_mp.algorithim.maxTapeYDistRatio) {
              float xDistance = fabs(tapeVector[l].rRect.center.x - tapeVector[r].rRect.center.x);
              float xDistanceFactor = xDistance / leftHeight;

              if (xDistanceFactor >= g_mp.algorithim.minTapeXDistRatio && xDistanceFactor <= g_mp.algorithim.maxTapeXDistRatio) {
                TargetData t;
                t.left        = tapeVector[l];
                t.right       = tapeVector[r];
                float width   = t.right.bRect.br().x - t.left.bRect.tl().x;
                float side    = (c_tapeWidth * c_screenWidth) / (2.0 * width);
                t.distance    = side / c_tan35;
                t.heightRatio = heightRatio;
                m_tapeTargets.push_back(t);														
                break;
              }
            }
          }
        }
      }
    }
  }

  if (g_mp.algorithim.displayType == 5) {
    for (unsigned int i = 0; i < m_tapeTargets.size();i++) {			
      rectangle(source, m_tapeTargets[i].left.bRect.tl(), m_tapeTargets[i].right.bRect.br(), cv::Scalar(0, 255, 0));
      line(source, m_tapeTargets[i].left.rRectPoints[0], m_tapeTargets[i].right.rRectPoints[0], m_red);
      line(source, m_tapeTargets[i].left.rRectPoints[3], m_tapeTargets[i].right.rRectPoints[1], m_red);
      char text[32];
      sprintf(text, "%.3f", m_tapeTargets[i].distance);
      putText(source, text, m_tapeTargets[i].left.bRect.tl(), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 0, 0));			
    }
  }
}

void Camera::ProcessFloor(Mat &frame, Mat &source)
{
  vector<vector<Point>> contours;
  vector<Vec4i> 				hierarchy;

  findContours(frame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));

  for (int i=0; i<contours.size(); i++) {

    RotatedRect rRect = minAreaRect(Mat(contours[i]));

    if (rRect.size.width * rRect.size.height > g_mp.algorithim.minFloorArea) {

      Vec4f lines;    

      fitLine(contours[i], lines, 2, 0, 0.01 ,0.01);
                  
      int lefty  = (-lines[2]                * lines[1] / lines[0]) + lines[3];
      int righty = ((source.cols - lines[2]) * lines[1] / lines[0]) + lines[3];

      FloorMarker floorMarker;

      floorMarker.angle = (180.0f/CV_PI) * atan2(righty - lefty, source.cols);
      
      if (floorMarker.angle < 0) floorMarker.angle += 180.0f;

      floorMarker.bRect = boundingRect(Mat(contours[i]));      
      
      rRect.points(floorMarker.rRectPoints);
 
      m_floorMarkers.push_back(floorMarker);
    }
  }

  if (g_mp.algorithim.displayType == 5) {

    for (int i=0; i<m_floorMarkers.size(); i++) {
      for (unsigned int j=0; j<4; ++j)
        line(source, m_floorMarkers[i].rRectPoints[j], m_floorMarkers[i].rRectPoints[(j + 1) % 4], m_red);

      char text[32];
      sprintf(text, "%.3f", m_floorMarkers[i].angle);
      putText(source, text, m_floorMarkers[i].bRect.tl(), FONT_HERSHEY_PLAIN, 1.0, Scalar(0, 0, 0));			
    }
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
  m_tapeTargets.clear();
  m_floorMarkers.clear();
  

  if (g_mp.algorithim.displayType == 0) {
    pthread_mutex_lock(&g_lock);
    if (!g_bSimActive) {
      memcpy(g_frameBuffer, m_frameBuffer[m_bufferInfo.index].data, m_bufferInfo.bytesused);
      g_frameBufferSize = m_bufferInfo.bytesused;
    } 
    else {
      memcpy(g_frameBuffer, m_simFrame, m_simFrameLength);
      g_frameBufferSize = m_simFrameLength;
    }
    g_bNewFrame = true;
    pthread_mutex_unlock(&g_lock);
  }

  Mat mjpegData(1, g_bSimActive ? m_simFrameLength : m_bufferInfo.bytesused, CV_8UC1, g_bSimActive ? m_simFrame : m_frameBuffer[m_bufferInfo.index].data);
    
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

    bool bContinue = true;
      
    if (g_mp.algorithim.colorSpace[0] == 1) {			
      try {
        cvtColor(erodeDilateData, hsvFrame, COLOR_BGR2HSV);	
      }
      catch( Exception& e )
      {
        //const char* err_msg = e.what();
        //printf("%s\n", err_msg);
        return;
      }
      inRange(hsvFrame, Scalar(g_mp.algorithim.minRed[0], g_mp.algorithim.minGreen[0], g_mp.algorithim.minBlue[0]), Scalar(g_mp.algorithim.maxRed[0], g_mp.algorithim.maxGreen[0], g_mp.algorithim.maxBlue[0]), grayFrame);		
    }
    else
      inRange(erodeDilateData, Scalar(g_mp.algorithim.minRed[0], g_mp.algorithim.minGreen[0], g_mp.algorithim.minBlue[0]), Scalar(g_mp.algorithim.maxRed[0], g_mp.algorithim.maxGreen[0], g_mp.algorithim.maxBlue[0]), grayFrame);

    if (g_mp.algorithim.displayType == 3 && g_mp.algorithim.processingType == 1)
      SaveFrame(grayFrame);      

    ProcessTape(grayFrame, data);

    if (g_mp.algorithim.colorSpace[1] == 1) {			
      inRange(hsvFrame, Scalar(g_mp.algorithim.minRed[1], g_mp.algorithim.minGreen[1], g_mp.algorithim.minBlue[1]), Scalar(g_mp.algorithim.maxRed[1], g_mp.algorithim.maxGreen[1], g_mp.algorithim.maxBlue[1]), grayFrame);		
    }
    else
      inRange(erodeDilateData, Scalar(g_mp.algorithim.minRed[1], g_mp.algorithim.minGreen[1], g_mp.algorithim.minBlue[1]), Scalar(g_mp.algorithim.maxRed[1], g_mp.algorithim.maxGreen[1], g_mp.algorithim.maxBlue[1]), grayFrame);   

    if (g_mp.algorithim.displayType == 3 && g_mp.algorithim.processingType == 2)
      SaveFrame(grayFrame);  

    ProcessFloor(grayFrame, data);      

    if (g_mp.algorithim.displayType >= 4)
      SaveFrame(data);  
  }

  g_frameCount++;
}

bool GetSimFrame(char *pSimFrame, int &length);

bool Camera::GetSimBuffer()
{
  do {
    if (g_bSimFrameReady) {			
      if (GetSimFrame(m_simFrame, m_simFrameLength)) {
        g_bSimFrameReady = false;
        return true;			
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

  } while (1);

  return false;
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

void Camera::RunSim()
{
  int loopCount = 0;

  do	{
    if (!GetSimBuffer())
      break;

    ProcessBuffer();

  } while (!g_bStopThreads);

  printf("Camera thread terminated\n");
}

void CameraThread()
{
  Camera camera;

  if (g_bSimActive)
    camera.RunSim();
  else
    camera.Run();
}
