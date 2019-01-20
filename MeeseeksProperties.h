#ifndef SRC_MEESEEKSPROPERTIES_H_
#define SRC_MEESEEKSPROPERTIES_H_

#include <map>
#include <vector>
#include <string>
#include <set>
#include <atomic>

using namespace std;

typedef map<string, string> StringToStringMap;
typedef vector<string>      StringVector;
typedef map<int, string>    IntToStringMap;
typedef set<string>					StringSet;

typedef struct {
  int displayType;
  int minWidth;
  int minHeight;
  int maxWidth;
  int maxHeight;
  int minRed, maxRed;
  int minGreen, maxGreen;
  int minBlue, maxBlue;
  int jpgQuality;
  int colorSpace;
  int blur;
  int erode;
  int dilate;
  int minTapeArea;
  int minTapeAngle;
  int maxTapeAngle;
  double minTapeHeightRatio;
  double maxTapeHeightRatio;
  double minTapeYDistRatio;
  double maxTapeYDistRatio;
  double minTapeXDistRatio;
  double maxTapeXDistRatio;
} AlgorithimParams;

typedef struct {
   int autoExposure;
   int exposureAbsolute;
   int autoWhiteBalance;
   int whiteBalanceTemperature;
   int brightness;
   int gain;
   int autoFocus;
   int zoom;
   int contrast;
   int saturation;
   int sharpness;
   int focus;
} CameraParams;

#define ALGORITHIM_DISPLAY_TYPE  "algorithim-display-type"
#define ALGORITHIM_MIN_WIDTH  	 "algorithim-min-width"
#define ALGORITHIM_MIN_HEIGHT    "algorithim-min-height"
#define ALGORITHIM_MAX_WIDTH     "algorithim-max-width"
#define ALGORITHIM_MAX_HEIGHT    "algorithim-max-height"
#define ALGORITHIM_MIN_RED       "algorithim-min-red"
#define ALGORITHIM_MIN_GREEN     "algorithim-min-green"
#define ALGORITHIM_MIN_BLUE      "algorithim-min-blue"
#define ALGORITHIM_MAX_RED       "algorithim-max-red"
#define ALGORITHIM_MAX_GREEN     "algorithim-max-green"
#define ALGORITHIM_MAX_BLUE      "algorithim-max-blue"
#define ALGORITHIM_JPEG_QUALITY  "algorithim-jpeg-quality"
#define ALGORITHIM_COLOR_SPACE   "algorithim-color-space"
#define ALGORITHIM_BLUR          "algorithim-blur"
#define ALGORITHIM_ERODE         "algorithim-erode"
#define ALGORITHIM_DILATE        "algorithim-dilate"

#define ALGORITHIM_MIN_TAPE_AREA         "algorithim-min-tape-area"
#define ALGORITHIM_MIN_TAPE_ANGLE        "algorithim-min-tape-angle"
#define ALGORITHIM_MAX_TAPE_ANGLE        "algorithim-max-tape-angle"
#define ALGORITHIM_MIN_TAPE_HEIGHT_RATIO "algorithim-min-tape-height-ratio"
#define ALGORITHIM_MAX_TAPE_HEIGHT_RATIO "algorithim-max-tape-height-ratio"
#define ALGORITHIM_MIN_TAPE_Y_DIST_RATIO "algorithim-min-tape-y-dist-ratio"
#define ALGORITHIM_MAX_TAPE_Y_DIST_RATIO "algorithim-max-tape-y-dist-ratio"
#define ALGORITHIM_MIN_TAPE_X_DIST_RATIO "algorithim-min-tape-x-dist-ratio"
#define ALGORITHIM_MAX_TAPE_X_DIST_RATIO "algorithim-max-tape-x-dist-ratio"

#define ALGORITHIM_DEFAULT_DISPLAY_TYPE  1
#define ALGORITHIM_DEFAULT_MIN_WIDTH     10
#define ALGORITHIM_DEFAULT_MIN_HEIGHT    10
#define ALGORITHIM_DEFAULT_MAX_WIDTH     100
#define ALGORITHIM_DEFAULT_MAX_HEIGHT    100
#define ALGORITHIM_DEFAULT_MIN_RED       0
#define ALGORITHIM_DEFAULT_MIN_GREEN     0
#define ALGORITHIM_DEFAULT_MIN_BLUE      0
#define ALGORITHIM_DEFAULT_MAX_RED       255
#define ALGORITHIM_DEFAULT_MAX_GREEN     255
#define ALGORITHIM_DEFAULT_MAX_BLUE      255
#define ALGORITHIM_DEFAULT_JPEG_QUALITY  100
#define ALGORITHIM_DEFAULT_COLOR_SPACE   0
#define ALGORITHIM_DEFAULT_BLUR          3
#define ALGORITHIM_DEFAULT_ERODE         3
#define ALGORITHIM_DEFAULT_DILATE        3

#define ALGORITHIM_DEFAULT_MIN_TAPE_AREA         200
#define ALGORITHIM_DEFAULT_MIN_TAPE_ANGLE         65
#define ALGORITHIM_DEFAULT_MAX_TAPE_ANGLE        120
#define ALGORITHIM_DEFAULT_MIN_TAPE_HEIGHT_RATIO 0.8
#define ALGORITHIM_DEFAULT_MAX_TAPE_HEIGHT_RATIO 1.2
#define ALGORITHIM_DEFAULT_MIN_TAPE_Y_DIST_RATIO 0.0
#define ALGORITHIM_DEFAULT_MAX_TAPE_Y_DIST_RATIO 0.2
#define ALGORITHIM_DEFAULT_MIN_TAPE_X_DIST_RATIO 1.0
#define ALGORITHIM_DEFAULT_MAX_TAPE_X_DIST_RATIO 2.5

#define CAMERA_AUTO_EXPOSURE      "camera-auto-exposure"
#define CAMERA_EXPOSURE_ABSOLUTE  "camera-exposure-absolute"
#define CAMERA_AUTO_WHITE_BALANCE "camera-auto-white-balance"
#define CAMERA_WHITE_BALANCE_TEMP "camera-white-balance-temperature"
#define CAMERA_BRIGHTNESS         "camera-brightness"
#define CAMERA_SATURATION         "camera-saturation"
#define CAMERA_GAIN               "camera-gain"
#define CAMERA_SHARPNESS          "camera-sharpness"
#define CAMERA_AUTO_FOCUS         "camera-auto-focus"
#define CAMERA_ZOOM               "camera-zoom"
#define CAMERA_CONTRAST           "camera-contrast"
#define CAMERA_FOCUS              "camera-focus"

#define CAMERA_DEFAULT_AUTO_EXPOSURE        3
#define CAMERA_DEFAULT_EXPOSURE_ABSOLUTE    100
#define CAMERA_DEFAULT_AUTO_WHITE_BALANCE   0
#define CAMERA_DEFAULT_WHITE_BALANCE_TEMP   100
#define CAMERA_DEFAULT_BRIGHTNESS           100
#define CAMERA_DEFAULT_GAIN                 10
#define CAMERA_DEFAULT_AUTO_FOCUS           0
#define CAMERA_DEFAULT_ZOOM                 100
#define CAMERA_DEFAULT_CONTRAST             128 
#define CAMERA_DEFAULT_SATURATION           128
#define CAMERA_DEFAULT_SHARPNESS            128
#define CAMERA_DEFAULT_FOCUS                0

#define SERVER_PORT "camera-server-port"
#define NETWORK_TABLE_ADDRESS "network-table-address"
#define SIMULATION_ACTIVE "simulation-active"

#define DEFAULT_SERVER_PORT 5800
#define DEFAULT_NETWORK_TABLE_ADDRESS "10.74.0.2"
#define DEFAULT_SIMULATION_ACTIVE 0

class MeeseeksProperties
{
  public    : MeeseeksProperties();

              void Initialize();

              void LoadFromFile(const char *pFileName);
              void LoadFromString(const std::string &str);
              void SaveToString(std::string &str);
              void SaveToFile(const char *pFileName);
              void SetProperty(const std::string &nameValue);
              bool NewProperties();

              CameraParams      camera;
              AlgorithimParams  algorithim;
              std::string       networkTableAddress;
              int               serverPort, simulationPort;
              bool              bSimActive;
  protected :
              const char *GetString(const char *pName, const char *pDefaultValue);
              int         GetInt(const char *pName, int defaultValue);
              double      GetDouble(const char *pName, double defaultValue);
              const char *IntToString(int intValue, char *pStringValue);
              const char *DoubleToString(double doubleValue, char *pStringValue);
              void        SetProperties();

              StringToStringMap nameValueMap;
              IntToStringMap    lineNameMap;
              StringVector      lines;
              std::atomic<int>  newProperties;

};

#endif
