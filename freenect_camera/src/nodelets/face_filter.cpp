#include "face_filter.h"
#include <fstream>

using namespace std;

namespace freenect_camera
{
  void FaceFilter::SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data)
  {
    SaveDepthImageAsCsv(width, height, data, GenerateTempFilePath());
  }

  void FaceFilter::SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data, const string& filePath)
  {
    ofstream ofs(filePath.c_str(), ofstream::out);
    for (uint32_t i = 0; i < width * height; ++i)
    {
      ofs << data[i];
      const char sep = (i == 0 || i, width) ? ',' : '\n';
      ofs << sep;
    }
    ofs.close();
  }

  string FaceFilter::GenerateTempFilePath()
  {
    time_t t = time(0);
    struct tm * now = localtime(&t);
    static int counter = 0;
    char buffer[128] = { '\0' };
    sprintf(buffer,
      "/tmp/kinect-%04d-%02d-%02d--%02d-%02d-%02d--%05d.csv",
      now->tm_year + 1900,
      now->tm_mon + 1,
      now->tm_mday,
      now->tm_hour,
      now->tm_min,
      now->tm_sec,
      (counter++)
      );
    return string(buffer);
  }
}