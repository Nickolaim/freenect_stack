#include "face_filter.h"
#include <fstream>

namespace freenect_camera
{
  void FaceFilter::SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data)
  {
    SaveDepthImageAsCsv(width, height, data, GenerateTempFilePath());
  }

  void FaceFilter::SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data, const std::string& filePath)
  {
    std::ofstream ofs(filePath.c_str(), std::ofstream::out);
    for (uint32_t i = 0; i < width * height; ++i)
    {
      ofs << data[i];
      const char sep = ((i + 1) % width) ? ',' : '\n';
      ofs << sep;
    }
    ofs.close();
  }

  std::string FaceFilter::GenerateTempFilePath()
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
    return std::string(buffer);
  }

  void FaceFilter::LoadDepthImageFromCsv(const std::string& filePath, uint32_t expectedWidth, uint32_t expectedHeight, uint16_t* data)
  {
    std::ifstream ifs(filePath.c_str(), std::ifstream::in);
    if (!ifs)
      throw new std::runtime_error(std::string("Cannot open file for reading:") + filePath + std::string("."));

    std::string line;
    size_t row = 0;
    while (std::getline(ifs, line))
    {
      std::istringstream iss(line);
      for (size_t column = 0; column < expectedWidth; ++column)
      {
        uint16_t value;
        char ch;
        if (!(iss >> value))
          throw new std::runtime_error("Too few column values.");

        const size_t index = row * expectedWidth + column;
        assert(index < expectedWidth * expectedWidth && "Reaching outside of array upper bound.");
        data[index] = value;

        if (column == expectedWidth - 1)
          continue;

        if (!(iss >> ch))
          throw new std::runtime_error("Cannot read separator.");
        if (ch != ',')
          throw new std::runtime_error("Separator is not ','.");
      }
      row++;
    }
    if (row != expectedHeight)
      throw new std::runtime_error("Unexpected number of rows" + std::to_string(row - 1) + ".");

  }

}