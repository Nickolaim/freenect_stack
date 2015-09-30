#include "face_filter.h"
#include <fstream>

namespace freenect_camera
{
  template<typename T>
  void FaceFilter::SaveDataAsCsv(uint32_t width, uint32_t height, const T* data)
  {
    SaveDataAsCsv(width, height, data, GenerateTempFilePath());
  }

  template<typename T>
  void FaceFilter::SaveDataAsCsv(uint32_t width, uint32_t height, const T* data, const std::string& filePath)
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

  template<typename T>
  void FaceFilter::LoadDataFromCsv(const std::string& filePath, uint32_t expectedWidth, uint32_t expectedHeight, T* data)
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
      throw new std::runtime_error("Unexpected number of rows" + boost::lexical_cast<std::string>(row - 1) + ".");

  }
}
