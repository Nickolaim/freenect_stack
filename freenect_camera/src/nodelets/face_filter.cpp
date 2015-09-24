#include "face_filter.h"
#include <fstream>

namespace freenect_camera
{
  struct Mask
  {
    static const uint16_t _maxScore = 20000;
    uint16_t _lengthOneSide;
    std::vector<int16_t> _matrix;

    Mask(uint16_t diameter, uint16_t innerHole);
  };

  struct FaceFilterHistogramTransform::FaceFilterHistogramTransformData
  {
    static const uint32_t _expectedWidth = 640;
    static const uint32_t _expectedHeight = 480;
    static const uint32_t _layersCount = 30;
    static const uint32_t _depthMax = 4000;
    static const uint32_t _segmentsCount = 20;

    std::vector<std::vector<uint16_t>> _layeredSegments;
    // TODO: pre-generate the mask
    Mask _mask;

    FaceFilterHistogramTransform::FaceFilterHistogramTransformData();
    void PlacePoint(uint32_t x, uint32_t y, uint16_t value);
  };

  Mask::Mask(uint16_t maxDiameter, uint16_t minDiameter)
  {
    _lengthOneSide = maxDiameter + 2;
    if (maxDiameter == 0)
      return;

    _matrix = std::vector<int16_t>(_lengthOneSide * _lengthOneSide);
    double_t maxRadius = maxDiameter / 2.;
    double_t minRadius = minDiameter / 2.;
    double_t cx = 1 + maxRadius;
    double_t cy = cx;

    // Calculate matrix
    uint16_t i = 0;
    uint16_t score1Count = 0;
    uint16_t score2Count = 0;
    for (uint16_t y = 0; y < _lengthOneSide; ++y)
    {
      for (uint16_t x = 0; x < _lengthOneSide; ++x)
      {
        assert(i == y * _lengthOneSide + x && "Index must be always increasing by 1.");
        double d = sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy));
        if (d >= minRadius && d <= maxRadius)
        {
          score1Count++;
          _matrix[i] = 1;
        }
        else if (d >= maxRadius)
        {
          score2Count++;
          _matrix[i] = 2;
        }

        i++;
      }
    }

    // Assign scores
    i = 0;
    int16_t score1 = _maxScore / 2 / score1Count;
    int16_t score2 = _maxScore / 2 / score2Count;
    for (uint16_t y = 0; y < _lengthOneSide; ++y)
    {
      for (uint16_t x = 0; x < _lengthOneSide; ++x)
      {
        if (_matrix[i] == 1)
        {
          _matrix[i] = score1;
        }
        else if (_matrix[i] == 2)
        {
          _matrix[i] = -score2;
        }
      }
    }
  }

  FaceFilterHistogramTransform::FaceFilterHistogramTransformData::FaceFilterHistogramTransformData()
    : _mask(6, 1)
  {
    for (auto i = 0; i < _layersCount; ++i)
    {
      _layeredSegments.push_back(std::vector<uint16_t>(_segmentsCount * _segmentsCount));
    }
  }

  void FaceFilterHistogramTransform::FaceFilterHistogramTransformData::PlacePoint(uint32_t x, uint32_t y, uint16_t value)
  {
    const uint32_t xSegment = x * _segmentsCount / _expectedWidth;
    const uint32_t ySegment = y * _segmentsCount / _expectedHeight;
    uint32_t layer = 0;

    if (value != 0)
    {
      if (value >= _depthMax)
        layer = _layersCount - 1;
      else
        layer = value * _layersCount / _depthMax;
    }

    _layeredSegments[layer][ySegment * _segmentsCount + xSegment] ++;
  }

  FaceFilterHistogramTransform::FaceFilterHistogramTransform()
    : _data(new FaceFilterHistogramTransformData)
  {
  }

  FaceFilterHistogramTransform::~FaceFilterHistogramTransform()
  {
  }

  void FaceFilterHistogramTransform::Transform(uint32_t width, uint32_t height, uint16_t* data)
  {
    if (width != FaceFilterHistogramTransformData::_expectedWidth || height != FaceFilterHistogramTransformData::_expectedHeight || data == nullptr)
      return;

    uint32_t index = 0;
    for (uint32_t y = 0; y < FaceFilterHistogramTransformData::_expectedHeight; ++y){
      for (uint32_t x = 0; x < FaceFilterHistogramTransformData::_expectedWidth; ++x){
        _data->PlacePoint(x, y, data[index]);
        assert((index == y * FaceFilterHistogramTransformData::_expectedWidth + x) && "Index must be always increasing by 1.");
        index++;
      }
    }
  }

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
      throw new std::runtime_error("Unexpected number of rows" + boost::lexical_cast<std::string>(row - 1) + ".");

  }

}
