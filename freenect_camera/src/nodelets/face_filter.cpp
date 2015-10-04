#include "face_filter.h"
#include "face_filter.hpp"

namespace freenect_camera
{
  struct Mask
  {
    static const uint16_t _maxScore = 20000;
    uint16_t _lengthOneSide;
    std::vector<int16_t> _matrix;
    std::vector<int16_t> _selectionIndexes;

    Mask(uint16_t diameter, uint16_t innerHole);
    void ApplySelection(std::vector<char>& filter, uint32_t segmentsOneSide, uint32_t layersCount);
  };

#ifdef _MSC_VER
#pragma warning (disable: 4512)  // Cannot generate assignment operator automatically due to const members
#endif

  struct FaceFilterHistogramTransformData
  {
    FaceFilterHistogramTransformData(uint32_t layersCount, uint32_t segmentsCount = 20, uint32_t depthMax = 4000, bool tracingEnabled = false, const std::string& fileNameBaseTrace = std::string());
    void PlacePoints(uint32_t width, uint32_t height, uint16_t* data);
    void ApplyMask();
    void FilterDepthData(uint32_t width, uint32_t height, uint16_t* data);

  private:
    const uint32_t _layersCount;
    const uint32_t _depthMax;
    const uint32_t _segmentsCount;
    bool _tracingEnabled;
    const std::string& _fileNameBaseTrace;

    std::vector<std::vector<uint16_t> > _layeredSegments;
    std::vector<char> _segmentFilter;

    // TODO: pre-generate the mask
    Mask _mask;
    uint32_t LayerToDepth(uint32_t layer);
    uint32_t DepthToLayer(uint32_t depth);

    void PlacePoint(uint32_t x, uint32_t y, uint32_t width, uint32_t height, uint16_t value);
    void ApplyMask(const std::vector<uint16_t>& layer, const Mask& mask, std::vector<uint16_t>& scores);
    inline uint32_t GetSegmentIndex(uint32_t x, uint32_t y, uint32_t width, uint32_t height);
    template<typename T> void Trace(const std::string& name, const std::vector<T>& data, uint32_t width, uint32_t height, uint32_t counter);
    template<typename T> void Trace(const std::string& name, const T* data, uint32_t width, uint32_t height, uint32_t counter);
  };

#ifdef _MSC_VER
#pragma warning (default: 4512)
#endif


  Mask::Mask(uint16_t maxDiameter, uint16_t minDiameter)
  {
    _lengthOneSide = maxDiameter + 2;
    if (maxDiameter == 0)
      return;

    _matrix = std::vector<int16_t>(_lengthOneSide * _lengthOneSide);
    double_t maxRadius = maxDiameter / 2.;
    double_t minRadius = minDiameter / 2.;
    double_t cx = .5 + maxRadius;
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
        assert(i == y * _lengthOneSide + x && "Index must be always increasing by 1.");
        if (_matrix[i] == 1)
        {
          _matrix[i] = score1;
        }
        else if (_matrix[i] == 2)
        {
          _matrix[i] = -score2;
        }
        i++;
      }
    }
  }

  void Mask::ApplySelection(std::vector<char>& filter, uint32_t segmentsOneSide, uint32_t layersCount)
  {
    uint32_t index = 0;
    for (uint32_t y = 0; y < segmentsOneSide; ++y) {
      for (uint32_t x = 0; x < segmentsOneSide; ++x) {
        assert((index == y * segmentsOneSide + x) && "Index must be always increasing by 1.");
        uint32_t filterValue = filter[index];
        if (filterValue > 0 && filterValue < layersCount){
          uint16_t centerOffset = _lengthOneSide / 2;
          int32_t maskIndex = 0;
          for (uint16_t my = 0; my < _lengthOneSide; ++my) {
            for (uint16_t mx = 0; mx < _lengthOneSide; ++mx) {
              int32_t tx = x + mx - centerOffset;
              int32_t ty = y + my - centerOffset;
              int32_t tIndex = ty * segmentsOneSide + tx;
              assert((maskIndex == my * _lengthOneSide + mx) && "Index must be always increasing by 1.");
              const int32_t segmentsCount = static_cast<int32_t>(segmentsOneSide);

              if (tx >= 0 && tx < segmentsCount && ty >= 0 && ty < segmentsCount){
                if (_matrix[maskIndex] >= 0 && filter[tIndex] == 0) {
                  filter[tIndex] = filter[index] + static_cast<char>(layersCount);
                }
              }
              maskIndex++;
            }
          }
        }
        index++;
      }
    }
  }

  FaceFilterHistogramTransformData::FaceFilterHistogramTransformData(uint32_t layersCount, uint32_t segmentsCount, uint32_t depthMax, bool tracingEnabled, const std::string& fileNameBaseTrace)
    : _mask(5, 1)
    , _layersCount(layersCount)
    , _depthMax(depthMax)
    , _segmentsCount(segmentsCount)
    , _tracingEnabled(tracingEnabled)
    , _fileNameBaseTrace(fileNameBaseTrace)
  {
    for (uint32_t i = 0; i < _layersCount; ++i)
    {
      _layeredSegments.push_back(std::vector<uint16_t>(_segmentsCount * _segmentsCount));
    }

    _segmentFilter = std::vector<char>(_segmentsCount * _segmentsCount);
  }

  void FaceFilterHistogramTransformData::PlacePoints(uint32_t width, uint32_t height, uint16_t* data)
  {
    Trace("mask", _mask._matrix, _mask._lengthOneSide, _mask._lengthOneSide, 0);

    uint32_t index = 0;
    for (uint32_t y = 0; y < height; ++y){
      for (uint32_t x = 0; x < width; ++x){
        PlacePoint(x, y, width, height, data[index]);
        assert((index == y * width + x) && "Index must be always increasing by 1.");
        index++;
      }
    }
  }

  uint32_t FaceFilterHistogramTransformData::LayerToDepth(uint32_t layer)
  {
    assert(layer <= _layersCount && "Invalid layer.");
    uint32_t result = layer == _layersCount ? _depthMax : layer * _depthMax / _layersCount;
    return result;
  }

  uint32_t FaceFilterHistogramTransformData::DepthToLayer(uint32_t depth)
  {
    uint32_t result = depth >= _depthMax ? _layersCount - 1U: depth * _layersCount / _depthMax;
    assert(depth >= LayerToDepth(result) && (depth <= LayerToDepth(result + 1U) || depth >= _depthMax ) && "DepthToLayer(), roundtrip verification check failed.");
    return result;
  }

  void FaceFilterHistogramTransformData::PlacePoint(uint32_t x, uint32_t y, uint32_t width, uint32_t height, uint16_t value)
  {
    if (value > 0)
    {
      const uint32_t segmentIndex = GetSegmentIndex(x, y, width, height);
      const uint32_t layer = DepthToLayer(value);

      _layeredSegments[layer][segmentIndex] ++;
    }
  }

  void FaceFilterHistogramTransformData::ApplyMask()
  {
    // The first and the last layers are ignored
    // TODO: has a matrix with layers of interest, based on some criteria (like having at least radiusMax not empty sequential points)
    for (uint32_t j = 1; j < _layeredSegments.size() - 1; ++j)
    {
      Trace("layer", _layeredSegments[j], _segmentsCount, _segmentsCount, j);
      std::vector<uint16_t> scores(_segmentsCount * _segmentsCount);
      ApplyMask(_layeredSegments[j], _mask, scores);

      Trace("score", scores, _segmentsCount, _segmentsCount, j);

      for (uint16_t i = 0; i < scores.size(); ++i){
        if (scores[i] > Mask::_maxScore * .78) {
          _segmentFilter[i] = std::max(_segmentFilter[i], static_cast<char>(j));
        }
      }
      Trace("segmentFilter", _segmentFilter, _segmentsCount, _segmentsCount, j);
    }

    _mask.ApplySelection(_segmentFilter, _segmentsCount, _layersCount);
    Trace("segmentSelection", _segmentFilter, _segmentsCount, _segmentsCount, 0);
  }

  void FaceFilterHistogramTransformData::ApplyMask(const std::vector<uint16_t>& layer, const Mask& mask, std::vector<uint16_t>& scores)
  {
    uint32_t index = 0;
    for (uint32_t y = 0; y < _segmentsCount; ++y) {
      for (uint32_t x = 0; x < _segmentsCount; ++x) {
        assert((index == y * _segmentsCount + x) && "Index must be always increasing by 1.");
        uint16_t score = 0;
        uint16_t centerOffset = mask._lengthOneSide / 2;
        int32_t maskIndex = 0;
        for (uint16_t my = 0; my < mask._lengthOneSide; ++my) {
          for (uint16_t mx = 0; mx < mask._lengthOneSide; ++mx) {
            int32_t tx = x + mx - centerOffset;
            int32_t ty = y + my - centerOffset;
            int32_t tIndex = ty * _segmentsCount + tx;
            assert((maskIndex == my * _mask._lengthOneSide + mx) && "Index must be always increasing by 1.");
            const int32_t segmentsCount = static_cast<int32_t>(_segmentsCount);

            if (tx >= 0 && tx < segmentsCount && ty >= 0 && ty < segmentsCount){
              if (mask._matrix[maskIndex] > 0 && layer[tIndex] != 0) {
                score += mask._matrix[maskIndex];
              }
              else if (mask._matrix[maskIndex] < 0 && layer[tIndex] == 0) {
                score -= mask._matrix[maskIndex];
              }
            }
            maskIndex++;
          }
        }
        scores[index] = score;
        index++;
      }
    }
  }

  inline uint32_t FaceFilterHistogramTransformData::GetSegmentIndex(uint32_t x, uint32_t y, uint32_t width, uint32_t height)
  {
    const uint32_t xSegment = x * _segmentsCount / width;
    const uint32_t ySegment = y * _segmentsCount / height;
    uint32_t segmentIndex = ySegment * _segmentsCount + xSegment;
    assert(segmentIndex < _segmentsCount * _segmentsCount);

    return segmentIndex;
  }

  void FaceFilterHistogramTransformData::FilterDepthData(uint32_t width, uint32_t height, uint16_t* data)
  {
    uint32_t index = 0;
    for (uint32_t y = 0; y < height; ++y)
    {
      for (uint32_t x = 0; x < width; ++x)
      {
        uint32_t segmentIndex = GetSegmentIndex(x, y, width, height);
        uint32_t layerValueCoded = _segmentFilter[segmentIndex];
        uint32_t layer = layerValueCoded > _layersCount ? layerValueCoded - _layersCount : layerValueCoded;
        uint16_t maxAllowedValue = layer == 0 ? 0U : static_cast<uint16_t>(LayerToDepth(layer + 1));
        data[index] = data[index] > maxAllowedValue ? 0U : data[index];
        assert((index == y * width + x) && "Index must be always increasing by 1.");
        index++;
      }
    }
  }

  template<typename T>
  void FaceFilterHistogramTransformData::Trace(const std::string& name, const std::vector<T>& data, uint32_t width, uint32_t height, uint32_t counter)
  {
    assert(data.size() <= width * height && "width and height are invalid for this vector.");
    Trace(name, data.data(), width, height, counter);
  }

  template<typename T>
  void FaceFilterHistogramTransformData::Trace(const std::string& name, const T* data, const uint32_t width, const uint32_t height, const uint32_t counter)
  {
    if (!_tracingEnabled)
      return;

    char buffer[1024] = { '\0' };
    int stringLength = sprintf(buffer,
      "%s_%02d_%s.csv",
      _fileNameBaseTrace.c_str(),
      counter,
      name.c_str()
    );

    if (stringLength <= 0) {
      assert(!"Error while creating file name.");
      return;
    }

    FaceFilter::SaveDataAsCsv(width, height, data, std::string(buffer, stringLength));
  }

  FaceFilterHistogramTransform::FaceFilterHistogramTransform(uint32_t layersCount, uint32_t segmentsCount, uint32_t depthMax, bool tracingEnabled, const std::string& fileNameBaseTrace)
    : _data(new FaceFilterHistogramTransformData(layersCount, segmentsCount, depthMax, tracingEnabled, fileNameBaseTrace))
  {
  }

  FaceFilterHistogramTransform::~FaceFilterHistogramTransform()
  {
  }

  void FaceFilterHistogramTransform::Transform(uint32_t width, uint32_t height, uint16_t* data)
  {
    if (data == NULL)
      return;

    _data->PlacePoints(width, height, data);

    _data->ApplyMask();

    _data->FilterDepthData(width, height, data);
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
}
