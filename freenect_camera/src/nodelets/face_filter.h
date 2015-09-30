#ifndef FREENECT_CAMERA_FACE_FILTER_H
#define FREENECT_CAMERA_FACE_FILTER_H

#ifndef _MSC_VER
// TODO:  Can it be removed at all?
#include <ros/ros.h>
#else
#include "vs/common.h"
#endif

namespace freenect_camera {

  class DepthDataTransform
  {
  public:
    virtual void Transform(uint32_t width, uint32_t height, uint16_t* data) = 0;
  };

  class FaceFilterHistogramTransform : public DepthDataTransform
  {
  public:
    FaceFilterHistogramTransform(uint32_t layersCount = 30, uint32_t segmentsCount = 20, uint32_t depthMax = 4000, bool tracingEnabled = false, const std::string& fileNameBaseTrace = std::string());
    void Transform(uint32_t width, uint32_t height, uint16_t* data) override;
    ~FaceFilterHistogramTransform();

  private:
    struct FaceFilterHistogramTransformData;
    std::unique_ptr<FaceFilterHistogramTransformData> _data;
  };

  class FaceFilter
  {
  public:
    template<typename T>
    static void SaveDataAsCsv(uint32_t width, uint32_t height, const T* data);
    template<typename T>
    static void SaveDataAsCsv(uint32_t width, uint32_t height, const T* data, const std::string& filePath);
    template<typename T>
    static void LoadDataFromCsv(const std::string& filePath, uint32_t expectedWidth, uint32_t expectedHeight, T* data);

  private:
    static std::string GenerateTempFilePath();
  };

}
#endif // FREENECT_CAMERA_FACE_FILTER_H