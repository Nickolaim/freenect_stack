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
    FaceFilterHistogramTransform();
    void Transform(uint32_t width, uint32_t height, uint16_t* data) override;

  private:
    struct FaceFilterHistogramTransformData;
    std::unique_ptr<FaceFilterHistogramTransformData> _data;
  };

  class FaceFilter
  {
  public:
    static void SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data);
    static void SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data, const std::string& filePath);
    static void LoadDepthImageFromCsv(const std::string& filePath, uint32_t expectedWidth, uint32_t expectedHeight, uint16_t* data);

  private:
    static std::string GenerateTempFilePath();
  };

}
#endif // FREENECT_CAMERA_FACE_FILTER_H