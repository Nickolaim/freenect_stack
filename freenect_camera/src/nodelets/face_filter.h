#ifndef FREENECT_CAMERA_FACE_FILTER_H
#define FREENECT_CAMERA_FACE_FILTER_H

#ifndef _MSC_VER
  // TODO:  Can it be removed at all?
  #include <ros/ros.h>
#else
  #include "vs/common.h"
#endif

namespace freenect_camera {

class FaceFilter
{
	public:
		static void SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data);
		static void SaveDepthImageAsCsv(uint32_t width, uint32_t height, uint16_t* data, const std::string& filePath);

	private:
		static std::string GenerateTempFilePath();
};

}
#endif // FREENECT_CAMERA_FACE_FILTER_H