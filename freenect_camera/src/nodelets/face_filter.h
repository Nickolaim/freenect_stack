#ifndef FREENECT_CAMERA_FACE_FILTER_H
#define FREENECT_CAMERA_FACE_FILTER_H

#include <ros/ros.h>

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