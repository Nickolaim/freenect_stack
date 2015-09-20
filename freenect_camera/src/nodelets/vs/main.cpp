#include "common.h"
#include <Windows.h>
#include "unittest.h"
#include "..\face_filter.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace freenect_camera;

EXTERN_C IMAGE_DOS_HEADER __ImageBase;

namespace UnitTest1
{
  TEST_CLASS(UnitTest1)
  {
  public:
    std::string pathToDataDir;

    UnitTest1()
    {
      char   dllPath[MAX_PATH] = { 0 };
      DWORD result = ::GetModuleFileNameA((HINSTANCE)&__ImageBase, dllPath, _countof(dllPath));
      assert(result > 0 && "GetModuleFileNameA returned error");
      auto path = std::string(dllPath);
      size_t pos = path.find_last_of('\\');  // this is Windows so use \.
      assert(pos <= MAX_PATH);
      pathToDataDir = path.substr(0, pos) + "\\data\\";
    }


    TEST_METHOD(TestMethod1)
    {
      Logger::WriteMessage("In TestMethod1");
      const std::string testFilePath = pathToDataDir + "kinect-2015-09-02--21-32-01--00000.csv";
      const uint32_t width = 640;
      const uint32_t heigth = 480;
      std::unique_ptr<uint16_t> data(new uint16_t[width * heigth]);
      FaceFilter::LoadDepthImageFromCsv(testFilePath, width, heigth, data.get());
    }

  };
}