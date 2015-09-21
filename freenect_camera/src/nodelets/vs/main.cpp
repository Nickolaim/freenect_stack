#include "common.h"
#define NOMINMAX
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
    std::string _pathToDataDir;
    std::string _pathToTestOutDir;

    UnitTest1()
    {
      char   dllPath[MAX_PATH] = { 0 };
      DWORD result = ::GetModuleFileNameA((HINSTANCE)&__ImageBase, dllPath, _countof(dllPath));
      assert(result > 0 && "GetModuleFileNameA returned error");
      auto path = std::string(dllPath);
      size_t pos = path.find_last_of('\\');  // this is Windows so use \.
      assert(pos <= MAX_PATH);
      auto currentDir = path.substr(0, pos);
      _pathToDataDir = currentDir + "\\data\\";
      _pathToTestOutDir = currentDir + "\\test_out\\";

      ::RemoveDirectoryA(_pathToTestOutDir.c_str());
      ::CreateDirectoryA(_pathToTestOutDir.c_str(), nullptr);
    }

    TEST_METHOD(TestMethod1)
    {
      Logger::WriteMessage("In TestMethod1");
      const std::string testFilePath = _pathToDataDir + "kinect-2015-09-02--21-32-01--00000.csv";
      const uint32_t width = 640;
      const uint32_t heigth = 480;
      std::unique_ptr<uint16_t> data(new uint16_t[width * heigth]);
      FaceFilter::LoadDepthImageFromCsv(testFilePath, width, heigth, data.get());
    }
    
    TEST_METHOD(SaveLoad)
    {
      const std::string testFilePath = _pathToTestOutDir + "save_load.csv";
      const uint32_t width = 640u;
      const uint32_t heigth = 480u;
      std::unique_ptr<uint16_t> data(new uint16_t[width * heigth]);
      for (auto i = 0; i < width * heigth; i++)
      {
        data.get()[i] = static_cast<uint16_t>(i % std::numeric_limits<uint16_t>::max());
      }

      FaceFilter::SaveDepthImageAsCsv(width, heigth, data.get(), testFilePath);

      std::unique_ptr<uint16_t> dataActual(new uint16_t[width * heigth]);
      FaceFilter::LoadDepthImageFromCsv(testFilePath, width, heigth, dataActual.get());
      for (auto i = 0; i < width * heigth; i++)
      {
        Assert::AreEqual(data.get()[i], dataActual.get()[i]);
      }
    }
  };
}