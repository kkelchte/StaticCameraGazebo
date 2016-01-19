#include "gazebo/gazebo.hh"
#include "plugins/CameraPlugin.hh"
using namespace std;

namespace gazebo
{
  class CameraDump : public CameraPlugin
  {
    public: CameraDump() : CameraPlugin(), saveCount(0), location("/esat/quaoar/kkelchte/simulation/data/box") {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      // Don't forget to load the camera plugin
      CameraPlugin::Load(_parent, _sdf);
    }

    // Update the controller
    public: void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height, unsigned int _depth,
        const std::string &_format)
    {
      char tmp[1024];
      snprintf(tmp, sizeof(tmp), "%s/%s-%05d.jpg",this->location.c_str(),
          this->parentSensor->GetCamera()->GetName().c_str(), this->saveCount);

      if (this->saveCount < 10)
      {
        this->parentSensor->GetCamera()->SaveFrame(
            _image, _width, _height, _depth, _format, tmp);
        gzmsg << "Saving frame [" << this->saveCount
              << "] as [" << tmp << "]\n";
        this->saveCount++;
      }
    }
    
    private: std::string location;
    private: int saveCount;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CameraDump)

}
