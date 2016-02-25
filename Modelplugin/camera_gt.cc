/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <iostream>
#include <boost/filesystem.hpp>

//#include "gazebo/physics/physics.hh"
#include "plugins/CameraPlugin.hh"

using namespace std;
namespace gazebo
{
  class Camera_gt  :  public CameraPlugin
  {
        int maxNumber = 0; //Upperbound in case the trajectory is not finished yet
        int state;
        transport::NodePtr node;
        transport::SubscriberPtr stateSub;
        transport::SubscriberPtr finishedSub;
        transport::SubscriberPtr locationSub;
        std::string location;
        int saveCount;
        bool wait;
        bool finished;//If finished =1 dont save
        

        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
        {
            
            state = 0;
            saveCount = 0;
            finished = false;
            wait = true;
            location = _sdf->Get<std::string>("location");
            maxNumber = _sdf->Get<int>("maxnumberframes");
            if(location == ""){
                location = "/esat/quaoar/kkelchte/simulation/data/no_location/RGB";
                boost::filesystem::path dir(location.c_str());
                if(boost::filesystem::create_directory(dir)) {
                        gzmsg << "[GT]:Success in creating: "<<location << "\n";
                }
            }
            if(maxNumber == 0) maxNumber = 1000;
            std::cout << "Location: "<<location << ". Max number: "<<maxNumber<<std::endl;
            
            node = transport::NodePtr(new transport::Node());
            // Don't forget to load the camera plugin
            CameraPlugin::Load(_parent, _sdf);
            // Initialize the node with the sensors name
            node->Init(_parent->GetName());
            std::cout << "Subscribing to: " << "trajectory" << std::endl;
            // Listen to Gazebo trajectory_state topic
            stateSub = node->Subscribe("/gazebo/moving/trajectory_state", &Camera_gt::callback_state, this);
            finishedSub = node->Subscribe("/gazebo/moving/finished_state", &Camera_gt::callback_finished, this);
            // subscribe with latching so that function is now directly called if there is a value on the topic
            // without that this value has to change
            locationSub = node->Subscribe("/gazebo/saving_location", &Camera_gt::callback_location, this,true); 
        }

        
        /////////////////////////////////////////////////
        // Function is called everytime a message is received.
        private: void callback_location(ConstGzStringPtr &_msg)
        {
            // Dump the message contents to stdout.
            gzmsg << "[GT:] received location: "<<_msg->data() << std::endl;
            if(_msg->data()!=""){
                location = _msg->data() + "/RGB";
                boost::filesystem::path dir(location.c_str());
                if(boost::filesystem::create_directory(dir)) {
                        gzmsg << "[GT]:Success in creating: "<<location << "\n";
                }
            }
        }
        private: void callback_finished(ConstIntPtr &_msg)
        {
            // Dump the message contents to stdout.
            if(_msg->data()==1){ finished=true; saveCount = 0;}
            if(_msg->data()==0) finished=false; 
            cout <<"[GT] received finished "<< finished << std::endl;
        }
        
        private: void callback_state(ConstIntPtr &_msg)
        {
            // Dump the message contents to stdout.
            this->state =  _msg->data();
            //std::cout << state << std::endl;
        }

        // Update the controller
        public: void OnNewFrame(const unsigned char *_image,
            unsigned int _width, unsigned int _height, unsigned int _depth,
            const std::string &_format)
        {
            if(wait){
                saveCount++;
                if(saveCount>7){ //the first 7 frames appear to be black even though the simulation waits untill
                    //everything is nicely loaded. This is strange but these first images are now just skipped.
                    wait = false;
                    saveCount =0;
                }
            }
            if(!finished && !wait){
                char tmp[1024];
                snprintf(tmp, sizeof(tmp), "%s/%05d-gt%01d.jpg",this->location.c_str(),
                    this->saveCount, this->state);
                
                if (this->saveCount < maxNumber)
                {
                    this->parentSensor->GetCamera()->SaveFrame(
                        _image, _width, _height, _depth, _format, tmp);
                    gzmsg << "Saving frame [" << this->saveCount
                        << "] as [" << tmp << "]\n";
                    this->saveCount++;
                }
            }
            
        }
        
  };
  
    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(Camera_gt)
}
