/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include <gazebo/msgs/msgs.hh>
using namespace std;

namespace gazebo
{
  class CameraMove : public ModelPlugin
  {
    public: CameraMove() : ModelPlugin() {
        frameNumber=0;
        innerState=0;//the state in which the drone flies in 11 steps around the object
        outerState=0;//the relative control states needed as ground truth
        finished=false;
        
        // Create a new transport node
        this->node= gazebo::transport::NodePtr (new gazebo::transport::Node());
        // Create a publisher on the ~/trajectory_state topic
        this->statePub = node->Advertise<msgs::Int>("/gazebo/moving/trajectory_state");
        this->finishedPub = node->Advertise<msgs::Int>("/gazebo/moving/finished_state");
        this->finishedSub = node->Subscribe("/gazebo/moving/finished_state", &CameraMove::callbackFinished, this);
            

    }

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Get a pointer to the model
        this->model = _model;
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&CameraMove::OnUpdate, this));
        
        // Initialize the node with the world name
        this->node->Init(model->GetName());

    }
    //Called whenever the trajectory is finished
    private: void callbackFinished(ConstIntPtr &_msg){
        cout<<"[MOV]:trajectory finished? "<< _msg->data()<< endl<<flush;
        if(_msg->data()==0){
            finished = false;
            innerState = 0;
            outerState = 0;
        }
    }
    // Called by the world update start event
    public: void OnUpdate()
    {
        if(!finished){
        float speed = 0.5;
        float yaw = -0.314;

        math::Pose pose = this->model->GetWorldPose();
        math::Vector3 v(0,0,0.01);//translational velocity: z direction for gravity compensation
        math::Vector3 a(0,0,0);//angular velocity

        frameNumber = frameNumber+1;
        if((frameNumber%5000)==0){
            innerState = innerState+1;
            cout << "current state " << innerState << endl;
            if(innerState == 11){ 
                innerState = 0;
                finished = 1;
            }
        }
        //TODO do this line away
        //if(innerState==1){finished = true;}

        if(finished){
            // Make sure to shut everything down.
            //gazebo::shutdown();
            // Publish that everything is done
            msgs::Int msg;
            msg.set_data(1);
            // Send the message
            finishedPub->Publish(msg);
        }
        
        switch(innerState){
            case 0://go up: outerState 0
                v+=math::Vector3(0,0,0.2*speed);
                outerState = 0;
                break;
            case 1:
            case 9://go half left
                v+=math::Vector3(0,0.5*speed,0);
                outerState = 1;
                break;
            case 2:
            case 4:
            case 6:
            case 8://turn: 
                a+=math::Vector3(0,0,yaw);
                outerState = 2;
                break;
            case 3://go forward absolute
            case 5://go right absolute
            case 7://go backward absolute
                v+=math::Vector3(0, speed, 0);
                outerState = 1;
                break;/*
            case 5://go left
                v+=math::Vector3(0,-speed,0);
                break;
            case 7://go back
                v+=math::Vector3(-speed,0,0);
                break;*/
            case 10://land
                v+=math::Vector3(0,0,-0.2*speed);
                outerState=3;
                break;                
        }
        // Apply a small linear velocity to the model.
        v = pose.rot * v; // change relative speed Vector to absolute frame
        this->model->SetLinearVel(v);
        this->model->SetAngularVel(a);
        //cout << "outerState: "<< outerState<< endl;
        // Sent the state out there!
        
        // Create the message
        msgs::Int msg;
        msg.set_data(outerState);
        // Send the message
        statePub->Publish(msg);
    
        }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Counters for changing the direction
    private: int frameNumber;
    private: int innerState;
    private: int outerState;
    private: bool finished;
    private: transport::NodePtr node;
    private: transport::PublisherPtr statePub;
    private: transport::PublisherPtr finishedPub;
    private: transport::SubscriberPtr finishedSub;
    //private: transport::SubscriberPtr modelSub;    
        
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraMove)
}
