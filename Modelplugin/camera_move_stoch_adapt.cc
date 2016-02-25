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
#include <stdlib.h>

using namespace std;

namespace gazebo
{
  class CameraMove : public ModelPlugin
  {   
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Counters for changing the direction
    private: const int durationNoise = 5; //variation among number of updates before in next state
    private: const float directionNoise = 0.006; // small perturbations on the direction of the speed vector
    private: const float orientationNoise = 0.014; // small perturbations on the orientation of the Camera
    private: const float speedNoise = 0.001; // small perturbations on the absolute value of speed
    private: const int updateNoise = 100; // number of updates before noise is updated to make perturbations clearer
    private: math::Vector3 v;//translational velocity: z direction for gravity compensation
    private: math::Vector3 a;//angular velocity
    private: math::Vector3 s;//standard stepsize in x,y,z direction
    private: float speed = 1.0f;//size of velocity vector
    private: math::Vector3 currents;//size the current stepsize adapted to the current size of the focus object
    private: int numberofruns = 1;//the number of runs it goes through state 0->9 for a tall object
    private: int currentrun = 1;//the number of runs it goes through state 0->9 for a tall object
    
    private: int frameNumber;
    private: int nextThreshold;
    private: int innerState;
    private: int outerState;
    private: bool finished;
    private: transport::NodePtr node;
    private: transport::PublisherPtr statePub;
    private: transport::PublisherPtr finishedPub;
    private: transport::SubscriberPtr finishedSub;   
    private: transport::SubscriberPtr sizeSub;   
    
    public: CameraMove() : ModelPlugin() {
        frameNumber=0;
        nextThreshold= rand() % durationNoise + 5000-floor(durationNoise/2);
        innerState=0;//the state in which the drone flies in 11 steps around the object
        outerState=0;//the relative control states needed as ground truth
        finished=false;
	
        // Apply noise to direction, orientation and size of speed vector
        float vx = ((rand()%10)-5)*directionNoise/10;
        float vy = ((rand()%10)-5)*directionNoise/10;
        float vz = ((rand()%10)-5)*directionNoise/10+0.01;
        float ar = ((rand()%10)-5)*orientationNoise/10;
        float ap = ((rand()%10)-5)*orientationNoise/10;
        float ay = ((rand()%10)-5)*orientationNoise/10;
        speed = speed+((rand()%10)-5)*speedNoise/10;
        v = math::Vector3(vx,vy,vz);//translational velocity: z direction for gravity compensation
        a = math::Vector3(ar,ap,ay);//angular velocity
        
        
        // Trajectory is defined by stepsize in certain direction: these are the weights for an object of size 1 1 1 
        float sx = 0.5; //stepsize of x direction defined by objects size
        float sy = 0.5; //stepsize of y direction defined by objects size
        float sz = 0.1;
        s = math::Vector3(sx, sy, sz);
        
        //Current stepsize for object of size 1 1 1
        currents = math::Vector3(sx, sy, sz);
        
        
        // Create a new transport node
        this->node= gazebo::transport::NodePtr (new gazebo::transport::Node());
        // Create a publisher on the ~/trajectory_state topic
        this->statePub = node->Advertise<msgs::Int>("/gazebo/moving/trajectory_state");
        this->finishedPub = node->Advertise<msgs::Int>("/gazebo/moving/finished_state");
        this->finishedSub = node->Subscribe("/gazebo/moving/finished_state", &CameraMove::callbackFinished, this);
        this->sizeSub = node->Subscribe("/gazebo/moving/object_size", &CameraMove::callbackSize, this,true);
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
        gzmsg<<"[MOV]:trajectory finished? "<< _msg->data()<< endl<<flush;
        if(_msg->data()==0){
            finished = false;
            innerState = 0;
            outerState = 0;
            frameNumber = 0;
            nextThreshold= rand() % durationNoise + 5000-floor(durationNoise/2);
            currentrun = 1;
            gazebo::common::Time::MSleep(20);//only start flying when everything is certainly ready.
        }
    }
    //Called when a new focus object is spawned and the size is changed
    private: void callbackSize(ConstVector3dPtr &_msg){
        gzmsg<<"[MOV]:Size received: "<< _msg->x()<< ","<<_msg->y()<<","<<_msg->z()<< endl<<flush;
        double sx = (_msg->x()+1.5)*1000/5000; //stepsize = distance(sizeOfFocusObject) / numberOfFrames
        double sy = (_msg->y()+1.5)/5;
        double sz = (_msg->z()*0.5)/5;//stepsize = objectsize * 0.5 *1000/5000 ~ circle around half way
        if(_msg->z()>1){
            sz = 0.1;//=(1*0.5/5)
            numberofruns = ceil(_msg->z()*2);//divide by 0.5 
            gzmsg<<"[MOV]: Number of runs: "<<numberofruns<<endl;
        }else{
            numberofruns = 1;
        }
        currents = math::Vector3(sx,sy,sz);
        
        
    }
    
    // Called by the world update start event
    public: void OnUpdate()
    {
        if(!finished){
        math::Pose pose = this->model->GetWorldPose();
        frameNumber = frameNumber+1;
        //cout<<"threshold: "<<changingThreshold<<endl<<flush;
        if(frameNumber > nextThreshold){
            innerState = innerState+1;
            if(innerState == 11){ 
                innerState = 0;
                finished = 1;
            }else{
                int dur = 5000;
                if(innerState == 10){
                    if(numberofruns != currentrun){
                        currentrun=currentrun+1;//one run down
                        innerState = 0;//go up
                    }else{
                        dur = numberofruns * dur;//give it more time to go down according to the numbersofruns it went up
                    }
                }
                if(innerState==1 || innerState==9) dur = 2500; //doing half of x direction so only half of length is needed
                nextThreshold = rand() % durationNoise + dur -floor(durationNoise/2);//mean of threshold is 5000
                frameNumber = 0;
                cout << "current state " << innerState << "next: "<<nextThreshold<<endl;
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
        
        float yaw = -0.314;
        if(frameNumber % updateNoise == 0){
            // Apply noise to direction, orientation and size of speed vector
            float vx = ((rand()%10)-5)*directionNoise/10;
            float vy = ((rand()%10)-5)*directionNoise/10;
            float vz = ((rand()%10)-5)*directionNoise/10+0.01;//+0.01 to compensate gravity
            float ar = ((rand()%10)-5)*orientationNoise/10;
            float ap = ((rand()%10)-5)*orientationNoise/10;
            float ay = ((rand()%10)-5)*orientationNoise/10;
            speed = speed+((rand()%10)-5)*speedNoise/10;
            
            v=math::Vector3(vx,vy,vz);//translational velocity: z direction for gravity compensation
            a=math::Vector3(ar,ap,ay);//angular velocity
            //cout<<"randomness of the speed vector: "<<endl;
            //cout<<v[0]<<"; "<<v[1]<<"; "<<v[2]<<"; "<<a[0]<<"; "<<a[1]<<"; "<<a[2]<<"; "<<endl<<flush;
        }
	
        math::Vector3 vt(0,0,0);//temp translational speed vector
        math::Vector3 at(0,0,0);//temp angular speed vector
        switch(innerState){
            case 0://go up: outerState 0
                vt=v+math::Vector3(0,0,currents[2]*speed);
		at=a;
                outerState = 0;
                break;
            case 2:
            case 4:
            case 6:
            case 8://turn: 
                at=a+math::Vector3(0,0,yaw);
		vt=v;
                outerState = 2;
                break;
            case 1:
            case 5:
            case 9://flying in x direction according to stepsize in x directoin
                vt=v+math::Vector3(0, currents[0]*speed, 0);//still put in y direction because its relative
		at=a;
                outerState = 1;
                break;
            case 3://go forward absolute
            case 7://go backward absolute 
                vt=v+math::Vector3(0, currents[1]*speed, 0);//fly with speed of y direction
		at=a;
                outerState = 1;
                break;
            case 10://land
                vt=v+math::Vector3(0,0,-currents[2]);//*speed*numberofruns);//if we went 3x up we have to go 3x faster down
		at=a;
                outerState=3;
                break;
            default:
                gzerr << "[MOV]: unknown innerState."<<endl;
                break;
        }

        // Apply a small linear velocity to the model.
        vt = pose.rot * vt; // change relative speed Vector to absolute frame
        this->model->SetLinearVel(vt);
        this->model->SetAngularVel(at);
        //cout << "outerState: "<< outerState<< endl;
        // Sent the state out there!
        
        //cout<<"pose of the speed vector: "<<endl;
	//cout<<vt[0]<<"; "<<vt[1]<<"; "<<vt[1]<<"; "<<at[0]<<"; "<<at[1]<<"; "<<at[2]<<"; "<<endl<<flush;

        // Create the message
        msgs::Int msg;
        msg.set_data(outerState);
        // Send the message
        statePub->Publish(msg);
    
        }
    }

    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraMove)
}
