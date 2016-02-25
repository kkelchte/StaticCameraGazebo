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
    private: float speed;//size of velocity vector
    
    private: int frameNumber;
    private: int nextThreshold;
    private: int innerState;
    private: int outerState;
    private: bool finished;
    private: transport::NodePtr node;
    private: transport::PublisherPtr statePub;
    private: transport::PublisherPtr finishedPub;
    private: transport::SubscriberPtr finishedSub;   
        
    public: CameraMove() : ModelPlugin() {
        frameNumber=0;
        nextThreshold= rand() % durationNoise + 5000-floor(durationNoise/2);
        innerState=0;//the state in which the drone flies in 11 steps around the object
        outerState=0;//the relative control states needed as ground truth
        finished=false;
        speed = 0.5f;
	
        // Apply noise to direction, orientation and size of speed vector
            float vx = ((rand()%10)-5)*directionNoise/10;
	    float vy = ((rand()%10)-5)*directionNoise/10;
	    float vz = ((rand()%10)-5)*directionNoise/10+0.01;
	    float ar = ((rand()%10)-5)*orientationNoise/10;
	    float ap = ((rand()%10)-5)*orientationNoise/10;
	    float ay = ((rand()%10)-5)*orientationNoise/10;
	    speed = speed+((rand()%10)-5)*speedNoise/10;
        
        this->v = math::Vector3(vx,vy,vz);//translational velocity: z direction for gravity compensation
        this->a = math::Vector3(ar,ap,ay);//angular velocity
        //v = math::Vector3(0,0,0.01);//translational velocity: z direction for gravity compensation
        //a = math::Vector3(0,0,0);//angular velocity
        
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
        
        math::Pose pose = this->model->GetWorldPose();
        
        frameNumber = frameNumber+1;
        //cout<<"threshold: "<<changingThreshold<<endl<<flush;
        if(frameNumber > nextThreshold){
            innerState = innerState+1;
            nextThreshold = rand() % durationNoise + 5000-floor(durationNoise/2);//mean of threshold is 5000
            frameNumber = 0;
            cout << "current state " << innerState << "next: "<<nextThreshold<<endl;
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
        
        float yaw = -0.314;
        if(frameNumber % updateNoise == 0){
            // Apply noise to direction, orientation and size of speed vector
            float vx = ((rand()%10)-5)*directionNoise/10;
            float vy = ((rand()%10)-5)*directionNoise/10;
            float vz = ((rand()%10)-5)*directionNoise/10+0.01;
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
                vt=v+math::Vector3(0,0,0.2*speed);
		at=a;
                outerState = 0;
                break;
            case 1:
            case 9://go half left
                vt=v+math::Vector3(0,0.5*speed,0);
		at=a;
                outerState = 1;
                break;
            case 2:
            case 4:
            case 6:
            case 8://turn: 
                at=a+math::Vector3(0,0,yaw);
		vt=v;
                outerState = 2;
                break;
            case 3://go forward absolute
            case 5://go right absolute
            case 7://go backward absolute
                vt=v+math::Vector3(0, speed, 0);
		at=a;
                outerState = 1;
                break;
            case 10://land
                vt=v+math::Vector3(0,0,-0.2*speed);
		at=a;
                outerState=3;
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
