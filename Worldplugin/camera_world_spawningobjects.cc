#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <iostream>
#include <boost/filesystem.hpp>

#include <sensors/sensors.hh>

using namespace std;

namespace gazebo
{
    class Camera_world : public WorldPlugin
    {
        private: bool reloading;
        
        private: string ground_plane;
        private: string sun;
        private: string surroundings; //index for current surrounding used in simulation
        
        private: string currentFocus; //index for current focus used in simulation
        private: vector<float> currentSize;//keep the size of the current focus object as set in the .world file 
        private: vector<vector<float>>  sizeList;
        private: vector<string>  focusList;
        private: string camera;
        private: string savingLocation;
        
        private: physics::WorldPtr world;
        private: transport::NodePtr node;
        private: transport::PublisherPtr locationPub;
        private: transport::SubscriberPtr finishedSub;
        private: transport::PublisherPtr finishedPub;
        private: transport::SubscriberPtr modelsub;
        private: transport::PublisherPtr sizePub;
        
        private: map<string,bool>  spawnMap;//Map with all focus objects to be spawned
        
        public: Camera_world() : WorldPlugin(){
            ground_plane = "ground_plane";
            spawnMap[ground_plane] = false;
            sun = "sun";
            //spawnMap[sun] = false;//The sun is lighting and so doesnt get spawned like normal model? or is spawned too soon.
            savingLocation="/esat/quaoar/kkelchte/simulation/no_location";
            reloading = false;
        }
        
        
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {
            
            this->world = _parent;
            
            // Keep the simulation paused
            this->world->SetPaused(true);
            
            //Standard objects
            world->InsertModelFile("model://"+ground_plane);
            world->InsertModelFile("model://"+sun);
            
            //Insert surroundings
            string tmp = _sdf->Get<string>("surroundings");
            if(tmp != ""){
                surroundings = tmp;
                cout << "sur: "<<surroundings<<endl;
            }else{
                surroundings = "Surroundingwalls";
            }
            spawnMap[surroundings] = false;
            world->InsertModelFile("model://"+surroundings);
            
            //Inser focus object
            //load in focusList vector
            tmp = _sdf->Get<string>("focus_objects");
            if(tmp != ""){
                string focusString = tmp;
                cout << "focus: "<<focusString<<endl;
                size_t found;
                found = focusString.find(" ",0);
                while(found!=std::string::npos){//as long as there are more " " found
                    string focusObject = focusString.substr(0,found);
                    focusList.push_back(focusObject);
                    focusString = focusString.substr(found+1);
                    found=focusString.find(" ");
                }
                focusList.push_back(focusString);
            }else{
                focusList.push_back("box");
            }
            currentFocus = focusList.back();
            focusList.pop_back();
            world->InsertModelFile("model://"+currentFocus);
            spawnMap[currentFocus] = false;
            
            //Load the size of the different focus objects
            tmp = _sdf->Get<string>("size_of_objects");
            if(tmp != ""){
                string sizeString = tmp;
                //cout << "size: "<<sizeString<<endl;
                size_t found;
                found = sizeString.find(";",0);
                while(found!=std::string::npos){//as long as there are more " " found
                    string sizeObjectString = sizeString.substr(0,found);//achieve the string "sx sy sz" of the object
                    vector<float>sizeObject(3,0);//create a vector with these floats
                    try{
                        sizeObject[0] = stof(sizeObjectString.substr(0,3));//get float from string
                        sizeObject[1] = stof(sizeObjectString.substr(4,3));
                        sizeObject[2] = stof(sizeObjectString.substr(8));
                    }
                    catch(std::out_of_range e){
                        gzerr << "failed to parse sizes of object models in world file"<<endl;
                    }
                    sizeList.push_back(sizeObject);//save it in the list
                    sizeString = sizeString.substr(found+1);//substract the interpreted part of the string
                    found=sizeString.find(";");
                }//if there is no more ; add the last vector
                vector<float>sizeObject(3,0);
                try{
                    sizeObject[0] = stof(sizeString.substr(0,3));//get float from string
                    sizeObject[1] = stof(sizeString.substr(4,3));
                    sizeObject[2] = stof(sizeString.substr(8));}
                catch(std::out_of_range e){
                    gzerr << "failed to parse sizes of object model"<<endl;
                }           
                sizeList.push_back(sizeObject);
            }else{
                vector<float> sizeObject(3,1);
                sizeList.push_back(sizeObject);
            }
            /*cout << "sizes: "<<sizeList[0][0]<<","<<sizeList[0][1]<<","<<sizeList[0][2]<<endl;
            cout << "sizes: "<<sizeList[1][0]<<","<<sizeList[1][1]<<","<<sizeList[1][2]<<endl;
            cout << "sizes: "<<sizeList[2][0]<<","<<sizeList[2][1]<<","<<sizeList[2][2]<<endl;
            cout << "sizes: "<<sizeList[3][0]<<","<<sizeList[3][1]<<","<<sizeList[3][2]<<endl;*/
            currentSize = sizeList.back();//load the current size
            sizeList.pop_back();
            
            //Insert camera from model            
            tmp = _sdf->Get<string>("camera");
            if(tmp != ""){
                camera = tmp;
                cout << "cam: "<<camera<<endl;
            }else{
                camera="distorted_camera_k";
            }
            world->InsertModelFile("model://"+camera);
            spawnMap[camera] = false;
            
            
            //Read saving location
            tmp = _sdf->Get<string>("savingLocation");
            if(tmp != ""){
                savingLocation = tmp;
                cout << "loc: "<<savingLocation<<endl;
            }
            boost::filesystem::path dir(savingLocation.c_str());
            if(boost::filesystem::create_directory(dir)) {
                    cout << "Success in creating: "<<savingLocation << "\n";
            }
            if(focusList.size()!=0){
                savingLocation = savingLocation+"/"+currentFocus;
                boost::filesystem::path dir(savingLocation.c_str());
                if(boost::filesystem::create_directory(dir)) {
                        cout << "Success in creating: "<<savingLocation << "\n";
                }
                
            }
            
            // Initialize the node listening to the moving camera with the world name
            this->node =  transport::NodePtr(new transport::Node());
            this->node->Init(_parent->GetName());
            this->finishedSub = node->Subscribe("/gazebo/moving/finished_state", &Camera_world::callbackFinishedTrajectory, this);
            this->modelsub = node->Subscribe("~/model/info", &Camera_world::callbackCheckLoad, this, true);
            this->locationPub = node->Advertise<msgs::GzString>("/gazebo/saving_location");
            this->finishedPub = node->Advertise<msgs::Int>("/gazebo/moving/finished_state");
            this->sizePub = node->Advertise<msgs::Vector3d>("/gazebo/moving/object_size");
            
            // Send the message with proper saving location
            msgs::GzString msg;
            msg.set_data(savingLocation);
            locationPub->Publish(msg);
            
        }
        
        //Called whenever an object is spawn:
        //Start simulation after all objects are ready
        private: void callbackCheckLoad(ConstModelPtr &_msg){
            cout<<"spawn message: "<<_msg->name()<<endl<<flush;
            bool ready = true;
            for (map<string,bool>::iterator it=spawnMap.begin(); it!=spawnMap.end(); ++it){
                string cname = it->first;
                bool value = it->second;
                cout << "Map: "<< cname << " => " << value << '\n';
                if(!value){
                    if(cname.compare(_msg->name())==0){//Set the spawned object that to true
                        spawnMap[cname]=true;
                        cout<<"set "<<cname<<" to "<< spawnMap[cname]<<endl;
                    }else{
                        ready = false;
                    }
                } 
            }
            //if the focusobject is spawned adapt the trajectory to its size
            if(currentFocus.compare(_msg->name())==0){
                msgs::Vector3d msg = msgs::Vector3d();//reference to 3d size vector
                msg.set_x(currentSize[0]);
                msg.set_y(currentSize[1]);
                msg.set_z(currentSize[2]);
                sizePub->Publish(msg);
                
            }
            //give the camera the right position
            if(camera.compare(_msg->name())==0){
                physics::ModelPtr cameraModel = world->GetModel(camera);
                if(cameraModel == NULL){gzerr<<"no model found"<<endl;exit();}
                else{ 
                    physics::LinkPtr cameraLink = cameraModel->GetLink("link");
                    //move the camera 0.25 tile away from the surface of the object
                    math::Pose p (-0.75-currentSize[1]/2,0.0,0.05,0,0,0);
                    const math::Pose& campose = p;
                    cameraLink->SetWorldPose(campose, true, true);
                }
            }
            //start simulation when everything is ready
            if(ready){
                reloading = false;
                msgs::Int msg;
                msg.set_data(0);
                finishedPub->Publish(msg);
            }
            cout<<flush;
        }
        
        
        //Called whenever the trajectory is finished
        private: void callbackFinishedTrajectory(ConstIntPtr &_msg){
            //cout<<"trajectory finished? "<< _msg->data()<< endl<<flush;
            if(_msg->data()==1){
                if(!reloading){ 
                    reload();
                    reloading=true;
                }
            }
            if(_msg->data()==0){
                this->world->SetPaused(false);
            }
        }
        //quit the program
        private: void exit(){
            cout << "Gazebo is shutting down... "<<endl<<flush;
            transport::PublisherPtr exitPub = node->Advertise<msgs::ServerControl>("/gazebo/server/control");
            msgs::ServerControl msg;
            msg.set_stop(true);
            exitPub->Publish(msg);
        }
        
        //reload a simulation
        private: void reload(){
            
            this->world->SetPaused(true);
            //world->PrintEntityTree();
            // Update focus model
            string prevFocus = currentFocus;
            currentFocus = focusList.back();
            focusList.pop_back();
            currentSize = sizeList.back();
            sizeList.pop_back();
            physics::ModelPtr prevFocusModel = world->GetModel(prevFocus);
            if(prevFocusModel==NULL){cerr<<"no model found"<<endl<<flush;exit();}
                
            //delete current focus object
            transport::requestNoReply(this->node, "entity_delete", prevFocusModel->GetName());
            spawnMap.erase(prevFocus);
                
            //Load new focus object
            world->InsertModelFile("model://"+currentFocus);
            spawnMap[currentFocus]=false;
            
            //cout<<currentFocus<<" is next"<<endl<<flush;
            savingLocation.replace(savingLocation.find(prevFocus), prevFocus.length(), currentFocus);
            
            boost::filesystem::path dir(savingLocation.c_str());
            if(boost::filesystem::create_directory(dir)) {
                    cout << "Success in creating: "<<savingLocation << "\n";
            }
            
            // Change location for saving the images:
            //cout << "savingLocation: "<< savingLocation <<endl<<flush;
            msgs::GzString msg;
            msg.set_data(savingLocation);
            locationPub->Publish(msg);
            
            physics::ModelPtr cameraModel = world->GetModel(camera);
            if(cameraModel == NULL){gzerr<<"no model found"<<endl;exit();}
            else{ 
                physics::LinkPtr cameraLink = cameraModel->GetLink("link");
                //move the camera 0.25 tile away from the surface of the object
                math::Pose p (-0.75-currentSize[1]/2,0.0,0.05,0,0,0);
                const math::Pose& campose = p;
                cameraLink->SetWorldPose(campose, true, true);
            }
            

        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(Camera_world)
}
