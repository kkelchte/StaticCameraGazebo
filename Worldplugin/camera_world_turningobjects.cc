#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <sensors/sensors.hh>

using namespace std;

namespace gazebo
{
    class Camera_world : public WorldPlugin
    {
        private: bool reloading;
        
        private: string ground_plane;
        private: string sun;
        private: int si; //index for current surrounding used in simulation
        private: vector<string>  surroundingsList;
        private: int fi; //index for current focus used in simulation
        private: vector<string>  focusList;
        private: vector<physics::ModelPtr> focusModels;
        private: string camera;
        private: string savingLocation;
        private: math::Pose under;
        private: math::Pose above;
                
        
        private: physics::WorldPtr world;
        private: transport::NodePtr node;
        private: transport::PublisherPtr locationPub;
        private: transport::SubscriberPtr finishedSub;
        private: transport::PublisherPtr finishedPub;
        private: transport::SubscriberPtr modelsub;
        
        private: map<string,bool>  spawnMap;//Map with all focus objects to be spawned
        
        public: Camera_world() : WorldPlugin(){
            ground_plane = "ground_plane";
            //spawnMap[ground_plane] = false;
            sun = "sun";
            //spawnMap[sun] = false;//The sun is lighting and so doesnt get spawned like normal model? or is spawned too soon.
            camera="distorted_camera_k";
            //spawnMap[camera] = false;
            savingLocation="/esat/quaoar/kkelchte/simulation/data";
            reloading = false;
            under=math::Pose(0,0,-1,3.14,0,0);
            above=math::Pose(0,0,0,0,0,0);
                
        }
        
        
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {
            this->world = _parent;
            //Standard objects
            world->InsertModelFile("model://"+ground_plane);
            world->InsertModelFile("model://"+sun);
            
            //Insert surroundings
            string tmp = _sdf->Get<string>("surroundings");
            if(tmp != ""){
                string surroundingsString = tmp;
                cout << "sur: "<<surroundingsString<<endl;
                size_t found;
                found = surroundingsString.find(" ",0);
                while(found!=std::string::npos){//as long as there are more " " found
                    string surroundingsObject = surroundingsString.substr(0,found);
                    surroundingsList.push_back(surroundingsObject);
                    surroundingsString = surroundingsString.substr(found+1);
                    found=surroundingsString.find(" ");
                }
                surroundingsList.push_back(surroundingsString);
                //for(int i = 0;i!=focusList.size();i++) cout<<"final: "<<focusList[i]<<endl<<flush;
            }else{
                surroundingsList.push_back("Surroundingwalls");
            }
            si = 0;
            //spawnMap[surroundingsList.at(si)] = false;
            world->InsertModelFile("model://"+surroundingsList.at(si));
            
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
                    //focusList.push_back(focusObject);
                    focusString = focusString.substr(found+1);
                    found=focusString.find(" ");
                    world->InsertModelFile("model://"+focusObject);
                    spawnMap[focusObject] = false;
                }
                //focusList.push_back(focusString);
                world->InsertModelFile("model://"+focusString);
                spawnMap[focusString] = false;
            }else{
                //focusList.push_back("box");
                spawnMap["box"] = false;
                world->InsertModelFile("model://box");
            }
            fi = 0;
            
            //Insert camera from model            
            tmp = _sdf->Get<string>("camera");
            if(tmp != ""){
                camera = tmp;
                cout << "cam: "<<camera<<endl;
            }
            world->InsertModelFile("model://"+camera);
            
            //Read saving location
            tmp = _sdf->Get<string>("savingLocation");
            if(tmp != ""){
                savingLocation = tmp;
                cout << "loc: "<<savingLocation<<endl;
            }
            if(focusList.size()!=0){
                savingLocation = savingLocation+"/"+focusList.at(fi);
            }
            if(surroundingsList.size()!=0){
                savingLocation = savingLocation+"/"+surroundingsList.at(si);
            }
            
            // Initialize the node listening to the moving camera with the world name
            this->node =  transport::NodePtr(new transport::Node());
            this->node->Init(_parent->GetName());
            this->finishedSub = node->Subscribe("/gazebo/moving/finished_state", &Camera_world::callbackFinishedTrajectory, this);
            this->modelsub = node->Subscribe("~/model/info", &Camera_world::callbackCheckLoad, this, true);
            this->locationPub = node->Advertise<msgs::GzString>("/gazebo/savingLocation");
            this->finishedPub = node->Advertise<msgs::Int>("/gazebo/moving/finished_state");
            
            // Send the message with proper saving location
            msgs::GzString msg;
            msg.set_data(savingLocation);
            locationPub->Publish(msg);
            
            // Keep the simulation paused
            this->world->SetPaused(true);
            
        }
        
        //Called whenever an object is spawn:
        //Start simulation after all objects are ready
        private: void callbackCheckLoad(ConstModelPtr &_msg){
            //cout<<"spawn message: "<<_msg->name()<<endl<<flush;
            bool ready = true;
            for (map<string,bool>::iterator it=spawnMap.begin(); it!=spawnMap.end(); ++it){
                string cname = it->first;
                bool value = it->second;
                //std::cout << "Map: "<< cname << " => " << value << '\n';
                if(!value){
                    if(cname.compare(_msg->name())==0){//Set the spawned object that to true
                        spawnMap[cname]=true;
                        //std::cout<<"set "<<cname<<" to "<< spawnMap[cname]<<endl;
                        //if(find(focusList.begin(), focusList.end(), cname)!=focusList.end()){//if focusmodel is not selected turn it upside down
                            //cerr<<"0";
                            physics::ModelPtr model = world->GetModel(cname);
                            focusModels.push_back(model);
                            focusList.push_back(cname);
                            if(focusList.size()!=1){//turn all except the first model upside down
                                const math::Pose& unpose = this->under;
                                model->GetLink("link")->SetWorldPose(unpose, true, true);
                                cout << "Turned "<<cname<<" around."<<endl;
                            }
                        //}
                    }else{
                        ready = false;
                    }
                } 
            }
            if(ready){
                reloading = false;
                this->world->SetPaused(false);
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
        }
        //quit the program
        private: void exit(){
            transport::PublisherPtr exitPub = node->Advertise<msgs::ServerControl>("/gazebo/server/control");
            msgs::ServerControl msg;
            msg.set_stop(true);
            exitPub->Publish(msg);
        }
        //run through focus objects and surrounding object lists
        private: void updateIndices(){
            fi = fi+1;
            if(fi==focusList.size()){
                fi = 0;
                si = si+1;
                if(si==surroundingsList.size()){
                    cerr<<"Run through all simulation lists"<<endl;
                    exit();
                }
            }
        }
        //reload a simulation
        private: void reload(){
            this->world->SetPaused(true);
            
            int oldfi = fi;//save previous index
            int oldsi = si;//
            updateIndices();
            
            //world->PrintEntityTree();
            // Update focus model if needed:
            if(oldfi != fi){
                //Put the previous model under the floor
                /*physics::ModelPtr prevFocusModel = world->GetModel(focusList.at(oldfi));
                if(prevFocusModel==NULL){cerr<<"no model found"<<endl<<flush;exit();}*/
                
                //delete current focus object
                //transport::requestNoReply(this->node, "entity_delete", prevFocusModel->GetName());
                //spawnMap.erase(focusList.at(oldfi));
                
                //hide previous focus object
                const math::Pose& unpose = this->under;
                focusModels[oldfi]->GetLink("link")->SetWorldPose(unpose, true, true);
                //show new focus object
                const math::Pose& newpose = this->above;
                focusModels[fi]->GetLink("link")->SetWorldPose(newpose, true, true);
                
                //Load new focus object
                //world->InsertModelFile("model://"+focusList.at(fi));
                //spawnMap[focusList.at(fi)]=false;
                
                cout<<focusList[fi]<<" is next"<<endl<<flush;
                savingLocation.replace(savingLocation.find(focusList.at(oldfi)), focusList.at(oldfi).length(), focusList.at(fi));
            }
            // Update surroundingsmodel model if needed:
            /*if(oldsi != si){
                physics::ModelPtr surroundingsModel = world->GetModel(surroundingsList.at(oldsi));
                if(surroundingsModel==NULL){cerr<<"no model found"<<endl<<flush;exit();}
                //delete current surroundings object
                transport::requestNoReply(this->node, "entity_delete", surroundingsModel->GetName());
                spawnMap.erase(surroundingsList.at(oldsi));
                //Load new surroundings object
                spawnMap[surroundingsList.at(si)]=false;
                cout<<surroundingsList.at(si)<<" is next"<<endl<<flush;
                world->InsertModelFile("model://"+surroundingsList.at(si));
                savingLocation.replace(savingLocation.find(surroundingsList.at(oldsi)), surroundingsList.at(oldsi).length(), surroundingsList.at(si));
            }*/
            // Change location for saving the images:
            cout << "savingLocation: "<< savingLocation <<endl<<flush;
            msgs::GzString msg;
            msg.set_data(savingLocation);
            locationPub->Publish(msg);
            
            //Change pose of camera object
            physics::ModelPtr cameraModel = world->GetModel(camera);
            if(cameraModel == NULL){cerr<<"no model found"<<endl;exit();}
            else{ 
                physics::LinkPtr cameraLink = cameraModel->GetLink("link");
                math::Pose p (-1.5,0.05,0.05,0,0,0);
                const math::Pose& campose = p;
                cameraLink->SetWorldPose(campose, true, true);
            }
                
            /* ALTERNATIVE WAYS OF DELETING MODEL
             * cout<<focusModel->GetName()<<endl<<flush;
            int total = focusModel->GetChildCount();
            for(int i=0;i<total;i++){
                //cout<<"Removing: "<<focusModel->GetChildCount()<<" children."<<endl<<flush;
                focusModel->RemoveChild(i);
            }*/
            /*uint32_t id = focusModel->GetChild();
            physics::BasePtr parent = focusModel->GetParent();
            parent->RemoveChild(id);*/
            // show content:
            /*for(map<string,bool>::iterator it=spawnMap.begin(); it!=spawnMap.end(); ++it){
                cout << it->first << " => " << it->second << '\n';
            }*/
            /*string n = focusModel->GetName();
            const string & modelName = n;
            world->RemoveModel(modelName);*/
            //gazebo::common::Time::MSleep(100);
            
            //-----------------------------------
            //world->PrintEntityTree();
            //get camera_gt plugin from camera object with _sdf
            /*physics::ModelPtr cameraModel = world->GetModel(camera);
            if(cameraModel == NULL){cerr<<"no model found"<<endl;gazebo::shutdown();}
            else{ 
                sdf::ElementPtr camerasdf = cameraModel->GetSDF();
                camera = "camera_"+currentFocus;
                camerasdf->GetAttribute("name")->SetFromString(camera);
                sdf::ElementPtr myplug = camerasdf->GetElement("link")->GetElement("sensor")->GetElement("plugin");                
                myplug->GetElement("location")->Set<string>("/esat/quaoar/kkelchte/simulation/data/"+fius);
                //myplug->GetElement("location")->PrintValues("the values: ");
                
                sdf::SDF cameraSDF;
                string camerasdfString = "<sdf version ='1.5'> \n" + camerasdf->ToString("") + "\n </sdf>";
                transport::requestNoReply(this->node, "entity_delete", cameraModel->GetName());
                
                //cout<<camerasdfString<<endl<<flush;
                cameraSDF.SetFromString(camerasdfString);
                world->InsertModelSDF(cameraSDF);
                
                /*physics::LinkPtr cameraLink = cameraModel->GetLink("link");
                math::Pose p (-1.5,0.05,0.05,0,0,0);
                const math::Pose& pose = p;
                cameraLink->SetWorldPose(pose, true, true);
                
                
            }*/

        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(Camera_world)
}
