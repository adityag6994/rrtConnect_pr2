#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <openrave/planningutils.h>

#include <boost/bind.hpp>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split

#include <iostream>
#include <string.h>
#include <tuple>

#include "myplugin.h"

using namespace OpenRAVE;
using namespace std;

size_t MAX_ITERATIONS = 100;
int RRTNode::CurrentID = 0;
dReal GOAL_BIAS = 0.2;
dReal delta_Q = 0.05;
int flag = 0;

typedef std::vector<OpenRAVE::dReal> Config;
typedef boost::shared_ptr<RRTNode> Node;
typedef boost::shared_ptr<NodeTree> Tree;

class RRTModule : public ModuleBase
{
public:
    RRTModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        _penv = penv;
        RegisterCommand("MyCommand",boost::bind(&RRTModule::MyCommand,this,_1,_2),
                        "This is an example command hello");
    }
    virtual ~RRTModule() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        cout << "*PLUGIN STARTED" << endl;   

        //setenvironment and robot
        _penv->GetRobots(_robots);
        _robot = _robots.at(0);
        _robot->GetActiveDOFValues(_startConfig);
        _goalConfig = getStringVector(sout, sinput);
         
         //get limits
        _robot->GetActiveDOFLimits(_lowerLimit, _upperLimit);
        /*
        Lower Limits:  Config:  -0.564602 -0.3536 -2.12131 -2.00001 -10000 -2.00001 -10000 
        Upper Limits:  Config:   2.13539   1.2963 -0.15    -0.1      10000 -0.1      10000 
        */
        
        //we have goal and start config with us now, add them to NodeTree
        _weight.push_back(3.17104);
        _weight.push_back(2.75674);
        _weight.push_back(2.2325);
        _weight.push_back(1.78948);
        _weight.push_back(0);
        _weight.push_back(0.809013);
        _weight.push_back(0);



        _startNode = Node(new RRTNode());
        _startNode->setConfig(_startConfig);
        _startNode->setParent(NULL);
        Config test = _startNode->getConfig();
        
        //adding start to tree
        _mainTree.setrootNode(_startNode);
        _mainTree.nodeAdd(_startNode);

        _randomConfig = _startConfig;
        //Main Algorithm Starts Here
        count = 0;
        while(count++ < 100000){
            
            cout << "{ count :: " << count <<  " }" << endl;
            
            //------------------------get random node
            _randomConfig = randomnodeGenerator();
            // printConfig("Random",_randomConfig);  
            
            
            //------------------------check for nearest node in tree
            _nearestNode = NNNode();
            // printConfig("Nearest", _nearestNode->getConfig());

            
            
            //------------------------Get the step size
             _stepSize = step();
            //printConfig("Step",_stepSize);
            // _diffGoal = differenceCost();
            // cout << "_diffGoal ---> "<< _diffGoal << endl;
            // cout << ""
            if(flag){
                
                cout << "-------------------------------------------------------" <<endl;
                // cout << "_diffGoal ---> "<< _diffGoal << endl;
                _mainTree.printFullTree();
                cout << "Goal Found" << endl;
                break;
                
            }else{
            //------------------------get the next node
                EXTEND(count);
                cout << "Searching.."<< endl;

            }
            // extendExtend(count);
            //------------------------
            
            // if(flag == 1){
            //     break;
            // }else{
            //     cout << "Running..." << endl; 
            // }
            cout << endl;

        }
        cout << "SIZE =================> " << _mainTree.getTreeSize() << endl;
        // ExecuteTrajectory();
        cout << "*PLUGIN FINISHED" << endl;   

        return true;

    }

    //Print config function
    void printConfig(string s ,Config config){
        cout << s <<" Config:  " ;
        for(size_t i = 0; i < 7 ; i++){
                cout << config[i] << ' ';
            }
            cout << endl;
    }

    //getStringVector
    vector<dReal> getStringVector(std::ostream& sout, std::istream& sinput){
        vector<std::string> words;
        string s;
        ostringstream os;
        os<<sinput.rdbuf();
        s=os.str();
        std::vector<dReal> goal_config;
        boost::split(words, s, boost::is_any_of(", "), boost::token_compress_on);
        for(int i=0; i<7; i++){
            dReal num = atof(words.at(i).c_str());
            goal_config.push_back(num);
        }
        return goal_config;
    }

    //check if config are in limits
    bool checkConfigLimits(Config config){
    /*
        Lower Limits:  Config:  -0.564602 -0.3536 -2.12131 -2.00001 -10000 -2.00001 -10000 
        Upper Limits:  Config:   2.13539   1.2963 -0.15    -0.1      10000 -0.1      10000 
    */
        int sum = 0;
        for(size_t i = 0; i<7 ;i++){
            if(config[i] < _upperLimit[i] && config[i] > _lowerLimit[i]){
                sum++;
                cout << " i : " << i;
            }
        }
        cout << endl;
        cout << "sum :" << sum << endl;
        if(sum==7)
            return true;
        else
            return false;

    }

    //generate random node
    Config randomnodeGenerator(){

        dReal bias= RaveRandomFloat();
        Config temp_config = _goalConfig;
        
        if(bias < GOAL_BIAS){
            temp_config = _goalConfig;
        }else{
            do{

                for (size_t i = 0; i < 7; ++i) {

                    temp_config[i] = static_cast<dReal>(bias*(_upperLimit[i] - _lowerLimit[i]) + _lowerLimit[i]);
                    bias= RaveRandomFloat();//change random configration next time <common sense , not so common :'D  :| >

                   }    
                }while(!CheckCollision(temp_config));   
        }
        return temp_config; 
    
    }

    //nearest node 
   boost::shared_ptr<RRTNode> NNNode(){
    
        dReal temp_val = 0;
        dReal min_val = 100;
        dReal diff;
        size_t range = _mainTree.getTreeSize();
        vector< boost::shared_ptr<RRTNode> > temp_tree = _mainTree.getfullPath();
    
        for(size_t i=0; i<range; i++){
    
            Config temp_config = temp_tree[i]->getConfig();

            for(size_t j=0; j<7; j++){
                if(j == 4 || j == 6)
                  temp_val += pow((_randomConfig[j]/10000 - temp_config[j]/10000)*_weight[j], 2);
                else
                  temp_val += pow((-temp_config[j] + _randomConfig[j])*_weight[j], 2);
            }

            temp_val = sqrt(temp_val);
            // cout << "----------------------------------" << endl;
            // cout << "Temp :" << temp_val << "--";
            // printConfig("Temp", temp_config);
            // cout << "Min  :" << min_val << "--";
            diff = min_val - temp_val;
            // printThis("min_val - temp_val : ",  diff);

            if(diff > 0){
                // cout << "---------" << endl;
                min_val = temp_val;
                _nearestNode = temp_tree[i];
                _nearestNode->setUniqueId(i);
                // printThis("Min :", min_val);
                // printConfig("Min", _nearestNode->getConfig());
            }
        }
        // cout << "----------------------------------------------" << endl;
        // printThis("Final", min_val);
        // printConfig("Nearest ::",_nearestNode->getConfig());
        // cout << "min_index................." << _nearestNode->getUniqueId() << endl;

        return _nearestNode;
    }
   
   dReal differenceCost(Config config){
        dReal temp_val;
        
        for(size_t j=0; j<7; j++){
                if(j == 4 || j == 6)
                  temp_val += pow((config[j]/10000 - _goalConfig[j]/10000)*_weight[j], 2);
                else
                  temp_val += pow((-_goalConfig[j] + config[j])*_weight[j], 2);
            }

            temp_val = sqrt(temp_val);
            return temp_val;
   }

   //function for printing
    void printThis(string s,dReal g){
        cout << s << " " << g << endl;
    }

    void printThose(string s,dReal g, dReal h){
        cout << s << " " << g  << " " << h << endl;
    }
   //step size calculation
   Config step(){
        
        Config temp_step = _randomConfig;
        dReal magnitude = 0.0;

        _robot->SubtractActiveDOFValues(temp_step, _nearestNode->getConfig());

        for(size_t i=0; i<7; i++){
            magnitude += pow(temp_step[i], 2.0);
        }
        magnitude = sqrt(magnitude);

        for (size_t i = 0; i < 7; ++i) {
            temp_step[i] = (temp_step[i] / magnitude) * delta_Q;
        }

        return temp_step;
   }



   //extend
    void EXTEND(int count){
        //extend a step on nearest neibhour until obstacle is found
        //find new node config
        // int flag = 0;
        string s;
        //flag = 0;
        Config temp_new_config = _nearestNode->getConfig();

        do{
        // do{
            // printConfig("Before Adding Nearest Node", temp_new_config);
            temp_new_config = _nearestNode->getConfig();
            // cout << "count " << count << endl;
            // printConfig("Before Adding", temp_new_config);
            // printConfig("Extend Config :", temp_new_config);
            
            // if(_nearestNode->getConfig() == _goalConfig){
            //     cout << "--------------------------------------------Found Goal" << endl;
            //     flag = 1;
            // }

            // if(flag == 0){
            //     cout << " searching ... " << count << endl; 
            // }else{
            //     break;
            //     cout << " GOAL Found " ;
            // }

            for(size_t i=0; i<7 ; i++){
                temp_new_config[i] += _stepSize[i];
                //_robot->SetDOFValues(temp_new_config);
            }
            // printConfig("After Adding", temp_new_config);
            
             _mainTree.nodeAdd(Node(new RRTNode(temp_new_config, _nearestNode)));

            // if(temp_new_config == _goalConfig){
            //     return false;
            // }else{
            //     return true;
            // }
            _diffGoal = differenceCost(temp_new_config);
            if(_diffGoal < delta_Q){
                // printConfig("final config",)
                cout << _diffGoal << endl;
                flag = 1;
                break;
            }

            if(!CheckCollision(temp_new_config)){
                 //_mainTree.nodeAdd(_temp);
                // cout << " added ... " << endl;
                _nearestNode = _mainTree.getLast();       

            }



        }while(!CheckCollision(temp_new_config));// && checkConfigLimits(temp_new_config));
        // return s;
        // return true;
    }



    // bool extendExtend(int count){
    //     string pp;
    //     int j = count;
    //     pp = Extend(count);

    //     if(pp == "Searching .."){
    //         return true;
    //     }else{
    //         return false;
    //     }
    // }

    bool CheckCollision(Config config){
        EnvironmentMutex& lock = _penv->GetMutex();
        lock.lock();
        // if(config[5] < -2.00001){
        //     config[5] = -2.00000;
        // }
        _robot->SetActiveDOFValues(config);
        bool check = _penv->CheckCollision(_robot);
        _robot->SetActiveDOFValues(_startConfig);
        lock.unlock();
        //cout << "----" << endl;
        return check;
    }

    void ExecuteTrajectory(){
    
        EnvironmentMutex& lock = _penv->GetMutex();
        lock.lock();
        TrajectoryBasePtr traj = RaveCreateTrajectory(_penv);
        traj->Init(_robot->GetActiveConfigurationSpecification());

        vector< boost::shared_ptr<RRTNode> > temp_tree = _mainTree.getfullPath();

        for(int i=0; i< _mainTree.getTreeSize(); i++){
            traj->Insert(0, temp_tree[i]->getConfig());
        } 
            traj->Insert(0, _startConfig);
            planningutils::RetimeActiveDOFTrajectory(traj, _robot);
            _robot->GetController()->SetPath(traj);
            lock.unlock();
    }


    
private:
    //openrave variables
    vector<RobotBasePtr> _robots;

    EnvironmentBasePtr _penv;
    RobotBasePtr _robot;

    Config _startConfig ;
    Config _goalConfig;
    Config _randomConfig; //get it from random generator
    Config _lowerLimit;
    Config _upperLimit;
    Config _stepSize; //get from config
    Config _extendConfig_new; //has new config from extend function
    Config _weight; //weight matrix used while calculating cost

    Node _startNode;
    Node _nearestNode;
    Node _extendNode_new;
    // Node _temp;

    NodeTree _mainTree;
    
    dReal _diffGoal;

    size_t count;
};



// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtmodule" ) {
        return InterfaceBasePtr(new RRTModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("RRTModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

