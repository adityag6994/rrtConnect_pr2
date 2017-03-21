#include <openrave/openrave.h>
#include <openrave/plugin.h>

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
dReal delta_Q = 0.1;

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
        //RegisterCommand("load",boost::bind(&RRTModule::Load, this,_1,_2),"loads a given file");
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
        // printConfig("Lower Limits: ",_lowerLimit);
        // printConfig("Upper Limits: ",_upperLimit);
        // cout << "start config .. :: " ;
        //printConfig(_startConfig);
        // printConfig(_goalConfig);
        //we have goal and start config with us now
        //add them to NodeTree
        _weight.push_back(6);
        _weight.push_back(5);
        _weight.push_back(4);
        _weight.push_back(3);
        _weight.push_back(0);
        _weight.push_back(1);
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
        //get a random config
        count = 0;
        while(count++ < 10){
            //------------------------get random node
            // cout << "...................................... COUNT ::::: " <<count << endl;
            _randomConfig = randomnodeGenerator();
            printConfig("Random",_randomConfig);  
            
            
            //------------------------check for nearest node in tree
            _nearestNode = NNNode();
            //cout << _mainTree.getTreeSize() << endl;
            // printConfig("nearest config",_nearestNode->getConfig());
            cout << "{ count :: " << count << " || unique id :: " << _nearestNode->getUniqueId() << " }" << endl;
            
            
            //------------------------Extend into that direction
            _stepSize = step();
            //printConfig("Step",_stepSize);

            EXTEND(count);
            // printConfig("Extend",_stepSize);
            // _mainTree.nodeAdd(Node(new RRTNode(_extendConfig_new, _nearestNode)));

            //_extendNode_new = Node(new RRTNode());
            //_extendNode_new->setConfig(_extendConfig_new);
            //_extendNode_new->setParent(_nearestNode);

            // cout << "Tree Size :: "<< _mainTree.getTreeSize() << endl;
             // _mainTree.nodeAdd(newNode);

            //------------------------
            cout << endl;

        }

        vector< boost::shared_ptr<RRTNode> > temp_tree_1 = _mainTree.getfullPath();
        
        // for(size_t i=0; i<_mainTree.getTreeSize(); i++){
        //     printConfig("stored config: ",temp_tree_1[i]->getConfig());
        // }


        cout << "*PLUGIN FINISHED" << endl;   

        return true;

    }

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

    bool checkConfigLimits(Config config){
    /*
        Lower Limits:  Config:  -0.564602 -0.3536 -2.12131 -2.00001 -10000 -2.00001 -10000 
        Upper Limits:  Config:   2.13539   1.2963 -0.15    -0.1      10000 -0.1      10000 
    */
        int sum = 0;
        for(size_t i = 0; i<7 ;i++){
            if(config[i] < _upperLimit[i] && config[i] > _lowerLimit[i]){
                sum++;
            }
        }

        if(sum==7)
            return true;
        else
            return false;

    }

    //generate random node
    Config randomnodeGenerator(){

        //cout << "--Random Generator--" << endl;
        dReal bias= RaveRandomFloat();
        Config temp_config = _goalConfig;
        // cout << "bias : " << bias << endl;
        if(bias < GOAL_BIAS){
            temp_config = _goalConfig;
            // printConfig("goal config: ",temp_config);
        }else{
            do{
                // cout << "bias_ before ::: " << bias << endl;
                // cout << "bias : " << bias << endl;
                // printConfig("r: ",temp_config);
                for (size_t i = 0; i < 7; ++i) {
                    // printConfig("ra: ",temp_config);
                    // cout << "bias ::: " << bias << endl;
                    temp_config[i] = static_cast<dReal>(bias*(_upperLimit[i] - _lowerLimit[i]) + _lowerLimit[i]);
                    bias= RaveRandomFloat();//change random configration next time <common sense , not so common :'D  :| >
                    // cout << "bias ::: " << bias << endl;
                    //printConfig("random: ",temp_config);
                   }    
                }while(CheckCollision(temp_config));   
        }
        // cout << "bias ::: " << bias << endl;
        // printConfig("random config: ",temp_config);
        return temp_config; 
    }

    //nearest node 
   boost::shared_ptr<RRTNode> NNNode(){
    
        dReal temp_val = 0;
        dReal min_val = 100;
        size_t range = _mainTree.getTreeSize();
        vector< boost::shared_ptr<RRTNode> > temp_tree = _mainTree.getfullPath();
        // cout << "range :: " << range << endl;
        for(size_t i=0; i<range; i++){
            //Node temp_node = _mainTree[i];
            
            Config temp_config = temp_tree[i]->getConfig();


            for(size_t j=0; j<7; j++){
                if(j == 4 || j == 6)
                  temp_val += pow((_randomConfig[j]/10000 - temp_config[j]/10000)*_weight[j], 2);
                else
                  temp_val += pow((-temp_config[j] + _randomConfig[j])*_weight[j], 2);
                // temp_val += (- temp_config[j] + _randomConfig[j])*_weight[j];// TODO j+1 for weight...can be opposite
              // cout << "temp_val--- " << temp_val << endl;
            }

            temp_val = sqrt(temp_val);
            
            // cout << "temp_val " << temp_val << endl;
            // cout << "outside " << i << endl;
             // cout << "temp_val " << temp_val << endl;
            // cout << "min_index.....|||." << _nearestNode->getCurrentID() << endl;
            // printConfig("nn ",temp_tree[i]->getConfig());
            if(temp_val < min_val){
                // cout << "inseide " << i << endl;
                // cout << "min_index................." << temp_tree[i]->getCurrentID() << endl;
                // cout << "min_val " << min_val << endl;
                min_val = temp_val;
                _nearestNode = temp_tree[i];
                _nearestNode->setUniqueId(i);
                // printConfig("nn-- ",_nearestNode->getConfig());
                // printConfig("nn ",temp_tree[i]->getConfig());
                //_nearestNode->getCurrentID() = temp_tree[i]->getCurrentID
                // cout << "min_val " << min_val << endl;
                // cout << "min_index................." << _nearestNode->getCurrentID() << endl;
                // cout << "min_index................." << temp_tree[i]->getCurrentID() << endl;
                // cout << "min_index................." << _nearestNode->getUniqueId() << endl;
            }
        }
        printConfig("Nearest Config ::",_nearestNode->getConfig());
        // cout << "min_index................." << _nearestNode->getUniqueId() << endl;

        return _nearestNode;
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
        Config temp_new_config = _nearestNode->getConfig();

        do{
        // do{
            // printConfig("Before Adding Nearest Node", temp_new_config);
            temp_new_config = _nearestNode->getConfig();
            cout << "count " << count << endl;
            printConfig("Before Adding", temp_new_config);
            // printConfig("Extend Config :", temp_new_config);
            
            if(_nearestNode->getConfig() == _goalConfig){
                cout << "--------------------------------------------Found Goal" << endl;
            }

        // do{
            for(size_t i=0; i<7 ; i++){
                temp_new_config[i] += _stepSize[i];
                //_robot->SetDOFValues(temp_new_config);
            }
            printConfig("After Adding", temp_new_config);
            
            // printConfig("Step Config :", _stepSize);
            // printConfig("New Config :", temp_new_config);

            //Node _temp = Node(new RRTNode(temp_new_config, _nearestNode));

             _mainTree.nodeAdd(Node(new RRTNode(temp_new_config, _nearestNode)));

            if(!CheckCollision(temp_new_config)){
                 //_mainTree.nodeAdd(_temp);
                _nearestNode = _mainTree.getLast();       

            }
        // }while();
        }while(CheckCollision(temp_new_config));// && checkConfigLimits(temp_new_config));

        //      _mainTree.nodeAdd(Node(new RRTNode(_extendConfig_new, _nearestNode)));
        // }while(CheckCollision(temp_new_config)); 
        // //printConfig("Extended", temp_new_config);
        //return temp_new_config;
    }

    //check condition for extend
    // string Connect(){
    //     string s;
    //     do{
    //            s = EXTEND(); 
    //     }while(s == "GO");

    //     return s;
    // }


    bool CheckCollision(Config config){
        EnvironmentMutex& lock = _penv->GetMutex();
        lock.lock();
        _robot->SetActiveDOFValues(config);
        bool check = _penv->CheckCollision(_robot);
        _robot->SetActiveDOFValues(_startConfig);
        lock.unlock();
        //cout << "----" << endl;
        return check;
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

