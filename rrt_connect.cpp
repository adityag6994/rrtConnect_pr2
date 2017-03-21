#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include "rrt_connect.h"

#include <iostream>
#include <math>

using namespace std;
using namespace OpenRAVE;



int RRTNode::CurrentID = 0;

/*************************************************PART_1**************************************************/

/*************************************************PART_2**************************************************/
    
class RRTConnect_Module : public ModuleBase
{
    public:

        RRTConnect_Module(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
            _penv = penv;
            __description = "RRTConnect PR2";
            RegisterCommand("RRTConnect",boost::bind(&RRTConnect_Module::Extend,this,_1,_2),
            "Gives a path calculated from RRTConnect.");
        } 
    
        RRTConnect(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv))    
    
        void randomnodeGenerator();
        Node getrandomConfig(){return randomConfig;}

        shared_ptr<RRTNode> nnNode(Node random_config);
        //shared_ptr<RRTNode> getNNNode(){return nearestNode;}

        Node stepSize();
        //Node getstepCongif(){return stepConfig;}

        void Extend(Node Goal, Node Start);

    private:
        
        int GOAL_BIAS = 0.4;//bias towards goal
        float delta_Q = 1; //step size(minimum)


        shared_ptr<RRTNode> currentNode; //the latest node added
        shared_ptr<RRTNode> nearestNode; //nearest node to random config
        shared_ptr<RRTNode> lastNode; //last node in tree, if goal congig
        shared_ptr<RRTNode> base; //first node in tree _ start node

        Node randomConfig; //saves the random config
        Node Goal; //start config
        Node Start; //goal config
        Node stepConfig; //step to be taken into the direction of random node
        
        shared_ptr<NodeTree> tree; //main tree
        shared_ptr<NodeTree> path; //final path

};

shared_ptr<RRTNode> RRTConnect::nnNode(Node random_config){
        
        vector<shared_ptr<RRTNode>> tempTree = tree->getfullPath();
        vector<shared_ptr<RRTNode>>::iterator it;
        dReal  temp=0;
        dReal  min=0;
        Node tempConfig = random_config;
        shared_ptr<RRTNode> final;

        for(it = tempTree.begin(); it != tempTree.end(); ++it){
            Node tempNode = (*it)->getConfig();
            for(int i=0; i < 7; i++){
                temp += (tempNode[i] - tempConfig[i])*(i+1);            
            }
            if(temp < min){
                min = temp;
                final = (*it);
            }  
        }
        //nearestNode = final;
        return final;
    }

void RRTConnect::randomnodeGenerator(){
    
        dReal bias = (rand() % 100 + 1)/100; //0.1 - 1
        Node temp;
    
        /*change according to limits*/

        if(bias < GOAL_BIAS)
            temp = Goal;
        else{
            do{
                for (size_t i = 0; i < 7; ++i) {
                    temp[i] = (rand() * (static_cast<dReal>(joint_limits_upper[i] - joint_limits_lower[i])) + joint_limits_lower[i]);                   
                }while(collisoinCheck(temp))    
            }
        }
        randomConfig = temp;    
    }

Node RRTConnect::stepSize(Node random_config, shared_ptr<RRTNode> nearestNode){

        stepConfig = randomConfig;
        robot->SubtractActiveDOFValues(stepConfig, nearestNode->getConfig());

        //calculate magnitude
        dReal magnitude = 0.0;

        for(int i=0; i<7; i++){
            magnitude += pow(stepConfig[i], 2.0);
        }
        magnitude = sqrt(magnitude);

        for (size_t i = 0; i < 7; ++i) {
            stepConfig[i] = (stepConfig[i] / magnitude) * delta_Q;
        }

        return stepConfig;
    }
     
void RRTConnect::Extend(Node goal, Node start){

    //add start goal to tree
    base->setConfig(start);
    tree->nodeAdd(base);

    //while random naode is not goal
    lastNode = tree->getlastNode();
    
    if(lastNode->getConfig() == goal){
        cout << "Reached" << endl;
        break;
    }else{
        //call random config
        Node rand_config_temp = this->getrandomConfig();
        //calculate nearest neibhour
        shared_ptr<RRTNode> nn_node_temp = this->randomnodeGenerator(rand_config_temp);
        //get the step size
        Node step_config_temp = stepConfig(rand_config_temp, nn_node_temp);
        //add this node in the end
        Node temp = nn_node_temp->getConfig();
        //RRTNode pointer to be added
        boost::shared_ptr<RRTNode> new_node_ptr(RRTNode());

        //extend the node considering collision
        do{
            for(int i=0 ; i<7; ++i){
               temp[i] =  temp[i] + step_config_temp[i];
            }  
        }while(collisoinCheck(temp)

        new_node_ptr->setParent(nn_node_temp);
        new_node_ptr->setConfig(temp);
        lastNode = new_node_ptr;
    }

}

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtConnect" ) {
        return InterfaceBasePtr(new RRTConnect(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("MyNewModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

