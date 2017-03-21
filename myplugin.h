#ifndef MYPLUGIN_H
#define MYPLUGIN_H

    #include <vector>
    #include <tuple>
	  #include <algorithm>
    #include <boost/bind.hpp>
    #include <openrave/plugin.h>
    #include <openrave/openrave.h>

	using namespace std;
  using namespace OpenRAVE;

	typedef std::vector<OpenRAVE::dReal> Config;
	//typedef std::vector<RRTNode> RRTNodePtr;

	
	class RRTNode
    {
    	private:
    		Config _configration;
    		boost::shared_ptr<RRTNode> _parent;
        int uniqueId;
        static int CurrentID;

    	public:
    			RRTNode(Config configration,boost::shared_ptr<RRTNode> parent){
                        _configration = configration;
                        _parent = parent;
                        CurrentID++;
                        uniqueId = 0;
                        // cout << "RRTNode Generated.." << endl;
                    }

                RRTNode(){
                	CurrentID++;
                  cout << "First RRTNode Generated.." << endl;
                  uniqueId = 0;
                }
                
                ~RRTNode();
    		  
                void setConfig(Config config){_configration = config;}

                void setParent(boost::shared_ptr<RRTNode> parent){_parent = parent;}

                void setUniqueId(int num){uniqueId = num;}

    			boost::shared_ptr<RRTNode> getParent() const {return _parent;}		
    
    			Config getConfig() const {return _configration;}

                int getCurrentID() const {return CurrentID;}

                int getUniqueId(){return uniqueId;}
    };	  

    class NodeTree
    {

    	private:
    		vector< boost::shared_ptr<RRTNode> > _tree; 
            boost::shared_ptr<RRTNode> _rootNode;
    	
    	public:
    			NodeTree(){
            cout << "Node Tree Generated .." << endl;
          };
                
          NodeTree(boost::shared_ptr<RRTNode> root){
                    _rootNode = root;
          }
          ~NodeTree();
    		
          void setrootNode(boost::shared_ptr<RRTNode> newNode){
            _rootNode = newNode;
          }
    			void nodeAdd(boost::shared_ptr<RRTNode> newNode){
                    _tree.push_back(newNode);
                }

          int getTreeSize() const { return  _tree.size(); }  
		

    			boost::shared_ptr<RRTNode> getNode(int index) const{
    
                for(boost::shared_ptr<RRTNode> i : _tree){
                 	if(i->getCurrentID() == index){
                 		return i;
                 	}
                }
                throw 0;
          }
          
          boost::shared_ptr<RRTNode> getLast() {
            return _tree.back();
          }
          // vector<boost::shared_ptr<RRTNode>> getPath(boost::shared_ptr<RRTNode> from){
          //       //get path from present index to root
          //       vector< boost::shared_ptr<RRTNode> > path;
          //       boost::shared_ptr<RRTNode> temp = from;

          //       while(temp->getParent()){
          //           path.push_back(temp->getParent());
          //       }

          //       return path;
          // }

            vector<boost::shared_ptr<RRTNode>> getfullPath(){
                    return _tree;
            }

            boost::shared_ptr<RRTNode> getlastNode(){return _tree.back();}

            //delete last node
            void deleteNode(){
                  //delete last node
                 _tree.pop_back();
            }

            void deleteNodeIndex(int index){
                    
                for(size_t i=0; i<_tree.size(); i++){

                  if(_tree[i]->getCurrentID() == index){
                    _tree.erase(_tree.begin()+i);
                  }

                }
            }

    };

 #endif