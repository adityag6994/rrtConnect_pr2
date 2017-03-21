
    #ifndef RRT_CONNECT_H
	#define RRT_CONNECT_H
    
    #include <vector>
    #include <boost/bind.hpp>
    #include <openrave/plugin.h>
    #include <openrave/openrave.h>

	using namespace std;
    using namespace OpenRAVE;

	int index;

    typedef std::vector<OpenRAVE::dReal> Node;
    //typedef shared_ptr<Base>  

    class RRTNode
    {
    	private:
    		Node _configration;
    		shared_ptr<RRTNode> _parent = NULL;
            
            static int CurrentID;

    	public:
    			RRTNode(const Node configration,const shared_ptr<RRTNode> parent){
                        _configration = configration;
                        _parent = parent;
                        RRTNode::CurrentID++;
                    }

                RRTNode();
                
                ~RRTNode();
    		  
                void setConfig(Node config){_configration = config;}

                void setParent(shared_ptr<RRTNode> parent){_parent = parent;}

    			shared_ptr<RRTNode> getParent() const {return _parent;}		
    
    			Node getConfig() const {return _configration;}

                int getCurrentID() const {return CurrentID;}

    };

    
    class NodeTree
    {

    	private:
    		vector<shared_ptr<RRTNode>> _tree; 
            shared_ptr<RRTNode> rootNode;
    	
    	public:
    			NodeTree();
                NodeTree(shared_ptr<Node> root){
                    _tree.push_back(root);
                    rootNode = root;
                }
                ~NodeTree();
    		
    			void nodeAdd(shared_ptr<RRTNode> newNode){
                    _tree.push_back(newNode);
                }

                int getTreeSize() const { return  _tree.size(); }  
		

    			shared_ptr<RRTNode> getNode(int index) const{
                    vector<RRTNode*>::iterator it;
                    //shared_ptr<RRTNode> item;
                    for(it=_tree.begin(); it!=_tree.end(); ++it){
                        if((*it)->getCurrentID == index){
                            //item = (*it);
                            break;
                        }
                    }
                    return item;

                }

                vector<shared_ptr<RRTNode>> getPath(shared_ptr<RRTNode> from){
                    //get path from present index to root
                    vector<RRTNode*> path;
                    shared_ptr<RRTNode> temp = from;

                    while(temp->getParent()){
                        path.push_back(temp->getParent());
                    }

                    return path;
                }

                vector<shared_ptr<RRTNode>> getfullPath(){
                    return _tree;
                }

                shared_ptr<RRTNode> getlastNode(){return _tree.back();}

                void deleteNode(){
                    //delete last node
                    _tree.pop_back();
                }

                void deleteNodeIndex(int index){
                    
                    vector<shared_ptr<RRTNode>>::iterator it;
                    //RRTNode* item;
                    for(it=_tree.begin(); it!=_tree.end(); ++it){
                        if((*it)->getCurrentID == index){
                            _tree.erase(*it);
                        }
                    }
                }

    };

    #endif