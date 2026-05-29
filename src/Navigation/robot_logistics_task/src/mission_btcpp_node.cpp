#include <chrono>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
using namespace std::chrono_literals;
static double steady_seconds(){using clock=std::chrono::steady_clock;static const auto start=clock::now();return std::chrono::duration<double>(clock::now()-start).count();}
static double wall_seconds(rclcpp::Node& n){return n.now().seconds();}
static std::string read_file_or_empty(const std::string& path){std::ifstream ifs(path);if(!ifs.good())return "";std::stringstream ss;ss<<ifs.rdbuf();return ss.str();}
static std::string read_file_required(const std::string& path){std::ifstream ifs(path);if(!ifs.good())throw std::runtime_error("failed to open file: "+path);std::stringstream ss;ss<<ifs.rdbuf();return ss.str();}
static void ensure_parent_dir(const std::string& path){std::filesystem::path p(path);if(p.has_parent_path())std::filesystem::create_directories(p.parent_path());}
static bool get_string_json(const std::string& json,const std::string& key,std::string& out){auto p=json.find("\""+key+"\"");if(p==std::string::npos)return false;p=json.find(":",p);if(p==std::string::npos)return false;p=json.find("\"",p);if(p==std::string::npos)return false;auto q=json.find("\"",p+1);if(q==std::string::npos)return false;out=json.substr(p+1,q-p-1);return true;}
static bool get_number_json(const std::string& json,const std::string& key,double& out){auto p=json.find("\""+key+"\"");if(p==std::string::npos)return false;p=json.find(":",p);if(p==std::string::npos)return false;p++;while(p<json.size()&&std::isspace((unsigned char)json[p]))p++;auto q=p;while(q<json.size()){char c=json[q];if(!(std::isdigit((unsigned char)c)||c=='-'||c=='+'||c=='.'))break;q++;}if(q==p)return false;out=std::stod(json.substr(p,q-p));return true;}
static bool get_bool_json(const std::string& json,const std::string& key,bool& out){auto p=json.find("\""+key+"\"");if(p==std::string::npos)return false;p=json.find(":",p);if(p==std::string::npos)return false;p++;while(p<json.size()&&std::isspace((unsigned char)json[p]))p++;if(json.compare(p,4,"true")==0){out=true;return true;}if(json.compare(p,5,"false")==0){out=false;return true;}return false;}
struct Pose2D{bool valid=false;double x=0,y=0,yaw=0,confidence=0,stamp=0;std::string source;};
struct TaskData{bool active=false;std::string raw,task_type="pickup",shelf_id,cell_id,col;int row=0;bool command_has_pose=false;Pose2D target_pose;};
struct AnchorData{bool valid=false;std::string raw,shelf_id,anchor_type="front_qr";double x=0,y=0,yaw=0,confidence=0.8,stamp=0;};
struct SharedContext{rclcpp::Node::SharedPtr node;std::mutex mutex;TaskData task;Pose2D resolved_pose;AnchorData latest_anchor;std::string latest_current_shelf,latest_platform_status,state="IDLE";std::string semantic_map_path,semantic_session_path,map_session_id,map_frame="map";double session_pose_max_age_sec=600,min_session_confidence=0.5;bool allow_demo_anchor_if_missing=true;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub,state_pub,goal_json_pub,trace_pub,bt_xml_pub,platform_pub,explore_pub,control_mode_pub;rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub;void set_state(const std::string&s){state=s;state_pub->publish(std_msgs::msg::String().set__data(s));}void status(const std::string&st,const std::string&msg){std_msgs::msg::String m;std::stringstream ss;ss<<"{\"state\":\""<<state<<"\",\"status\":\""<<st<<"\",\"message\":\""<<msg<<"\"}";m.data=ss.str();status_pub->publish(m);}void trace(const std::string&node,const std::string&res){trace_pub->publish(std_msgs::msg::String().set__data("{\"node\":\""+node+"\",\"result\":\""+res+"\"}"));}};
static std::shared_ptr<SharedContext> get_ctx(const BT::TreeNode& node){return node.config().blackboard->get<std::shared_ptr<SharedContext>>("ctx");}
static bool find_session_pose_in_text(const std::string& text,const std::string& shelf_id,const std::string& session,double max_age,double min_conf,double now,Pose2D& out){if(shelf_id.empty()||text.empty())return false;auto p=text.find("\""+shelf_id+"\"");if(p==std::string::npos)return false;std::string sub=text.substr(p,std::min<size_t>(1200,text.size()-p));std::string s;get_string_json(sub,"map_session_id",s);if(!s.empty()&&s!=session)return false;bool valid=true;if(get_bool_json(sub,"valid",valid)&&!valid)return false;Pose2D pose;if(!get_number_json(sub,"x",pose.x)||!get_number_json(sub,"y",pose.y))return false;if(!get_number_json(sub,"yaw",pose.yaw))pose.yaw=0;if(!get_number_json(sub,"confidence",pose.confidence))pose.confidence=0.5;if(!get_number_json(sub,"last_seen_time",pose.stamp))pose.stamp=now;if(pose.confidence<min_conf||now-pose.stamp>max_age)return false;pose.valid=true;pose.source="session_pose";out=pose;return true;}
static void write_session_pose(const std::string& path,const std::string& session,const std::string& shelf_id,const Pose2D& pose){ensure_parent_dir(path);std::ofstream ofs(path,std::ios::out|std::ios::trunc);ofs<<"{\n  \"map_session_id\": \""<<session<<"\",\n  \"shelves\": {\n    \""<<shelf_id<<"\": {\n      \"session_pose\": {\n        \"map_session_id\": \""<<session<<"\",\n        \"frame\": \"map\",\n        \"x\": "<<pose.x<<",\n        \"y\": "<<pose.y<<",\n        \"yaw\": "<<pose.yaw<<",\n        \"confidence\": "<<pose.confidence<<",\n        \"last_seen_time\": "<<pose.stamp<<",\n        \"source\": \""<<pose.source<<"\",\n        \"valid\": true\n      }\n    }\n  }\n}\n";}
#define SIMPLE_PORTS static BT::PortsList providedPorts(){return {};}
class ReceiveShelfGoal:public BT::ConditionNode{public:ReceiveShelfGoal(const std::string&n,const BT::NodeConfiguration&c):BT::ConditionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("ReceiveShelfGoal");std::lock_guard<std::mutex>l(ctx->mutex);if(ctx->task.active){ctx->status("SUCCESS","task received");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}};
class CheckShelfSessionPoseValid:public BT::ConditionNode{public:CheckShelfSessionPoseValid(const std::string&n,const BT::NodeConfiguration&c):BT::ConditionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("CheckShelfSessionPoseValid");std::lock_guard<std::mutex>l(ctx->mutex);if(!ctx->task.active){ctx->status("FAILURE","no active task");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}if(ctx->task.command_has_pose){ctx->resolved_pose=ctx->task.target_pose;ctx->status("SUCCESS","direct command pose is valid for this test");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}Pose2D pose;double now=wall_seconds(*ctx->node);auto text=read_file_or_empty(ctx->semantic_session_path);if(find_session_pose_in_text(text,ctx->task.shelf_id,ctx->map_session_id,ctx->session_pose_max_age_sec,ctx->min_session_confidence,now,pose)){ctx->resolved_pose=pose;ctx->status("SUCCESS","valid current-session shelf pose found");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}ctx->status("FAILURE","no valid current-session pose; need anchor relocalization");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}};

static bool extract_json_object_after_key(const std::string& s,const std::string& key,size_t from,std::string& out){
    std::string pat="\""+key+"\"";
    size_t k=s.find(pat,from);
    if(k==std::string::npos)return false;
    size_t c=s.find(":",k+pat.size());
    if(c==std::string::npos)return false;
    size_t b=s.find("{",c);
    if(b==std::string::npos)return false;

    int depth=0;
    bool in_str=false;
    bool esc=false;

    for(size_t i=b;i<s.size();++i){
        char ch=s[i];

        if(in_str){
            if(esc){esc=false;continue;}
            if(ch=='\\'){esc=true;continue;}
            if(ch=='"'){in_str=false;continue;}
            continue;
        }

        if(ch=='"'){
            in_str=true;
            continue;
        }

        if(ch=='{')depth++;
        else if(ch=='}'){
            depth--;
            if(depth==0){
                out=s.substr(b,i-b+1);
                return true;
            }
        }
    }
    return false;
}

static bool get_bool_json_loose(const std::string& s,const std::string& key,bool& v){
    std::string pat="\""+key+"\"";
    size_t k=s.find(pat);
    if(k==std::string::npos)return false;
    size_t c=s.find(":",k+pat.size());
    if(c==std::string::npos)return false;
    size_t t=s.find_first_not_of(" \t\r\n",c+1);
    if(t==std::string::npos)return false;
    if(s.compare(t,4,"true")==0){v=true;return true;}
    if(s.compare(t,5,"false")==0){v=false;return true;}
    return false;
}

static bool pose_from_json_object(const std::string& obj,Pose2D& pose,const std::string& source){
    bool valid=false;
    if(get_bool_json_loose(obj,"valid",valid)&&!valid)return false;

    double x=0.0,y=0.0,yaw=0.0;
    if(!get_number_json(obj,"x",x))return false;
    if(!get_number_json(obj,"y",y))return false;
    get_number_json(obj,"yaw",yaw);

    pose.valid=true;
    pose.x=x;
    pose.y=y;
    pose.yaw=yaw;

    double conf=1.0,stamp=0.0;
    get_number_json(obj,"confidence",conf);
    get_number_json(obj,"stamp",stamp);
    pose.confidence=conf;
    pose.stamp=stamp;
    pose.source=source;
    return true;
}

static bool find_semantic_library_pose_in_text(const std::string& text,const std::string& shelf_id,Pose2D& pose){
    if(shelf_id.empty())return false;

    std::string shelf_obj;
    if(!extract_json_object_after_key(text,shelf_id,0,shelf_obj))return false;

    std::string approach_obj;
    if(extract_json_object_after_key(shelf_obj,"approach_pose",0,approach_obj)){
        Pose2D p;
        if(pose_from_json_object(approach_obj,p,"semantic_library_approach_pose")){
            pose=p;
            return true;
        }
    }

    std::string current_obj;
    if(extract_json_object_after_key(shelf_obj,"current_map_pose",0,current_obj)){
        Pose2D p;
        if(pose_from_json_object(current_obj,p,"semantic_library_current_map_pose")){
            pose=p;
            return true;
        }
    }

    return false;
}

class QueryShelfPose:public BT::SyncActionNode{
public:
    QueryShelfPose(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}
    SIMPLE_PORTS

    BT::NodeStatus tick()override{
        auto ctx=get_ctx(*this);
        ctx->set_state("QueryShelfPose");

        std::lock_guard<std::mutex>l(ctx->mutex);

        if(!ctx->task.active){
            ctx->status("FAILURE","no active task");
            ctx->trace(name(),"FAILURE");
            return BT::NodeStatus::FAILURE;
        }

        if(ctx->resolved_pose.valid){
            ctx->status("SUCCESS","resolved shelf pose ready");
            ctx->trace(name(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        if(ctx->task.command_has_pose){
            ctx->resolved_pose=ctx->task.target_pose;
            ctx->status("SUCCESS","direct pose from task command");
            ctx->trace(name(),"SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        {
            Pose2D pose;
            auto text=read_file_or_empty(ctx->semantic_map_path);
            if(find_semantic_library_pose_in_text(text,ctx->task.shelf_id,pose)){
                ctx->resolved_pose=pose;
                ctx->status("SUCCESS","semantic library pose loaded");
                ctx->trace(name(),"SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
        }

        {
            Pose2D pose;
            double now=wall_seconds(*ctx->node);
            auto text=read_file_or_empty(ctx->semantic_session_path);
            if(find_session_pose_in_text(text,ctx->task.shelf_id,ctx->map_session_id,ctx->session_pose_max_age_sec,ctx->min_session_confidence,now,pose)){
                ctx->resolved_pose=pose;
                ctx->status("SUCCESS","session pose loaded");
                ctx->trace(name(),"SUCCESS");
                return BT::NodeStatus::SUCCESS;
            }
        }

        ctx->status("FAILURE","shelf pose not found in semantic library or current session");
        ctx->trace(name(),"FAILURE");
        return BT::NodeStatus::FAILURE;
    }
};
class EnterExploreMode:public BT::SyncActionNode{public:EnterExploreMode(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("EnterExploreMode");std::string id;{std::lock_guard<std::mutex>l(ctx->mutex);id=ctx->task.shelf_id;}ctx->control_mode_pub->publish(std_msgs::msg::String().set__data("auto"));ctx->explore_pub->publish(std_msgs::msg::String().set__data("{\"mode\":\"explore_shelf\",\"target_shelf_id\":\""+id+"\"}"));ctx->status("RUNNING","explore mode requested");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}};
class SearchShelfReference:public BT::StatefulActionNode{double start_=0;public:SearchShelfReference(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus onStart()override{start_=steady_seconds();auto ctx=get_ctx(*this);ctx->set_state("SearchShelfReference");ctx->status("RUNNING","searching shelf reference/anchor");ctx->trace(name(),"RUNNING");return BT::NodeStatus::RUNNING;}BT::NodeStatus onRunning()override{auto ctx=get_ctx(*this);std::lock_guard<std::mutex>l(ctx->mutex);if(ctx->latest_anchor.valid&&ctx->latest_anchor.shelf_id==ctx->task.shelf_id){ctx->status("SUCCESS","target shelf anchor observed");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}if(ctx->allow_demo_anchor_if_missing&&steady_seconds()-start_>1.0){ctx->latest_anchor.valid=true;ctx->latest_anchor.shelf_id=ctx->task.shelf_id;ctx->latest_anchor.anchor_type="demo_anchor";ctx->latest_anchor.x=0.5;ctx->latest_anchor.y=0.0;ctx->latest_anchor.yaw=0.0;ctx->latest_anchor.confidence=0.8;ctx->latest_anchor.stamp=wall_seconds(*ctx->node);ctx->status("SUCCESS","demo anchor generated");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}return BT::NodeStatus::RUNNING;}void onHalted()override{get_ctx(*this)->trace(name(),"HALTED");}};
class DetectShelfAnchor:public BT::ConditionNode{public:DetectShelfAnchor(const std::string&n,const BT::NodeConfiguration&c):BT::ConditionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("DetectShelfAnchor");std::lock_guard<std::mutex>l(ctx->mutex);if(ctx->latest_anchor.valid&&ctx->latest_anchor.shelf_id==ctx->task.shelf_id){ctx->status("SUCCESS","anchor detection available");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}ctx->status("FAILURE","no target anchor detection");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}};
class EstimateShelfPoseFromAnchor:public BT::SyncActionNode{public:EstimateShelfPoseFromAnchor(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("EstimateShelfPoseFromAnchor");std::lock_guard<std::mutex>l(ctx->mutex);if(!ctx->latest_anchor.valid||ctx->latest_anchor.shelf_id!=ctx->task.shelf_id){ctx->status("FAILURE","no matching anchor for pose estimation");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}Pose2D p;p.valid=true;p.x=ctx->latest_anchor.x;p.y=ctx->latest_anchor.y;p.yaw=ctx->latest_anchor.yaw;p.confidence=ctx->latest_anchor.confidence;p.stamp=wall_seconds(*ctx->node);p.source=ctx->latest_anchor.anchor_type;ctx->resolved_pose=p;ctx->status("SUCCESS","shelf pose estimated from anchor");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}};
class UpdateSemanticSessionPose:public BT::SyncActionNode{public:UpdateSemanticSessionPose(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("UpdateSemanticSessionPose");std::lock_guard<std::mutex>l(ctx->mutex);if(!ctx->task.active||!ctx->resolved_pose.valid){ctx->status("FAILURE","no pose to update");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}write_session_pose(ctx->semantic_session_path,ctx->map_session_id,ctx->task.shelf_id,ctx->resolved_pose);ctx->status("SUCCESS","semantic session pose updated");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}};
class NavigateToShelf:public BT::SyncActionNode{public:NavigateToShelf(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("NavigateToShelf");std::lock_guard<std::mutex>l(ctx->mutex);if(!ctx->task.active||!ctx->resolved_pose.valid){ctx->status("FAILURE","no resolved target pose");ctx->trace(name(),"FAILURE");return BT::NodeStatus::FAILURE;}geometry_msgs::msg::PoseStamped goal;goal.header.stamp=ctx->node->now();goal.header.frame_id=ctx->map_frame;goal.pose.position.x=ctx->resolved_pose.x;goal.pose.position.y=ctx->resolved_pose.y;goal.pose.orientation.z=std::sin(ctx->resolved_pose.yaw*0.5);goal.pose.orientation.w=std::cos(ctx->resolved_pose.yaw*0.5);ctx->goal_pose_pub->publish(goal);ctx->goal_json_pub->publish(std_msgs::msg::String().set__data(ctx->task.raw));ctx->status("RUNNING","published /goal_pose");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}};
class TimedAction:public BT::StatefulActionNode{std::string state_;double seconds_,start_=0;public:TimedAction(const std::string&n,const BT::NodeConfiguration&c,std::string s,double sec):BT::StatefulActionNode(n,c),state_(std::move(s)),seconds_(sec){}SIMPLE_PORTS BT::NodeStatus onStart()override{start_=steady_seconds();auto ctx=get_ctx(*this);ctx->set_state(state_);ctx->status("RUNNING",name()+" running");ctx->trace(name(),"RUNNING");return BT::NodeStatus::RUNNING;}BT::NodeStatus onRunning()override{if(steady_seconds()-start_>=seconds_){auto ctx=get_ctx(*this);ctx->status("SUCCESS",name()+" success");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}return BT::NodeStatus::RUNNING;}void onHalted()override{get_ctx(*this)->trace(name(),"HALTED");}};
class WaitNavArrive:public TimedAction{public:WaitNavArrive(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,"WaitNavArrive",2.0){}SIMPLE_PORTS};
class SearchShelfQR:public TimedAction{public:SearchShelfQR(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,"SearchShelfQR",1.0){}SIMPLE_PORTS};
class AlignToCell:public TimedAction{public:AlignToCell(const std::string&n,const BT::NodeConfiguration&c):TimedAction(n,c,"AlignToCell",1.0){}SIMPLE_PORTS};
class VerifyShelfQR:public BT::StatefulActionNode{double start_=0;public:VerifyShelfQR(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus onStart()override{start_=steady_seconds();auto ctx=get_ctx(*this);ctx->set_state("VerifyShelfQR");ctx->status("RUNNING","verifying shelf identity");ctx->trace(name(),"RUNNING");return BT::NodeStatus::RUNNING;}BT::NodeStatus onRunning()override{auto ctx=get_ctx(*this);std::lock_guard<std::mutex>l(ctx->mutex);if(!ctx->latest_current_shelf.empty()&&ctx->latest_current_shelf==ctx->task.shelf_id){ctx->status("SUCCESS","current shelf matches target");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}if(ctx->allow_demo_anchor_if_missing&&steady_seconds()-start_>1.5){ctx->status("SUCCESS","demo shelf verification passed");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}return BT::NodeStatus::RUNNING;}void onHalted()override{get_ctx(*this)->trace(name(),"HALTED");}};
class SendPlatformTarget:public BT::SyncActionNode{public:SendPlatformTarget(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("SendPlatformTarget");std::lock_guard<std::mutex>l(ctx->mutex);std::stringstream ss;ss<<"{\"action\":\"pickup\",\"shelf_id\":\""<<ctx->task.shelf_id<<"\",\"row\":"<<ctx->task.row<<",\"col\":\""<<ctx->task.col<<"\",\"cell_id\":\""<<ctx->task.cell_id<<"\"}";ctx->platform_pub->publish(std_msgs::msg::String().set__data(ss.str()));ctx->status("RUNNING","published /platform/target");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}};
class WaitPickupDone:public BT::StatefulActionNode{double start_=0;public:WaitPickupDone(const std::string&n,const BT::NodeConfiguration&c):BT::StatefulActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus onStart()override{start_=steady_seconds();auto ctx=get_ctx(*this);ctx->set_state("WaitPickupDone");ctx->status("RUNNING","waiting pickup completion");ctx->trace(name(),"RUNNING");return BT::NodeStatus::RUNNING;}BT::NodeStatus onRunning()override{auto ctx=get_ctx(*this);std::lock_guard<std::mutex>l(ctx->mutex);if(ctx->latest_platform_status.find("done")!=std::string::npos||ctx->latest_platform_status.find("DONE")!=std::string::npos){ctx->status("SUCCESS","platform reports done");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}if(ctx->allow_demo_anchor_if_missing&&steady_seconds()-start_>1.0){ctx->status("SUCCESS","demo pickup done");ctx->trace(name(),"SUCCESS");return BT::NodeStatus::SUCCESS;}return BT::NodeStatus::RUNNING;}void onHalted()override{get_ctx(*this)->trace(name(),"HALTED");}};
class PublishTaskDone:public BT::SyncActionNode{public:PublishTaskDone(const std::string&n,const BT::NodeConfiguration&c):BT::SyncActionNode(n,c){}SIMPLE_PORTS BT::NodeStatus tick()override{auto ctx=get_ctx(*this);ctx->set_state("PublishTaskDone");ctx->status("DONE","pickup task completed");ctx->trace(name(),"SUCCESS");{std::lock_guard<std::mutex>l(ctx->mutex);ctx->task=TaskData();ctx->resolved_pose=Pose2D();}return BT::NodeStatus::SUCCESS;}};
class MissionNode:public rclcpp::Node{std::shared_ptr<SharedContext>ctx_;BT::BehaviorTreeFactory factory_;BT::Blackboard::Ptr blackboard_;std::unique_ptr<BT::Tree>tree_;std::unique_ptr<BT::PublisherZMQ>groot_pub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_,cancel_sub_,anchor_sub_,current_shelf_sub_,platform_status_sub_;rclcpp::TimerBase::SharedPtr timer_;public:MissionNode():Node("mission_btcpp_node"){}void init(){declare_parameter<std::string>("bt_xml","/host-home/src/Navigation/robot_logistics_task/behavior_tree/pickup_task_full_groot1.xml");declare_parameter<std::string>("semantic_map_path","/host-home/maps/semantic_library.json");declare_parameter<std::string>("semantic_session_path","/host-home/maps/semantic_session_pose.json");declare_parameter<std::string>("map_session_id","current_slam_session");declare_parameter<std::string>("shelf_goal_topic","/task/shelf_goal");declare_parameter<std::string>("cancel_topic","/task/cancel");declare_parameter<std::string>("task_status_topic","/task/status");declare_parameter<std::string>("current_state_topic","/task/current_state");declare_parameter<std::string>("current_goal_topic","/task/current_goal");declare_parameter<std::string>("bt_trace_topic","/task/bt_trace");declare_parameter<std::string>("bt_xml_topic","/task/bt_tree_xml");declare_parameter<std::string>("goal_pose_topic","/goal_pose");declare_parameter<std::string>("platform_target_topic","/platform/target");declare_parameter<std::string>("platform_status_topic","/platform/status");declare_parameter<std::string>("explore_cmd_topic","/task/explore_cmd");declare_parameter<std::string>("control_mode_topic","/control_mode");declare_parameter<std::string>("anchor_detection_topic","/semantic/anchor_detection");declare_parameter<std::string>("current_shelf_topic","/semantic/current_shelf");declare_parameter<std::string>("map_frame","map");declare_parameter<std::string>("base_frame","base_link");declare_parameter<double>("tick_hz",10.0);declare_parameter<double>("session_pose_max_age_sec",600.0);declare_parameter<double>("min_session_confidence",0.5);declare_parameter<bool>("allow_demo_anchor_if_missing",true);declare_parameter<bool>("enable_groot_monitoring",true);declare_parameter<int>("groot_publisher_port",1666);declare_parameter<int>("groot_server_port",1667);declare_parameter<int>("groot_max_msg_per_second",25);ctx_=std::make_shared<SharedContext>();ctx_->node=shared_from_this();ctx_->semantic_map_path=get_parameter("semantic_map_path").as_string();ctx_->semantic_session_path=get_parameter("semantic_session_path").as_string();ctx_->map_session_id=get_parameter("map_session_id").as_string();ctx_->map_frame=get_parameter("map_frame").as_string();ctx_->session_pose_max_age_sec=get_parameter("session_pose_max_age_sec").as_double();ctx_->min_session_confidence=get_parameter("min_session_confidence").as_double();ctx_->allow_demo_anchor_if_missing=get_parameter("allow_demo_anchor_if_missing").as_bool();ctx_->status_pub=create_publisher<std_msgs::msg::String>(get_parameter("task_status_topic").as_string(),10);ctx_->state_pub=create_publisher<std_msgs::msg::String>(get_parameter("current_state_topic").as_string(),10);ctx_->goal_json_pub=create_publisher<std_msgs::msg::String>(get_parameter("current_goal_topic").as_string(),10);ctx_->trace_pub=create_publisher<std_msgs::msg::String>(get_parameter("bt_trace_topic").as_string(),10);ctx_->bt_xml_pub=create_publisher<std_msgs::msg::String>(get_parameter("bt_xml_topic").as_string(),1);ctx_->goal_pose_pub=create_publisher<geometry_msgs::msg::PoseStamped>(get_parameter("goal_pose_topic").as_string(),10);ctx_->platform_pub=create_publisher<std_msgs::msg::String>(get_parameter("platform_target_topic").as_string(),10);ctx_->explore_pub=create_publisher<std_msgs::msg::String>(get_parameter("explore_cmd_topic").as_string(),10);ctx_->control_mode_pub=create_publisher<std_msgs::msg::String>(get_parameter("control_mode_topic").as_string(),10);goal_sub_=create_subscription<std_msgs::msg::String>(get_parameter("shelf_goal_topic").as_string(),10,[this](std_msgs::msg::String::SharedPtr m){on_goal(m);});cancel_sub_=create_subscription<std_msgs::msg::String>(get_parameter("cancel_topic").as_string(),10,[this](std_msgs::msg::String::SharedPtr){std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->task=TaskData();ctx_->resolved_pose=Pose2D();ctx_->status("CANCELLED","task cancelled");if(tree_)tree_->haltTree();});anchor_sub_=create_subscription<std_msgs::msg::String>(get_parameter("anchor_detection_topic").as_string(),10,[this](std_msgs::msg::String::SharedPtr m){on_anchor(m);});current_shelf_sub_=create_subscription<std_msgs::msg::String>(get_parameter("current_shelf_topic").as_string(),10,[this](std_msgs::msg::String::SharedPtr m){std::lock_guard<std::mutex>l(ctx_->mutex);std::string shelf;if(get_string_json(m->data,"shelf_id",shelf))ctx_->latest_current_shelf=shelf;else ctx_->latest_current_shelf=m->data;});platform_status_sub_=create_subscription<std_msgs::msg::String>(get_parameter("platform_status_topic").as_string(),10,[this](std_msgs::msg::String::SharedPtr m){std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->latest_platform_status=m->data;});factory_.registerNodeType<ReceiveShelfGoal>("ReceiveShelfGoal");factory_.registerNodeType<CheckShelfSessionPoseValid>("CheckShelfSessionPoseValid");factory_.registerNodeType<QueryShelfPose>("QueryShelfPose");factory_.registerNodeType<EnterExploreMode>("EnterExploreMode");factory_.registerNodeType<SearchShelfReference>("SearchShelfReference");factory_.registerNodeType<DetectShelfAnchor>("DetectShelfAnchor");factory_.registerNodeType<EstimateShelfPoseFromAnchor>("EstimateShelfPoseFromAnchor");factory_.registerNodeType<UpdateSemanticSessionPose>("UpdateSemanticSessionPose");factory_.registerNodeType<NavigateToShelf>("NavigateToShelf");factory_.registerNodeType<WaitNavArrive>("WaitNavArrive");factory_.registerNodeType<VerifyShelfQR>("VerifyShelfQR");factory_.registerNodeType<SearchShelfQR>("SearchShelfQR");factory_.registerNodeType<AlignToCell>("AlignToCell");factory_.registerNodeType<SendPlatformTarget>("SendPlatformTarget");factory_.registerNodeType<WaitPickupDone>("WaitPickupDone");factory_.registerNodeType<PublishTaskDone>("PublishTaskDone");blackboard_=BT::Blackboard::create();blackboard_->set("ctx",ctx_);auto xml=get_parameter("bt_xml").as_string();ctx_->bt_xml_pub->publish(std_msgs::msg::String().set__data(read_file_required(xml)));tree_=std::make_unique<BT::Tree>(factory_.createTreeFromFile(xml,blackboard_));if(get_parameter("enable_groot_monitoring").as_bool()){int pub_port=get_parameter("groot_publisher_port").as_int();int srv_port=get_parameter("groot_server_port").as_int();int max_msg=get_parameter("groot_max_msg_per_second").as_int();groot_pub_=std::make_unique<BT::PublisherZMQ>(*tree_,max_msg,pub_port,srv_port);RCLCPP_INFO(get_logger(),"Groot v1 PublisherZMQ enabled. publisher_port=%d server_port=%d",pub_port,srv_port);}double hz=get_parameter("tick_hz").as_double();timer_=create_wall_timer(std::chrono::milliseconds((int)(1000.0/hz)),[this](){tick();});RCLCPP_INFO(get_logger(),"Full semantic relocalization mission BT started.");}private:void on_goal(const std_msgs::msg::String::SharedPtr m){
    if(tree_){
        tree_->haltTree();
    }

    TaskData t;
    t.active=true;
    t.raw=m->data;

    get_string_json(m->data,"task_type",t.task_type);
    get_string_json(m->data,"shelf_id",t.shelf_id);
    get_string_json(m->data,"cell_id",t.cell_id);
    get_string_json(m->data,"target_cell_id",t.cell_id);
    get_string_json(m->data,"col",t.col);

    if(t.shelf_id.empty()){
        std::string target_shelf;
        get_string_json(m->data,"target_shelf_id",target_shelf);
        t.shelf_id=target_shelf;
    }

    if(t.shelf_id.empty()){
        auto d=m->data.find("-");
        t.shelf_id=d==std::string::npos?m->data:m->data.substr(0,d);
    }

    double row=0;
    if(get_number_json(m->data,"row",row))t.row=(int)row;

    double x=0,y=0,yaw=0;
    if(get_number_json(m->data,"x",x)&&get_number_json(m->data,"y",y)){
        t.command_has_pose=true;
        t.target_pose.valid=true;
        t.target_pose.x=x;
        t.target_pose.y=y;
        if(get_number_json(m->data,"yaw",yaw))t.target_pose.yaw=yaw;
        t.target_pose.confidence=1.0;
        t.target_pose.stamp=wall_seconds(*this);
        t.target_pose.source="command_direct_pose";
    }

    {
        std::lock_guard<std::mutex>l(ctx_->mutex);
        ctx_->task=t;
        ctx_->resolved_pose=Pose2D();
        ctx_->latest_anchor=AnchorData();
        ctx_->set_state("TASK_ACCEPTED");
    }

    ctx_->status("ACCEPTED","task accepted");
    RCLCPP_INFO(get_logger(),"Task received: %s",m->data.c_str());
}void on_anchor(const std_msgs::msg::String::SharedPtr m){AnchorData a;a.raw=m->data;a.stamp=wall_seconds(*this);get_string_json(m->data,"shelf_id",a.shelf_id);get_string_json(m->data,"anchor_type",a.anchor_type);get_number_json(m->data,"x",a.x);get_number_json(m->data,"y",a.y);get_number_json(m->data,"yaw",a.yaw);get_number_json(m->data,"confidence",a.confidence);if(!a.shelf_id.empty()){a.valid=true;std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->latest_anchor=a;}}void tick(){bool has=false;{std::lock_guard<std::mutex>l(ctx_->mutex);has=ctx_->task.active;}if(!has){ctx_->set_state("IDLE");return;}auto st=tree_->tickRoot();if(st==BT::NodeStatus::SUCCESS){tree_->haltTree();}else if(st==BT::NodeStatus::FAILURE){ctx_->status("FAILED","behavior tree returned FAILURE");{std::lock_guard<std::mutex>l(ctx_->mutex);ctx_->task=TaskData();ctx_->resolved_pose=Pose2D();}tree_->haltTree();}}};
int main(int argc,char**argv){rclcpp::init(argc,argv);auto n=std::make_shared<MissionNode>();n->init();rclcpp::spin(n);rclcpp::shutdown();return 0;}
