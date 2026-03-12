#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  rclcpp::Node::SharedPtr get_node() const { return node_; }
  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<moveit_msgs::msg::CollisionObject> objects;

  moveit_msgs::msg::CollisionObject box_left;
  box_left.id = "box_left";
  box_left.header.frame_id = "world";
  box_left.primitives.resize(1);
  box_left.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box_left.primitives[0].dimensions = { 0.05, 0.05, 0.05 }; 
  box_left.pose.position.x = 0.5;
  box_left.pose.position.y = 0.2; 
  box_left.pose.position.z = 0.025;
  box_left.pose.orientation.w = 1.0;
  objects.push_back(box_left);

  moveit_msgs::msg::CollisionObject box_right;
  box_right.id = "box_right";
  box_right.header.frame_id = "world";
  box_right.primitives.resize(1);
  box_right.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box_right.primitives[0].dimensions = { 0.05, 0.05, 0.05 };

  box_right.pose.position.x = 0.4;
  box_right.pose.position.y = -0.45; 
  box_right.pose.position.z = 0.025;

  box_right.pose.orientation.x = 0.0;
  box_right.pose.orientation.y = 0.0;
  box_right.pose.orientation.z = 0.3826834;
  box_right.pose.orientation.w = 0.9238795;
  objects.push_back(box_right);

  psi.addCollisionObjects(objects);
}

void MTCTaskNode::doTask()
{
  task_ = createTask();
  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  task_.execute(*task_.solutions().front());
}

mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("Stack Cubes Task");
  task.loadRobotModel(node_);
  task.setProperty("publish_planning_scene", true);

  const auto& arm_group_name = "panda_arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "panda_hand";

  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  mtc::Stage* attach_object_stage = nullptr;

  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick box_left");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
    stage->properties().set("marker_ns", "approach_object");
    stage->properties().set("link", hand_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(0.1, 0.15);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = hand_frame;
    vec.vector.z = 1.0;
    stage->setDirection(vec);
    grasp->insert(std::move(stage));

    auto grasp_pose = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    grasp_pose->properties().configureInitFrom(mtc::Stage::PARENT);
    grasp_pose->setPreGraspPose("open");
    grasp_pose->setObject("box_left"); 
    grasp_pose->setAngleDelta(M_PI / 4);
    grasp_pose->setMonitoredStage(current_state_ptr);

    Eigen::Isometry3d grasp_frame_transform;
    Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    grasp_frame_transform.linear() = q.matrix();
    grasp_frame_transform.translation().z() = 0.1;

    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp_pose));
    wrapper->setMaxIKSolutions(8);
    wrapper->setIKFrame(grasp_frame_transform, hand_frame);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    grasp->insert(std::move(wrapper));

    auto allow_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
    allow_coll->allowCollisions("box_left", task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(), true);
    grasp->insert(std::move(allow_coll));

    auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    close_hand->setGroup(hand_group_name);
    close_hand->setGoal("close");
    grasp->insert(std::move(close_hand));

    auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("attach box_left");
    attach->attachObject("box_left", hand_frame);
    attach_object_stage = attach.get();
    grasp->insert(std::move(attach));

    auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    lift->setMinMaxDistance(0.1, 0.2);
    lift->setIKFrame(hand_frame);
    vec.header.frame_id = "world"; vec.vector.z = 1.0; vec.vector.x = 0;
    lift->setDirection(vec);
    grasp->insert(std::move(lift));

    task.add(std::move(grasp));
  }

  auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
      "move to place", mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_place->setTimeout(5.0);
  stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_place));

  {
    auto place = std::make_unique<mtc::SerialContainer>("place on box_right");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    auto place_pose = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    place_pose->properties().configureInitFrom(mtc::Stage::PARENT);
    place_pose->setObject("box_left");

    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = "box_right";
    target_pose_msg.pose.position.z = 0.05; 
    target_pose_msg.pose.orientation.w = 1.0;
    place_pose->setPose(target_pose_msg);
    place_pose->setMonitoredStage(attach_object_stage);

    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(place_pose));
    wrapper->setMaxIKSolutions(2);
    wrapper->setIKFrame("box_left");
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
    place->insert(std::move(wrapper));

    auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    open_hand->setGroup(hand_group_name);
    open_hand->setGoal("open");
    place->insert(std::move(open_hand));

    auto forbid_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision");
    forbid_coll->allowCollisions("box_left", task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(), false);
    place->insert(std::move(forbid_coll));

    auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach box_left");
    detach->detachObject("box_left", hand_frame);
    place->insert(std::move(detach));

    auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    retreat->setMinMaxDistance(0.1, 0.2);
    retreat->setIKFrame(hand_frame);
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world"; 
    vec.vector.z = 1.0; 
    vec.vector.x = 0;
    retreat->setDirection(vec);
    place->insert(std::move(retreat));

    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  
  std::thread spin_thread([mtc_task_node]() { 
    rclcpp::spin(mtc_task_node->getNodeBaseInterface()); 
  });

  int mode = mtc_task_node->get_node()->get_parameter("mode").as_int();

  switch (mode)
  {
    case 0: {
        RCLCPP_INFO_STREAM(LOGGER, "Running in MODE 0: Only setting up the planning scene.");
        mtc_task_node->setupPlanningScene();
        break;
    }
    case 1:{
        RCLCPP_INFO_STREAM(LOGGER, "Running in MODE 1: Setting up the planning scene and executing the task.");
        mtc_task_node->setupPlanningScene();
        mtc_task_node->doTask();
        break;
    }
  }

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}