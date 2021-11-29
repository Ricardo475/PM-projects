/**
  * FOR MORE INFORMATION CHECK msg2tf_node.h
  */

#include "gazebo2odom.h"

GAZ2ODOParams getParameters(ros::NodeHandle n_public_, ros::NodeHandle n_private_)
{
    GAZ2ODOParams parameters;

    n_public_.param<std::string>("frame_robot_no_prefix", parameters.robot_frame, "base_link");
    n_public_.param<std::string>("frame_world", parameters.world_frame, "odom");
    n_public_.param<bool>("global_verbose", parameters.global_verbose, false);

    n_private_.param<std::string>("topic_pub_base", parameters.topic_pub_base, "odom");
    n_private_.param<std::string>("topic_gazebo_properties", parameters.topic_gazebo_properties, "/gazebo/get_world_properties");

    n_private_.param<std::string>("element_descriptor", parameters.element_descriptor, "heron");

    n_private_.param<double>("freq_rate", parameters.time, 50.0);
    parameters.time = 1.0 / parameters.time;

    n_private_.param<double>("time_wait_data", parameters.time_wait_data, 5.0);
    parameters.time_wait_data = 1.0 / parameters.time_wait_data;

    n_private_.param<bool>("debug", parameters.debug, true);
    parameters.debug = parameters.debug && parameters.global_verbose;


    return parameters;
}

void setModels()
{
    ros::NodeHandle n_public;

    ros::ServiceClient client = n_public.serviceClient<gazebo_msgs::GetWorldProperties>(g_method_params.topic_gazebo_properties);
    gazebo_msgs::GetWorldProperties srv;

    bool has_service = true;

    while(!client.exists())
    {
        has_service = false;
        ROS_INFO("WAITING FOR GAZEBO! (gazebo2odom_node)");
    }

    if(!has_service)
    {
        ros::Rate rate(g_method_params.time_wait_data);
        ROS_WARN("PAUSED TO GET MODELS INFO! (gazebo2odom_node)");
        rate.sleep();
    }

    if(client.call(srv))
    {
        std::vector<std::string> models = srv.response.model_names;

        for(uint it=0; it < models.size(); it++)
        {
            std::string name = models[it];
            std::size_t found = name.find(g_method_params.element_descriptor);

            if(found == 0)
                g_element_list.push_back(Element(name, it, n_public.advertise<nav_msgs::Odometry>("/" + name + "/"+ g_method_params.topic_pub_base, 1)));
        }
    }
}

bool update_elements(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{
    g_has_model = false;
    g_element_list.clear();
    g_element_list.shrink_to_fit();

    setModels();

    ROS_WARN("CREATING PUBLISHERS! (gazebo2odom_node)");
    ros::Rate rate(g_method_params.time_wait_data);
    rate.sleep();

    return true;
}

void elementsCallback(const gazebo_msgs::ModelStates::ConstPtr& msg_)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = g_method_params.world_frame;

    for(Element& element: g_element_list)
    {
        element.header = header;
        element.pose = msg_->pose[element.index];
        element.velocity = msg_->twist[element.index];
    }

    g_has_model=true;

    return;
}

void timerCallback(const ros::TimerEvent&)
{
    if(!g_has_model)
    {
        if(g_method_params.debug)
            ROS_ERROR_STREAM("NO element FOUND! (gazebo2odom_node)");
        return;
    }

    for(Element const element: g_element_list)
    {
        nav_msgs::Odometry odom;

        odom.header         = element.header;
        odom.child_frame_id = g_method_params.robot_frame;
        odom.twist.twist    = element.velocity;
        odom.pose.pose      = element.pose;

        element.pub_odom.publish(odom);
    }

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gazebo2odom_node");

    ros::NodeHandle n_public;
    ros::NodeHandle n_private("~");

    /// Initialize tf broadcaster
    broadcaster = new tf::TransformBroadcaster();

    ///Get parameters
    g_method_params = getParameters(n_public, n_private);

    ///Get subscriber topic name
    std::string topic_element_list_sub;
    n_private.param<std::string>("topic_element_list_sub", topic_element_list_sub, "/gazebo/model_states");
    ///Create subscribers
    ros::Subscriber sub_elements = n_public.subscribe(topic_element_list_sub, 100, elementsCallback);

    ros::Timer timer = n_public.createTimer(ros::Duration(g_method_params.time), timerCallback);

    ros::ServiceServer service = n_private.advertiseService("update_elements", update_elements);

    setModels();

    ros::spin();
    return 0;
}
