#include <utility>
#include <iostream>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace boost;
using namespace ros;

namespace tmadeira_ns {

    //
    class Team
    {
        public:
            explicit Team(string name)
            {
                this->name = name;
                // read team players from param
                n.getParam("/team_" + name, player_names);
            }

            void addPlayer(string name)
            {
                player_names.push_back(name);
            }

            void printPlayerNames()
            {
                cout << "Team " << name << " has players: " << endl;

                for (int i = 0; i < player_names.size(); ++i)
                {
                    cout << "Player " << i << ": " << player_names[i] << " " << endl;
                }
            }

            bool playerBelongsToTeam(string player_name)
            {
                for (string& player : player_names)
                {
                    if (player == player_name) return true;
                }
                return false;
            }

            // Getter for team name
            string getName()
            {
                return this->name;
            }


        private:
            string name;
            vector<string> player_names;
            NodeHandle n;
    };

    class Player {

        public:
            // Constructor
            explicit Player(string name)
            {
                this->name = name;
                this->team_name = "";
            }

            // string setter for team name
            void setTeamName(string name)
            {
                if (name == "red" || name == "green" || name == "blue") team_name = name;
                else cout << "Cannot set team name " << name << ". Not part of the options!" << endl;
            }

            // int setter for team name
            void setTeamName(int idx)
            {
                if (idx == 0) setTeamName("red");
                else if (idx == 1) setTeamName("green");
                else if (idx == 2) setTeamName("blue");
                else setTeamName("");
            }

            string getName()
            {
                return this->name;
            }

            // Getter for team name
            string getTeamName()
            {
                return this->team_name;
            }


        private:
            string name;
            string team_name;

    };

    // Extending player class
    class MyPlayer : public Player
    {
        public:

            MyPlayer(string name, string team_name) : Player(name)
            {
                team_red = (boost::shared_ptr<Team>) new Team("red");
                team_green = (boost::shared_ptr<Team>) new Team("green");
                team_blue = (boost::shared_ptr<Team>) new Team("blue");

                ros::NodeHandle n;
                vis_pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
                *vis_pub = n.advertise<visualization_msgs::Marker>("player_names", 0);

                if(team_red->playerBelongsToTeam(name))
                {
                    team_mine = team_red;
                    team_prey = team_green;
                    team_hunters = team_blue;
                } else if (team_green->playerBelongsToTeam(name))
                {
                    team_mine = team_green;
                    team_prey = team_blue;
                    team_hunters = team_red;
                } else if(team_blue->playerBelongsToTeam(name))
                {
                    team_mine = team_blue;
                    team_prey = team_red;
                    team_hunters = team_green;
                } else
                {
                    cout << "ERROR: Something went wrong in team parametrization!" << endl;
                }

                setTeamName(team_mine->getName());

                //define intial position
                float sx = randomizePosition();
                float sy = randomizePosition();
                tf::Transform T1;
                T1.setOrigin( tf::Vector3(sx, sy, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, M_PI);
                T1.setRotation(q);

                //define global movement
                tf::Transform Tglobal = T1;
                br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getName()));
                ros::Duration(0.1).sleep();
                br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getName()));

                //printInfo();
            }

            void printInfo()
            {
                ROS_INFO_STREAM("My name is " << this->getName() << " and my team is " << team_mine->getName());
                ROS_INFO_STREAM("I am hunting " << team_prey->getName() << " and fleeing from " << team_hunters->getName());
            }

            void makeAPlayCallBack(rws2019_msgs::MakeAPlayConstPtr msg)
            {
                ROS_INFO("Received a new msg");
                static tf::TransformListener ls;

                // Determine where player is
                tf::StampedTransform T0;
                try
                {
                    ls.lookupTransform("/world", this->getName(), ros::Time(0), T0);
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.1).sleep();
                }

                // Define strategy of movement
                //TODO:
                float dx = 0.1;
                float angle = M_PI;

                float dx_max = msg->turtle;
                dx > dx_max ? dx = dx_max : dx = dx;

                double angleMax = M_PI/30;
                fabs(angle) > fabs(angleMax) ? angle = angleMax * angle / fabs(angle): angle = angle;

                // Define movement in own referential
                tf::Transform T1;
                T1.setOrigin( tf::Vector3(dx, 0.0, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, angle);
                T1.setRotation(q);

                // Calculate movement in world
                tf::Transform Tglobal = T0*T1;
                br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getName()));

                visualization_msgs::Marker marker;
                marker.header.frame_id = this->getName();
                marker.header.stamp = ros::Time();
                marker.ns = this->getName();
                marker.id = 0;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::Marker::ADD;
//            marker.pose.position.x = 1;
//            marker.pose.position.y = 1;
//            marker.pose.position.z = 1;
//            marker.pose.orientation.x = 0.0;
//            marker.pose.orientation.y = 0.0;
//            marker.pose.orientation.z = 0.0;
//            marker.pose.orientation.w = 1.0;
//            marker.scale.x = ;
//            marker.scale.y = 0.1;
                marker.scale.z = 0.6;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.text = this->getName();

//only if using a MESH_RESOURCE marker type:
//            marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                vis_pub->publish( marker );
            }

        private:
            boost::shared_ptr<Team> team_red;
            boost::shared_ptr<Team> team_blue;
            boost::shared_ptr<Team> team_green;
            boost::shared_ptr<Team> team_hunters;
            boost::shared_ptr<Team> team_mine;
            boost::shared_ptr<Team> team_prey;
            tf::TransformBroadcaster br;
            boost::shared_ptr<ros::Publisher> vis_pub;

            float randomizePosition()
            {
                srand(3483*time(NULL)); // set initial seed value to 5323
                return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
            }

    };
}

int main(int argc, char** argv)
{
    // Initialize node
    init(argc, argv, "player_tmadeira");
    NodeHandle n;

    string player_name = "tmadeira";
    // Creating an instance of class Player
    tmadeira_ns::MyPlayer player1 = tmadeira_ns::MyPlayer(player_name, "red");

    // Create a team
    tmadeira_ns::Team team_red("red");

    Subscriber sub = n.subscribe("/make_a_play", 100, &tmadeira_ns::MyPlayer::makeAPlayCallBack, &player1);

    ros::Rate r(20);
    // Life cycle
    while(ros::ok())
    {
        spinOnce();
        r.sleep();
    }
}