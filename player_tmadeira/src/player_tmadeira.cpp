#include <utility>
#include <iostream>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <rws2019_msgs/DoTheMath.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost;
using namespace ros;

namespace tmadeira_ns {

    /**
     * General description: definition of the team.
     */
    class Team
    {

    private:
        /**
         * Internal data
         */
        string name;
        vector<string> player_names;
        NodeHandle n;

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

            for (int i = 0; i < player_names.size(); i++)
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

        // Getter for team name
        vector<string> getPlayerNames()
        {
            return this->player_names;
        }
    };

    /**
     * General description: definition of the Player.
     */
    class Player
    {

    /**
     * Internal data
     */
    private:
        string name;
        string team_name;

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

    };

    /**
     * General description: definition of MyPlayer, extending the Player class.
     */
    class MyPlayer : public Player
    {

    private:
        /**
         * Internal data
         */
        boost::shared_ptr<Team> team_red;
        boost::shared_ptr<Team> team_blue;
        boost::shared_ptr<Team> team_green;
        boost::shared_ptr<Team> team_hunters;
        boost::shared_ptr<Team> team_mine;
        boost::shared_ptr<Team> team_prey;

        NodeHandle n;

        tf::TransformListener listener;
        tf::TransformBroadcaster br;

        string hunting, fleeing;

        boost::shared_ptr<ros::Publisher> vis_pub;
        boost::shared_ptr<ros::Publisher> bocas_pub;

        boost::shared_ptr<ros::Subscriber> sub;
        boost::shared_ptr<ros::ServiceServer> game_query_srv;

        float randomizePosition(int seed)
        {
            srand(seed*time(NULL));
            return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
        }

    public:

        /**
         * Constructor
         */
        MyPlayer(string name, string team_name) : Player(name)
        {
            team_red = (boost::shared_ptr<Team>) new Team("red");
            team_green = (boost::shared_ptr<Team>) new Team("green");
            team_blue = (boost::shared_ptr<Team>) new Team("blue");

            hunting = "";
            fleeing = "";

            ros::NodeHandle n;
            vis_pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
            *vis_pub = n.advertise<visualization_msgs::Marker>("player_names", 0);

            bocas_pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
            *bocas_pub = n.advertise<visualization_msgs::Marker>("bocas", 0);

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

            //define initial position
            float sx = randomizePosition(8736);
            float sy = randomizePosition(3468);
            tf::Transform T;
            T.setOrigin( tf::Vector3(sx, sy, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, M_PI);
            T.setRotation(q);

            //define global movement
            tf::Transform Tglobal = T;
            br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", name));
            ros::Duration(0.1).sleep();
            br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", name));

            // TODO:

            sub = boost::shared_ptr<ros::Subscriber> (new ros::Subscriber());
            *sub = n.subscribe("/make_a_play", 100, &MyPlayer::makeAPlayCallBack, this);

            game_query_srv = boost::shared_ptr<ros::ServiceServer> (new ros::ServiceServer());
            *game_query_srv = n.advertiseService("/do_the_math", &tmadeira_ns::MyPlayer::doTheMathCallback, this);


            //printInfo();
        }


        void printInfo()
        {
            ROS_INFO_STREAM("My name is " << this->getName() << " and my team is " << team_mine->getName());
            ROS_INFO_STREAM("I am hunting " << team_prey->getName() << " and fleeing from " << team_hunters->getName());
        }

        std::tuple<float, float> getDistanceAndAngleToArenaCenter()
        {
            return getDistanceAndAngleToPlayer("world");
        }

        std::tuple<float, float> getDistanceAndAngleToPlayer(string other_player)
        {
            tf::StampedTransform T0;
            try
            {
                listener.lookupTransform(this->getName(), other_player, ros::Time(0), T0);
            }
            catch (tf::TransformException ex)
            {
                // ROS_ERROR("%s",ex.what());
                ros::Duration(0.01).sleep();
                return {1000.0, 0.0};
            }

            float d = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y() );
            float a = atan2( T0.getOrigin().y(), T0.getOrigin().x());
            return {d,a};
        }

        bool doTheMathCallback(rws2019_msgs::DoTheMath::Request  &req, rws2019_msgs::DoTheMath::Response &res)
        {
            if (req.op == "+")
                res.result = req.a + req.b;
            else if (req.op == "-")
                res.result = req.a - req.b;
            else if (req.op == "*")
                res.result = req.a * req.b;
            else if (req.op == "/")
                res.result = req.a / req.b;
            else
                return false;

            // ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
            // ROS_INFO("sending back response: [%ld]", (long int)res.result);
            return true;
        }

        void makeAPlayCallBack(rws2019_msgs::MakeAPlayConstPtr msg)
        {
            // ROS_INFO("Received a new msg");
            bool stratChange = false;

            // Determine where player is
            tf::StampedTransform T0;
            try
            {
                listener.lookupTransform("/world", this->getName(), ros::Time(0), T0);
            }
            catch (tf::TransformException ex)
            {
                // ROS_ERROR("%s",ex.what());
                ros::Duration(0.1).sleep();
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Define strategy of movement
            // TODO:

            // Compute closest prey
            vector<float> distance_to_prey;
            vector<float> angle_to_prey;
            for (size_t i =0; i< team_prey->getPlayerNames().size(); i++)
            {
                // ROS_WARN_STREAM("team_preys = " << team_prey->getPlayerNames()[i]);

                std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_prey->getPlayerNames()[i]);
                distance_to_prey.push_back( std::get<0>(t));
                angle_to_prey.push_back( std::get<1>(t));
            }

            int idx_closest_prey = 0;
            float distance_closest_prey = 1000;
            for (int i =0; i < distance_to_prey.size(); i++)
            {
                if (distance_to_prey[i] < distance_closest_prey)
                {
                    idx_closest_prey = i;
                    distance_closest_prey = distance_to_prey[i];
                }
            }

            ///////////////////////////////////////////////////////////////////////////////////////////////////////
            // Compute closest hunter

            vector<float> distance_to_hunter;
            vector<float> angle_to_hunter;
            for (size_t i =0; i < team_hunters->getPlayerNames().size(); i++)
            {
                // ROS_WARN_STREAM("team_hunters = " << team_hunters->getPlayerNames()[i]);

                std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_hunters->getPlayerNames()[i]);
                distance_to_hunter.push_back( std::get<0>(t));
                angle_to_hunter.push_back( std::get<1>(t));
            }

            int idx_closest_hunter = 0;
            float distance_closest_hunter = 1000;
            for (int i =0; i< distance_to_hunter.size(); i++)
            {
                if (distance_to_hunter[i] < distance_closest_hunter)
                {
                    idx_closest_hunter = i;
                    distance_closest_hunter = distance_to_hunter[i];
                }
            }

            ////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Compute distance to arena center

            float distance_to_arena_center;
            float angle_to_arena_center;

            std::tuple<float, float> t = getDistanceAndAngleToArenaCenter();
            distance_to_arena_center = std::get<0>(t);
            angle_to_arena_center = std::get<1>(t);

            ////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Compare prey distance to hunter distance

            // Step
            float dx = 0;
            // Angle of step
            float angle = 0;

            string boca = "";


            if (distance_closest_hunter < 3 && distance_closest_hunter < distance_closest_prey)
            {   // RUNNING

                dx = 10;
                // TODO:
                angle = angle_to_hunter[idx_closest_hunter] + M_PI;

                // Check if strat changed to shout
                if (team_hunters->getPlayerNames()[idx_closest_hunter] != fleeing)
                {
                    hunting = "";
                    fleeing = team_hunters->getPlayerNames()[idx_closest_hunter];
                    stratChange = true;
                }

                boca = "Coming for " + fleeing;
            }
            else
            {   // CHASING

                dx = 10;
                angle = angle_to_prey[idx_closest_prey];

                // Check if strat changed to shout
                if (team_prey->getPlayerNames()[idx_closest_prey] != hunting)
                {
                    fleeing = "";
                    hunting = team_prey->getPlayerNames()[idx_closest_prey];
                    stratChange = true;
                }

                boca = "Coming for " + hunting;
            }


            ////////////////////////////////////////////////////////////////////////////////////////////////////////

            // Movement restrictions
            float dx_max = msg->turtle;
            dx > dx_max ? dx = dx_max : dx = dx;

            double angleMax = M_PI/30;
            if (angle != 0) fabs(angle) > fabs(angleMax) ? angle = angleMax * angle / fabs(angle): angle = angle;

            // Define movement in own referential
            tf::Transform T1;
            T1.setOrigin( tf::Vector3(dx, 0.0, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, angle);
            T1.setRotation(q);

            // Calculate movement in world
            tf::Transform Tglobal = T0 * T1;

            br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getName()));

            // Marker for the name
            visualization_msgs::Marker marker;
            marker.header.frame_id = this->getName();
            marker.header.stamp = ros::Time();
            marker.ns = this->getName();
            marker.id = 0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.z = 0.6;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.text = this->getName();

            vis_pub->publish( marker );

            // Maker for the bocas
            visualization_msgs::Marker bocas_marker;
            bocas_marker.header.frame_id = this->getName();
            bocas_marker.header.stamp = ros::Time();
            bocas_marker.ns = this->getName();
            bocas_marker.id = 0;
            bocas_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            bocas_marker.action = visualization_msgs::Marker::ADD;
            bocas_marker.scale.z = 0.4;
            bocas_marker.color.a = 1.0; // Don't forget to set the alpha!
            bocas_marker.pose.position.y = 0.4;
            bocas_marker.color.r = 0.0;
            bocas_marker.color.g = 0.0;
            bocas_marker.color.b = 0.0;
            bocas_marker.text = boca;
            bocas_marker.lifetime = ros::Duration(2);
            bocas_marker.frame_locked = 1;

            if (stratChange)
                bocas_pub->publish( bocas_marker );
        }
    };
}

/**
 * Creation of the node and classes needed.
 */
int main(int argc, char** argv)
{
    string player_name = "tmadeira";

    // Initialize node
    init(argc, argv, player_name);

    // Creating an instance of class Player
    tmadeira_ns::MyPlayer player1(player_name, "red");

    // Create a team
    tmadeira_ns::Team team_red("red");

    ros::Rate r(20);
    // Life cycle
    while(ros::ok())
    {
        spinOnce();
        r.sleep();
    }
}