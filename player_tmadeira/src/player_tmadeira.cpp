#include <utility>
#include <iostream>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>

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
            }

            void printInfo()
            {
                ROS_INFO_STREAM("My name is " << this->getName() << " and my team is " << team_mine->getName());
                ROS_INFO_STREAM("I am hunting " << team_prey->getName() << " and fleeing from " << team_hunters->getName());
            }

            void makeAPlayCallBack(rws2019_msgs::MakeAPlayConstPtr msg)
            {
                ROS_INFO("Received a new msg");

                // Publish the tranformation
                static tf::TransformBroadcaster br;

                tf::Transform transform1;
                transform1.setOrigin( tf::Vector3(1.0, 2.0, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform1.setRotation(q);

                br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", this->getName()));
            }

        private:
            boost::shared_ptr<Team> team_red;
            boost::shared_ptr<Team> team_blue;
            boost::shared_ptr<Team> team_green;
            boost::shared_ptr<Team> team_hunters;
            boost::shared_ptr<Team> team_mine;
            boost::shared_ptr<Team> team_prey;

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

    // Life cycle
    while(ros::ok())
    {

        // Sleep one second
        Duration(1).sleep();
        player1.printInfo();
        spinOnce();

    }
}