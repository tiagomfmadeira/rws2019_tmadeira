#include <utility>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace boost;
using namespace ros;

namespace tmadeira_ns {

    //
    class Team
    {
    public:
        Team(string name)
        {
            this->name = name;
            // read team players from param
            n.getParam("/team_" + name, player_names);
        }

        // Setter used for testing purposes!
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
            for (std::string& player : player_names)
            {
                if (player == player_name) return true;
            }
            return false;
        }

        // Getter for team name
        string getName() {
            return this->name;
        }


    private:
        string name;
        vector<string> player_names;
        ros::NodeHandle n;
    };

    class Player {

        public:

            // Constructor
            explicit Player(string name) {
                this->name = name;
                this->team_name = "";
            }

            // string setter for team name
            void setTeamName(string name) {
                if (name == "red" || name == "green" || name == "blue") {
                    team_name = name;
                } else {
                    cout << "Cannot set team name " << name << ". Not part of the options!" << endl;
                }
            }

            // int setter for team name
            void setTeamName(int idx) {
                if (idx == 0) setTeamName("red");
                else if (idx == 1) setTeamName("green");
                else if (idx == 2) setTeamName("blue");
                else setTeamName("");
            }

            string getName() {
                return this->name;
            }

            // Getter for team name
            string getTeamName() {
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

            MyPlayer(string name, string team_name) : Player(name) {
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
                    cout << "Something went wrong in team parametrization!" << endl;
                }

                setTeamName(team_mine->getName());
            }

            void printInfo()
            {
                ROS_INFO_STREAM("My name is " << this->getName() << " and my team is " << team_mine->getName());
                ROS_INFO_STREAM("I am hunting " << team_prey->getName() << " and fleeing from " << team_hunters->getName());
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
    ros::init(argc, argv, "player_tmadeira");

    ros::NodeHandle n;

    string player_name = "tmadeira";

    // Creating an instance of class Player
    tmadeira_ns::MyPlayer player1 = tmadeira_ns::MyPlayer(player_name, "red");

    //cout << "Created an instance of class Player with public name " << player1.getName() << " of team " << player1.getTeamName() << "!" << endl;
    player1.printInfo();

    // Create a team
    tmadeira_ns::Team team_red("red");

    /*
    // Life cycle
    while(ros::ok())
    {
        team_red.printPlayerNames();

        // test playerBelongsToTeam() function
        string test_player = "tmadeira";
        if(team_red.playerBelongsToTeam(test_player))
        {
            cout << "Player " << test_player << " belongs to team!" << endl;
        } else
        {
            cout << "Player " << test_player << " does not belong to team!" << endl;
        }

        // Sleep one second
        ros::Duration(1).sleep();
    }
     */
}