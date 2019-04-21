#include <boost/config.hpp>
#include <boost/dll/alias.hpp>

#include <grsim/team.h>
#include <iostream>

// Robojacket has its own version of the command packet
#include "rj_grSim_Packet.pb.h"

namespace robojacket {

class RoboJacketTeam : public DefaultTeam {
public:
    RoboJacketTeam(bool is_yellow) :
        DefaultTeam(is_yellow){}

    virtual ~RoboJacketTeam() {}

    virtual std::vector<Team::Status> handle_cmds(char *in_buffer, int size) {
        rj_grSim_Packet packet;
        packet.ParseFromArray(in_buffer, size);

        if (!packet.has_commands() || packet.commands().isteamyellow() != is_yellow) {
            return {};
        }
        std::vector<Status> statuses;
        statuses.reserve(packet.commands().robot_commands_size());
        for (int i = 0; i < packet.commands().robot_commands_size(); i++)
        {
            const rj_grSim_Robot_Command& cmd = packet.commands().robot_commands(i);
            if (!cmd.has_id())
                continue;
            unsigned int id = cmd.id();
            if (id >= robots.size())
                continue;
//            PtrRobot robot = robots.at(id);
            if (cmd.has_wheelsspeed() && cmd.wheelsspeed()) {
                if (cmd.has_wheel1()) robots[id]->setSpeed(0, (double)cmd.wheel1());
                if (cmd.has_wheel2()) robots[id]->setSpeed(1, (double)cmd.wheel2());
                if (cmd.has_wheel3()) robots[id]->setSpeed(2, (double)cmd.wheel3());
                if (cmd.has_wheel4()) robots[id]->setSpeed(3, (double)cmd.wheel4());
            } else {
                dReal vx = cmd.has_veltangent() ? cmd.veltangent() : 0;
                dReal vy = cmd.has_velnormal() ? cmd.velnormal() : 0;
                dReal vw = cmd.has_velangular() ? cmd.velangular() : 0;
                robots[id]->setSpeed(vx, vy, vw);
            }

            dReal kickx = cmd.kickspeedx();
            dReal kickz = cmd.kickspeedz();
            bool kick = (kickx>0.0001 || kickz>0.0001);
            if (cmd.has_triggermode())
            {
                rj_grSim_Robot_Command_TriggerMode mode = cmd.triggermode();
                switch (mode) {
                    case rj_grSim_Robot_Command_TriggerMode_STAND_DOWN:
//                        if (id == 3) std::cout << "STAND_DOWN"<< std::endl;
                        kickx = 0;
                        kickz = 0;
                        kick = false;
                        break;
                    case rj_grSim_Robot_Command_TriggerMode_IMMEDIATE:
                        break;
                    case rj_grSim_Robot_Command_TriggerMode_ON_BREAK_BEAM:
                        if (!robots[id]->kicker->isTouchingBall()) {
                            kickx = 0;
                            kickz = 0;
                            kick = false;
                        }
                        break;
                }
            }


            if (kick)
                robots[id]->kicker->kick(kickx, kickz);
            int rolling = cmd.spinner()? 1 : 0;
            robots[id]->kicker->setRoller(rolling);

            char status = 0;
            status = (char)id;
            const uint8_t touching_ball = 0x1 << 3;
            const uint8_t just_kicked = 0x1 << 4;
            const uint8_t robot_on = 0x1 << 5;

            if (robots[id]->kicker->isTouchingBall()) status = status | touching_ball;
//            if (robot->kicker->justKicked()) status |= just_kicked; // TODO
            if (robots[id]->on) status = status |= robot_on;
            statuses.push_back(status);
        }
        return statuses;
    }
};

Team* createRobojacketTeam(bool is_yellow) {
    return new RoboJacketTeam(is_yellow);
}

BOOST_DLL_ALIAS(robojacket::createRobojacketTeam, create_robojacket_team)
}
