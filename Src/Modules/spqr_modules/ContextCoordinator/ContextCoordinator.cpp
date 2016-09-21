/**
* @file ContextCoordinator.cpp
*	This file implements the team coordination module
* @author Francesco Riccio, Emanuele Borzi
*/

#include "ContextCoordinator.h"

#include <unistd.h>
#include <iostream>
#include <Manfield/configfile/configfile.h>


#include <fstream> // ofstream

#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"["<<theRobotInfo.number<<"]"<<" [ContextCoordination] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"["<<theRobotInfo.number<<"]"<<" [ContextCoordination] " << x << "\033[0m" << std::endl;
#define SPQR_WARN(x) std::cerr << "\033[0;33;1m" <<"["<<theRobotInfo.number<<"]"<<" [ContextCoordination] " << x << "\033[0m" << std::endl;
#define SPQR_SUCC(x) std::cerr << "\033[0;32;1m" <<"["<<theRobotInfo.number<<"]"<<" [ContextCoordination] " << x << "\033[0m" << std::endl;

#define CONTEXT(x) \
    if(x == 2) std::cerr << "\033[22;32;1m"<<"Playing"<<"\033[0m" << std::endl; \
    else if(x == 3) std::cerr << "\033[22;31;1m"<<"Search for ball"<<"\033[0m" << std::endl; \
    else if(x == 4) std::cerr << "\033[22;33;1m"<<"Throw-In"<<"\033[0m" << std::endl; \

#define SPQR_ROLE(x) \
    if(x == 1) std::cerr << "\e[22;40m"<<"Goalie"<<"\e[0m" << std::endl; \
    else if(x == 2) std::cerr << "\033[22;32;1m"<<"Defender"<<"\033[0m" << std::endl; \
    else if(x == 3) std::cerr << "\033[22;30;1m"<<"Jolly"<<"\033[0m" << std::endl; \
    else if(x == 4) std::cerr << "\033[22;31;1m"<<"Supporter"<<"\033[0m" << std::endl; \
    else if(x == 5) std::cerr << "\033[22;33;1m"<<"Striker"<<"\033[0m" << std::endl; \
    else if(x == 6) std::cerr << "\033[22;30;1m"<<"Searcher 1"<<"\033[0m" << std::endl; \
    else if(x == 7) std::cerr << "\033[22;31;1m"<<"Searcher 2"<<"\033[0m" << std::endl; \
    else if(x == 8) std::cerr << "\033[22;33;1m"<<"Searcher 3"<<"\033[0m" << std::endl; \
    else if(x == 9) std::cerr << "\033[22;30;1m"<<"ThrowIn Searcher 1"<<"\033[0m" << std::endl; \
    else if(x == 10) std::cerr << "\033[22;31;1m"<<"ThrowIn Searcher 2"<<"\033[0m" << std::endl; \
    else if(x == 11) std::cerr << "\033[22;33;1m"<<"ThrowIn Searcher 3"<<"\033[0m" << std::endl; \
    else if(x == 12) std::cerr << "\033[22;32;1m"<<"Guard"<<"\033[0m" << std::endl; \
    else if(x == 13) std::cerr << "\e[22;40m"<<"No Role"<<"\e[0m" << std::endl; \

#define NORM(x, y) sqrt(x*x + y*y)

MAKE_MODULE(ContextCoordinator, spqr_modules)

#ifdef TARGET_SIM
std::vector<std::mutex> ContextCoordinator::mutex_req(5);
std::vector<std::mutex> ContextCoordinator::mutex_res(5);
std::vector<std::mutex> ContextCoordinator::mutex_need(5);
std::vector<std::vector<ContextCoordinator::RoleRequest>> ContextCoordinator::requests(5);
ContextCoordinator::Ack ContextCoordinator::acks[5][5];
ContextCoordinator::RoleRequest ContextCoordinator::robots_need[5][5];
bool ContextCoordinator::leave_role[5];
#else
std::mutex ContextCoordinator::mutex_req;
std::mutex ContextCoordinator::mutex_res;
std::mutex ContextCoordinator::mutex_need;
std::vector<ContextCoordinator::RoleRequest> ContextCoordinator::requests(0);
ContextCoordinator::Ack ContextCoordinator::acks[5];
ContextCoordinator::RoleRequest ContextCoordinator::robots_need[5];
bool ContextCoordinator::leave_role;
#endif

ContextCoordinator::ContextCoordinator():
    intention(-1), goalie_pose(Pose2f()), defender_pose(Pose2f()), supporter_pose(Pose2f()),
    jolly_pose(Pose2f()), striker_pose(Pose2f()), striker_nokickoff_pose(Pose2f()),
    tmp_role(ContextCoordination::no_role), myRole(ContextCoordination::no_role), role_hysteresis_cycle(0)
{
    SPQR::ConfigurationParameters();
    //update config file
    configure();

    for(unsigned int i=0; i<theSpqrDWKcombiner.robots_poses.size();++i) // 5 number of robots
    {
        std::vector<int> init_uv(theSpqrDWKcombiner.robots_poses.size(),0); // 5 number of roles
        utility_matrix.push_back(init_uv);

        mapped_robots.push_back(false);
        penalized_robots.push_back(fall_down_penalty);
    }

    role_time.setToNow();
    stamp.setToNow();

    last_utility_computation = -1000; //sure to compute initially
    pos_infos_to_add = -1;
    pos_infos_to_check = 0;

    for(int r = 0; r < 5; ++r)
    {
#ifdef TARGET_SIM
        requests.at(r) = std::vector<ContextCoordinator::RoleRequest>(0);
#else
        acks[r] = Ack();
        robots_need[r] = RoleRequest();
        acks[r].ack = false;
        acks[r].ts = 0;
#endif
    }

    waiting_response = false;

}

void ContextCoordinator::configure()
{
    goalie_pose = Pose2f(goalie_pose_t, goalie_pose_x, goalie_pose_y);
    goalie_pose.translation.x() = goalie_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    goalie_pose.translation.y() = goalie_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    defender_pose = Pose2f(defender_pose_t, defender_pose_x, defender_pose_y);
    defender_pose.translation.x() = defender_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    defender_pose.translation.y() = defender_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    supporter_pose = Pose2f(supporter_pose_t, supporter_pose_x, supporter_pose_y);
    supporter_pose.translation.x() = supporter_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    supporter_pose.translation.y() = supporter_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    jolly_pose = Pose2f(jolly_pose_t, jolly_pose_x, jolly_pose_y);
    jolly_pose.translation.x() = jolly_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    jolly_pose.translation.y() = jolly_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    striker_pose = Pose2f(striker_pose_t, striker_pose_x, striker_pose_y);
    striker_pose.translation.y() = striker_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    striker_pose.translation.y() = striker_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
    striker_nokickoff_pose = Pose2f(striker_pose_t, striker_nokickoff_pose_x, striker_pose_y);
    striker_nokickoff_pose.translation.y() = striker_nokickoff_pose.translation.x()*SPQR::FIELD_DIMENSION_X;
    striker_nokickoff_pose.translation.y() = striker_nokickoff_pose.translation.y()*SPQR::FIELD_DIMENSION_Y;
}

bool ContextCoordinator::isInCurrentContext(ContextCoordination::SpqrRole currentRole)
{
    if (((int) currentRole) == 13)
        return false;
    //playing context
    if((theSpqrDWKcombiner.current_context == SpqrDWKcombiner::playing) && (((int) currentRole) < 6))
        return true;
    //search_for_ball context
    if(theSpqrDWKcombiner.current_context == SpqrDWKcombiner::search_for_ball &&

            ( ((int) currentRole >5 && (int) currentRole < 9) || (int) currentRole ==  12))
        return true;
    //throw_in context

    if((theSpqrDWKcombiner.current_context == SpqrDWKcombiner::throw_in) && ((int) currentRole) > 8)
        return true;
    return false;
}

int ContextCoordinator::getUtilityOrientationValue(float vx, float vy) const
{
    if( NORM(vx,vy) < 500.f)
        return 200;
    else
        return (M_PI - Vector2f(vx,vy).angle())/M_PI;
}

int ContextCoordinator::getUtilityRoleHistory(int j) const
{
    //from the time since this particular role has been mapped
    uint time;
    time = (int)(PTracking::Timestamp() - role_time).getSeconds();

    if(tmp_role == j)
    {
        if(time*50 <= 500)
            return time*50;
        else return 500;
    }

    return 0;
}

int ContextCoordinator::biasWeight(int role) const
{
    if(role == theRobotInfo.number && role != 5) return 500.f;
    else return 0.f;
}

bool ContextCoordinator::isMaxRoleScore(int c)
{
    unsigned int max_r = 0;
    for(unsigned int r=0; r<utility_matrix.size(); ++r)
    {
        if(mapped_robots.at(r)) continue;
        if( utility_matrix.at(max_r).at(c-1) < utility_matrix.at(r).at(c-1) )
            max_r  = r;
    }

    if(max_r != 0)
        mapped_robots.at(max_r) = true;

    if( (int) max_r+1 == theRobotInfo.number) return true;
    return false;
}

bool ContextCoordinator::isMaxRoleScoreBetweenPoors(int c)
{
    if( theSpqrDWKcombiner.robots_out_of_coordination.size() == 0 ) return false;
    //    SPQR_SUCC("out are: " << theSpqrDWKcombiner.robots_out_of_coordination.size() );
    unsigned int max_r = theSpqrDWKcombiner.robots_out_of_coordination.at(0);
    std::vector<int>::const_iterator r = theSpqrDWKcombiner.robots_out_of_coordination.begin();
    for ( ; r != theSpqrDWKcombiner.robots_out_of_coordination.end(); ++r)
    {
        if(mapped_robots.at(*r)) continue;
        if( utility_matrix.at(max_r).at(c-1) < utility_matrix.at(*r).at(c-1) )
            max_r  = *r;
    }

    if(max_r != 0)
        mapped_robots.at(max_r) = true;

    if( (int) max_r+1 == theRobotInfo.number ) { return true; }
    return false;
}


bool ContextCoordinator::isRobotPenalized(int r)
{
    for(unsigned int t=0; t<theTeammateData.teammates.size(); ++t)
    {
        if(theTeammateData.teammates.at(t).number == r)
            return theTeammateData.teammates.at(t).status != Teammate::FULLY_ACTIVE;
    }

    return false;
}
bool ContextCoordinator::amINearest()
{
    Vector2f ball = theSpqrDWKcombiner.estimated_ball_global;
    float distanceMax = 10000;
    bool returningValue = false;

    std::vector<Pose2f>::const_iterator r = theSpqrDWKcombiner.robots_poses.begin();
    int displacement = 0;
    for ( ; r != theSpqrDWKcombiner.robots_poses.end(); ++r)
    {
        float distTemp = ( ( *r ).translation - ball ).norm();
        if(distTemp <= distanceMax) // minore
        {
            distanceMax = distTemp;
            if( (theRobotInfo.number-1) == displacement ) // sono io
            {
                returningValue = true;
            }
            else
            {
                returningValue = false;
            }
        }
        ++displacement;
    }
    return returningValue;

}

void ContextCoordinator::computePlayingRoleAssignment()
{
    for(unsigned int r=0; r<utility_matrix.size(); ++r)
        mapped_robots.at(r) = false;

#ifdef QUALITY_CONSENSUS

    if( ( ( theSpqrDWKcombiner.robots_poses.at(theRobotInfo.number-1).translation - theSpqrDWKcombiner.estimated_ball_global).norm() <= threshold_to_striker || amINearest() )
            && isMaxRoleScoreBetweenPoors((int) ContextCoordination::striker) )
    {
        tmp_role = ContextCoordination::striker;
    }
    else if( isMaxRoleScoreBetweenPoors( (int) ContextCoordination::jolly ) )
        tmp_role = ContextCoordination::jolly;
    else if( isMaxRoleScoreBetweenPoors( (int) ContextCoordination::supporter ) )
        tmp_role = ContextCoordination::supporter;
    else if( isMaxRoleScoreBetweenPoors( (int) ContextCoordination::defender ) )
        tmp_role = ContextCoordination::defender;
    else if( isMaxRoleScoreBetweenPoors((int) ContextCoordination::striker) )
        tmp_role = ContextCoordination::striker;
    else

#endif

        if( isMaxRoleScore((int) ContextCoordination::striker) )
            tmp_role = ContextCoordination::striker;
        else if( isMaxRoleScore((int) ContextCoordination::defender) /*&& theOwnTeamInfo.teamColor == TEAM_RED*/ ) //ptl
            tmp_role = ContextCoordination::defender;
        else if( isMaxRoleScore((int) ContextCoordination::supporter) )
            tmp_role = ContextCoordination::supporter;
        else if( isMaxRoleScore((int) ContextCoordination::jolly) )
            tmp_role = ContextCoordination::jolly;
        else
            tmp_role = ContextCoordination::no_role;

#ifdef DEEP_DEBUG_CONTEXT_COORD
    SPQR_INFO("tmp: "<<tmp_role); /// when no role has been selected the tmp role assigns the no_role enum.
    for(unsigned int r=0;r<mapped_robots.size(); ++r)
        SPQR_INFO("mapped robot "<<mapped_robots.at(r));
#endif
}

void ContextCoordinator::computeSearchRoleAssignment()
{
    for(unsigned int r=0; r<utility_matrix.size(); ++r)
        mapped_robots.at(r) = false;

    if( isMaxRoleScore((int) ContextCoordination::guard -10) )
        tmp_role = ContextCoordination::guard;
    else if( isMaxRoleScore((int) ContextCoordination::searcher_1 -3) )
        tmp_role = ContextCoordination::searcher_1;
    else if( isMaxRoleScore((int) ContextCoordination::searcher_2 -3) )
        tmp_role = ContextCoordination::searcher_2;
    else if( isMaxRoleScore((int) ContextCoordination::searcher_3 -3) )
        tmp_role = ContextCoordination::searcher_3;
    else
        tmp_role = ContextCoordination::no_role;

#ifdef DEEP_DEBUG_CONTEXT_COORD
    SPQR_INFO("tmp: "<<tmp_role); /// when no role has been selected the tmp role assigns the no_role enum.
    for(unsigned int r=0;r<mapped_robots.size(); ++r)
        SPQR_INFO("mapped robot "<<mapped_robots.at(r));
#endif
}

void ContextCoordinator::computeThrowInRoleAssignment()
{
    for(unsigned int r=0; r<utility_matrix.size(); ++r)
        mapped_robots.at(r) = false;

    if( isMaxRoleScore((int) ContextCoordination::guard -10) )
        tmp_role = ContextCoordination::guard;
    else if( isMaxRoleScore((int) ContextCoordination::throwin_searcher_1 -6) )
        tmp_role = ContextCoordination::throwin_searcher_1;
    else if( isMaxRoleScore((int) ContextCoordination::throwin_searcher_2 -6) )
        tmp_role = ContextCoordination::throwin_searcher_2;
    else if( isMaxRoleScore((int) ContextCoordination::throwin_searcher_3 -6) )
        tmp_role = ContextCoordination::throwin_searcher_3;
    else
        tmp_role = ContextCoordination::no_role;

#ifdef DEEP_DEBUG_CONTEXT_COORD
    SPQR_INFO("tmp: "<<tmp_role); /// when no role has been selected the tmp role assigns the no_role enum.
    for(unsigned int r=0;r<mapped_robots.size(); ++r)
        SPQR_INFO("mapped robot "<<mapped_robots.at(r));
#endif
}

void ContextCoordinator::computeUtilityMatrix(const std::vector<Pose2f>& targets)
{

#ifdef COORDINATION_TODO
    /*
    * test for guarantee consistent state. TODO: check validity of vector (mean?) and do not consider entry r=0 (goalie)
    */

    bool ready = false, late = false;
    unsigned now = SystemCall::getCurrentSystemTime(), diff = 9999;
    int best_time = 0;
    for(int t = 0; t<10; ++t)
    {
        unsigned diff_tmp = now - theTeammateData.coordinationBuffer.references[t];//( ( theTeammateData.coordinationBuffer.getMax(t) - theTeammateData.coordinationBuffer.getMin(t) ) / 2);
        if( diff_tmp < 3000 ) // vector not so distant in time
        {

            if( //theTeammateData.coordinationBuffer.informationMatrix[r][t].ts != 0 && // is it just started ? - maybe this check can be erase
                    theTeammateData.coordinationBuffer.fallHere( t, 3000 )) // values not distant more than 'range'
            {
                ready = true;
                if( diff_tmp < diff && diff_tmp > 0)
                {
                    diff = diff_tmp;
                    best_time = t;
                }
            }
        }
    }

    if( !ready && (SystemCall::getCurrentSystemTime() - last_utility_computation) > /*threshold for computation*/ 3000) // too late, I must compute something - now it is large for testing
    {
        SPQR_INFO( "LATE!");
        late = true;
    }


    if(ready)
    {
        for(unsigned int r=1; r<5; ++r) // how many robots are alive ?
        {
            //            SPQR_SUCC(theTeammateData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.x() << ", "
            //                      << theTeammateData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.y());
            if(theTeammateData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.x() == .0f) continue;
            for(unsigned int j=1; j<targets.size(); ++j)
            {
                penalty = 0;
                if(isRobotPenalized(r)) penalty = 1;

                utility_matrix.at(r).at(j) = (1-penalty) /** theTeammateData.coordinationBuffer.informationMatrix[r][best_time].validity */ * (
                            + translation_weight * (1000*(SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD -
                                                          norm(targets.at(j).translation.x()-theTeammateData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.x(),
                                                               targets.at(j).translation.y()-theTeammateData.coordinationBuffer.informationMatrix[r][best_time].pose.translation.y()))/
                                                    SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD)
                            + bias_weight * biasWeight(j)
                            + history_weight * getUtilityRoleHistory(j)
                            + orientation_weight * getUtilityOrientationValue(targets.at(j).translation.x(), targets.at(j).translation.y()));
            }
        }
        last_utility_computation = SystemCall::getCurrentSystemTime();
    }
    else if(late) // previous utility computation
    {
        for(unsigned int r=1; r<theSpqrDWKcombiner.robots_poses.size(); ++r)
        {
            if(theSpqrDWKcombiner.robots_poses.at(r).translation.x() == .0f) continue;
            for(unsigned int j=1; j<targets.size(); ++j)
            {
                penalty = 0;
                if(isRobotPenalized(r)) penalty = 1;

                utility_matrix.at(r).at(j) = (1-penalty) * theRobotPose.validity * (
                            + translation_weight * (1000*(SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD -
                                                          norm(targets.at(j).translation.x()-theSpqrDWKcombiner.robots_poses.at(r).translation.x(),
                                                               targets.at(j).translation.y()-theSpqrDWKcombiner.robots_poses.at(r).translation.y()))/
                                                    SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD)
                            + bias_weight * biasWeight(j)
                            + history_weight * getUtilityRoleHistory(j)
                            + orientation_weight * getUtilityOrientationValue(targets.at(j).translation.x(), targets.at(j).translation.y()));
            }
        }
        last_utility_computation = SystemCall::getCurrentSystemTime();
    }

#endif

    for(unsigned int r=1; r<theSpqrDWKcombiner.robots_poses.size(); ++r) // how many robots are alive ?
    {
        for(unsigned int j=1; j<targets.size(); ++j)
        {
            if(theSpqrDWKcombiner.robots_poses.at(r).translation.x() == .0f) continue;
            int penalty = 0;
            if(isRobotPenalized(r)) penalty = 1;
//            if(theRobotInfo.number == 2 && theOwnTeamInfo.teamColor == TEAM_BLUE) penalty = 1; // ptl bug


            utility_matrix.at(r).at(j) = (1-penalty) /** theTeammateData.coordinationBuffer.informationMatrix[r][best_time].validity */ * (
                        + translation_weight * (1000*(SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD -
                                                      norm(targets.at(j).translation.x()-theSpqrDWKcombiner.robots_poses.at(r).translation.x(),
                                                           targets.at(j).translation.y()-theSpqrDWKcombiner.robots_poses.at(r).translation.y()))/
                                                SPQR::MAXIMUM_DISTANCE_ON_THE_FIELD)
                        + bias_weight * biasWeight(j)
                        + history_weight * getUtilityRoleHistory(j)
                        + orientation_weight * getUtilityOrientationValue(targets.at(j).translation.x(), targets.at(j).translation.y()));
        }
    }
}

void ContextCoordinator::processRequests(ContextCoordination& contextCoordination)
{
#ifdef TARGET_SIM
    std::vector<ContextCoordinator::RoleRequest>::iterator it = requests.at(theRobotInfo.number-1).begin();
    for( ; it != requests.at(theRobotInfo.number-1).end(); ++it )
#else
    std::vector<ContextCoordinator::RoleRequest>::iterator it = requests.begin();
    for( ; it != requests.end(); ++it )
#endif
    {

        if( (SystemCall::getCurrentSystemTime() - it->inMyTime) < (unsigned int)request_timeout ) //TODO this time need to be modified
        {
            //SPQR_INFO("I'M IN TIME FOR RESPONSE");
            if(
                    ( (it->role == intention) && (timestamp_request < it->inMyTime) ) || // is my intention older than the other one?
                    ( ( ((int)contextCoordination.robotRole ) == it->role) && (intention == -1) ) //I don't want to modify my role!
                    )
            {
                //SPQR_INFO("(" << it->robot_id << ") YOU CAN'T BECOME " << it->role << ", My intention is " << intention);
                // it sends back to A 'byebye (NO), ts(A), ts(Me)'
                ContextCoordinator::RoleResponse rs = ContextCoordinator::RoleResponse(*it,-1); //I'm currently not sending ts(me) - TOCHECK
                //TEAM_OUTPUT(idRoleResponse, bin, rs); TODO
            }
            else if (!compareOthers(*it) )
            {
                //SPQR_INFO("(" << it->robot_id << ") NOT COMPARE OTHERS");
                // it sends back to A 'byebye (NO), ts(A), ts(Me)'
                ContextCoordinator::RoleResponse rs = ContextCoordinator::RoleResponse(*it,-1); //I'm currently not sending ts(me) - TOCHECK
                //TEAM_OUTPUT(idRoleResponse, bin, rs); TODO
            }
            else
            {
                // it sends back to A 'You're welcome (YES), ts(A), ts(Me)'; robots_need[A] = {Role, ts[A]}
#ifdef TARGET_SIM
                mutex_need.at(theRobotInfo.number-1).lock();
                robots_need[theRobotInfo.number-1][it->robot_id - 1] = *it;
                mutex_need.at(theRobotInfo.number-1).unlock();
#else
                mutex_need.lock();
                robots_need[it->robot_id - 1] = *it;
                mutex_need.unlock();
#endif
                //SPQR_INFO("(" << it->robot_id << ") YOU CAN BE " << it->role);
                ContextCoordinator::RoleResponse rs = ContextCoordinator::RoleResponse(*it,1); //I'm currently not sending ts(me) - TOCHECK
                //TEAM_OUTPUT(idRoleResponse, bin, rs); TODO
            }
        }
        else
        {
            //SPQR_INFO("NOT IN TIME FOR RESPONSE: " << (SystemCall::getCurrentSystemTime() - it->inMyTime));
            ContextCoordinator::RoleResponse rs = ContextCoordinator::RoleResponse(*it,-1); //I'm currently not sending ts(me) - TOCHECK
            //TEAM_OUTPUT(idRoleResponse, bin, rs); TODO
        }

    }
#ifdef TARGET_SIM
    requests.at(theRobotInfo.number-1).clear();
#else
    requests.clear();
#endif
}

bool ContextCoordinator::compareOthers(RoleRequest rr)
{
#ifdef TARGET_SIM
    mutex_need.at(theRobotInfo.number-1).lock();
#else
    mutex_need.lock();
#endif
    for (int r = 0; r<5; ++r) // compare intention of others and I want to guarantee FCFS order (I HOPE)
    {
        if(
        #ifdef TARGET_SIM
                robots_need[theRobotInfo.number-1][r].role == rr.role && // someone else wants this role
                robots_need[theRobotInfo.number-1][r].inMyTime < rr.inMyTime && // and it has request this one before the current Request
                (rr.inMyTime - robots_need[theRobotInfo.number-1][r].inMyTime) < 3000 // not so distant in time the requests
        #else
                robots_need[r].role == rr.role && // someone else wants this role
                robots_need[r].inMyTime < rr.inMyTime && // and it has request this one before the current Request
                (rr.inMyTime - robots_need[r].inMyTime) < (unsigned int)request_timeout_others // not so distant in time the requests
        #endif
                )
        {
            //it will sends back to A 'byebye (NO), ts(A), ts(Me)' (out of this function)
#ifdef TARGET_SIM
            mutex_need.at(theRobotInfo.number-1).unlock();
#else
            mutex_need.unlock();
#endif
            return false;
        }
        //        else
        //            SPQR_ERR((rr.inMyTime - robots_need[theRobotInfo.number-1][r].inMyTime));

    }
#ifdef TARGET_SIM
    mutex_need.at(theRobotInfo.number-1).unlock();
#else
    mutex_need.unlock();
#endif
    return true;
}

void ContextCoordinator::processMyRequest()
{
    need_to_change.change = false;
    ContextCoordinator::RoleRequest rr = ContextCoordinator::RoleRequest(theRobotInfo.number, timestamp_request, intention);
    //TEAM_OUTPUT(idRoleRequest, bin, rr); TODO
    //TODO stream the RoleRequest
}

void ContextCoordinator::processForget()
{
    forget.forget = false;
    //TEAM_OUTPUT(idForgetRoleRequest, bin, forget.ts); TODO
    //TODO stream the forget request
}

void ContextCoordinator::processChanged()
{
    changed.change = false;
    ChangeNotification cn = ChangeNotification(changed.ts, changed.role);
    //TEAM_OUTPUT(idChangedRole, bin, cn); TODO
    //TODO stream the role's request change TODO
}

int ContextCoordinator::evaluateAcks()
{
    int count = 0;
#ifdef TARGET_SIM
    mutex_res.at(theRobotInfo.number-1).lock();
#else
    mutex_res.lock();
#endif
    for( int r = 0; r < 5; ++r)
    {
        if(r+1 != theRobotInfo.number)
        {
#ifdef TARGET_SIM
            if( (acks[theRobotInfo.number-1][r].ts != 0) && (acks[theRobotInfo.number-1][r].ts == timestamp_request) && !acks[theRobotInfo.number-1][r].ack )
            {
                mutex_res.at(theRobotInfo.number-1).unlock();
                SPQR_INFO("FAILING: " << -1);
                return -1;
            }
            else if( (acks[theRobotInfo.number-1][r].ts != 0) && (acks[theRobotInfo.number-1][r].ts == timestamp_request) && acks[theRobotInfo.number-1][r].ack )
            {
                ++count;
            }
#else
            if( (acks[r].ts != 0) && (acks[r].ts == timestamp_request) && (!acks[r].ack) )
            {
                mutex_res.unlock();
                return -1;
            }
            else if( (acks[r].ts != 0) && (acks[r].ts == timestamp_request) && (acks[r].ack) )
            {
                ++count;
            }
#endif
        }
    }
#ifdef TARGET_SIM
    mutex_res.at(theRobotInfo.number-1).unlock();
#else
    mutex_res.unlock();
#endif
    return count;
}

void ContextCoordinator::updateRobotRole(ContextCoordination& contextCoordination)
{

#ifdef WRITE_DYNAMIC_HYSTERESIS
    //if(theGameInfo.state == STATE_PLAYING)
    {
        static PTracking::Timestamp lastPrintTime;
        bool isPrintTime = ((PTracking::Timestamp() - lastPrintTime).getMs() > 2000.0);
        if ( isPrintTime )
        {
            lastPrintTime.setToNow();
            std::stringstream to_write;
            to_write << ( 200 - ( 150 * theEQualityNetworkEstimation.convex_quality / 100 ) ) << "\n";
            ofstream myFile;
            std::stringstream ss;
            ss << "dynamic_hysteresis_robot" << theRobotInfo.number << ".txt";
            myFile.open(ss.str(),std::ofstream::ate | std::ofstream::app);
            myFile << to_write.str() << "\n";
            myFile.close();
        }
    }
#endif

    /// WARNING: handle cases like: contextCoordination.robotRole == 0
    if( (contextCoordination.robotRole == ContextCoordination::no_role && tmp_role != ContextCoordination::no_role) ||
            //contextCoordination.robotRole == 0 ||
            !isInCurrentContext(contextCoordination.robotRole) )
    {
        contextCoordination.robotRole = tmp_role;
        myRole = tmp_role;
    }
    else
    {
        if(contextCoordination.robotRole != tmp_role) ++role_hysteresis_cycle;
        else role_hysteresis_cycle = 0;

#if defined(DYNAMIC_HYSTERESIS)

        if (
                ( TEAM_RED == theOwnTeamInfo.teamColor && role_hysteresis_cycle >= time_hysteresis ) ||
                ( TEAM_BLUE == theOwnTeamInfo.teamColor && role_hysteresis_cycle >= ( 200 - ( 150 * theEQualityNetworkEstimation.convex_quality / 100 ) ) ) )
#elif defined(NO_HYSTERESIS)

#else
        if ( role_hysteresis_cycle >= time_hysteresis )
#endif
        {
            contextCoordination.robotRole = tmp_role;
            myRole = tmp_role;
            role_hysteresis_cycle = 0;
            role_time.setToNow();
        }
    }
}

void ContextCoordinator::updatePlayingRoleSpace(ContextCoordination& contextCoordination)
{
    if(theRobotInfo.number == 1)
    {
        contextCoordination.robotRole = ContextCoordination::goalie;
        myRole = ContextCoordination::goalie;
    }
    else
    {
        /// define targets: the order is crazy important
        std::vector<Pose2f> targets;
        targets.push_back(goalie_pose);
        targets.push_back(defender_pose);
        targets.push_back(jolly_pose);
        targets.push_back(supporter_pose);
        targets.push_back(Pose2f(theSpqrDWKcombiner.estimated_ball_global));
#if defined(DEBUG_AND_WRITE_ROLE_PERSISTENCE)
        Pose2f posa_palla = Pose2f(theSpqrDWKcombiner.estimated_ball_global);
#endif

        /// compute the utility matrix
        computeUtilityMatrix(targets);

        /// map the current configuration in the 'role space'
        computePlayingRoleAssignment();

        /// update spqr robot role
        updateRobotRole(contextCoordination);

        /*
        * PLOTTING
        */
#if defined(DEBUG_AND_WRITE_ROLE_PERSISTENCE)
        Vector2f my_pos = Vector2f(
                    theSpqrDWKcombiner.robots_poses.at( theRobotInfo.number-1 ).translation.x(),
                    theSpqrDWKcomer.robots_poses.at( theRobotInfo.number-1 ).translation.y()
                    );
        Vector2f my_role_pos;
        std::stringstream to_plot;
        switch(contextCoordination.robotRole)
        {
        case ContextCoordination::goalie:
            my_role_pos = Vector2f( goalie_pose.translation.x(), goalie_pose.translation.y() );
            to_plot << "Goalie";
            break;
        case ContextCoordination::defender:
            my_role_pos = Vector2f( defender_pose.translation.x(), defender_pose.translation.y() );
            to_plot << "Defender";
            break;
        case ContextCoordination::jolly:
            my_role_pos = Vector2f( jolly_pose.translation.x(), jolly_pose.translation.y() );
            to_plot << "Jolly";
            break;
        case ContextCoordination::supporter:
            my_role_pos = Vector2f( supporter_pose.translation.x(), supporter_pose.translation.y() );
            to_plot << "Supporter";
            break;
        case ContextCoordination::striker:
            my_role_pos = Vector2f( posa_palla.translation.x(), posa_palla.translation.y() );
            to_plot << "Striker";
            break;
        case ContextCoordination::no_role:
            my_role_pos = Vector2f( 0,0 );
            to_plot << "NO_ROLE";
            break;
        }

        if( contextCoordination.robotRole == ContextCoordination::no_role )
            to_plot << " <> MY POSITION: " << my_pos.x << "," << my_role_pos.y <<
                       " <> MY ROLE POSITION: " << 0 << "," << 0 <<
                       " <> DISTANCE: " << 9000.0;
        else
            to_plot << " <> MY POSITION: " << my_pos.x << "," << my_pos.y <<
                       " <> MY ROLE POSITION: " << my_role_pos.x << "," << my_role_pos.y <<
                       " <> DISTANCE: " << (my_pos - my_role_pos).absFloat();

        //SPQR_INFO(to_plot.str());

        std::ofstream myfile;
        std::stringstream ss;
        ss << "log_roles/log_roles_" << theRobotInfo.number << (theOwnTeamInfo.teamColor == TEAM_BLUE ? "_(BLUE)" : "_(RED)" ) << ".txt";
        myfile.open(ss.str(),std::ofstream::ate | std::ofstream::app);
        myfile << to_plot.str() << "\n";
        myfile.close();
#endif

    }
}

void ContextCoordinator::updateSearchRoleSpace(ContextCoordination& contextCoordination)
{
    if(theRobotInfo.number == 1)
    {
        contextCoordination.robotRole = ContextCoordination::goalie;
        myRole = ContextCoordination::goalie;
    }
    else
    {
        /// define targets
        std::vector<Pose2f> targets;
        targets.push_back(goalie_pose);
        targets.push_back(defender_pose);
        for(unsigned int i=0; i<theSpqrDWKcombiner.unexplored_clusters_centroids.size(); ++i)
            targets.push_back(theSpqrDWKcombiner.unexplored_clusters_centroids.at(i));

        /// compute the utility matrix
        computeUtilityMatrix(targets);

        /// map the current configuration in the 'role space'
        computeSearchRoleAssignment();

        /// update spqr robot role
        updateRobotRole(contextCoordination);
    }
}

void ContextCoordinator::updateThrowInRoleSpace(ContextCoordination& contextCoordination)
{
    if(theRobotInfo.number == 1)
    {
        contextCoordination.robotRole = ContextCoordination::goalie;
        myRole = ContextCoordination::goalie;
    }
    else
    {
        /// define targets
        std::vector<Pose2f> targets;
        targets.push_back(goalie_pose);
        targets.push_back(defender_pose);
        for(unsigned int i=0; i<theSpqrDWKcombiner.unexplored_clusters_centroids.size(); ++i)
        {
            /// forcing that in a throwIn context the ball is on a side of the field
            if(theSpqrDWKcombiner.unexplored_clusters_centroids.at(i).y() > 0.f)
                targets.push_back(Vector2f(theSpqrDWKcombiner.unexplored_clusters_centroids.at(i).x(), SPQR::FIELD_DIMENSION_Y-300));
            else
                targets.push_back(Vector2f(theSpqrDWKcombiner.unexplored_clusters_centroids.at(i).x(), -(SPQR::FIELD_DIMENSION_Y-300)));
        }

        /// compute the utility matrix
        computeUtilityMatrix(targets);

        /// map the current configuration in the 'role space'
        computeThrowInRoleAssignment();

        /// update spqr robot role
        updateRobotRole(contextCoordination);
    }
}

void ContextCoordinator::update(ContextCoordination& contextCoordination)
{
    if(theGameInfo.state == STATE_PLAYING)
    {
        // EMANUELE COORDINATION
        if( theTeammateData.sendThisFrame ) // only if I can send packets in this period of time
        {
#ifdef TARGET_SIM
            mutex_req.at(theRobotInfo.number-1).lock();
            processRequests(contextCoordination);
            mutex_req.at(theRobotInfo.number-1).unlock();
#else
            mutex_req.lock();
            processRequests(contextCoordination);
            mutex_req.unlock();
#endif
            //            if ( forget.forget ){ processForget(); }
            //            if ( need_to_change.change ){ processMyRequest(); }
            //            if ( changed.change ){ processChanged(); }
        }

        // ---------------------------------------------------------------

#ifdef LEAVE_DOUBLE_ROLE
#ifdef TARGET_SIM
        mutex_need.at(theRobotInfo.number-1).lock();
        if(ContextCoordinator::leave_role[theRobotInfo.number-1])
        {
            contextCoordination.robotRole = ContextCoordination::no_role;
            myRole = ContextCoordination::no_role; // 13
            ContextCoordinator::leave_role[theRobotInfo.number-1] = false;
        }
        mutex_need.at(theRobotInfo.number-1).unlock();
#else
        mutex_need.lock();
        if(ContextCoordinator::leave_role)
        {
            contextCoordination.robotRole = ContextCoordination::no_role;
            myRole = ContextCoordination::no_role; // 13
            ContextCoordinator::leave_role = false;
        }
        mutex_need.unlock();
#endif
#endif


        /// playing context
        if(theSpqrDWKcombiner.current_context == SpqrDWKcombiner::playing)
            updatePlayingRoleSpace(contextCoordination);

        /// search for ball context
        else if(theSpqrDWKcombiner.current_context == SpqrDWKcombiner::search_for_ball)
            updateSearchRoleSpace(contextCoordination);

        /// throw in context
        else if(theSpqrDWKcombiner.current_context == SpqrDWKcombiner::throw_in)
            updateThrowInRoleSpace(contextCoordination);


        /// assign keeper role   ----------------- Goal Keeper
        if(theRobotInfo.number == 1)
        {
            contextCoordination.robotRole = ContextCoordination::goalie;
            myRole = ContextCoordination::goalie;
        }
    }

#ifdef DEBUG_CONTEXT_COORD
    if(theGameInfo.state == STATE_PLAYING && theRobotInfo.number != 1)
        //        if(theGameInfo.state == STATE_PLAYING && (theRobotInfo.number == 2 || theRobotInfo.number == 5) )
    {
        if ((PTracking::Timestamp() - stamp).getMs() > 3000)
        {
            for(unsigned int i=0; i<theSpqrDWKcombiner.unexplored_clusters_centroids.size(); ++i)
                SPQR_INFO("Centroid "<<i<<": "<<theSpqrDWKcombiner.unexplored_clusters_centroids.at(i).x<<", "<<
                          theSpqrDWKcombiner.unexplored_clusters_centroids.at(i).y);
        }
    }
#endif

#ifdef UM_VIEW
    //        if(theGameInfo.state == STATE_PLAYING && (theRobotInfo.number == 5) )
    if(theGameInfo.state == STATE_PLAYING)
    {
        if( theRobotInfo.number != 1 )
        {
            if ((PTracking::Timestamp() - stamp).getMs() > 3000)
            {
                std::cout<<"Context: "; CONTEXT(theSpqrDWKcombiner.current_context);
                std::cerr<<"Spqr Role mapped to robot ["<<theRobotInfo.number<<"] => ";
                SPQR_ROLE((int) contextCoordination.robotRole);
                for(unsigned int i=0; i<utility_matrix.size(); ++i)
                {
                    for(unsigned int j=0; j<utility_matrix.at(i).size(); ++j)
                    {
                        if(utility_matrix.at(i).at(j) >= 850) std::cout<<"\033[0;32;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                        else if(utility_matrix.at(i).at(j) < 100) std::cout<<"\033[22;31;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                        else std::cout<<"\033[22;34;1m"<<"["<<utility_matrix.at(i).at(j)<<"]--- \033[0m";
                    }

                    std::cout<<std::endl;
                }
                std::cout<<std::endl;
                stamp.setToNow();
            }
        }
    }
#endif
}

void ContextCoordinator::update(Role& role)
{
    switch( (int) myRole )
    {
    case 13:
        role.role = Role::RoleType::no_role;
        break;
    case 1:
        role.role = Role::RoleType::goalie;
        break;
    case 2:
        role.role = Role::RoleType::defender;
        break;
    case 3:
        role.role = Role::RoleType::jolly;
        break;
    case 4:
        role.role = Role::RoleType::supporter;
        break;
    case 5:
        role.role = Role::RoleType::striker;
        break;
    }
}

