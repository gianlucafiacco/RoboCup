/**
* @file ContextCoordinator.h
*	This file implements the team coordination module
* @author Francesco Riccio, Emanuele Borzi
*/


#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/SpqrDWKcombiner.h"
#include "Representations/spqr_representations/ContextCoordination.h"
#include "Representations/spqr_representations/EQualityNetworkEstimation.h"
#include "Representations/spqr_representations/OurDefinitions.h"

#include "Representations/BehaviorControl/Role.h"

#include <Core/Processors/Processor.h>

#include <mutex>

MODULE(ContextCoordinator, //TODO need to change Forget and Change as streamable and sand to all also the role toghether with the timestamp.. byebye
{,
 REQUIRES(GameInfo),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotInfo),
 REQUIRES(RobotPose),
 REQUIRES(BallModel),
 REQUIRES(FrameInfo),
 REQUIRES(FallDownState),
 REQUIRES(TeammateData),
 REQUIRES(EQualityNetworkEstimation),
 REQUIRES(SpqrDWKcombiner),
 PROVIDES(ContextCoordination),
 PROVIDES(Role),
 LOADS_PARAMETERS(
 {,
  (int) dead_robot_time_threshold,
  (int) time_hysteresis,
  (int) fall_down_penalty,
  (float) orientation_weight,
  (float) translation_weight,
  (float) striker_weight,
  (float) history_weight,
  (float) bias_weight,
  (float) goalie_pose_x,
  (float) goalie_pose_y,
  (float) goalie_pose_t,
  (float) defender_pose_x,
  (float) defender_pose_y,
  (float) defender_pose_t,
  (float) supporter_pose_x,
  (float) supporter_pose_y,
  (float) supporter_pose_t,
  (float) jolly_pose_x,
  (float) jolly_pose_y,
  (float) jolly_pose_t,
  (float) striker_pose_x,
  (float) striker_nokickoff_pose_x,
  (float) striker_pose_y,
  (float) striker_pose_t,
  (float) threshold_striker,
  (int) request_timeout,
  (int) request_timeout_others,
  (int) countdown_threshold,
 }),
       });

class ContextCoordinator: public ContextCoordinatorBase
{

    class InformationState{
    public:
        InformationState(){ timestamp = 0; }
        InformationState(int myId, Pose2f my_pose)
        {
            timestamp = SystemCall::getCurrentSystemTime();
            my_pose_stored = my_pose;
        }

        unsigned timestamp;
        Pose2f my_pose_stored;

    };

public:
    STREAMABLE(RoleRequest,
    {
                   public:

                   RoleRequest() = default;

                   RoleRequest(int id, unsigned ts_input, int role_in) { ts = ts_input; robot_id = id; role = role_in; },

                   ( int ) robot_id,
                   ( int ) role,
                   ( unsigned ) ts,
                   ( unsigned ) inMyTime,
               });

    STREAMABLE(RoleResponse,
    {
                   public:

                   RoleResponse() = default;

                   RoleResponse(ContextCoordinator::RoleRequest rr, int is_ok) { ts = rr.ts; robot_id = rr.robot_id; role = rr.role; ok =is_ok; },

                   ( int ) robot_id,
                   ( int ) role,
                   ( unsigned ) ts,
                   ( int ) ok,
               });

    STREAMABLE(ChangeNotification,
    {
                   public:

                   ChangeNotification() = default;

                   ChangeNotification(unsigned ts_in, int role_in) { role = role_in; ts = ts_in; },

                   ( unsigned ) ts,
                   ( int ) role,
               });

private:

    /** Utilities */
    float norm(float x, float y){ return sqrt(x*x + y*y); }
    float sign(float x){ if (x >= 0) return 1.f; else return -1.f; }
    template <typename T>
    std::string num2str( T num )
    {
        ostringstream ss;
        ss << num;
        return ss.str();
    }
    float str2float(std::string ss)
    {
        return atof(ss.c_str());
    }
    int str2int(std::string ss)
    {
        return atoi(ss.c_str());
    }

    InformationState info_for_utility[3];
    unsigned last_utility_computation;
    unsigned char pos_infos_to_add, pos_infos_to_check;


    //EMANUELE COORDINATION

    class Change
    {
    public:
        Change(){ change = false; ts = 0; role = -1; }
        bool change;
        unsigned ts;
        int role;
    };

    class Need_Change
    {
    public:
        Need_Change(){ change = false; ts = 0; }
        bool change;
        unsigned ts;
    };

    class Forget
    {
    public:
        Forget(){ forget = false; ts = 0; }
        bool forget;
        unsigned ts;
    };

    class Ack
    {
    public:
        Ack() { ack = false; ts = 0; }
        bool ack;
        unsigned ts;
    };

    unsigned timestamp_request;
    bool waiting_response;
    unsigned timestamp_change;
    int intention;
    Need_Change need_to_change;
    Change changed;
    Forget forget;

#ifdef TARGET_SIM
    static RoleRequest robots_need[5][5];
    static std::vector<std::vector<RoleRequest>> requests;
    static std::vector<std::mutex> mutex_req;
    static std::vector<std::mutex> mutex_res;
    static std::vector<std::mutex> mutex_need;
    static Ack acks[5][5];
    static bool leave_role[5];
#else
    static RoleRequest robots_need[5];
    static std::vector<RoleRequest> requests;
    static std::mutex mutex_req;
    static std::mutex mutex_res;
    static std::mutex mutex_need;
    static Ack acks[5];
    static bool leave_role;
#endif

public:

    PTracking::Timestamp stamp, role_time;

    //    int dead_robot_time_threshold;
    //    int time_hysteresis;
    //    int fall_down_penalty;
    //    int penalty;

#ifdef QUALITY_CONSENSUS
    float threshold_to_striker;
#endif

    //    float orientation_weight, translation_weight,
    //    striker_weight, history_weight, bias_weight;

    Pose2f goalie_pose, defender_pose, supporter_pose,
    jolly_pose, striker_pose, striker_nokickoff_pose;

    std::vector<std::vector<int> > utility_matrix;
    ContextCoordination::SpqrRole tmp_role, myRole;
    int role_hysteresis_cycle;
    std::vector<bool> mapped_robots;
    std::vector<int> penalized_robots;

    void configure();
    void configureHandshake();

    bool isInCurrentContext(ContextCoordination::SpqrRole currentRole);
    int getUtilityOrientationValue(float vx, float vy) const;
    int getUtilityRoleHistory(int j) const;
    int biasWeight(int role) const;
    bool isMaxRoleScore(int c);
    bool isMaxRoleScoreBetweenPoors(int c);
    bool amINearest();
    bool isRobotPenalized(int r);
    void computePlayingRoleAssignment();
    void computeSearchRoleAssignment();
    void computeThrowInRoleAssignment();
    void computeUtilityMatrix(const std::vector<Pose2f>& targets);
    void processRequests(ContextCoordination& contextCoordination);
    bool compareOthers(RoleRequest);
    void processMyRequest();
    void processForget();
    void processChanged();
    int evaluateAcks();
    void updateRobotRole(ContextCoordination& contextCoordination);

    void updatePlayingRoleSpace(ContextCoordination& contextCoordination);
    void updateSearchRoleSpace(ContextCoordination& contextCoordination);
    void updateThrowInRoleSpace(ContextCoordination& contextCoordination);

    ContextCoordinator();
    void update(ContextCoordination& contextCoordination);
    void update(Role& role);

    static void handleRoleRequest(int id, ContextCoordinator::RoleRequest rr)
    {
        //std::cerr << "[" << id+1 <<  "] handleRoleRequest IN -> Role: " << rr.role << " REQUESTED BY: " << rr.robot_id << std::endl;
#ifdef TARGET_SIM
        mutex_req.at(id).lock();
        requests.at(id).push_back(rr);
        mutex_req.at(id).unlock();
#else
        mutex_req.lock();
        requests.push_back(rr);
        mutex_req.unlock();
#endif
    }

    static void handleRoleResponse(int id, int from, ContextCoordinator::RoleResponse rs)
    {
#ifdef TARGET_SIM
        mutex_res.at(id).lock();
        if(acks[id][from].ts == 0 || acks[id][from].ts < rs.ts) // dovrei poter togliere la prima condizione...
        {
            acks[id][from].ts = rs.ts;
            //std::cerr << "[" << id+1 <<  "] handleRoleResponse IN " << rs.ts << " ||| " << rs.ok << " IN MERITO A: " << rs.robot_id << " FROM: " << from+1 << std::endl;
            acks[id][from].ack = rs.ok == 1 ? true : false;
        }
        mutex_res.at(id).unlock();
#else
        mutex_res.lock();
        if(acks[from].ts == 0 || acks[from].ts < rs.ts)
        {
            acks[from].ts = rs.ts;
            //std::cerr << "[" << id <<  "] handleRoleResponse IN " << rs.ts << " ||| " << rs.ok << " FROM: " << id+1 << std::endl;
            acks[from].ack = rs.ok == 1 ? true : false;
        }
        mutex_res.unlock();
#endif
    }

    static void handleChangedRole(int me, int byId, unsigned ts, int role, int myRole)
    {
        //        ( robots_need[A] != null && robots_need[A].ts = ts[A] )
        //                 robots_need[A] = none;
#ifdef TARGET_SIM
        mutex_need.at(me).lock();
        if( robots_need[me][byId].role != -1 && robots_need[me][byId].ts == ts )
        {
            if( ( (int)myRole ) == role )
            {
                // INSERT HERE A FIX FOR DOUBLE ROLES
                leave_role[me] = true;
            }

            //std::cerr << "[" << me+1 <<  "] handleChangedRole - removing: " << robots_need[me][byId].role << std::endl;
            robots_need[me][byId].role = -1;
            robots_need[me][byId].ts = 0;
        }
        mutex_need.at(me).unlock();
#else
        mutex_need.lock();
        if( robots_need[byId].role != -1 && robots_need[byId].ts == ts )
        {
            if( ( (int)myRole ) == role )
            {
                // INSERT HERE A FIX FOR DOUBLE ROLES
                leave_role = true;
            }
            robots_need[byId].role = -1;
            //std::cerr << "[" << me+1 <<  "] handleChangedRole - removing: " << robots_need[byId].role << std::endl;
            robots_need[byId].ts = 0;
        }
        mutex_need.unlock();
#endif
    }

    static void handleForgetRole(int me, int byId, unsigned ts)
    {
        //        ( robots_need[A] != null && robots_need[A].ts = ts[A] )
        //                 robots_need[A] = none;
#ifdef TARGET_SIM
        mutex_need.at(me).lock();
        if( robots_need[me][byId].role != -1 && robots_need[me][byId].ts == ts )
        {
            robots_need[me][byId].role = -1;
            robots_need[me][byId].ts = 0;
        }
        mutex_need.at(me).unlock();
#else
        mutex_need.lock();
        if( robots_need[byId].role != -1 && robots_need[byId].ts == ts )
        {
            robots_need[byId].role = -1;
            robots_need[byId].ts = 0;
        }
        mutex_need.unlock();
        //std::cerr << "handleForgetRole IN byId: "  << byId << std::endl;
#endif
    }

};
