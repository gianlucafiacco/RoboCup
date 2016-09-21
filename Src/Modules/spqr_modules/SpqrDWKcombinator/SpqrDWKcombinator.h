/**
* @file SpqrDWKcombinator.h
*	This file implements the team Distributed World Model
* @author Francesco Riccio, Emanuele Borzi, Vincenzo Suriani
*/

#pragma once

#include <iostream>
#include <set>
#include <mutex>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/RobotPoseSpqrFiltered.h"
#include "Representations/spqr_representations/GlobalBallEstimation.h"
#include "Representations/spqr_representations/SpqrDWKcombiner.h"
#include "Representations/spqr_representations/EQualityNetworkEstimation.h"
#include "Representations/spqr_representations/ContextCoordination.h"

#include "Representations/spqr_representations/OurDefinitions.h"

#include <Core/Processors/Processor.h>

#define RO 1000
#define Kap 0.1f
#define Kbp 100.f
#define Kr 100
#define TEAMMATE_CO 500
#define ETA 1000
#define GAMMA 2

#define VEL_K 10

#define ATTR_VIS_W 0.1

#define SCALING_VIS 10


MODULE(SpqrDWKcombinator,
{,
 REQUIRES(GameInfo),
 REQUIRES(OpponentTeamInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotInfo),
 REQUIRES(BallModel),
 REQUIRES(TeamPlayersModel),
 REQUIRES(FrameInfo),
 REQUIRES(TeammateData),
 REQUIRES(RobotPoseSpqrFiltered),
 REQUIRES(GlobalBallEstimation),
 REQUIRES(EQualityNetworkEstimation),
 REQUIRES(FieldDimensions),
 USES(ContextCoordination),
 USES(SpqrDWKcombiner),
 PROVIDES(SpqrDWKcombiner),
 LOADS_PARAMETERS(
 {,
  (float) goto_target_update,
  (float) throw_in_time_threshold,
  (int) k_clusters,
  (int) max_kmean_iteration,
  (int) pkg_send_rate,
  (unsigned int) time_when_last_seen,
  (int) node_distance_x,
  (int) node_distance_y,
  (float) node_init_weight,
  (float) node_min_weight,
  (float) node_max_weight,
  (int) node_neightborhood,
  (int) node_score_recover_rate,
  (int) node_viewer,
  (int) viewer_translation_x,
  (int) viewer_translation_y,
  (int) viewer_tolerance,
  (float) mu_dynamic,
  (float) g_constant,
  (float) ball_radius,
 }),
});


class SpqrDWKcombinator: public SpqrDWKcombinatorBase
{

private:

    class Cluster
    {
    public:

        Cluster(Vector2f vect, int pb)
        {
            center = vect;
            perceivedBy = pb; //no multiple values
        }

        Cluster(){}

        Vector2f center;
        int perceivedBy;
    };

    class Node
    {
    public:
        Vector2f pos;
        bool throwIn;
        float score;
        PTracking::Timestamp timeSinceNodeWasSeen;


        Node(Vector2f np = Vector2f(), bool _tI = false, float _s = 0.5f):
            pos(np), throwIn(_tI), score(_s)
        {
            timeSinceNodeWasSeen.setToNow();
        }

        Node(const Node& node)
        {
            pos = node.pos;
            throwIn = node.throwIn;
            score = node.score;
            timeSinceNodeWasSeen = node.timeSinceNodeWasSeen;
        }

        ~Node(){}
    };

    class NodePF
    {
    public:
        Vector2f pos;
        Vector2f potential;
        NodePF():pos(Vector2f(0,0)), potential(Vector2f(0,0)){;}
        NodePF(Vector2f _pos, Vector2f _pot):pos(_pos), potential(_pot){;}
        ~NodePF(){}
    };

    /** Utilities */
    Vector2f glob2Rel(float x, float y) const;
    Vector2f rel2Glob(float x, float y) const;
    float norm(float x, float y){ return sqrt(x*x + y*y); }
    float sign(float x){ if (x >= 0) return 1.f; else return -1.f; }
    template <typename T>
    std::string num2str( T Number )
    {
        ostringstream ss;
        ss << Number;
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
    void tokenizeString(std::string _string, std::vector<std::string>* tokens);

    /** Members */
    float initial_time;
    PTracking::Timestamp stamp, searchForBallTimeUpdate, nodeViewerTimer,
    timeSinceLastMsgWasSent, timeSinceBallWasSeen, restoreNode;
    std::vector<Node*> occupancyGraph;
    SpqrDWKcombiner::Context context, prev_context;
    Vector2f tmpGlobEstimation;
    Vector2f tmpGlobPrediction;
    std::vector<Vector2f> unexplored_centroids;
    std::vector<Vector2f> explored_centroids;

    /** Config parameters */
    bool ballSeen;

    /** Methods */
    void configure();
    void resetGraph();
    void updateBallStatus(SpqrDWKcombiner& spqrDWKcombiner);
    Vector2f endRollingEstimatePosition();
    void updateRobotPoses(SpqrDWKcombiner& spqrDWKcombiner);
    void updateOpponentsBelief(SpqrDWKcombiner& spqrDWKcombiner);
    void updateNodeScoreWhenLastTimeBallSeen();
    void computeClusters(float value_treshold, bool reverse=false);
    void nodeViewer();
    void restoreNodeScore();
    void throwInNodesUpdate();
    SpqrDWKcombiner::Context contextProvider();
    void mergeTeammatesLM();
    void agglomerative_clustering(std::vector<SpqrDWKcombinator::Cluster>* adversaries);
    int getNearestPosInRF(Vector2f teammate_pose, Vector2f current_q, std::vector<NodePF*>* _pot_field);
    void computePF(SpqrDWKcombiner& spqrDWKcombiner);

    Pose2f getDefenderPlayingPosition(SpqrDWKcombiner &spqrDWKcombiner);
    Pose2f getSupporterPlayingPosition(SpqrDWKcombiner& spqrDWKcombiner);
    Pose2f getJollyPlayingPosition(SpqrDWKcombiner& spqrDWKcombiner);
    bool between(float value, float min, float max);



    float threshold = 0; // threshold for agglomerative clustering ( 340 )

#ifdef QUALITY_CONSENSUS

#ifdef DEBUG_WHO_IS_OUT
    PTracking::Timestamp stampForWho;
#endif

    float parameter_for_consider_low_channel; // 40 - 70
    float parameter_for_consider_good_channel; // 60 - 80

    int agentId, agentBasePort, agentPort;
    std::string agentAddress;
    std::vector<std::pair<std::string,int> > receivers;

    int n_robots;
    int base_port; //11937

    int how_many_for_trigger;
    int timer_for_clean, timer_for_clean_bad, timer_for_clean_good, timer_for_master, time_for_consider_dead; // 2000000, 5000, 5000, 25000, 4000

    bool proposed[5] = {false};
    bool proposed_for_good[5] = {false};
    //bool decided[5] = {false};
    bool out_of_coordination[5] = {false};
    std::set< int > response_received[5], response_received_for_good[5];
    std::set< int > ping[5], ping_for_good[5];
    std::set< int > replies[5], replies_for_good[5];
    std::mutex mutex; // TODO: forse meglio un mutex per robot da proporre...
    int master = 1;
    PTracking::Timestamp countdown_for_bad[5], countdown_for_good[5], countdown_for_master;

#endif

    void configureAgents(const string& configDirectory);
    void findLowChannelQualities();

    // callback
    void waiting();
    void timerForResponse();

    /*Anchor to callback*/
    static void* waitingThread(SpqrDWKcombinator* spqrDWKcombinator)
    {
        spqrDWKcombinator->waiting();
        return 0;
    }
    static void* timerThread(SpqrDWKcombinator* spqrDWKcombinator)
    {
        spqrDWKcombinator -> timerForResponse();
        return 0;
    }

public:
    SpqrDWKcombinator();
    ~SpqrDWKcombinator(){}
    void update(SpqrDWKcombiner& spqrDWKcombiner);
};
