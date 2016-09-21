/**
* @file SpqrDWKcombinator.cpp
*	This file implements the team Distributed World Model
* @author Francesco Riccio, Emanuele Borzi, Vincenzo Suriani
*/

#include "SpqrDWKcombinator.h"

#include <string.h>
#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <UdpSocket.h>
#include <Utils/AgentPacket.h>
#include <Manfield/configfile/configfile.h>
#include <stdlib.h>

using namespace std;
using namespace PTracking;
using GMapping::ConfigFile;

///definisco le maschere (Attenzione!! Si lavora sui bit e verifico la membership applicando le maschere)
///per aggiungere un membro si fa uno OR
///per confrontare si fa un AND e si verifica se != 0
/// Fiero di questa soluzione!! :D
//#define MASK_1 1 //00001
//#define MASK_2 2 //00010
//#define MASK_3 4 //00100
//#define MASK_4 8 //01000
//#define MASK_5 16 //10000

#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"["<<theRobotInfo.number<<"]"<<" [SqprDWKcombinator] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"["<<theRobotInfo.number<<"]"<<" [SqprDWKcombinator] " << x << "\033[0m" << std::endl;
#define SPQR_SUCC(x) std::cerr << "\033[0;32;1m" <<"["<<theRobotInfo.number<<"]"<<" [SqprDWKcombinator] " << x << "\033[0m" << std::endl;
#define SPQR_WARN(x) std::cerr << "\033[0;33;1m" <<"["<<theRobotInfo.number<<"]"<<" [SqprDWKcombinator] " << x << "\033[0m" << std::endl;
#define SPQR_SPECIAL(x) std::cerr << "\033[0;35;1m" <<"["<<theRobotInfo.number<<"]"<<" [SqprDWKcombinator] " << x << "\033[0m" << std::endl;

#define CONTEXT(x) \
    if(x == 2) std::cerr << "\033[22;32;1m"<<"Playing"<<"\033[0m" << std::endl; \
    else if(x == 3) std::cerr << "\033[22;31;1m"<<"Search for ball"<<"\033[0m" << std::endl; \
    else if(x == 4) std::cerr << "\033[22;33;1m"<<"Throw-In"<<"\033[0m" << std::endl; \

//#define DEEP_DEBUG_DWK
//#define DEBUG_DWK
//#define GRAPH_VIEWER

MAKE_MODULE(SpqrDWKcombinator, spqr_modules)

SpqrDWKcombinator::SpqrDWKcombinator(): context(SpqrDWKcombiner::no_context), prev_context(SpqrDWKcombiner::no_context),
    tmpGlobEstimation(Vector2f()), tmpGlobPrediction(Vector2f()), ballSeen(false)
{
    //load configs
    configure();
    SPQR::ConfigurationParameters();
    initial_time = SystemCall::getCurrentSystemTime();

    //init graph
    for(int i=SPQR::FIELD_DIMENSION_X; i>-SPQR::FIELD_DIMENSION_X; i-= node_distance_x)
    {
        for(int j=SPQR::FIELD_DIMENSION_Y; j>-SPQR::FIELD_DIMENSION_Y; j-= node_distance_y)
        {
            if(j-node_distance_y== -SPQR::FIELD_DIMENSION_Y || j == SPQR::FIELD_DIMENSION_Y)
                occupancyGraph.push_back(new Node( Vector2f(i-node_distance_x/2,j-node_distance_y/2), true));
            else
                occupancyGraph.push_back(new Node( Vector2f(i-node_distance_x/2,j-node_distance_y/2) ));
        }
    }
#ifdef DEEP_DEBUG_DWK
    SPQR_INFO("Initializing graph...");
    for(unsigned int n=0; n<occupancyGraph.size(); ++n)
    {
        std::cout<<"["<<occupancyGraph.at(n)->pos.x<<", "<<occupancyGraph.at(n)->pos.y<<"]---";
        if( n%(SPQR::FIELD_DIMENSION_Y/500) == SPQR::FIELD_DIMENSION_Y/500 -1) std::cout<<std::endl;
    }
#endif

    for(int i=0; i<k_clusters; ++i)
    {
        explored_centroids.push_back(Vector2f());
        unexplored_centroids.push_back(Vector2f());
    }

    stamp.setToNow();
    searchForBallTimeUpdate.setToNow();
    nodeViewerTimer.setToNow();
    restoreNode.setToNow();
    timeSinceLastMsgWasSent.setToNow();
    timeSinceBallWasSeen.setToNow();

#ifdef QUALITY_CONSENSUS

#ifdef DEBUG_WHO_IS_OUT
    stampForWho.setToNow();
#endif

    for(int i=0;i < n_robots;i++)
    {
        countdown_for_bad[i].setToNow();
        countdown_for_good[i].setToNow();
    }
    countdown_for_master.setToNow();

    /*--------Thread for receive packets and evaluate the quality------------*/
    pthread_t waitingThreadId;
    pthread_create(&waitingThreadId,0,(void*(*)(void*)) waitingThread,this);

    string configDirectory = "";
    char currentWorkingDirectory[1024];
    if (SystemCall::getMode() == SystemCall::simulatedRobot)
    {
        if (getcwd(currentWorkingDirectory,1024)) {;}

        configDirectory = currentWorkingDirectory;

        configDirectory = configDirectory.substr(0,configDirectory.rfind("/")) + "/";
    }
    else configDirectory = "Config/";
    std::string location = Global::getSettings().location;
    configDirectory += string("Locations/") + location + "/";

    //setting up agents and parameters
    configureAgents(configDirectory);

    pthread_t timerThreadId;
    pthread_create(&timerThreadId,0,(void*(*)(void*)) timerThread, this); //avvio il thread che ogni tanto mi pulisce le strutture dati

#endif

}

void SpqrDWKcombinator::tokenizeString(std::string _string, std::vector<std::string>* tokens)
{
    int j= 0;
    std::string token="";
    for( unsigned int i=0; i<_string.size(); ++i )
    {
        if(i==_string.size()-1)
            tokens->at(j)=(token+_string[_string.size()-1]);

        if(_string[i] != ' ')
            token += _string[i];
        else
        {
            tokens->at(j)=(token);
            token=""; ++j;
        }
    }
}

Vector2f SpqrDWKcombinator::glob2Rel(float x, float y) const
{
    Vector2f result;

    float tempX = x - theRobotPoseSpqrFiltered.x;
    float tempY = y - theRobotPoseSpqrFiltered.y;

    result.x() = tempX * cos(theRobotPoseSpqrFiltered.theta) + tempY * sin(theRobotPoseSpqrFiltered.theta);
    result.y() = -tempX * sin(theRobotPoseSpqrFiltered.theta) + tempY * cos(theRobotPoseSpqrFiltered.theta);

    return Vector2f(result.x(),result.y());
}

Vector2f SpqrDWKcombinator::rel2Glob(float x, float y) const
{
    Vector2f result;
    float rho = sqrt((x * x) + (y * y));

    result.x() = theRobotPoseSpqrFiltered.x + (rho * cos(theRobotPoseSpqrFiltered.theta + atan2(y,x)));
    result.y() = theRobotPoseSpqrFiltered.y + (rho * sin(theRobotPoseSpqrFiltered.theta + atan2(y,x)));

    return Vector2f(result.x(),result.y());
}

void SpqrDWKcombinator::configure()
{

#ifdef QUALITY_CONSENSUS

    if (!fCfg.read(configDirectory + string(std::string("Locations/") + location + "/consensusDWKParameters.cfg")))
    {
        SPQR_ERR("Error reading file '" << configDirectory + std::string("Locations/") + location + "/networkParameters.cfg'. Exiting...");
        exit(-1);
    }

    try
    {
        section = "parameters";

        threshold = atof(string(fCfg.value(section,"threshold_for_agg_clustering")).c_str());

        parameter_for_consider_low_channel = atof(string(fCfg.value(section, "parameter_for_consider_low_channel")).c_str());

        parameter_for_consider_good_channel = atof(string(fCfg.value(section, "parameter_for_consider_good_channel")).c_str());

        n_robots = atoi(string(fCfg.value(section, "n_robots")).c_str());

        base_port = atoi(string(fCfg.value(section, "base_port")).c_str());

        how_many_for_trigger = atoi(string(fCfg.value(section, "how_many_for_trigger")).c_str());

        timer_for_clean = atoi(string(fCfg.value(section, "timer_for_clean")).c_str());

        timer_for_clean_bad = atoi(string(fCfg.value(section, "timer_for_clean_bad")).c_str());

        timer_for_clean_good = atoi(string(fCfg.value(section, "timer_for_clean_good")).c_str());

        timer_for_master = atoi(string(fCfg.value(section, "timer_for_master")).c_str());

        time_for_consider_dead = atoi(string(fCfg.value(section, "time_for_consider_dead")).c_str());

    }
    catch (...)
    {
        SPQR_ERR("Not existing value. Exiting...");
        exit(-1);
    }

#endif

}

void SpqrDWKcombinator::agglomerative_clustering(std::vector<SpqrDWKcombinator::Cluster>* adversaries) //actually cost is O(N^3).. consider that N max is 20..
{
    //initializing clusters (each element as a cluster - bottom up approach) - Keep only adversaries
    /*
    for(int i=1; i<theTeammateData.numberOfActiveTeammates; i++)
    {
        for (std::vector<SpqrRobotPerception::SpqrRobotBox>::const_iterator it = theTeammateData.robotPercepts[i].robots.begin(); it != theTeammateData.robotPercepts[i].robots.end(); ++it)
        {
            ///TODO initial release: testing with red as opponent team.
            if(it -> detectedJersey && ((it -> teamRed && TEAM_BLUE == theOwnTeamInfo.teamColor) || (TEAM_RED == theOwnTeamInfo.teamColor && !it -> teamRed)))
            {
                adversaries -> push_back(SpqrDWKcombinator::Cluster(Vector2<>(it -> x_glob, it -> y_glob), 1 << (i-1)));
            }
        }
    }
    if(adversaries -> size()==0)
    {
        return;
    }

    float threshold_temp = threshold; //TODO think about this
    bool out = false;
    while(true)
    {
        int pos_A = 0, pos_B = 0, a_temp = 0;
        float distance = 99999;
        SpqrDWKcombinator::Cluster a , b;
        bool modified = false;
        for (std::vector<SpqrDWKcombinator::Cluster>::const_iterator it_A = adversaries -> begin(); it_A != adversaries -> end(); ++it_A, a_temp++)
        {
            int b_temp = 0;
            for (std::vector<SpqrDWKcombinator::Cluster>::const_iterator it_B = adversaries -> begin(); it_B != adversaries -> end(); ++it_B, b_temp++)
            {

                if(!out && (it_A->perceivedBy & it_B->perceivedBy)!=0) // Esclude anche il caso in cui A == B
                {
                    continue;
                }
                float distFound = (it_A->center - it_B->center).absFloat();

                if(distFound != 0 && distFound < distance && distFound < threshold_temp)
                {
                    modified = true;

                    distance = distFound;
                    a = *it_A;
                    b = *it_B;
                    pos_A = a_temp, pos_B = b_temp;
                }

            }
        }
        if(!modified)
        {
            if(adversaries -> size() > 5)
            {
                out = true;
                threshold_temp += 50.0;
                continue;
            }
            break;
        }

        //MERGE CLUSTERS

        //        if(theRobotInfo.number==3) std::cerr << "MERGING" << std::endl;
        //        if(theRobotInfo.number==3)
        //        {
        //            std::cerr << "elem of A:   " << a.perceivedBy << std::endl;
        //            std::cerr << "elem of B:   " << b.perceivedBy << std::endl;
        //        }
        //        if(theRobotInfo.number==3)
        //            for(std::vector<SpqrDWKcombinator::Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); it++)
        //            {
        //                std::cerr << "PRIMA x: " << it -> center.x << " | y: " << it -> center.y << std::endl;
        //            }

        if(pos_A < pos_B)
        {
            adversaries -> erase(adversaries -> begin() + pos_B);
            adversaries -> erase(adversaries -> begin() + pos_A);
        }
        else
        {
            adversaries -> erase(adversaries -> begin() + pos_A);
            adversaries -> erase(adversaries -> begin() + pos_B);
        }

        //        if(theRobotInfo.number==3)
        //            for(std::vector<SpqrDWKcombinator::Cluster>::const_iterator it = clusters.begin(); it != clusters.end(); it++)
        //            {
        //                std::cerr << "DOPO x: " << it -> center.x << " | y: " << it -> center.y << std::endl;
        //            }
        //        if(theRobotInfo.number==3) std::cerr << "Removing at position A: " << pos_A << " | Removing at position B: " << pos_B << std::endl;
        //        if(theRobotInfo.number==3) std::cerr << "Inserting -> x: " << ((a.center+b.center)/2).x << "  -> y: " << ((a.center+b.center)/2).y << "  -> PB: " << (a.perceivedBy | b.perceivedBy) << std::endl;

        adversaries -> push_back(SpqrDWKcombinator::Cluster(Vector2<>((a.center+b.center)/2), (a.perceivedBy | b.perceivedBy))); // add the mean of two points!
    }
*/return; // TODO Vincenzo
}

void SpqrDWKcombinator::updateRobotPoses(SpqrDWKcombiner& spqrDWKcombiner)
{
    spqrDWKcombiner.robots_poses.at(theRobotInfo.number-1) = Pose2f(theRobotPoseSpqrFiltered.theta,theRobotPoseSpqrFiltered.x,theRobotPoseSpqrFiltered.y);
    if(!theTeammateData.teammates.size()) return;

    for(unsigned int i=0; i<theTeammateData.teammates.size(); ++i)
    {
        Pose2f tmp_pose = Pose2f(theTeammateData.teammates.at(i).robotPoseSpqrFiltered.theta,
                                 theTeammateData.teammates.at(i).robotPoseSpqrFiltered.x,
                                 theTeammateData.teammates.at(i).robotPoseSpqrFiltered.y);

        spqrDWKcombiner.robots_poses.at(theTeammateData.teammates.at(i).number-1) = tmp_pose;
    }
}

void SpqrDWKcombinator::updateBallStatus(SpqrDWKcombiner& spqrDWKcombiner)
{
    //update ball status: seen or not seen, last known position xy, elapsed time since not seen,

    // if at least one of the robots in the team perceives the ball...
    if ( theFrameInfo.time - theBallModel.timeWhenLastSeen < time_when_last_seen || theGlobalBallEstimation.isSingleRobotValid ||
         theGlobalBallEstimation.isMultiRobotValid )
    {
        ballSeen=true;
        timeSinceBallWasSeen.setToNow();
        spqrDWKcombiner.timeSinceWasSeen = 0;
    }
    else
    {
        ballSeen=false;
        spqrDWKcombiner.timeSinceWasSeen = str2float( num2str((float) (PTracking::Timestamp() - timeSinceBallWasSeen).getMs()) );
    }

    if( ballSeen && (theFrameInfo.time - theBallModel.timeWhenLastSeen < time_when_last_seen) )
    {
        spqrDWKcombiner.estimated_ball_global = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
        tmpGlobEstimation = Vector2f(theGlobalBallEstimation.singleRobotX, theGlobalBallEstimation.singleRobotY);
    }
    else if( ballSeen && theGlobalBallEstimation.isSingleRobotValid)
    {
        spqrDWKcombiner.estimated_ball_global = Vector2f(theGlobalBallEstimation.singleRobotX, theGlobalBallEstimation.singleRobotY);
        tmpGlobEstimation = Vector2f(theGlobalBallEstimation.singleRobotX, theGlobalBallEstimation.singleRobotY);
    }
    else if( ballSeen && theGlobalBallEstimation.isMultiRobotValid )
    {
        spqrDWKcombiner.estimated_ball_global = Vector2f(theGlobalBallEstimation.multiRobotX, theGlobalBallEstimation.multiRobotY);
        tmpGlobEstimation = Vector2f(theGlobalBallEstimation.multiRobotX, theGlobalBallEstimation.multiRobotY);
    }

    spqrDWKcombiner.estimated_ball_relative = glob2Rel(tmpGlobEstimation.x(), tmpGlobEstimation.y());

#ifdef DEEP_DEBUG_DWK

    if(ballSeen)
    {
        SPQR_SUCC("Ball seen");
        SPQR_INFO("Ball position: "<<spqrDWKcombiner.estimated_ball_global.x<<", "<<spqrDWKcombiner.estimated_ball_global.y);
    }
    else
    {
        //TODO: output a float meaning the time interval
        SPQR_ERR("ball not seen since: "<< spqrDWKcombiner.timeSinceWasSeen);
        SPQR_INFO("Last known ball position: "<<spqrDWKcombiner.estimated_ball_global.x<<", "<<spqrDWKcombiner.estimated_ball_global.y);
    }
#endif
}

Vector2f SpqrDWKcombinator::endRollingEstimatePosition()
{
    float ballDirectionAngle = theBallModel.estimate.velocity.angle();

    //Compute the ball friction acceleration
    float acceleration = ( mu_dynamic * g_constant ) /ball_radius;
    float rFAx = acceleration * cos(ballDirectionAngle);
    float rFAy = acceleration * sin(ballDirectionAngle);

    //Compute the time to stop the ball
    float timeToStop  = 0.f;
    if(theBallModel.estimate.velocity.norm() != 0)
        timeToStop = theBallModel.estimate.velocity.norm()/acceleration;

    //Compute the end position
    return Vector2f(theBallModel.estimate.position.x() + (theBallModel.estimate.velocity.x() * timeToStop) - (0.5 * rFAx * timeToStop * timeToStop),
                    theBallModel.estimate.position.y() + (theBallModel.estimate.velocity.y() * timeToStop) - (0.5 * rFAy * timeToStop * timeToStop));
}

void SpqrDWKcombinator::updateOpponentsBelief(SpqrDWKcombiner& spqrDWKcombiner)
{

    // merge all the teammates perception and generate a guess of opponents positions
    //     std::cout<<theTeammateData.robotPercepts[0].robots.size()<<std::endl;
    //    std::vector<SpqrDWKcombinator::Cluster> adversaries;
    //    SpqrDWKcombinator::agglomerative_clustering(&adversaries);

    //    if( adversaries.size() > 5 ) SPQR_ERR( "TROPPI AVVERSARI PERCEPITI!" );

    //    for(int i=0; i<(int)adversaries.size(); i++)
    //    {
    //        spqrDWKcombiner.opponents_estimated_poses.at(i) = Vector2f(adversaries.at(i).center.x(),adversaries.at(i).center.y());
    //    }

    //    return; //ptl

    if (SystemCall::getTimeSince(initial_time)  > 2000 )
    {
        std::vector<OpponentModel> tmp_opp = theSpqrDWKcombiner.opponents;
        spqrDWKcombiner.opponents.clear();
        initial_time = SystemCall::getCurrentSystemTime();
        for(unsigned int i=0; i<theTeamPlayersModel.obstacles.size(); ++i)
        {
            if(theTeamPlayersModel.obstacles.at(i).type == Obstacle::opponent)
            {
                OpponentModel om;
                om.estimated_pose = theTeamPlayersModel.obstacles.at(i).center;
                //            if(tmp_opp.size())
                //            {
                //                om.estimated_velocity = tmp_opp.at(i).estimated_velocity;
                //                if((PTracking::Timestamp() - stamp).getMs() > 1000) //BUG
                //                {
                //                    Vector2f tmp_v1 = rel2Glob(om.estimated_pose.translation.x(), om.estimated_pose.translation.y());
                //                    Vector2f tmp_v2 = rel2Glob(tmp_opp.at(i).estimated_pose.translation.x(), tmp_opp.at(i).estimated_pose.translation.y());
                //                    if((tmp_v1 - tmp_v2).norm() != 0)
                //                        om.estimated_velocity = tmp_v1 - tmp_v2;
                //                    stamp.setToNow();
                //                }
                //            }
                spqrDWKcombiner.opponents.push_back( om );
            }
        }
    }

    //    for(unsigned int i=0; i<theObstacleModel.obstacles.size(); ++i)
    //    {
    //        if(theObstacleModel.obstacles.at(i).type == Obstacle::opponent)
    //        {
    //            OpponentModel om;
    //            om.estimated_pose = Pose2f(rel2Glob(theObstacleModel.obstacles.at(i).center.x(), theObstacleModel.obstacles.at(i).center.y()));
    //            if(tmp_opp.size())
    //            {
    //                om.estimated_velocity = tmp_opp.at(i).estimated_velocity;

    //                if((PTracking::Timestamp() - stamp).getMs() > 1000) //BUG
    //                {
    //                    Vector2f tmp_v1 = rel2Glob(om.estimated_pose.translation.x(), om.estimated_pose.translation.y());
    //                    Vector2f tmp_v2 = rel2Glob(tmp_opp.at(i).estimated_pose.translation.x(), tmp_opp.at(i).estimated_pose.translation.y());

    //                    if((tmp_v1 - tmp_v2).norm() != 0)
    //                        om.estimated_velocity = tmp_v1 - tmp_v2;
    //                    stamp.setToNow();
    //                }
    //            }
    ////            spqrDWKcombiner.opponents.push_back( om );
    //        }
    //    }





}

void SpqrDWKcombinator::updateNodeScoreWhenLastTimeBallSeen()
{
    for(unsigned int n=0; n<occupancyGraph.size(); ++n)
    {
        if( abs(tmpGlobEstimation.x() - occupancyGraph.at(n)->pos.x()) <= node_neightborhood*node_distance_x &&
                abs(tmpGlobEstimation.y() - occupancyGraph.at(n)->pos.y()) <= node_neightborhood*node_distance_y)
            occupancyGraph.at(n)->score = 0.9f;
        else occupancyGraph.at(n)->score = 0.3f;
    }
}

void SpqrDWKcombinator::computeClusters(float value_treshold, bool reverse)
{
    std::vector<Node*> instances;
    std::vector<Vector2f> centroids;
    std::vector<std::vector<Node*> > clusters(k_clusters);

    for(unsigned int n=0; n<occupancyGraph.size(); ++n)
    {
        if(reverse)
        {
            if(occupancyGraph.at(n)->score < value_treshold)
                instances.push_back(occupancyGraph.at(n));
        }
        else
        {
            if(occupancyGraph.at(n)->score > value_treshold)
                instances.push_back(occupancyGraph.at(n));
        }
    }

    if(instances.size() == 0)
    {
        //        if(reverse) computeClusters(value_treshold*1.1, true);
        //        else computeClusters(value_treshold*0.9);
        //resetGrid();
        return;
    }
    else
    {
        // Initializing centroids
        for(int i=0; i<k_clusters; ++i)
            centroids.push_back( Vector2f(instances.at( i*(instances.size()/4) )->pos) );

        int interationcount = 0;
        while(interationcount < max_kmean_iteration)
        {

            // compute distances
            for(unsigned int i=0; i<instances.size(); ++i)
            {
                float minDistance = FLT_MAX;
                int minDistanceCentroid = -1;
                for(unsigned int j=0; j<centroids.size(); ++j)
                {
                    // evaluate min distance
                    if(minDistance > norm(instances.at(i)->pos.x() - centroids.at(j).x(),
                                          instances.at(i)->pos.y() - centroids.at(j).y()))
                    {
                        minDistance = norm(instances.at(i)->pos.x() - centroids.at(j).x(),
                                           instances.at(i)->pos.y() - centroids.at(j).y());
                        minDistanceCentroid = j;
                    }
                }
                // updates the K vector of instances
                if( minDistanceCentroid != -1)
                    clusters.at(minDistanceCentroid).push_back( instances.at(i));
            }


            // update centroids
            for(unsigned int i=0; i<clusters.size(); ++i)
            {
                float centroidX = 0.f;
                float centroidY = 0.f;

                for(unsigned int j=0; j<clusters.at(i).size(); ++j)
                {
                    centroidX += /*clusters.at(i).at(j)->score **/ clusters.at(i).at(j)->pos.x();
                    centroidY += /*clusters.at(i).at(j)->score **/ clusters.at(i).at(j)->pos.y();
                }
                if(clusters.at(i).size() != 0)
                    centroids.at(i) = Vector2f(centroidX/ clusters.at(i).size(), centroidY/ clusters.at(i).size());
                else
                    centroids.at(i) = Vector2f(0.f, 0.f);
            }
            ++interationcount;
        }

#ifdef DEEP_DEBUG_DWK
        for(unsigned int j=0; j<centroids.size(); ++j)
            SPQR_INFO("["<<theRobotInfo.number<<"] Centroid ["<<j<<"] reverse: "<<reverse<<" : " << centroids.at(j).x<<", "<<centroids.at(j).y);
        std::cout<<std::endl;
#endif

        if(reverse)
        {
            for(int j=0; j<k_clusters; ++j)
                explored_centroids.at(j) = centroids.at(j);
        }
        else
        {
            for(int j=0; j<k_clusters; ++j)
                unexplored_centroids.at(j) = centroids.at(j);
        }
    }
}

void SpqrDWKcombinator::nodeViewer()
{
    if ((PTracking::Timestamp() - nodeViewerTimer).getMs() > node_viewer)
    {
        for(unsigned int n=0; n<occupancyGraph.size(); ++n)
        {
            bool modify = false;

            if( norm(rel2Glob(viewer_translation_x, 0.f).x() - occupancyGraph.at(n)->pos.x(),
                     rel2Glob(viewer_translation_x, 0.f).y() - occupancyGraph.at(n)->pos.y()) < viewer_tolerance ) modify = true;

            if( norm(rel2Glob(viewer_translation_x, viewer_translation_y).x() - occupancyGraph.at(n)->pos.x(),
                     rel2Glob(viewer_translation_x, viewer_translation_y).y() - occupancyGraph.at(n)->pos.y()) < viewer_tolerance ) modify = true;

            if( norm(rel2Glob(viewer_translation_x, -viewer_translation_y).x() - occupancyGraph.at(n)->pos.x(),
                     rel2Glob(viewer_translation_x, -viewer_translation_y).y() - occupancyGraph.at(n)->pos.y()) < viewer_tolerance ) modify = true;

            if( norm(rel2Glob(viewer_translation_x*2, 0.f).x() - occupancyGraph.at(n)->pos.x(),
                     rel2Glob(viewer_translation_x*2, 0.f).y() - occupancyGraph.at(n)->pos.y()) < viewer_tolerance ) modify = true;

            if(modify)
            {
                // if the robot didn't find the ball
                if(!ballSeen)
                    occupancyGraph.at(n)->score = 0.2f;

                occupancyGraph.at(n)->timeSinceNodeWasSeen.setToNow();
            }
        }
        nodeViewerTimer.setToNow();
    }
}

void SpqrDWKcombinator::restoreNodeScore()
{
    if((PTracking::Timestamp() - restoreNode).getMs() > 12000)
    {
        for(unsigned int n=0; n<occupancyGraph.size(); ++n)
        {
            if(occupancyGraph.at(n)->score <= 0.5f)
                occupancyGraph.at(n)->score +=
                        (PTracking::Timestamp() - occupancyGraph.at(n)->timeSinceNodeWasSeen).getMs()/node_score_recover_rate;

            if(occupancyGraph.at(n)->score > 1.f) occupancyGraph.at(n)->score = 0.9f;
        }

        restoreNode.setToNow();
    }
}

void SpqrDWKcombinator::throwInNodesUpdate()
{
    resetGraph();

    // if the last known position is reliable
    if ((PTracking::Timestamp() - timeSinceBallWasSeen).getMs() < time_when_last_seen*2)
    {
        for(unsigned int n=0; n<occupancyGraph.size(); ++n)
        {
            if(occupancyGraph.at(n)->throwIn)
            {
                // the node is on the side of the last percepted ball (w.r.t. y-Axis)
                if( sign(tmpGlobEstimation.y())*sign(occupancyGraph.at(n)->pos.y()) > 0 )
                {
                    // if our team kicks the ball out of the field...
                    if(theGameInfo.dropInTeam == theOwnTeamInfo.teamColor)
                    {
                        if( tmpGlobEstimation.x() > SPQR::FIELD_DIMENSION_X-1000 )
                        {
                            if(abs(occupancyGraph.at(n)->pos.x()) < node_distance_x)
                                occupancyGraph.at(n)->score = 0.8f;
                            else occupancyGraph.at(n)->score = 0.5f;
                        }
                        else if( tmpGlobEstimation.x() < -SPQR::FIELD_DIMENSION_X+1000 )
                        {
                            if( abs(-SPQR::FIELD_DIMENSION_X -occupancyGraph.at(n)->pos.x()) <= node_distance_x)
                                occupancyGraph.at(n)->score = 0.8f;
                            else occupancyGraph.at(n)->score = 0.5f;
                        }

                    }
                    else
                    {
                        if( tmpGlobEstimation.x() > SPQR::FIELD_DIMENSION_X-1000 )
                        {
                            // position computed according to the game rules
                            if(abs(occupancyGraph.at(n)->pos.x() - SPQR::FIELD_DIMENSION_X) <= node_distance_x)
                                occupancyGraph.at(n)->score = 0.8f;
                            else occupancyGraph.at(n)->score = 0.5f;

                        }
                        else if( tmpGlobEstimation.x() < -SPQR::FIELD_DIMENSION_X+1000 )
                        {
                            if(abs(occupancyGraph.at(n)->pos.x()) < node_distance_x)
                                occupancyGraph.at(n)->score = 0.8f;
                            else occupancyGraph.at(n)->score = 0.5f;
                        }
                    }
                }
                //                else occupancyGraph.at(n)->score = 0.6f;
            }
            else occupancyGraph.at(n)->score = 0.3f;
        }
    }
    else
    {
        for(unsigned int n=0; n<occupancyGraph.size(); ++n)
        {
            if(occupancyGraph.at(n)->throwIn) occupancyGraph.at(n)->score = 0.7f;
            else occupancyGraph.at(n)->score = 0.4f;
        }
    }
}

SpqrDWKcombiner::Context SpqrDWKcombinator::contextProvider()
{
    //formalize contexts defined by the ctor

    //set init context
    SpqrDWKcombiner::Context current_status = SpqrDWKcombiner::no_context;

    // throw-In interrupt
    if(!ballSeen)
    {
        if(theGameInfo.dropInTime < 15 && theGameInfo.dropInTime != 0) current_status = SpqrDWKcombiner::throw_in;
        else current_status = SpqrDWKcombiner::search_for_ball;
    }
    else current_status = SpqrDWKcombiner::playing;
    return current_status;
}

void SpqrDWKcombinator::mergeTeammatesLM()
{
    for(unsigned int node=0; node<occupancyGraph.size(); ++node)
    {
        bool node_controlled = false;
        for(unsigned int i=0; i<theTeammateData.teammates.size(); ++i) //number of robots in the team, to retrieve from sent data, size vector
        {
            if( (int) i+1 == theRobotInfo.number ) continue;

            for(unsigned int j=0; j<theTeammateData.teammates.at(i).spqrDWKcombination.explored_clusters_centroids.size(); ++j)
            {
                if( theTeammateData.teammates.at(i).spqrDWKcombination.explored_clusters_centroids.at(j).x() !=0 ||
                        theTeammateData.teammates.at(i).spqrDWKcombination.explored_clusters_centroids.at(j).y() !=0 )
                {
                    if( std::abs(occupancyGraph.at(node)->pos.x() - theTeammateData.teammates.at(i).spqrDWKcombination.explored_clusters_centroids.at(j).x())
                            < node_neightborhood*node_distance_x &&

                            std::abs(occupancyGraph.at(node)->pos.y() - theTeammateData.teammates.at(i).spqrDWKcombination.explored_clusters_centroids.at(j).y())
                            < node_neightborhood*node_distance_y)
                        node_controlled = true;
                }
            }
        }

        if(occupancyGraph.at(node)->score > node_init_weight && node_controlled)
            occupancyGraph.at(node)->score = 0.3f;
    }
}

void SpqrDWKcombinator::resetGraph()
{
    for(unsigned int i=0; i< occupancyGraph.size(); ++i)
    {
        occupancyGraph.at(i)->score = node_init_weight;
        occupancyGraph.at(i)->timeSinceNodeWasSeen.setToNow();
    }
}

int SpqrDWKcombinator::getNearestPosInRF(Vector2f teammate_pose, Vector2f current_q, vector<NodePF*>* _pot_field)
{
    float min_err = SPQR::FIELD_DIMENSION_X * 2;
    int min_err_idx = -1;

    for(unsigned int n=0; n<_pot_field->size(); ++n)
    {
        if( (teammate_pose - _pot_field->at(n)->pos ).norm() < TEAMMATE_CO) 	//TODO check Vincenzo
            if( (teammate_pose - current_q).norm() < min_err)
            {
                min_err = (teammate_pose - current_q ).norm();
                min_err_idx = n;
            }
    }
    return min_err_idx;
}

void SpqrDWKcombinator::computePF(SpqrDWKcombiner& spqrDWKcombiner)
{
    Vector2f my_pos = Vector2f(theRobotPoseSpqrFiltered.x,theRobotPoseSpqrFiltered.y);

    vector<NodePF> potential_field;
    //init obstacle model
    int d = 200;
    for(int i=SPQR::FIELD_DIMENSION_X; i>-SPQR::FIELD_DIMENSION_X; i-= d) //TODO parametrize
    {
        for(int j=SPQR::FIELD_DIMENSION_Y; j>-SPQR::FIELD_DIMENSION_Y; j-= d)
        {
            potential_field.push_back( NodePF(Vector2f( i-d/2,j-d/2 ), Vector2f()) );
            if(i==SPQR::FIELD_DIMENSION_X - 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(i== -SPQR::FIELD_DIMENSION_X + 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(j== SPQR::FIELD_DIMENSION_Y - 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(j== -SPQR::FIELD_DIMENSION_Y + 300) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
            if(i>=SPQR::FIELD_DIMENSION_X-(SPQR::FIELD_DIMENSION_X/50) && j<=800 && j >= -800) potential_field.push_back( NodePF(Vector2f( i,j), Vector2f()) );
        }
    }

    //Attractive potential field toward the center of the soccer field + Repulsive potential field written as an opposite-attractive component
    vector<Vector2f > attractive_field(potential_field.size());

    Pose2f goalPose;
    if(theContextCoordination.robotRole == ContextCoordination::defender)
        goalPose = getDefenderPlayingPosition(spqrDWKcombiner);
    else if(theContextCoordination.robotRole == ContextCoordination::supporter)
        goalPose = getSupporterPlayingPosition(spqrDWKcombiner);
    else if(theContextCoordination.robotRole == ContextCoordination::jolly)
        goalPose = getJollyPlayingPosition(spqrDWKcombiner);
    else return;

    Vector2f goal_pos = goalPose.translation;


    for(unsigned int i=0; i<attractive_field.size(); ++i)
    {
        Vector2f tmp_err = goal_pos - potential_field.at(i).pos;

        if( tmp_err.norm() <= RO)
            attractive_field.at(i) = Vector2f(Kap * tmp_err.x(), Kap * tmp_err.y());
        else
            attractive_field.at(i) = Vector2f((Kbp/tmp_err.norm()) * tmp_err.x(), (Kbp/tmp_err.norm()) * tmp_err.y());


        //        if(theRobotInfo.number == 1)
        //        {
        //            if( i % 9 != 0 ) cerr << "[" << tmp_err.x << ":" << tmp_err.y << "]" << " , ";
        //            else cerr <<  "[" << tmp_err.x << ":" << tmp_err.y << "]" << "\n";
        //        }

    }


    //Repulsive potential field from other robots and/or regions of the field
    vector<Vector2f > repulsive_field(potential_field.size());
    for(unsigned int i=0; i<repulsive_field.size(); ++i)
        repulsive_field.at(i) = Vector2f(0,0);

    for(unsigned int i=0; i<repulsive_field.size(); ++i)
    {
        for(unsigned int r=0; r<spqrDWKcombiner.robots_poses.size(); ++r)
        {

            if(r == (uint) theRobotInfo.number-1) continue;
            if (spqrDWKcombiner.robots_poses.at(r).translation.x() == 0 || spqrDWKcombiner.robots_poses.at(r).translation.y() == 0) continue;

            Vector2f tmp_err = spqrDWKcombiner.robots_poses.at(r).translation - potential_field.at(i).pos;
            if( (tmp_err).norm() < ETA)
            {

                // Compute tmp_eta and tmp_rep through getNearestPosInRF
                //                float tmp_eta = (potential_field.at(
                //                         getNearestPosInRF(spqrDWKcombiner.robots_poses.at(r).translation, potential_field.at(i)->pos, &potential_field))->pos - potential_field.at(i)->pos).absFloat();

                //                Vector2<> tmp_rep = potential_field.at(getNearestPosInRF(spqrDWKcombiner.robots_poses.at(r).translation, potential_field.at(i)->pos, &potential_field))->pos
                //                        - potential_field.at(i)->pos;

                float tmp_eta = (spqrDWKcombiner.robots_poses.at(r).translation-my_pos).norm();

                Vector2f tmp_rep = spqrDWKcombiner.robots_poses.at(r).translation-my_pos;


                repulsive_field.at(i)  += Vector2f( -tmp_rep * (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (1/tmp_err.norm()));
                /// original else bbody
                /// repulsive_field.at(i)  += (Kr/GAMMA) * pow(1000*(1/tmp_eta)-(1/ETA),GAMMA-1) * (-tmp_rep*(1/tmp_err.translation.absFloat()));

            }
            else
            {
                repulsive_field.at(i) += Vector2f(0.f, 0.f);
            }
        }
    }


    Vector2f vel_avg(0,0);
    unsigned int vel_size=0;

    for(unsigned int p=0; p<potential_field.size(); ++p)
    {
        potential_field.at(p).potential = (attractive_field.at(p)  + repulsive_field.at(p));

        if( (potential_field.at(p).pos - my_pos).norm() < d )
        {
            vel_avg += potential_field.at(p).potential;
            ++vel_size;
        }

        //        delete potential_field[p];
        //        potential_field.erase(potential_field.begin()+p);
    }

    vel_avg.x() = vel_size == 0 ? 0.0 : vel_avg.x()/vel_size;
    vel_avg.y() = vel_size == 0 ? 0.0 : vel_avg.y()/vel_size;


    //if(theRobotInfo.number == 1) cerr<<"\t\t\tORG X: "<< vel_avg.x << ", ORG Y: "<< vel_avg.y <<endl;
    //    vel_avg.x = vel_avg.x/vel_avg.absFloat();
    //    vel_avg.y = vel_avg.y/vel_avg.absFloat();

    float angle = -theRobotPoseSpqrFiltered.theta;
    //    if(theRobotPoseSpqrFiltered.theta > 0.f) angle = theRobotPoseSpqrFiltered.theta;
    //    else angle = 6.28f + theRobotPoseSpqrFiltered.theta;

    // rotate
    Vector2f vel = Vector2f(cos(angle)*vel_avg.x() - sin(angle)*vel_avg.y(),
                            sin(angle)*vel_avg.x() + cos(angle)*vel_avg.y());

    spqrDWKcombiner.vel_avg = vel;

    potential_field.clear();
    attractive_field.clear();
    repulsive_field.clear();
}

void SpqrDWKcombinator::update(SpqrDWKcombiner& spqrDWKcombiner)
{
#ifdef QUALITY_CONSENSUS
    findLowChannelQualities();

    std::vector < int > out_temp;
    for(int i = 0; i < 5; i++)
    {
        if(out_of_coordination[i]) {
#ifdef DEBUG_WHO_IS_OUT
            if( (PTracking::Timestamp() - stampForWho).getMs() > 3000 )
            {
                SPQR_SPECIAL("Number " << ( i + 1 ) << " is out! And master is: " << master);
            }
#endif
            out_temp.push_back(i);
        }
    }
#ifdef DEBUG_WHO_IS_OUT
    if( (PTracking::Timestamp() - stampForWho).getMs() > 3000 )
    {
        stampForWho.setToNow();
    }
#endif
    spqrDWKcombiner.robots_out_of_coordination = out_temp;
#endif

    if(theGameInfo.state == STATE_READY || theGameInfo.state == STATE_SET)
    {
        resetGraph();
    }
    else
    {
        //f ball status
        updateBallStatus(spqrDWKcombiner);
        // update own robots' poses
        updateRobotPoses(spqrDWKcombiner);

#ifdef DWK_PF
        // potential fields
        computePF(spqrDWKcombiner);
#endif
        // update opponent robots' poses
        updateOpponentsBelief(spqrDWKcombiner);

        //        SPQR_INFO("prev: "<<prev_context);
        //        SPQR_INFO("curr: "<<context);

        //update context
        SpqrDWKcombiner::Context tmp_context;
        tmp_context = contextProvider();
        // SPQR_INFO("temp: "<<tmp_context);
        if(tmp_context != context) prev_context = context;
        context = tmp_context;
        spqrDWKcombiner.current_context = context;

        if( context == SpqrDWKcombiner::playing )                                     /** -------------------PLAYING------------------ */
        {
            resetGraph();
            tmpGlobPrediction = rel2Glob(endRollingEstimatePosition().x(),endRollingEstimatePosition().y());
            spqrDWKcombiner.predicted_ball_global = rel2Glob(endRollingEstimatePosition().x(),endRollingEstimatePosition().y());
            spqrDWKcombiner.predicted_ball_relative = endRollingEstimatePosition();
        }
        else if( context == SpqrDWKcombiner::throw_in && theGameInfo.dropInTime < 2 ) /** -------------------THROW-IN----------------- */
        {
            throwInNodesUpdate();
            computeClusters(node_init_weight);
            tmpGlobPrediction = unexplored_centroids.at(0);
            // compute explored centroids
            computeClusters(node_init_weight, true);

            //update the three unexplored area centroids
            for(unsigned int i=0;i<spqrDWKcombiner.unexplored_clusters_centroids.size();++i)
                spqrDWKcombiner.unexplored_clusters_centroids.at(i) = unexplored_centroids.at(i);
        }
        else if( context == SpqrDWKcombiner::search_for_ball )                        /** -------------------SEARCH------------------- */
        {
            //if previous context was 'playing' then
            // check case: when it goes in this state and after a reasonable amount of time  what should do the graph
            ///____________________________ TODO be tested
            if(prev_context == SpqrDWKcombiner::playing &&
                    (PTracking::Timestamp() - timeSinceBallWasSeen).getMs() < time_when_last_seen*3)
                updateNodeScoreWhenLastTimeBallSeen();
            else
            {
                nodeViewer();
                //maybe to solve the occlusion of other robots..
                //the best way to do this is to reason on the position of the robot and decide wheter
                //the function nodeViewer needs to be executed

                //merge teammates data
                /// TODO tune the merge frequency, sometimes the msg do not arrive constantly
                mergeTeammatesLM();
                return;
            }
            // the agent influences the score of the viewed cells
            nodeViewer();
            restoreNodeScore();

            // compute unexplored centroids
            computeClusters(node_init_weight*0.8);
            tmpGlobPrediction = unexplored_centroids.at(0);
            // compute explored centroids
            computeClusters(node_init_weight, true);

            for(unsigned int i=0;i<spqrDWKcombiner.explored_clusters_centroids.size();++i)
            {
                //send the three explored area centroids
                spqrDWKcombiner.explored_clusters_centroids.at(i) = explored_centroids.at(i);
                //update the three unexplored area centroids
                spqrDWKcombiner.unexplored_clusters_centroids.at(i) = unexplored_centroids.at(i);
            }
        }

#ifdef DEBUG_DWK
        if(theGameInfo.state == STATE_PLAYING && theRobotInfo.number == 1)
        {
            if ((PTracking::Timestamp() - stamp).getMs() > pkg_send_rate)
            {
                for(unsigned int i=0;i<explored_centroids.size();++i)
                {
                    SPQR_ERR("explored ["<<i<<"]: "<<explored_centroids.at(i).x<<", "<<explored_centroids.at(i).y);
                }
                for(unsigned int i=0;i<unexplored_centroids.size();++i)
                {
                    SPQR_SUCC("unexplored ["<<i<<"]: "<<unexplored_centroids.at(i).x<<", "<<unexplored_centroids.at(i).y);
                }

                for(unsigned int i=0; i<5; ++i)
                {
                    if( (int) i+1 == theRobotInfo.number ) continue;
                    for(unsigned int j=0; j<theTeammateData.spqrDWKcombination[i+1].explored_clusters_centroids.size(); ++j)
                    {
                        SPQR_INFO("received centroids from robot "<<i+1<<": "<<
                                  theTeammateData.spqrDWKcombination[i+1].explored_clusters_centroids.at(j).x<<", "
                                                                                                            <<theTeammateData.spqrDWKcombination[i+1].explored_clusters_centroids.at(j).y);
                    }
                }
            }
        }
#endif

#ifdef GRAPH_VIEWER
        if(theGameInfo.state == STATE_PLAYING && theRobotInfo.number == 1)
        {
            if ((PTracking::Timestamp() - stamp).getMs() > pkg_send_rate)
            {
                std::cout<<"Prev context: "; CONTEXT(prev_context);
                std::cout<<"Context: "; CONTEXT(context);
                SPQR_INFO("ball position: "<<tmpGlobEstimation.x<<", "<<tmpGlobEstimation.y);
                SPQR_INFO("ball prediction: "<<tmpGlobPrediction.x<<", "<<tmpGlobPrediction.y);

                std::cout<<std::endl;
                for(unsigned int n=0; n<occupancyGraph.size(); ++n)
                {
                    if(occupancyGraph.at(n)->score >= 0.8f) std::cout<<"\033[0;32;1m"<<"["<<occupancyGraph.at(n)->score<<"]--- \033[0m";
                    else if(occupancyGraph.at(n)->score <= 0.4f) std::cout<<"\033[22;31;1m"<<"["<<occupancyGraph.at(n)->score<<"]--- \033[0m";
                    else std::cout<<"\033[22;34;1m"<<"["<<occupancyGraph.at(n)->score<<"]--- \033[0m";

                    if( n%(SPQR::FIELD_DIMENSION_Y/500) == SPQR::FIELD_DIMENSION_Y/500 -1) std::cout<<std::endl;
                }
                std::cout<<std::endl;
                stamp.setToNow();
            }
        }
#endif
    }
}

/*
*
* EMANUELE
*
*/

void SpqrDWKcombinator::configureAgents(const string& configDirectory)
{
#ifdef QUALITY_CONSENSUS
    vector<int> agentVector;
    ConfigFile fCfg;
    stringstream s;
    string agents, key, section;
    bool isPresent;

    if (!fCfg.read(configDirectory + string("PTracking/agentsForDWK.cfg")))
    {
        SPQR_ERR("Error reading file '" << configDirectory + "PTracking/agentsForDWK.cfg'. Exiting...");
        exit(-1);
    }

    try
    {
        section = "parameters";

        /// It could be set by using the constructor.
        if (agentId == -1)
        {
            key = "agentId";
            agentId = fCfg.value(section,key); // 1
        }

        key = "agents";
        agents = string(fCfg.value(section,key)); // 1,2,3,4,5

        s << agents;

        while (s.good())
        {
            string temp;

            if (s.eof()) break;

            getline(s,temp,',');

            agentVector.push_back(atoi(temp.c_str()));
        }

        key = "agentBasePort";
        agentBasePort = fCfg.value(section,key);

        if (SystemCall::getMode() == SystemCall::simulatedRobot) section = "LocalAgent";
        else section = "Agent";

        const vector<string>& agentNames = Utils::getAgentsName(configDirectory + string("PTracking/agentsForDWK.cfg"),string("[") + section + string("]"));

        int counter = 1;

        isPresent = false;

        for (vector<string>::const_iterator it = agentNames.begin(); it != agentNames.end(); ++it, ++counter)
        {
            //if (find(agentVector.begin(),agentVector.end(),counter) == agentVector.end()) continue;

            key = (*it) + "Address";
            const string address = fCfg.value(section,key);

            key = (*it) + "Port";
            int p = fCfg.value(section,key);

            if (SystemCall::getMode() != SystemCall::simulatedRobot)
            {
                /// Checking if the agentId is present in the receivers' list.
                /// If so, the information between the local and global layer are exchanged by using the main memory.
                if (strcasecmp(SystemCall::getHostName(),it->c_str()) == 0)
                {
                    isPresent = true;
                    agentAddress = address;
                    agentPort = p;

                    continue;
                }
            }
            if(receivers.size() == 5) continue; //TODO - TOREMOVE

            receivers.push_back(make_pair(address,p));
        }

        if (SystemCall::getMode() != SystemCall::simulatedRobot)
        {
            if (!isPresent)
            {
                SPQR_ERR("The agent id " << agentId << " is not present in the list of the receivers... Please check the configuration! Exiting...");
                exit(-1);
            }
        }
    }
    catch (...)
    {
        SPQR_ERR("Not existing value '" << section << "/" << key << "'. Exiting...");

        exit(-1);
    }
#endif
}

void SpqrDWKcombinator::findLowChannelQualities()
{
#ifdef QUALITY_CONSENSUS
    UdpSocket senderSocket;
    int ret;

    while( theFrameInfo.getTimeSince( theTeammateData.timeStamps[ master ] ) > static_cast<int>( time_for_consider_dead ) || out_of_coordination[ master-1 ] )
    {
        master = ( (master + 1) % 6 ) == 0 ? 1 : (master + 1) % 6;
#ifdef DEBUG_QUALITY_CONSENSUS
        SPQR_WARN("[Out of coordination for master]:  New Master is " << master)
        #endif
    }

    for(int r = 0; r < n_robots; r++)
    {
        /// se non è stato proposto allora non puo' andare out of coordination..
        /// per di più, non potendo essere settato da nessuno a proposed, posso evitare il mutex prima dell'if
        /// DA APPURARE

        if( theEQualityNetworkEstimation.quality.at(r) <= parameter_for_consider_low_channel && ( !proposed[r] ) && ( !out_of_coordination[r] ) )
        {
            if(master != theRobotInfo.number) // non sono il master, quindi propongo
            {
                //DEVO MANDARE UNA PROPOSTA PER IL CONSENSUS.  --- time:mio numero (base uno):robot che propongo:porta
#ifdef DEBUG_QUALITY_CONSENSUS
                SPQR_WARN( "INVIO DI UNA PROPOSTA PER IL CONSENSUS. PROPONGO: " << ( r+1 ) );
#endif
                struct timeval tp;
                gettimeofday(&tp,NULL);
                unsigned long int time = tp.tv_sec * 1000 + tp.tv_usec / 1000;

                if(SystemCall::getMode() != SystemCall::simulatedRobot) // REAL
                {
                    ret = senderSocket.send("P_REQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(r) + ":" + to_string(agentPort) ,InetAddress(receivers.at(master-1).first, receivers.at(master-1).second));
                }
                else // SIMULATA
                {
                    ret = senderSocket.send("P_REQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(r) ,InetAddress(receivers.at(master-1).first, receivers.at(master-1).second));
                }
                if(ret < 0)
                {
                    SPQR_ERR("send for proposing - failed.");
                    exit(-1);
                }

            }
            mutex.lock();
            proposed[r] = true;
            response_received[r].insert(theRobotInfo.number);
            //replies[r].insert(theRobotInfo.number);
            countdown_for_bad[r].setToNow();
            mutex.unlock();
        }
        else if( theEQualityNetworkEstimation.quality.at(r) >= parameter_for_consider_good_channel && ( !proposed[r] ) ) // dobbiamo farlo riprendere
        {
            mutex.lock();
            if(out_of_coordination[r])
            {
                if(master != theRobotInfo.number)
                {
#ifdef DEBUG_QUALITY_CONSENSUS
                    SPQR_WARN( "INVIO DI UNA PROPOSTA PER LA RIPRESA DI: " << ( r+1 ) );
#endif
                    struct timeval tp;
                    gettimeofday(&tp,NULL);
                    unsigned long int time = tp.tv_sec * 1000 + tp.tv_usec / 1000;

                    if(SystemCall::getMode() != SystemCall::simulatedRobot) // REAL
                    {
                        ret = senderSocket.send("PRREQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(r) + ":" + to_string(agentPort) ,InetAddress(receivers.at(master-1).first, receivers.at(master-1).second));
                    }
                    else // SIMULATA
                    {
                        ret = senderSocket.send("PRREQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(r) ,InetAddress(receivers.at(master-1).first, receivers.at(master-1).second));
                    }
                    if(ret < 0)
                    {
                        SPQR_ERR("send for proposing - failed.");
                        exit(-1);
                    }
                }
                proposed[r] = true;
                response_received[r].insert(theRobotInfo.number); // TODO vedi sopra.
                //replies[r].insert(theRobotInfo.number); // TODO vedi sopra.
            }
            mutex.unlock();
        }
    }
#endif
}

void SpqrDWKcombinator::waiting()
{
#ifdef QUALITY_CONSENSUS

    UdpSocket receiverSocket;
    InetAddress sender;
    string dataReceived;
    int ret;
    int binding;

    do
    {
        usleep(100e3);
    }
    while (theRobotInfo.number == 0);

    if (SystemCall::getMode() == SystemCall::simulatedRobot) // SIMULATA
    {
        if(TEAM_BLUE == theOwnTeamInfo.teamColor)
        {
            binding = receiverSocket.bind(base_port + theRobotInfo.number);
        }
        else
            binding = receiverSocket.bind(base_port + 5 + theRobotInfo.number);
    }
    else binding = receiverSocket.bind(agentPort);

    if (binding < 0)
    {
        SPQR_ERR("Error during the binding operation. Data Fusion among robots is not possible...exiting!");
        exit(-1);
    }

    while (true)
    {
        ret = receiverSocket.recv(dataReceived,sender);

        if (ret < 0)
        {
            SPQR_ERR("Error in receiving message from: '" << sender.toString() << "'");
            continue;
        }

        string dispatch = dataReceived.substr(0,5).c_str();

        dataReceived = dataReceived.substr(dataReceived.find(":") + 1).c_str();


        if(strcmp(dispatch.c_str(),"P_REQ") == 0) // il master riceve la proposta (da clientId) del robot che deve andare fuori controllo (robot_proposed)
        {

            if( master != theRobotInfo.number ) { continue; }

            string timestamp = dataReceived.substr(0,dataReceived.find(":"));

            int clientId = 0, robot_proposed = 0, port = 0;
            if (SystemCall::getMode() == SystemCall::simulatedRobot) // SIMULATA
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number)
                {
                    continue;
                }
                port = base_port + clientId;
                robot_proposed = atoi(temp.substr(temp.find(":") + 1).c_str());
            }
            else // REALE
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number)
                    continue;
                temp = temp.substr(temp.find(":") + 1);
                robot_proposed = atoi(temp.substr(0,temp.find(":")).c_str());
                temp = temp.substr(temp.find(":") + 1);
                port = atoi(temp.c_str());
            }

#ifdef DEBUG_QUALITY_CONSENSUS
            SPQR_WARN("AGGIUNGO IN REPLIES LA PROPOSTA SU " << (robot_proposed+1) << " DA PARTE DI " << clientId << "\n SIZE SU " << (robot_proposed+1) <<  " ATTUALE: " << response_received[robot_proposed].size()+1);
#endif
            mutex.lock();
            //replies[robot_proposed].insert(clientId); // TODO mi sa che sono inutili questi ora.
            response_received[robot_proposed].insert(clientId); // notifico che ho ricevuto una risposta da parte di which_robot
            countdown_for_bad[robot_proposed].setToNow();
            if( (int)response_received[robot_proposed].size() >= ( how_many_for_trigger ) )
            {
#ifdef DEBUG_QUALITY_CONSENSUS
                SPQR_WARN("ESCLUSIONE DI " << (robot_proposed+1));
#endif

                struct timeval tp;
                gettimeofday(&tp,NULL);
                unsigned long int time = tp.tv_sec * 1000 + tp.tv_usec / 1000;
                UdpSocket senderSocket;
                int temp = 1;
                for (vector<pair<string,int> >::const_iterator it = receivers.begin(); it != receivers.end(); it++)
                {
                    if(temp++ == theRobotInfo.number)
                    {
                        out_of_coordination[robot_proposed] |= true;
                        //decided[robot_proposed] |= true;
                        proposed[robot_proposed] &= false;
                        ping[robot_proposed].insert(theRobotInfo.number);
                        continue;
                    }

                    if(SystemCall::getMode() != SystemCall::simulatedRobot) // REAL
                    {
                        ret = senderSocket.send("O_REQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(robot_proposed) + ":" + to_string(agentPort) , InetAddress(it -> first, it -> second));
                    }
                    else // SIMULATA
                    {
                        ret = senderSocket.send("O_REQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(robot_proposed) , InetAddress(it -> first, it -> second));
                    }
                    if(ret < 0)
                    {
                        SPQR_ERR("send for proposing - failed.");
                        exit(-1);
                    }
                }

            }
            mutex.unlock();
        }

        else if(strcmp(dispatch.c_str(),"O_REQ") == 0) // inviato dal master nel caso in cui ritiene che il robot sia da scartare.
        {
            string timestamp = dataReceived.substr(0,dataReceived.find(":"));

            int clientId = 0, robot_to_ban = 0, port = 0;
            if (SystemCall::getMode() == SystemCall::simulatedRobot) // SIMULATA
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number)
                {
                    continue;
                }
                port = base_port + clientId;
                robot_to_ban = atoi(temp.substr(temp.find(":") + 1).c_str());
            }
            else // REALE
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number || clientId != master )
                    continue;
                temp = temp.substr(temp.find(":") + 1);
                robot_to_ban = atoi(temp.substr(0,temp.find(":")).c_str());
                temp = temp.substr(temp.find(":") + 1);
                port = atoi(temp.c_str());
            }

            mutex.lock();
            out_of_coordination[robot_to_ban] |= true;
            //decided[robot_to_ban] |= true;
            proposed[robot_to_ban] &= false;
            mutex.unlock();

#ifdef DEBUG_QUALITY_CONSENSUS
            SPQR_WARN("DICO CHE HO ESCLUSO " << (robot_to_ban+1));
#endif
            UdpSocket senderSocket;
            ret = senderSocket.send("O_RES:" + timestamp + ":" + to_string(theRobotInfo.number) + ":" + to_string(robot_to_ban),InetAddress(inet_ntoa(sender.getAddress().sin_addr), port));
            if(ret == -1)
            {
                SPQR_ERR("Error when sending message to client");
                exit(-1);
            }
        }
        else if(strcmp(dispatch.c_str(),"O_RES") == 0) // ricevuto dal master.. il pacchetto gli dice che il client ha escluso 'banned_robot' dalla coordinazione
        {
            //string timestamp = dataReceived.substr(0,dataReceived.find(":"));

            if( master != theRobotInfo.number ) { continue; }

            string temp = dataReceived.substr(dataReceived.find(":") + 1);
            int which_robot = atoi(temp.substr(0,1).c_str());
            temp = temp.substr(temp.find(":") + 1);
            int banned_robot = atoi(temp.substr(0).c_str());

#ifdef DEBUG_QUALITY_CONSENSUS
            SPQR_WARN("CONFERMA DI RIMOZIONE DA " << which_robot);
#endif
            mutex.lock();
            ping[banned_robot].insert(which_robot);
            mutex.unlock();
        }
        else if(strcmp(dispatch.c_str(),"PRREQ") == 0) // proposta di ripresa ricevuta
        {

            if( master != theRobotInfo.number ) { continue; }

            //string timestamp = dataReceived.substr(0,dataReceived.find(":"));

            int clientId = 0, robot_proposed = 0, port = 0;
            if (SystemCall::getMode() == SystemCall::simulatedRobot) // SIMULATA
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number)
                {
                    continue;
                }
                port = base_port + clientId;
                robot_proposed = atoi(temp.substr(temp.find(":") + 1).c_str());
            }
            else // REALE
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number || clientId != master)
                    continue;
                temp = temp.substr(temp.find(":") + 1);
                robot_proposed = atoi(temp.substr(0,temp.find(":")).c_str());
                temp = temp.substr(temp.find(":") + 1);
                port = atoi(temp.c_str());
            }

#ifdef DEBUG_QUALITY_CONSENSUS
            SPQR_WARN("(GOOD) AGGIUNGO IN REPLIES LA PROPOSTA SU " << (robot_proposed+1) << " DA PARTE DI " << clientId << "\n SIZE SU " << (robot_proposed+1) <<  " ATTUALE: " << response_received[robot_proposed].size()+1);
#endif
            mutex.lock();
            //replies_for_good[robot_proposed].insert(clientId); // TODO mi sa che sono inutili questi ora.
            response_received_for_good[robot_proposed].insert(clientId); // notifico che ho ricevuto una risposta da parte di which_robot
            countdown_for_good[robot_proposed].setToNow();
            if( (int)response_received_for_good[robot_proposed].size() >= ( how_many_for_trigger )) // per il buoni si potrebbe usare il numero di robot attualmente attivi dato dal teammateData
            {
#ifdef DEBUG_QUALITY_CONSENSUS
                SPQR_WARN("ROBOT " << (robot_proposed+1) << " DA ESCLUDERE");
#endif

                struct timeval tp;
                gettimeofday(&tp,NULL);
                unsigned long int time = tp.tv_sec * 1000 + tp.tv_usec / 1000;
                UdpSocket senderSocket;
                int temp = 1;
                for (vector<pair<string,int> >::const_iterator it = receivers.begin(); it != receivers.end(); it++)
                {
                    if(temp++ == theRobotInfo.number)
                    {
                        out_of_coordination[robot_proposed] &= false;
                        ping_for_good[robot_proposed].insert(theRobotInfo.number);
                        //decided[robot_proposed] |= true;
                        continue;
                    }

                    if(SystemCall::getMode() != SystemCall::simulatedRobot) // REAL
                    {
                        ret = senderSocket.send("ORREQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(robot_proposed) + ":" + to_string(agentPort) ,InetAddress(it -> first, it -> second));
                    }
                    else // SIMULATA
                    {
                        ret = senderSocket.send("ORREQ:" + to_string(time) + ":" + to_string(theRobotInfo.number) + ":" + to_string(robot_proposed) ,InetAddress(it -> first, it -> second));
                    }
                    if(ret < 0)
                    {
                        SPQR_ERR("send for proposing - failed.");
                        exit(-1);
                    }
                }
            }
            mutex.unlock();
        }
        else if(strcmp(dispatch.c_str(),"ORREQ") == 0) // DA RIPRENDERE
        {
            string timestamp = dataReceived.substr(0,dataReceived.find(":"));

            int clientId = 0, robot_to_ban = 0, port = 0;
            if (SystemCall::getMode() == SystemCall::simulatedRobot) // SIMULATA
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number)
                {
                    continue;
                }
                port = base_port + clientId;
                robot_to_ban = atoi(temp.substr(temp.find(":") + 1).c_str());
            }
            else // REALE
            {
                string temp = dataReceived.substr(dataReceived.find(":") + 1);
                clientId = atoi(temp.substr(0,1).c_str());
                if(clientId == theRobotInfo.number)
                    continue;
                temp = temp.substr(temp.find(":") + 1);
                robot_to_ban = atoi(temp.substr(0,temp.find(":")).c_str());
                temp = temp.substr(temp.find(":") + 1);
                port = atoi(temp.c_str());
            }

            mutex.lock();
            out_of_coordination[robot_to_ban] &= false;
            //decided[robot_to_ban] |= true;
            mutex.unlock();

#ifdef DEBUG_QUALITY_CONSENSUS
            SPQR_WARN("DICO CHE " << (robot_to_ban+1) << " SI E' RIPRESO");
#endif
            UdpSocket senderSocket;
            ret = senderSocket.send("ORRES:" + timestamp + ":" + to_string(theRobotInfo.number) + ":" + to_string(robot_to_ban),InetAddress(inet_ntoa(sender.getAddress().sin_addr), port));
            if(ret == -1)
            {
                SPQR_ERR("Error when sending message to client");
                exit(-1);
            }
        }
        else if(strcmp(dispatch.c_str(),"ORRES") == 0) // DA ESCLUDERE
        {
            if( master != theRobotInfo.number ) { continue; }

            //string timestamp = dataReceived.substr(0,dataReceived.find(":"));

            string temp = dataReceived.substr(dataReceived.find(":") + 1);
            int which_robot = atoi(temp.substr(0,1).c_str());
            temp = temp.substr(temp.find(":") + 1);
            int banned_robot = atoi(temp.substr(0).c_str());

#ifdef DEBUG_QUALITY_CONSENSUS
            SPQR_WARN("PING RICEVUTO DA " << which_robot);
#endif
            mutex.lock();
            ping_for_good[banned_robot].insert(which_robot);
            mutex.unlock();
        }
    }
}

void SpqrDWKcombinator::timerForResponse() //per il ping penso sia opportuno mettere un timer a parte.
{
    while(true)
    {
        usleep( timer_for_clean );
        for( int r=0; r < 5; r++ )
        {
            if((PTracking::Timestamp() - countdown_for_bad[r]).getMs() > timer_for_clean_bad )
            {
#ifdef DEBUG_COUNTDOWN_CLEAN
                SPQR_INFO("DEVO RIPULIRE LE INFORMAZIONI CATTIVE PER " << ( r+1 ) << "!");
#endif
                mutex.lock();
                proposed[r] &= false;
                response_received[r].clear();
                ping[r].clear();
                //replies[r].clear();
                countdown_for_bad[r].setToNow();
                mutex.unlock();
            }
            if((PTracking::Timestamp() - countdown_for_good[r]).getMs() > timer_for_clean_good )
            {
#ifdef DEBUG_COUNTDOWN_CLEAN
                SPQR_INFO("DEVO RIPULIRE LE INFORMAZIONI BUONE PER " << ( r+1 ) << "!");
#endif
                mutex.lock();
                proposed_for_good[r] &= false;
                response_received_for_good[r].clear();
                ping_for_good[r].clear();
                //replies_for_good[r].clear();
                countdown_for_good[r].setToNow();
                mutex.unlock();
            }
        }
    }

    return;
#endif
}

Pose2f SpqrDWKcombinator::getDefenderPlayingPosition(SpqrDWKcombiner& spqrDWKcombiner)
{
    return Pose2f( -0.55*SPQR::FIELD_DIMENSION_X, -0.22*SPQR::FIELD_DIMENSION_Y );
}

Pose2f SpqrDWKcombinator::getSupporterPlayingPosition(SpqrDWKcombiner& spqrDWKcombiner)
{
    return Pose2f( -0.45*SPQR::FIELD_DIMENSION_X, +0.22*SPQR::FIELD_DIMENSION_Y );
}

Pose2f SpqrDWKcombinator::getJollyPlayingPosition(SpqrDWKcombiner& spqrDWKcombiner)
{
    return Pose2f( 0.25*SPQR::FIELD_DIMENSION_X, -0.33*SPQR::FIELD_DIMENSION_Y );
}
bool SpqrDWKcombinator::between(float value, float min, float max)
{
    return value >= min && value <= max;
}
