#include "EQualityNetwork.h"

#include <string.h>
#include <algorithm>
#include <iostream>
#include <UdpSocket.h>
#include <Utils/Timestamp.h>
#include <Manfield/configfile/configfile.h>
#include <unistd.h>
#include <stdlib.h> //atoi, rand

#include <sys/socket.h>
#include <sys/types.h>
#include <ctime>

#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" <<"["<<theRobotInfo.number<<"]"<<" [EQualityNetwork] " << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" <<"["<<theRobotInfo.number<<"]"<<" [EQualityNetwork] " << x << "\033[0m" << std::endl;
#define SPQR_SUCC(x) std::cerr << "\033[0;32;1m" <<"["<<theRobotInfo.number<<"]"<<" [EQualityNetwork] " << x << "\033[0m" << std::endl;

using namespace std;
using namespace PTracking;
using GMapping::ConfigFile;

MAKE_MODULE(EQualityNetwork, spqr_modules)

#ifdef TARGET_SIM
std::vector<std::mutex> EQualityNetwork::requestsMutex(5);
std::vector<std::mutex> EQualityNetwork::mutexRTT(5);
EQualityNetwork::ReceivedInfo EQualityNetwork::rtt_static[5][5];
std::vector<std::vector<EQualityNetwork::EQualityResponse>> EQualityNetwork::responsesToSend(5);
std::vector<std::vector<int>> EQualityNetwork::offset(5);
#else
std::mutex EQualityNetwork::requestsMutex;
std::mutex EQualityNetwork::mutexRTT;
EQualityNetwork::ReceivedInfo EQualityNetwork::rtt_static[5];
std::vector<EQualityNetwork::EQualityResponse> EQualityNetwork::responsesToSend(0);
int EQualityNetwork::offset[5];
#endif

EQualityNetwork::EQualityNetwork()
{
    //setting up agents and parameters
    //    configureAgents(configDirectory);
    parameter_under_which_half_good = (parameter_from_which_bad - parameter_for_good)/2 + parameter_for_good;
    parameter_from_which_very_bad = (parameter_from_which_lost - parameter_from_which_bad)/2 + parameter_from_which_bad;

    last_sent_for_evaluate = -1;

#ifndef TARGET_SIM
    memset(&wreq, 0, sizeof(struct iwreq));
    memset(&stats, 0, sizeof(struct iw_statistics));
    wreq.u.essid.length = IW_ESSID_MAX_SIZE+1;
    sprintf(wreq.ifr_name, IW_INTERFACE);
    wreq.u.data.length = sizeof(iw_statistics);
    wreq.u.data.pointer = &stats;
#endif

    //    for(int j = 0;  j < 5; j++)
    //    {
    //        rtt[j] = ReceivedInfo(0,0);
    //    }

#ifdef TARGET_SIM
    if(EQualityNetwork::offset.at(theRobotInfo.number-1).size() < 5) EQualityNetwork::offset.at(theRobotInfo.number-1).resize(5,1000);
#endif
    for(int j = 0;  j < 5; j++)
    {
#ifdef TARGET_SIM
        EQualityNetwork::rtt_static[theRobotInfo.number-1][j] = ReceivedInfo(0,0);
#else
        EQualityNetwork::rtt_static[j] = ReceivedInfo(0,0);
        if(j==theRobotInfo.number-1) EQualityNetwork::offset[j] = 0;
        else EQualityNetwork::offset[j] = 1000;
#endif
    }

}

void EQualityNetwork::update(EQualityNetworkEstimation& eQualityNetworkEstimation)
{

    //    SPQR_SUCC("\n1 " << theTeammateData.timeStamps[1] << "\n2 " << theTeammateData.timeStamps[2] << "\n3 " << theTeammateData.timeStamps[3]);
    //    SPQR_SUCC(SystemCall::getCurrentSystemTime() << "\n1 " << offset[0] << "\n2 " << offset[1] << "\n3 " << offset[2] << "\n4 " << offset[3] << "\n5 " << offset[4]);
    unsigned long int timeSinceLastSent = (time(0) * 1000) - last_sent_for_evaluate;

    if(timeSinceLastSent > ((unsigned long)seconds_for_evaluating_network * 1000)){

#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
        if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
        {
#endif
            SPQR_ERR("timeSinceLastSent maggiore di " << seconds_for_evaluating_network << " secondi.");
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
        }
#endif

#endif

        last_sent_for_evaluate = ( time(0) * 1000 );

        struct timeval tp2;

#ifdef DEBUG_NETWORK
        //SPQR_SUCC("---------------------------------------------------------------");
        std::stringstream to_log;
        to_log << "\n --------------------------------------------------------------- \n";
#endif
#ifdef WRITE_NETWORK_LOG
        std::stringstream to_write;
#endif
        int tot_quality = 0; // for estimate the average of all qualities
        float qualityAdjust = 0.0;
        int second_quality = eQualityNetworkEstimation.second_quality;


        /*----------------CRITICAL SECTION----------------*/
        /*
             * GOOD:   100 -> under parameter_for_good
             * GOOD/2: 75 -> above parameter_for_good and below parameter_under_which_half_good
             * BAD/2:    50 -> above parameter_under_which_half_good and below parameter_from_which_bad
             * BAD:  25 -> above parameter_from_which_bad and below parameter_from_which_very_bad
             * LOST:   0 -> above all
             */

        //        if(ioctl(senderSocket.getSocket(),SIOCGIWSTATS, &wreq) == -1)
        //        {
        //            fprintf(stderr, "Get ESSID ioctl failed \n");
        //        }
        //        else
        //        {
#if !defined(TARGET_SIM)
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
        if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
        {
#endif
            //            to_log << "\n QUAL: " << ((int)stats.qual.qual);
            //            to_log << "\n SIG_LEV: " << ((int)stats.qual.level);
            //            to_log << "\n NOISE_LEV: " << ((int)stats.qual.noise);
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
        }
#endif
#endif
#endif
        //        }

        gettimeofday(&tp2,NULL);
        unsigned long int time = tp2.tv_sec * 1000 + tp2.tv_usec / 1000;
        int dead_count = 0;

        for(int k = 0; k < /*n_robots*/5; k++)
        {

            if(k == theRobotInfo.number - 1)
            {
                continue;
            }
            if(theTeammateData.teammates.size())
            {
                if( /*theFrameInfo.getTimeSince( theTeammateData.timeStamps[ k+1 ] ) > static_cast<int>( 6500 ) )*/theTeammateData.teammates[k+1].status != Teammate::FULLY_ACTIVE  ) //Penalized or dead
                {
                    ++dead_count;
                    //#ifdef DEBUG_NETWORK
                    //                SPQR_INFO("DEAD ROBOT: " << k+1);
                    //#endif
                    continue;
                }
            }

            float qualityTemp = eQualityNetworkEstimation.quality.at(k);

            //            ReceivedInfo temp = rtt[k];
#ifdef TARGET_SIM
            EQualityNetwork::mutexRTT[theRobotInfo.number-1].lock();
            ReceivedInfo temp = EQualityNetwork::rtt_static[theRobotInfo.number-1][k];
            EQualityNetwork::mutexRTT[theRobotInfo.number-1].unlock();
#else
            EQualityNetwork::mutexRTT.lock();
            ReceivedInfo temp = EQualityNetwork::rtt_static[k];
            EQualityNetwork::mutexRTT.unlock();
#endif

            if(time - temp.updated > ((unsigned long)parameter_from_which_lost) && temp.updated != 0) // non mi arriva un pacchetto da oltre 'lost' secondi
            {
                qualityAdjust = -(qualityTemp * percentage_lost);
                second_quality += -(second_quality * percentage_lost);
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
                {
#endif
                    to_log << "\n LOST :" << (time-temp.updated) << ": From robot ";
                    //SPQR_SUCC((qualityTemp+qualityAdjust > 100 ? 100 : qualityTemp+qualityAdjust));
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                }
#endif
#endif
            }
            else if(temp.new_rtt < (parameter_for_good)) // GOOD
            {
                qualityAdjust = ( (100 - qualityTemp) * percentage_good); // increase based on the quality that remain to fill
                second_quality += ( (100 - second_quality) * percentage_good);
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
                {
#endif
                    to_log << "\n GOOD :: From robot ";
                    //SPQR_SUCC("GOOD :: From robot " << theRobotInfo.number << " to robot " << k + 1 << " = " << temp.rtt << " --- latest: " << temp.latest_received_id << " --- pending: " << temp.pending << " --- QUALITY: " << (qualityTemp+qualityAdjust > 100 ? 100 : qualityTemp+qualityAdjust));
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                }
#endif
#endif
            }
            else if(temp.new_rtt < (parameter_under_which_half_good)) // GOOD/2
            {
                qualityAdjust = ( (75 - qualityTemp) * percentage_half_good); // increase (decrease) based on the quality that remain to fill (based on the over quality)
                second_quality += ( (75 - second_quality) * percentage_half_good);
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
                {
#endif
                    to_log << "\n GOOD/2 :: From robot ";
                    //SPQR_SUCC("GOOD/2 :: From robot " << theRobotInfo.number << " to robot " << k + 1 << " = " << temp.rtt << " --- latest: " << temp.latest_received_id << " --- pending: " << temp.pending << " --- QUALITY: " << (qualityTemp+qualityAdjust > 100 ? 100 : qualityTemp+qualityAdjust));
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                }
#endif
#endif
            }
            else if(temp.new_rtt < (parameter_from_which_bad)) // BAD/2
            {
                qualityAdjust = ( (50 - qualityTemp) * percentage_half_bad);
                second_quality += ( (50 - second_quality) * percentage_half_bad);
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
                {
#endif
                    to_log << "\n BAD/2 :: From robot ";
                    //SPQR_SUCC("BAD/2 :: From robot " << theRobotInfo.number << " to robot " << k + 1 << " = " << temp.rtt << " --- latest: " << temp.latest_received_id << " --- pending: " << temp.pending << " --- QUALITY: " << (qualityTemp+qualityAdjust > 100 ? 100 : qualityTemp+qualityAdjust));
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                }
#endif
#endif
            }
            else if(temp.new_rtt < (parameter_from_which_very_bad)) // BAD
            {
                qualityAdjust = ( (25 - qualityTemp) * percentage_bad);
                second_quality += ( (25 - second_quality) * percentage_bad);
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
                {
#endif
                    to_log << "\n BAD :: From robot ";
                    //SPQR_SUCC("BAD :: From robot " << theRobotInfo.number << " to robot " << k + 1 << " = " << temp.rtt << " --- latest: " << temp.latest_received_id << " --- pending: " << temp.pending << " --- QUALITY: " << (qualityTemp+qualityAdjust > 100 ? 100 : qualityTemp+qualityAdjust));
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                }
#endif
#endif
            }
            else // LOST
            {
                qualityAdjust = -qualityTemp * percentage_lost;
                second_quality += -second_quality * percentage_lost;
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
                {
#endif
                    to_log << "\n LOST :: From robot ";
                    //SPQR_SUCC("LOST :: From robot " << theRobotInfo.number << " to robot " << k + 1 << " = " << temp.rtt << " --- latest: " << temp.latest_received_id << " --- pending: " << temp.pending << " --- QUALITY: " << (qualityTemp+qualityAdjust > 100 ? 100 : qualityTemp+qualityAdjust));
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
                }
#endif
#endif
            }
            qualityTemp += qualityAdjust;
            qualityTemp = qualityTemp < 100 ? qualityTemp : 100;
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
            if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
            {
#endif
                to_log << theRobotInfo.number << " to robot " << (k+1) << " = " << temp.new_rtt << " --- latest: " << temp.latest_received_id << " --- pending: " << temp.pending << " --- QUALITY: " << qualityTemp;
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
            }
#endif
#endif
#ifdef WRITE_NETWORK_LOG
            std::stringstream to_write_delay;
            to_write_delay << temp.new_rtt << "\n";
            ofstream myfileDelayChannel;
            std::stringstream ssDelayChannel;
            ssDelayChannel << "log_delays_from_" << theRobotInfo.number << "_to_" << k+1 << ".txt";
            myfileDelayChannel.open(ssDelayChannel.str(),std::ofstream::ate | std::ofstream::app);
            myfileDelayChannel << to_write_delay.str() << "\n";
            myfileDelayChannel.close();
#endif
            second_quality = second_quality < 100 ? second_quality : 100;
            tot_quality += qualityTemp;
            eQualityNetworkEstimation.quality.at(k) = qualityTemp;
        }

        /*----------------END OF CRITICAL SECTION----------------*/

        if(dead_count < 4)
        {
            //NOT ALL DEAD
            eQualityNetworkEstimation.network_quality = tot_quality / (/*n_robots*/5-dead_count-1);
            eQualityNetworkEstimation.second_quality = second_quality;
            eQualityNetworkEstimation.convex_quality = (0.7* tot_quality / (/*n_robots*/5-dead_count-1)) + (0.3 * second_quality);
#ifdef DEBUG_NETWORK
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
            if( theRobotInfo.number == WHICH_ONE_NET && theOwnTeamInfo.teamColor == (WHICH_ONE_COLOR_NET == 0 ? TEAM_BLUE: TEAM_RED))
            {
#endif
                to_log << "\n QUALITY OF ALL THE NETWORK: " << tot_quality/(/*n_robots*/5-dead_count-1) << "\n";
                to_log << " ALTERNATIVE QUALITY: " << second_quality << "\n";
                to_log << " CONVEX QUALITY: " << ((0.7* tot_quality / (/*n_robots*/5-dead_count-1)) + (0.3 * second_quality)) << "\n";
                to_log << " --------------------------------------------------------------- \n";
                SPQR_SUCC(to_log.str());
#ifdef ONLY_ONE_DEBUG_FOR_NETWORK
            }
#endif
#endif
#ifdef WRITE_NETWORK_LOG
            //WRITE THE CONVEX QUALITY
            to_write << ((0.7* tot_quality / (5-dead_count-1)) + (0.3 * second_quality)) << "\n";
#endif
        }

#ifdef WRITE_NETWORK_LOG
        //#ifdef DEBUG_NETWORK
        //        ofstream myfile;
        //        std::stringstream ss;
        //        ss << "log_network_" << theRobotInfo.number << ".txt";
        //        myfile.open(ss.str(),std::ofstream::ate | std::ofstream::app);
        //        myfile << to_log.str() << "\n";
        //        myfile.close();
        //#endif
        ofstream myfileConv;
        std::stringstream ss;
        ss << "log_convex_qual_" << theRobotInfo.number << ".txt";
        myfileConv.open(ss.str(),std::ofstream::ate | std::ofstream::app);
        myfileConv << to_write.str() << "\n";
        myfileConv.close();
#endif


    }
    PLOT("module:EQualityNetwork:ConvexQual", eQualityNetworkEstimation.convex_quality);
    PLOT("module:EQualityNetwork:channel1", eQualityNetworkEstimation.quality.at(0));
    PLOT("module:EQualityNetwork:channel2", eQualityNetworkEstimation.quality.at(1));
    PLOT("module:EQualityNetwork:channel3", eQualityNetworkEstimation.quality.at(2));
    PLOT("module:EQualityNetwork:channel4", eQualityNetworkEstimation.quality.at(3));
    PLOT("module:EQualityNetwork:channel5", eQualityNetworkEstimation.quality.at(4));

}
