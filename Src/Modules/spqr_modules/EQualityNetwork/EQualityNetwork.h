#pragma once

#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include <Representations/Infrastructure/TeamInfo.h>
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/EQualityNetworkEstimation.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include <mutex>
#include <queue>

#include <Utils/AgentPacket.h>
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/MessageQueue/MessageQueue.h"

#include <Core/Processors/Processor.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>

MODULE(EQualityNetwork,
{,
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotInfo),
 REQUIRES(TeammateData),
 PROVIDES(EQualityNetworkEstimation),
 LOADS_PARAMETERS(
 {,
  (int) seconds_for_evaluating_network,
  (int) parameter_for_good,
  (int) parameter_from_which_bad,
  (int) parameter_from_which_lost,
  (int) n_robots_par,
  (float) percentage_good,
  (float) percentage_half_good,
  (float) percentage_bad,
  (float) percentage_half_bad,
  (float) percentage_lost,
 }),
});

class EQualityNetwork: public EQualityNetworkBase
{

private:

    class ReceivedInfo
    {
    public:

        ReceivedInfo(unsigned long int rtt_temp, int latest_received_id_temp)
        {
            rtt = rtt_temp;
            latest_received_id = latest_received_id_temp;
        }
        ReceivedInfo(int rtt_temp, int latest_received_id_temp)
        {
            new_rtt = rtt_temp;
            latest_received_id = latest_received_id_temp;
        }

        ReceivedInfo(){}

        unsigned long int rtt;
        int new_rtt;
        int latest_received_id;
        int pending = 0;
        int from_robot = 0;
        unsigned long int updated = 0;

        unsigned long initial, how_many;

    };

    class ERoundTrip
    {
    public:

        ERoundTrip() : roundTrip(0), offset(0) {}

        int roundTrip;
        int offset;
    };


public:
    STREAMABLE(EQualityRequest,
    {
                   public:

                   unsigned receipt;

                   EQualityRequest() = default;

                   EQualityRequest(unsigned char sender) : sender(sender) { origin = SystemCall::getCurrentSystemTime(); },

                   (unsigned char) sender,
                   (unsigned) origin,
               });


    STREAMABLE(EQualityResponse,
    {
                   public:
                   unsigned receipt;

                   EQualityResponse() = default;

                   EQualityResponse(const EQualityRequest& request)
                   {
                       origin = request.origin;
                       reply_to = request.sender;
                   }

                   EQualityResponse(const EQualityRequest& request, unsigned char from)
                   {
                       origin = request.origin;
                       reply_to = request.sender;
                       sender = from;
                   }

                   ERoundTrip compute() const
                   {
                       ERoundTrip res;
                       res.roundTrip = int(receipt - origin) - int(responseSent - requestReceipt);
                       res.offset = ( int(requestReceipt - origin) + int(responseSent - receipt) ) / 2;
                       return res;
                   },

                   (unsigned char) reply_to,

                   (unsigned char) sender,

                   (unsigned) origin,

                   (unsigned) responseSent,

                   (unsigned) requestReceipt,
               });

#ifdef UNSTABLE_NETWORK

#endif

    int parameter_under_which_half_good,
        parameter_from_which_very_bad;

    int evaluation_id = 0;

    long int last_sent_for_evaluate;

#ifndef TARGET_SIM
    struct iwreq wreq;
    struct iw_statistics stats;
#endif

//    ReceivedInfo rtt[5];
    std::mutex mutex;

    void configureParameters(const string& configDirectory);

#ifdef TARGET_SIM
    static std::vector<std::vector<int>> offset;
#else
    static int offset[5];
#endif
private:

#ifdef TARGET_SIM
    static std::vector<vector<EQualityResponse>> responsesToSend;
    static std::vector<std::mutex> requestsMutex;
    static std::vector<std::mutex> mutexRTT;
    static ReceivedInfo rtt_static[5][5];
#else
    static vector<EQualityResponse> responsesToSend;
    static std::mutex requestsMutex;
    static std::mutex mutexRTT;
    static ReceivedInfo rtt_static[5];
#endif


public:
    EQualityNetwork();
    void update(EQualityNetworkEstimation& eQualityNetworkEstimation);
    static void handleRequestsAndResponses( int localId, OutMessage& out )
    {
        //I Have Requests at which reply?
#ifdef TARGET_SIM
        requestsMutex.at(localId-1).lock();
        for (vector<EQualityResponse>::iterator it = responsesToSend.at(localId-1).begin(); it != responsesToSend.at(localId-1).end(); it++)
        {
            it->responseSent = SystemCall::getCurrentSystemTime();
            out.bin << *it;
            out.finishMessage(idEQualityNTPResponse);
        }
        responsesToSend.at(localId-1).clear();
        requestsMutex.at(localId-1).unlock();
#else
        requestsMutex.lock();
        for (vector<EQualityResponse>::iterator it = responsesToSend.begin(); it != responsesToSend.end(); it++)
        {
            it->responseSent = SystemCall::getCurrentSystemTime();
            out.bin << *it;
            out.finishMessage(idEQualityNTPResponse);
        }
        responsesToSend.clear();
        requestsMutex.unlock();
#endif
        EQualityRequest request = EQualityRequest( ((unsigned char)localId) );
        out.bin << request;
        out.finishMessage(idEQualityNTPRequest);
    }
    static void replyToMessage( EQualityRequest request, unsigned reqReceipt, unsigned char localId )
    {
        EQualityResponse res(request, localId);
        res.requestReceipt = reqReceipt;
#ifdef TARGET_SIM
        requestsMutex.at((int)localId-1).lock();
        responsesToSend.at((int)localId-1).push_back(res);
        requestsMutex.at((int)localId-1).unlock();
#else
        requestsMutex.lock();
        responsesToSend.push_back(res);
        requestsMutex.unlock();
#endif
    }

    static void computeQualities(int myId, int from_which, int rtt, int offset_value)
    {
        struct timeval tp;
        gettimeofday(&tp,NULL);
        unsigned long int time = tp.tv_sec * 1000 + tp.tv_usec / 1000;
        ReceivedInfo ri = ReceivedInfo(rtt,0);
        ri.updated = time;

#ifdef TARGET_SIM
        if(offset_value >= 0)
            if( offset.at(myId-1).at(from_which-1) > offset_value ) offset.at(myId-1).at(from_which-1) = offset_value;
#else
        if(offset_value >= 0)
            if( offset[from_which-1] > offset_value ) offset[from_which-1] = offset_value;
#endif

#ifdef TARGET_SIM
        mutexRTT[myId-1].lock();
        rtt_static[myId-1][from_which-1] = ri;
        mutexRTT[myId-1].unlock();
#else
        mutexRTT.lock();
        rtt_static[((int)from_which)-1] = ri;
        mutexRTT.unlock();
#endif
    }
};
