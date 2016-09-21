/**
* @file Modules/Infrastructure/TeammateDataProvider.h
* This file declares a temporary wrapper from the old to the new TeammateData structure.
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Module/Module.h"
#include "Tools/Network/NTP.h"

#include "Representations/spqr_representations/RobotPoseSpqrFiltered.h"
#include "Representations/spqr_representations/SpqrDWKcombiner.h"
#include "Representations/spqr_representations/GlobalBallEstimation.h"
#include "Representations/spqr_representations/ContextCoordination.h"

MODULE(TeammateDataProvider,
{,
 REQUIRES(FrameInfo),
 REQUIRES(MotionInfo),
 REQUIRES(OwnTeamInfo),
 REQUIRES(RobotInfo),
 REQUIRES(RobotPose),
 USES(RobotPoseSpqrFiltered),
 USES(GlobalBallEstimation),
 USES(MotionRequest),
 USES(SpqrDWKcombiner),
 USES(ContextCoordination),
 PROVIDES(TeammateData),
 DEFINES_PARAMETERS(
 {,
  (int)(200) sendInterval, /** <  Time in ms between two messages that are sent to the teammates */
  (int)(4000) networkTimeout, /**< Time in ms after which teammates are considered as unconnected */
 }),
       });

/**
* @class TeammateDataProvider
* A module that provides information about the teammates
*/
class TeammateDataProvider : public TeammateDataProviderBase, public MessageHandler
{
private:
    static PROCESS_LOCAL TeammateDataProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */
    NTP ntp;                        /**< The Network Time Protocol. */
    unsigned lastSentTimestamp;     /**< The time when the last package to teammates was sent. */
    unsigned lastReceivedTimestamp; /**< The time when the incoming messages currently processed were received. */
    Teammate* currentTeammate;      /**< The teammate with which currently incoming messages are associated */
    TeammateData* teammateDataPtr;  /**< A pointer to have access to the teammate data struct from all methods */

    //EMANUELE
    int pos_infos_to_add;

    /** The main function, called every cycle
  * @param teammateData The data struct to be filled
  */
    void update(TeammateData& teammateData);

    /** Set the "currentTeammate" pointer to the representation of the robot that has the "robotNumber".
   *  If the robot does not exist yet, the object will be created.
   *  If robotNumber == theRobotInfo.number, currentTeammate will be set to 0.
   *  @param robotNumber The number of the robot to which the pointer should be set
   */
    void setCurrentTeammate(int robotNumber);

    /**
   * The method is called for every incoming team message by handleMessages.
   * @param message An interface to read the message from the queue.
   * @return true, if the message was handled
   */
    bool handleMessage(InMessage& message);

public:
    /** Default constructor */
    TeammateDataProvider();

    /** Destructor */
    ~TeammateDataProvider();

    /**
   * The method is called to handle all incoming team messages.
   * @param teamReceiver The message queue containing all team messages received.
   */
    static void handleMessages(MessageQueue& teamReceiver);
};
