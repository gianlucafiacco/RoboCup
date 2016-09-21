#ifndef OURDEFINITIONS_H
#define OURDEFINITIONS_H

//   NETWORK ----------------------------------------------------------------------------

//#define TWO_TEAMS_NETWORK //if we want to play a real game in lab with two teams. This divides the team's communications.
//#define UNSTABLE_NETWORK //if we want to simulate a bad network in the simulated environment.
#define IW_INTERFACE "wlan0"
//#define SLOW_SEND //in case of bad network we slow the packets sent. Go to TeamDataSender.cpp for more details.
#define GOOD_CONVEX_QUALITY 60 //above this quality we don't slow anything
#define NUMBER_OF_FRAMES_TO_WAIT 10 //number of frames to wait in case of bad quality network ( One frame is equal to 20ms, normally we send packets at 5Hz )

#define IP_FOR_UNSTABLE "192.168.1.185"

//#define DEBUG_NETWORK
//#define ONLY_ONE_DEBUG_FOR_NETWORK
//#define WHICH_ONE_NET 3
//#define WHICH_ONE_COLOR_NET 0
//#define DEBUG_UNSTABLE_NETWORK
//#define DEBUG_QUEUE_THREAD
//#define DEBUG_CONFIGURE_PARAMETERS
//#define WRITE_NETWORK_LOG


//   COORDINATION ----------------------------------------------------------------------------

//#define DYNAMIC_HYSTERESIS
//#define WRITE_DYNAMIC_HYSTERESIS
//#define NO_HYSTERESIS
//#define HANDSHAKE_COUNTDOWN
//#define LEAVE_DOUBLE_ROLE

//#define DEEP_DEBUG_CONTEXT_COORD
//#define DEBUG_CONTEXT_COORD
//#define UM_VIEW
//#define DEBUG_AND_WRITE_ROLE_PERSISTENCE

//   SPQR_DWK ----------------------------------------------------------------------------

//#define QUALITY_CONSENSUS

//#define DEBUG_COUNTDOWN_CLEAN
//#define DEBUG_WHO_IS_OUT
//#define DEBUG_QUALITY_CONSENSUS

#define DWK_PF


//    CORNER_KICK_CHALLENGE ------------------------------------------------------------------

//#define CORNER_KICK   //if active changed the kicker and taker's localizations in order to perform the corner kick
//#define DEBUG_ANGLE_FOR_KICK

//    AUDIO_IN_BEHAVIORS ------------------------------------------------------------------
#define AUDIO_IN_BEHAVIORS

// AUTO CAMERA SETTINGS ------------------------------------------------------------------
//#define AUTO_BALANCE



//  NO_GOAL_POSTS_SAMPLING ____________________________________________________________
//#define NO_GOAL_POSTS_SAMPLING

//#define NO_COORDINATED_ROLES  // if actives changes the ball perceptor settings by using the robot numbers instead of the robot roles

//    PENALTY ROLES ----------------------------------------------------------------------

//#define PENALTY_STRIKER_GOALIE // if active changes the goalie and striker in order to playing penalty kick and changes also the localizations of the robots
                                 //NB: penalty kicker must be the number 3 and the goalie the 1

//   DEMO MACROS ------------------------------------------------------------------------------

//#define DEMOKICK




#endif // OURDEFINITIONS_H







