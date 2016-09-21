/** All option files that belong to the current behavior have to be included by this file. */

#include "Options/Soccer.h"


#include "Options/GameControl/HandleGameState.h"
#include "Options/GameControl/HandlePenaltyState.h"
#include "Options/GameControl/PlayingState.h"
#include "Options/GameControl/ReadyState.h"

#include "Options/HeadControl/HeadControl.h"
#include "Options/HeadControl/LookForward.h"
#include "Options/HeadControl/LookAtBall.h"
#include "Options/HeadControl/LookAtGlobalBall.h"
#include "Options/HeadControl/LookAtLandmark.h"
#include "Options/HeadControl/LookLeftAndRight.h"
#include "Options/HeadControl/LookLeftAndRightReduced.h"
#include "Options/HeadControl/LookLeftAndRightForStriker.h"
#include "Options/HeadControl/LookLeftAndRightUpAndDown.h"
#include "Options/HeadControl/LookAroundForLocalizer.h"

#include "Options/Output/Annotation.h"
#include "Options/Output/ArmMotionRequest/KeyFrameArms.h"

#include "Options/Output/HeadMotionRequest/SetHeadPanTilt.h"
#include "Options/Output/HeadMotionRequest/SetHeadTarget.h"
#include "Options/Output/HeadMotionRequest/SetHeadTargetOnGround.h"

#include "Options/Output/MotionRequest/GetUpEngine.h"
#include "Options/Output/MotionRequest/InWalkKick.h"
#include "Options/Output/MotionRequest/PathToTarget.h"
#include "Options/Output/MotionRequest/SpecialAction.h"
#include "Options/Output/MotionRequest/Stand.h"
#include "Options/Output/MotionRequest/WalkAtSpeed.h"
#include "Options/Output/MotionRequest/WalkAtSpeedPercentage.h"
#include "Options/Output/MotionRequest/WalkToTarget.h"

#include "Options/Output/MotionRequest/GetIntoReadyPosition.h"
#include "Options/Output/MotionRequest/GotoPose.h"
#include "Options/Output/MotionRequest/InDribbleKick.h"
#include "Options/Output/MotionRequest/Kicks.h"
#include "Options/Output/MotionRequest/KickOffPushBallOverCenter.h"
#include "Options/Output/MotionRequest/KickToTarget.h"
#include "Options/Output/MotionRequest/StopBall.h"
#include "Options/Output/MotionRequest/KickAway.h"

#include "Options/Output/MotionRequest/Goalie/GoalieKickAway.h"
#include "Options/Output/MotionRequest/Goalie/GoalieSaveLeft.h"
#include "Options/Output/MotionRequest/Goalie/GoalieSaveRight.h"
#include "Options/Output/MotionRequest/Goalie/GoalieSitAndSaveLeft.h"
#include "Options/Output/MotionRequest/Goalie/GoalieSitAndSaveRight.h"

#include "Options/Output/PlaySound.h"

#include "Options/Roles/Playing/Striker.h"
#include "Options/Roles/Playing/Goalie.h"
#include "Options/Roles/Playing/SupporterPF.h"
#include "Options/Roles/Playing/DefenderPF.h"
#include "Options/Roles/Playing/JollyPF.h"

#include "Options/Roles/PenaltyKicks/PenaltyStriker.h"
#include "Options/Roles/Search/Guard.h"
#include "Options/Roles/Search/Searcher.h"
#include "Options/Roles/Throwin/Throwin.h"

#include "Options/Roles/Specials/GoalieLearner.h"
#include "Options/Roles/Specials/Walker.h"
#include "Options/Roles/Specials/GoToPotPose.h"

#include "Options/Roles/CornerKick/Kicker.h"
#include "Options/Roles/CornerKick/Taker.h"

#include "Options/Skills/ArmContact.h"
#include "Options/Skills/GetUp.h"

#include "Options/Tools/ButtonPressedAndReleased.h"

#include "Options/Roles/NoWifiChallenge/Sender.h"
#include "Options/Roles/NoWifiChallenge/Receiver.h"

#include "Options/Roles/DropIn/DropIn.h"

#ifdef DEMOKICK
#include "Options/Roles/Specials/PassBall.h"
#endif
