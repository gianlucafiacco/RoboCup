/**
 * @file Representations/MotionControl/SpecialActionRequest.h
 * This file declares a struct to represent special action requests.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Enum.h"

/**
 * @struct SpecialActionRequest
 * The struct represents special action requests.
 */
STREAMABLE(SpecialActionRequest,
{
  /** ids for all special actions */
  ENUM(SpecialActionID,
  {,
    playDead,
    sitDown,
    stand,
    standHigh,
    afterGenuflect,
    coachSit,
    genuflect,
    goUp,
    keeperJumpLeft,
    keeperJumpRight,
    keeperJumpLeftPenalty,
    keeperJumpLeftSign,
    keeperJumpLeftSim,
    keeperSitAndJumpLeft,
    keeperStandJumpLeft,
    sitDownKeeper,
    standUpBackNao,
    standUpFrontNao,
    stopBall,
    sumoPosition,
    pointingCenterLeft,
    pointingCenterLeftDown,
    pointingCenterRight,
    pointingCenterRightDown,
    pointingCloseLeft,
    pointingCloseRight,
    pointingExtremeLeft,
    pointingExtremeRight,
    pointingLeft,
    pointingModerateLeft,
    pointingModerateRight,
    pointingRight,
  });

  /**
   * The function searches the id for a special action name.
   * @param name The name of the special action.
   * @return The corresponding id if found, or numOfSpecialActions if not found.
   */
  static SpecialActionID getSpecialActionFromName(const char* name),

  (SpecialActionID)(playDead) specialAction, /**< The special action selected. */
  (bool)(false) mirror, /**< Mirror left and right. */
});
