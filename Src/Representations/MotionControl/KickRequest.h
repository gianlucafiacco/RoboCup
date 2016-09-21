/**
 * @file Representations/MotionControl/kickRequest.h
 * @author <a href="mailto:judy@informatik.uni-bremen.de">Judith Müller</a>
 */

#pragma once

#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/spqr_representations/OurDefinitions.h"

STREAMABLE(KickRequest,
{
  ENUM(KickMotionID,
  {,
    kickForward,
    kickSideward,
    backKick,
    sideKick,
    extSideKick,
    cornerKick,
    fastKickForward,
    veryFastKickForward,
#ifdef DEMOKICK
    realBallKickWithStand,
#endif
    none,
    newKick,
  });

  static KickMotionID getKickMotionFromName(const char* name),

  (KickMotionID)(none) kickMotionType,
  (bool)(false) mirror,
  (bool)(false) dynamical,
  (std::vector<DynPoint>) dynPoints,
});
