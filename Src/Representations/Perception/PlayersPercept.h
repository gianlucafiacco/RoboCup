/**
 * @file PlayersPercept.h
 * @author Michel Bartsch
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

STREAMABLE(PlayersPercept,
{
               STREAMABLE(Player,
               {,
                (int) x1, // left border in the image
                (int) y1, // top border in the image
                (int) x2, // right border in the image
                (int) y2, // bottom border in the image
                (int) realCenterX, // the real horizontal center of the robot, not allways middle between x1 and x2
                (int) x1FeetOnly, // left border of only the robots feet in the image
                (int) x2FeetOnly, // right border of only the robots feet in the image
                (bool) lowerCamera, // true, if the robot was spotted in the lower camera
                (bool) detectedJersey, // true, if a jersey was found
                (bool) detectedFeet, // true, if the bottom of the obstacle was visible; never calculate position on field without detectedFeet!
                (bool) ownTeam, // true, if a detected jersey was red
                (bool) fallen, // true, if the obstacle seems to lay on the field
                (Vector2f) centerOnField, // player center in relative to the robot in field coordinates
                (Vector2f) leftOnField, // player's left edge in relative to the robot in field coordinates
                (Vector2f) rightOnField, // player's right edge in relative to the robot in field coordinates
               });

               /** Draws this percept. */
               void draw() const,

               (bool) obstacleOnCamera, // Tned on when a too big obstacle is on one of the two camera (we are too close to the obstacle to detect it with bot)
               (bool)(false) stuck,
               (unsigned int) (0) whenObstacleOnCamera, // Time stamp that contains when the obstacleOnCamera has been seen
               (std::vector<Player>) players,
           });
