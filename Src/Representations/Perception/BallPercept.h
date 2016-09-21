/**
 * @file BallPercept.h
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(BallPercept,
{
               ENUM(Status,
               {,
                notSeen, //0
                seen, //1
                checkBallSpot, //2
                isRegionCircular, //3
                searchBallPoints, //4
                checkBallPoints, //5
                calculateBallInImage, //6
                checkBallInImage, //7
                calculateBallOnField, //8
                checkBallOnField, //9
                isGreenAround, //10
               });

               /** Draws the ball*/
               void draw() const,

               (Vector2f) positionInImage,         /**< The position of the ball in the current image */
               (float) radiusInImage,              /**< The radius of the ball in the current image */
               (Status)(notSeen) status,           /**< Indicates, if the ball was seen in the current image. */
               (Vector2f) relativePositionOnField, /**< Ball position relative to the robot. */
               (Vector2f)(0.f,0.f) previousRelativePositionOnField, /**< Ball position relative to the robot. */
               (float)(50) radiusOnField,          /**< The radius of the ball on the field in mm */
           });
