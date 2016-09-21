/**
 * Modification of the 2015 red ball BallPerceptor
 * @file BallPerceptor.h
 * This file declares a module that provides the ball percept for the balck and white ball.
 * If possible, it uses the full YCbCr422 image.
 * @author Dario Albani
 * @author Vincenzo Suriani
 * @author Ali Youssef
 */

#include "BallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>
#include <iostream>

MAKE_MODULE(BallPerceptor, perception)

bool unstuck = false; // if true the ball perception are not active

// Alternate drawing macros that scale down if required
#define LINE2(n, x1, y1, x2, y2, w, s, c) LINE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define CROSS2(n, x, y, r, w, s, c) CROSS(n, scale(x), scale(y), scale(r), w, s, c)
#define CIRCLE2(n, x, y, r, w, s1, c1, s2, c2) CIRCLE(n, scale(x), scale(y), scale(r), w, s1, c1, s2, c2)
#define RECTANGLE3(n, x1, y1, x2, y2, w, s, c) RECTANGLE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define DOT2(n, x, y, c1, c2) DOT(n, scale(x), scale(y), c1, c2)

void BallPerceptor::update(BallPercept& ballPercept)
{
    scaleInput();

    // first of all we think, that no ball was found...
    ballPercept.status = BallPercept::notSeen;
    if(!theCameraMatrix.isValid)
    {
        return;
    }
    // calculate the image limits
    height = theCameraInfo.height - 3;
    right = theCameraInfo.width - 3;

    horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
    horizon = std::min(horizon, height);

    fromBallSpots(ballPercept);
    scaleOutput(ballPercept);

    // set ball radius
    ballPercept.radiusOnField = theFieldDimensions.ballRadius;
}

void BallPerceptorScaler::scaleInput()
{
    typedef BallPerceptorBase B;
    theCameraInfo = B::theCameraInfo;
    theBallSpots = B::theBallSpots;

    // do not copy "table"
    theImageCoordinateSystem.rotation = B::theImageCoordinateSystem.rotation;
    theImageCoordinateSystem.invRotation = B::theImageCoordinateSystem.invRotation;
    theImageCoordinateSystem.origin = B::theImageCoordinateSystem.origin;
    theImageCoordinateSystem.offset = B::theImageCoordinateSystem.offset;
    theImageCoordinateSystem.a = B::theImageCoordinateSystem.a;
    theImageCoordinateSystem.b = B::theImageCoordinateSystem.b;

    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
    {
        theCameraInfo.width *= 2;
        theCameraInfo.height *= 2;
        theCameraInfo.opticalCenter *= 2.f;
        theCameraInfo.focalLength *= 2.f;
        theCameraInfo.focalLengthInv /= 2.f;
        theCameraInfo.focalLenPow2 *= 4.f;

        theImageCoordinateSystem.origin *= 2.f;
        theImageCoordinateSystem.b *= 0.5f;

        for(BallSpot& b : theBallSpots.ballSpots)
        {
            b.position *= 2;
            ++b.position.x(); // original y channel was the second one
        }
    }
    theImageCoordinateSystem.setCameraInfo(theCameraInfo);
}

void BallPerceptorScaler::scaleOutput(BallPercept& ballPercept) const
{
    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
    {
        ballPercept.positionInImage *= 0.5f;
        ballPercept.radiusInImage *= 0.5f;
    }
}

static bool ballSpotComparator(const BallSpot& b1, const BallSpot& b2)
{
    if(b1.position.y() != b2.position.y())
        return b1.position.y() > b2.position.y();
    else
        return b1.position.x() < b2.position.x();
}

BallPerceptor::BallPerceptor() : right(0), horizon(0), height(0)
{
    sqrMaxBallDistance = Vector2f(theFieldDimensions.xPosOpponentFieldBorder - theFieldDimensions.xPosOwnFieldBorder,
                                  theFieldDimensions.yPosLeftFieldBorder - theFieldDimensions.yPosRightFieldBorder).squaredNorm()/5;
}

BallPercept::Status BallPerceptor::analyzeBallSpot(BallSpot& ballSpot, BallPercept& ballPercept)
{
    //    if(theColorTable[getPixel(ballSpot.position.y(), ballSpot.position.x())].is(ColorClasses::black)){
    //        CROSS2("representation:BallPercept:Image", ballSpot.position.x(), ballSpot.position.y(), 4, 4, Drawings::solidPen, ColorRGBA::white);
    //    }else{
    //        CROSS2("representation:BallPercept:Image", ballSpot.position.x(), ballSpot.position.y(), 4, 4, Drawings::solidPen, ColorRGBA::black);
    //    }
    return  !checkBallSpot(ballSpot) ? BallPercept::checkBallSpot : //Check if above the horizon or if too far away
                                       !isRegionCircular(ballSpot) ? BallPercept::isRegionCircular : //Check if the region is circular, also check the size of the black ball spot in input
                                                                     !searchBallPoints(ballSpot) ? BallPercept::searchBallPoints : //Analyze the pattern and the shape of the ball pixel by pixel
                                                                                                   !checkBallPoints() ? BallPercept::checkBallPoints : //Validate the ball points obtained previously
                                                                                                                        !calculateBallInImage(ballPercept) ? BallPercept::calculateBallInImage :
                                                                                                                                                             !checkBallInImage(ballPercept) ? BallPercept::checkBallInImage :
                                                                                                                                                                                              !calculateBallOnField(ballPercept) ? BallPercept::calculateBallOnField :
                                                                                                                                                                                                                                   !checkBallOnField(ballPercept) ? BallPercept::checkBallOnField :
                                                                                                                                                                                                                                                                    !isGreenAround() ? BallPercept::isGreenAround : BallPercept::seen;                                                                                                                                                                                                                                                                   !isGreenAround() ? BallPercept::isGreenAround : BallPercept::seen;
}

// Here we get the  approxRadius1 based on the distance of the ball and the BallRadius
// DO NOT FORGET TO CALIBRATE THE CAMERAS
bool BallPerceptor::checkBallSpot(const BallSpot& ballSpot)
{
    // Discard either if there is a big obstacle in front of the robot or we are in the upper part of the upper camera
    if(thePlayersPercept.obstacleOnCamera || (ballSpot.position.y() < 150 && theCameraInfo.camera == CameraInfo::upper))
    {
        return false;
    }

    //If the ballSpot is far then do not look for ball on obstacles
    if(isOnObstacle(ballSpot)) return false;

    // calculate an approximation of the radius based on bearing distance of the ball spot
    const Vector2i& spot = ballSpot.position;
    Vector2f correctedStart = theImageCoordinateSystem.toCorrected(spot);
    Vector3f cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedStart.x(), theCameraInfo.opticalCenter.y() - correctedStart.y());
    Vector3f unscaledField = theCameraMatrix.rotation * cameraToStart;
    if(unscaledField.z() >= 0.f)
    {
        return false; // above horizon
    }
    const float scaleFactor = (theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / unscaledField.z();
    cameraToStart *= scaleFactor;
    unscaledField *= scaleFactor;
    if(unscaledField.topRows(2).squaredNorm() > sqrMaxBallDistance)
    {
        return false; // too far away, computed with respect to the field.
    }
    cameraToStart.y() += cameraToStart.y() > 0 ? -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
    cameraToStart /= scaleFactor;
    approxRadius1 = std::abs(theCameraInfo.opticalCenter.x() - cameraToStart.y() - correctedStart.x());
    return true;
}

// Check if the black region is almost circular (pentagon)
bool BallPerceptor::isRegionCircularAux(Vector2i &position, Vector2i increment, bool isWhite){
    ASSERT(increment.x() == 0 || increment.x() == 1 || increment.x() == -1);
    ASSERT(increment.y() == 0 || increment.y() == 1 || increment.y() == -1);

    Vector2i pos = position + increment;
    Image::Pixel pospixel;

    // Out of bounds?
    if(pos.x() > theCameraInfo.width-3 ||
            pos.x() < 3 ||
            pos.y() > theCameraInfo.height ||
            pos.y() < 3){
        return false;
    }

    pospixel = getPixel(pos.y(), pos.x());
    if(isWhite){
        if(theColorTable[pospixel].is(ColorClasses::black) ||
                theColorTable[pospixel].is(ColorClasses::none) ||
                theColorTable[pospixel].is(ColorClasses::white))
        {
            // Update borders
            position += increment;
        }else if(theColorTable[getPixel(pos.y() + (increment.y()), pos.x() + (increment.x()))].is(ColorClasses::black) || // Tollerance on one burned pixel
                 theColorTable[getPixel(pos.y() + (increment.y()), pos.x() + (increment.x()))].is(ColorClasses::none) ||
                 theColorTable[getPixel(pos.y() + (increment.y()), pos.x() + (increment.x()))].is(ColorClasses::white))
        {
            // Check also the next in the line
            // Update borders
            position += increment*2;
        }else{
            // If still not black or none assume that we are out
            return true;
        }
    }
    else
    {
        if(theColorTable[pospixel].is(ColorClasses::black) ||
                theColorTable[pospixel].is(ColorClasses::none))
        {
            // Update borders
            position += increment;
        }else if(theColorTable[getPixel(pos.y() + (increment.y()), pos.x() + (increment.x()))].is(ColorClasses::black) || // Tollerance on one burned pixel
                 theColorTable[getPixel(pos.y() + (increment.y()), pos.x() + (increment.x()))].is(ColorClasses::none))
        {
            // Check also the next in the line
            // Update borders
            position += increment*2;
        }else{
            // If still not black or none assume that we are out
            return true;
        }
    }
    return false;
}

// Check if the black region is almost circular (pentagon)
bool BallPerceptor::isRegionCircular(BallSpot &ballSpot){
    // Do not perform this check if we are dealing with a white spot
    if(theColorTable[getPixel(ballSpot.position.y(), ballSpot.position.x())].is(ColorClasses::white)) return true;

    bool isOut[8] = {false};
    std::fill_n(isOut, 8, false);

    //----------0-------------
    //------7-------1--------
    //--6---------------2----
    //------5-------3--------
    //----------4-------------
    Vector2i borders[8];
    std::fill_n(borders, 8, Vector2i(ballSpot.position.x(), ballSpot.position.y()));

    if(theColorTable[getPixel(ballSpot.position.y(), ballSpot.position.x())].is(ColorClasses::white)){
        for(int displacement = 0; displacement < approxRadius1*2; displacement++){
            if(!isOut[0]) isOut[0] = isRegionCircularAux(borders[0], Vector2i(0, -1), true);
            if(!isOut[1]) isOut[1] = isRegionCircularAux(borders[1], Vector2i(1,-1), true);
            if(!isOut[2]) isOut[2] = isRegionCircularAux(borders[2], Vector2i(1,0), true);
            if(!isOut[3]) isOut[3] = isRegionCircularAux(borders[3], Vector2i(1,1), true);
            if(!isOut[4]) isOut[4] = isRegionCircularAux(borders[4], Vector2i(0,1), true);
            if(!isOut[5]) isOut[5] = isRegionCircularAux(borders[5], Vector2i(-1,1), true);
            if(!isOut[6]) isOut[6] = isRegionCircularAux(borders[6], Vector2i(-1,0), true);
            if(!isOut[7]) isOut[7] = isRegionCircularAux(borders[7], Vector2i(-1,-1), true);

            // Stop the cycle before if we are out of the spot
            if(isOut[0] && isOut[1] && isOut[2] && isOut[3] && isOut[4] && isOut[5] && isOut[6] && isOut[7]){
                break;
            }
        }
    }
    else
    {
        for(int displacement = 0; displacement < 0.5 * approxRadius1; displacement++){
            if(!isOut[0]) isOut[0] = isRegionCircularAux(borders[0], Vector2i(0, -1), false);
            if(!isOut[1]) isOut[1] = isRegionCircularAux(borders[1], Vector2i(1,-1), false);
            if(!isOut[2]) isOut[2] = isRegionCircularAux(borders[2], Vector2i(1,0), false);
            if(!isOut[3]) isOut[3] = isRegionCircularAux(borders[3], Vector2i(1,1), false);
            if(!isOut[4]) isOut[4] = isRegionCircularAux(borders[4], Vector2i(0,1), false);
            if(!isOut[5]) isOut[5] = isRegionCircularAux(borders[5], Vector2i(-1,1), false);
            if(!isOut[6]) isOut[6] = isRegionCircularAux(borders[6], Vector2i(-1,0), false);
            if(!isOut[7]) isOut[7] = isRegionCircularAux(borders[7], Vector2i(-1,-1), false);

            // Stop the cycle before if we are out of the spot
            if(isOut[0] && isOut[1] && isOut[2] && isOut[3] && isOut[4] && isOut[5] && isOut[6] && isOut[7]){
                break;
            }
        }
    }
    float distances[4] = {
        (float)(borders[4] - borders[0]).norm(),
        (float)(borders[2] - borders[6]).norm(),
        (float)(borders[7] - borders[3]).norm(),
        (float)(borders[5] - borders[1]).norm()
    };

    //    if(!(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)){
    //        // Update the ball spot center
    //        ballSpot.position.x() = (float)((borders[2].x() - borders[6].x()/2) + (borders[1].x() - borders[5].x()/2) +(borders[3].x() + borders[7].x()/2))/3;
    //        ballSpot.position.y() = (float)((borders[4].y() - borders[0].y()/2) + (borders[3].y() - borders[7].y()/2) +(borders[5].y() - borders[1].y()/2))/3;
    //    }

    // Check that all the distances are similar
    float minDistance = *std::min_element(distances, distances+4);
    float maxDistance = *std::max_element(distances, distances+4);

    // Return true if the min distance and the max distance are not too small or not too big
    return minDistance > maxDistance * 0.6 && maxDistance < theCameraInfo.width*0.35;

}

// Ignore ball spots on the obstacle, accept only on the lower part
// Check also for big obstacle on upper or lower to update the obstacleOnLower and obstacleOnUpper global variables
bool BallPerceptor::isOnObstacle(const BallSpot &ballSpot){
    /*
     * Panic Definition for RoboCup 2016. If we have too many false positive on other robots, use this to remove ballSpots that lies on robots
     */
    for(const auto playerPercept : thePlayersPercept.players)
    {
        if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
        {
            if (ballSpot.position.x() > playerPercept.x1*2 && ballSpot.position.x() < playerPercept.x2*2 &&
                    ballSpot.position.y() >  playerPercept.y1*2 && ballSpot.position.y() < (playerPercept.y2)*2)
            {
                return true;
            }
        }
        else
        {
            if (ballSpot.position.x() > playerPercept.x1 && ballSpot.position.x() < playerPercept.x2 &&
                    ballSpot.position.y() >  playerPercept.y1 && ballSpot.position.y() < (playerPercept.y2)){
                return true;
            }
        }
    }
    return false;
}

void BallPerceptor::fromBallSpots(BallPercept& ballPercept)
{

    if(thePlayersPercept.stuck) // If we have an obstacle in front of us for more than 1.5 sec deactive the ball perception
    {
        unstuck = true;
    }else{
        unstuck = false;
    }

    // The information about the obstacles come from the previous state
    // Check isOnObstacle function
    if(unstuck || (wasOnLower && theCameraInfo.camera == CameraInfo::upper))
    {
        ballPercept.status = BallPercept::notSeen;
        wasOnLower = false;
        return;
    }

    std::vector<BallSpot> ballSpots = theBallSpots.ballSpots;

    // Sort according to y position
    std::sort(ballSpots.begin(), ballSpots.end(), ballSpotComparator);

    for(BallSpot& ballSpot : ballSpots)
    {
        Vector2f prevPositionInImage = ballPercept.positionInImage;
        ballPercept.positionInImage = ballSpot.position.cast<float>();

        BallPercept::Status status = analyzeBallSpot(ballSpot, ballPercept);

        if(status == BallPercept::seen || status > ballPercept.status){
            ballPercept.status = status;
        }
        else
            ballPercept.positionInImage = prevPositionInImage;
        if(status == BallPercept::seen)
        {
            if(theCameraInfo.camera == CameraInfo::lower){
                wasOnLower = true;
            }
            return;
        }
    }
}

bool BallPerceptor::searchBallPoints(const BallSpot& ballSpot)
{
    Vector2i start = ballSpot.position;
    const float approxDiameter = approxRadius1 * clippingApproxRadiusScale + clippingApproxRadiusPixelBonus;
    int halfApproxRadius = int(approxRadius1 * 0.6f);

    // try to improve the start point
    int resolutionWidth = theCameraInfo.width;
    int resolutionHeight = theCameraInfo.height;
    //Start from the spot coordinates
    startPixel = getPixel(start.y(), start.x());
    //span diagonally starting from the spot original coordinates
    Vector2i preScanPoints[4] =
    {
        start + Vector2i(halfApproxRadius, halfApproxRadius),
        start + Vector2i(-halfApproxRadius, -halfApproxRadius),
        start + Vector2i(halfApproxRadius, -halfApproxRadius),
        start + Vector2i(-halfApproxRadius, halfApproxRadius)
    };
    bool preScanResults[4];  // check flags related to the previous span
    for(int i = 0; i < 4; ++i)
    {
        if(preScanPoints[i].x() < 0 || preScanPoints[i].x() >= resolutionWidth ||
                preScanPoints[i].y() < 0 || preScanPoints[i].y() >= resolutionHeight)
        {
            i -= i % 2;
            preScanResults[i++] = false;
            ASSERT(i < 4);
            preScanResults[i] = false;
        }
        else
        {
            //2016 changes
            // Check the spanned points, compare with the origin coordinates (BallSPot)
            const Image::Pixel pixel = getPixel(preScanPoints[i].y(), preScanPoints[i].x());
            if (theColorTable[pixel].is(ColorClasses::white) ||
                    (theColorTable[pixel].is(ColorClasses::black)) ||
                    (theColorTable[pixel].is(ColorClasses::none)))
                preScanResults[i]  = true;
            else{
                preScanResults[i]  = false;
            }
        }
    }

    // 2016 Code
    // move the start point to a middle point when the spanned points dont lie on the ball
    if(preScanResults[0] != preScanResults[1] && preScanResults[2] != preScanResults[3])
    {
        start = Vector2i::Zero();
        if(preScanResults[0])
        {
            start += preScanPoints[0];
        }
        else
        {
            start += preScanPoints[1];
        }
        if(preScanResults[2])
        {
            start += preScanPoints[2];
        }
        else
        {
            start += preScanPoints[3];
        }
        start /= 2;
    }
    else if(preScanResults[0] != preScanResults[1])
    {
        start = preScanResults[0] ? preScanPoints[0] : preScanPoints[1];
    }
    else if(preScanResults[2] != preScanResults[3])
    {
        start = preScanResults[2] ? preScanPoints[2] : preScanPoints[3];
    }

    whiteCounter = 0;
    blackCounter = 0;

    // vertical scan
    searchBallPointFar(start,  Vector2i(0, 1), approxDiameter, ballPoints[0]);
    searchBallPointFar(start,  Vector2i(0, -1), approxDiameter, ballPoints[4]);


    //Check if at borders
    if(ballPoints[0].atBorder && ballPoints[4].atBorder) return false; // too large, opposite points cannot be both at border
    else if(ballPoints[0].atBorder)
    {
        start.y() = ballPoints[4].point.y() + int(approxRadius1);
        if(start.y() > ballPoints[0].point.y() - 1)
            start.y() = ballPoints[0].point.y() - 1;
    }
    else if(ballPoints[4].atBorder)
    {
        start.y() = ballPoints[0].point.y() - int(approxRadius1);
        if(start.y() < ballPoints[4].point.y() + 1)
            start.y() = ballPoints[4].point.y() + 1;
    }
    else
    {
        start.y() = (ballPoints[0].point.y() + ballPoints[4].point.y()) / 2;
    }

    // horizontal scan

    searchBallPointFar(start, Vector2i(1, 0), approxDiameter, ballPoints[2]);
    searchBallPointFar(start, Vector2i(-1, 0), approxDiameter, ballPoints[6]);

    //Check if at borders
    if(ballPoints[2].atBorder && ballPoints[6].atBorder) return false; // too large, opposite points cannot be both at border
    else if(ballPoints[2].atBorder)
    {
        start.x() = ballPoints[6].point.x() + int(approxRadius1);
        if(start.x() > ballPoints[2].point.x() - 1)
            start.x() = ballPoints[2].point.x() - 1;
    }
    else if(ballPoints[6].atBorder)
    {
        start.x() = ballPoints[2].point.x() - int(approxRadius1);
        if(start.x() < ballPoints[6].point.x() + 1)
            start.x() = ballPoints[6].point.x() + 1;
    }
    else
    {
        start.x() = (ballPoints[2].point.x() + ballPoints[6].point.x()) / 2;
    }
    approxCenter2 = start;

    // Diagonal scans
    searchBallPointFar(start, Vector2i(1, 1), approxDiameter, ballPoints[1]);
    searchBallPointFar(start, Vector2i(-1, -1), approxDiameter, ballPoints[5]);
    if(ballPoints[1].atBorder && ballPoints[5].atBorder) return false; // too large, opposite points cannot be both at border

    searchBallPointFar(start, Vector2i(1, -1), approxDiameter, ballPoints[3]);
    searchBallPointFar(start, Vector2i(-1, 1), approxDiameter, ballPoints[7]);
    if(ballPoints[3].atBorder && ballPoints[7].atBorder) return false; // too large, opposite points cannot be both at border

    return ((float) whiteCounter/((float)(whiteCounter+blackCounter)) < 0.85)
            && ((float) whiteCounter/((float)(whiteCounter+blackCounter)) > 0.25);
}

//Check where the ball ends and if there is green when it ends
//i.e. if the ball is close to a line or a nao we ignore it
void BallPerceptor::searchBallPointFar(const Vector2i& start, const Vector2i& step, const int maxLength, BallPoint& ballPoint){
    ASSERT(step.x() == 0 || step.x() == 1 || step.x() == -1);
    ASSERT(step.y() == 0 || step.y() == 1 || step.y() == -1);

    Vector2i pos = start;
    Image::Pixel pospixel;
    ballPoint.step = step;
    ballPoint.atBorder = false;
    ballPoint.start = start;
    ballPoint.point = start;

    unsigned int tolerance = 0;

    for(int i = 0; i<maxLength*1.2; i++){
        //Increase the position by the step
        pos += step;

        // Exit if out of bounds
        // Check if it is at border
        if(pos.x() < 0 || pos.x() >= theCameraInfo.width || pos.y() < 0 || pos.y() >= theCameraInfo.height){
            ballPoint.atBorder = true;
            break;
        }

        pospixel = getPixel(pos.y(), pos.x());

        if(theColorTable[pospixel].is(ColorClasses::black) ||
                theColorTable[pospixel].is(ColorClasses::none))
        {
            tolerance = 0;
            // Update borders
            ballPoint.point = pos;
            blackCounter++;
        }else if(theColorTable[pospixel].is(ColorClasses::white))
        {
            tolerance = 0;
            // Update borders
            ballPoint.point = pos;
            whiteCounter++;
        }else{
            ballPoint.point = pos;
            tolerance++;
            if(tolerance > 2) break;
        }
    }
}

bool BallPerceptor::checkBallPoints()
{
    // find "valid" ball points
    validBallPoints = 0;
    static const int countOfBallPoints = sizeof(ballPoints) / sizeof(*ballPoints);
    for(int i = 0; i < countOfBallPoints; ++i)
    {
        BallPoint& ballPoint(ballPoints[i]);
        ballPoint.isValid = !ballPoint.atBorder && ballPoint.point != ballPoint.start;
        if(ballPoint.isValid)
        {
            ++validBallPoints;
        }
    }
    if(validBallPoints < 4) //TODO DARIO check this number, maybe is too high for far black-ball-points based ball
    {
        return false;
    }

    // find duplicated ball points (may occur in small balls)
    for(int i = 0; i < countOfBallPoints; ++i)
    {
        BallPoint& ballPoint(ballPoints[i]);
        if(ballPoint.isValid)
        {
            const BallPoint& nextBallPoint(ballPoints[(i + 1) % countOfBallPoints]);
            if(nextBallPoint.isValid && ballPoint.point == nextBallPoint.point)
            {
                ballPoint.isValid = false;
                --validBallPoints;
            }
        }
    }
    if(validBallPoints < 4)
        return false;

    // drop mismatching ball points
    while(validBallPoints > 4)
    {
        Vector2f preCenter;
        float preRadius;
        if(!getBallFromBallPoints(preCenter, preRadius))
        {
            return false;
        }

        float minDist = 0;
        BallPoint* minDistBallPoint = 0;
        for(int i = 0; i < countOfBallPoints; ++i)
        {
            BallPoint& ballPoint(ballPoints[i]);
            if(ballPoint.isValid)
            {
                float dist = (ballPoint.pointf - preCenter).squaredNorm();
                if(!minDistBallPoint || dist < minDist)
                {
                    minDist = dist;
                    minDistBallPoint = &ballPoint;
                }
            }
        }
        minDistBallPoint->isValid = false;
        --validBallPoints;

        if((preRadius - (sqrt(minDist) + 2.f)) / preRadius < 0.1f)
        {
            break;
        }
    }
    return true;
}

bool BallPerceptor::getBallFromBallPoints(Vector2f& center, float& radius) const
{
    float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;

    for(const BallPoint* ballPoint = ballPoints, * end = ballPoints + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
        if(ballPoint->isValid)
        {
            float x = static_cast<float>(ballPoint->point.x());
            float y = static_cast<float>(ballPoint->point.y());
            float xx = x * x;
            float yy = y * y;
            float z = xx + yy;
            Mx += x;
            My += y;
            Mxx += xx;
            Myy += yy;
            Mxy += x * y;
            Mz += z;
            Mxz += x * z;
            Myz += y * z;
        }

    // Construct and solve matrix
    // Result will be center and radius of ball in theImage.
    Eigen::Matrix3d M;
    M << Mxx, Mxy, Mx,
            Mxy, Myy, My,
            Mx,  My,  validBallPoints;

    Eigen::Matrix3d Minv;
    bool invertible;
    M.computeInverseWithCheck(Minv, invertible);
    if(!invertible)
    {
        return false;
    }
    Eigen::Vector3d BCD = Minv * Eigen::Vector3d(-Mxz, -Myz, -Mz);

    center.x() = static_cast<float>(BCD.x() * -0.5);
    center.y() = static_cast<float>(BCD.y() * -0.5);
    float radicand = static_cast<float>(BCD.x() * BCD.x() / 4.0 + BCD.y() * BCD.y() / 4.0 - BCD.z());
    if(radicand <= 0.0f)
    {
        return false;
    }
    radius = std::sqrt(radicand);
    return true;
}

bool BallPerceptor::calculateBallInImage(BallPercept& ballPercept) const
{
    return getBallFromBallPoints(ballPercept.positionInImage, ballPercept.radiusInImage);
}

bool BallPerceptor::checkBallInImage(BallPercept& ballPercept) const
{
    const Vector2f center = ballPercept.positionInImage;
    const float radius = ballPercept.radiusInImage;

    return true;
    // check if the ball covers approxCenter2
    if((Vector2i(int(center.x() + 0.45), int(center.y() + 0.45)) - approxCenter2).squaredNorm() > sqr(radius))
    {
        return false;
    }

    // check ball radius
    if(radius - checkMaxRadiusPixelBonus > approxRadius1 * checkMaxRadiusDifference || radius + checkMinRadiusPixelBonus < approxRadius1 * checkMinRadiusDifference)
    {
        return false;
    }

    float noBallRadius = radius * checkOutlineRadiusScale + checkOutlineRadiusPixelBonus;

    Vector2i center32(int(center.x() * 45.f), int(center.y() * 45.f));
    int noBallRadius32 = int(noBallRadius * 45.f);
    int noBallDRadius32 = int(noBallRadius * 45.f / 1.41421326f);
    Vector2i noBallPoints32[8 + 4 * 4] =
    {
        Vector2i(center32.x() + noBallRadius32, center32.y()),
        Vector2i(center32.x() - noBallRadius32, center32.y()),
        Vector2i(center32.x(), center32.y() + noBallRadius32),
        Vector2i(center32.x(), center32.y() - noBallRadius32),
        Vector2i(center32.x() + noBallDRadius32, center32.y() + noBallDRadius32),
        Vector2i(center32.x() - noBallDRadius32, center32.y() - noBallDRadius32),
        Vector2i(center32.x() + noBallDRadius32, center32.y() - noBallDRadius32),
        Vector2i(center32.x() - noBallDRadius32, center32.y() + noBallDRadius32),
    };
    int noBallPointCount = 8;
    int resolutionWidth = theCameraInfo.width;
    int resolutionHeight = theCameraInfo.height;
    int borderDists[2] = {0, 3};
    if(center32.x() + noBallRadius32 >= resolutionWidth * 45)
    {
        for(int i = 0; i < 2; ++i)
        {
            int x = resolutionWidth - 1 - borderDists[i];
            float d = std::sqrt(float(sqr(noBallRadius) - sqr(x - center.x())));
            noBallPoints32[noBallPointCount++] = Vector2i(x * 45, int((center.y() + d) * 45.f));
            noBallPoints32[noBallPointCount++] = Vector2i(x * 45, int((center.y() - d) * 45.f));
        }
    }
    if(center32.y() + noBallRadius32 >= resolutionHeight * 45)
    {
        for(int i = 0; i < 2; ++i)
        {
            int y = resolutionHeight - 1 - borderDists[i];
            float d = std::sqrt(float(sqr(noBallRadius) - sqr(y - center.y())));
            noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() + d) * 45.f), y * 45);
            noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() - d) * 45.f), y * 45);
        }
    }
    if(center32.x() - noBallRadius32 < 0)
    {
        for(int i = 0; i < 2; ++i)
        {
            int x = borderDists[i];
            float d = std::sqrt(float(sqr(noBallRadius) - sqr(x - center.x())));
            noBallPoints32[noBallPointCount++] = Vector2i(x * 45, int((center.y() + d) * 45.f));
            noBallPoints32[noBallPointCount++] = Vector2i(x * 45, int((center.y() - d) * 45.f));
        }
    }
    if(center32.y() - noBallRadius32 < 0)
    {
        for(int i = 0; i < 2; ++i)
        {
            int y = borderDists[i];
            float d = std::sqrt(float(sqr(noBallRadius) - sqr(y - center.y())));
            noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() + d) * 45.f), y * 45);
            noBallPoints32[noBallPointCount++] = Vector2i(int((center.x() - d) * 45.f), y * 45);
        }
    }
    Image::Pixel duplicateBallCheck;

    for(int i = 0; i < noBallPointCount; ++i)
    {
        Vector2i pos(noBallPoints32[i] / 45);
        if(pos.x() < 0 || pos.x() >= resolutionWidth ||
                pos.y() < 0 || pos.y() >= resolutionHeight)
        {
            continue;
        }

        duplicateBallCheck = getPixel(pos.y(), pos.x());
    }
    return true;}

bool BallPerceptor::calculateBallOnField(BallPercept& ballPercept) const
{
    const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
    Vector3f cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedCenter.x(), theCameraInfo.opticalCenter.y() - correctedCenter.y());
    cameraToBall.normalize(theFieldDimensions.ballRadius * theCameraInfo.focalLength / ballPercept.radiusInImage);
    Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
    const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
    const Vector3f bearingBasedCenterOnField =  theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / rotatedCameraToBall.z());

    if(rotatedCameraToBall.z() < 0)
    {
        ballPercept.relativePositionOnField.x() = bearingBasedCenterOnField.x();
        ballPercept.relativePositionOnField.y() = bearingBasedCenterOnField.y();
    }
    else
    {
        ballPercept.relativePositionOnField.x() = sizeBasedCenterOnField.x();
        ballPercept.relativePositionOnField.y() = sizeBasedCenterOnField.y();
    }

    return true;
}

bool BallPerceptor::checkBallOnField(BallPercept& ballPercept) const
{
    // Not sure about self-localization => Do not use it to check ball position
    if(theRobotPose.validity < 1.f)
    {
        return true;
    }
    // Check, if the computed ball position is still on the carpet
    Pose2f currentRobotPose = theRobotPose + theOdometer.odometryOffset;
    Vector2f absoluteBallPosition = (currentRobotPose * ballPercept.relativePositionOnField);
    return ((fabs(absoluteBallPosition.x()) < theFieldDimensions.xPosOpponentFieldBorder + 300.f) &&
            (fabs(absoluteBallPosition.y()) < theFieldDimensions.yPosLeftFieldBorder + 300.f));
}

/** Returns false if not surorunded by green*/
bool BallPerceptor::isGreenAround() const
{
    unsigned int minGreen = 19;
    unsigned int green = 0;
    bool sideInbody = false;
    int ratio = std::abs(std::max((ballPoints[4].point.y() - ballPoints[0].point.y()),(ballPoints[2].point.x() - ballPoints[6].point.x()))*0.3);

#ifdef NO_COORDINATED_ROLES
    theRobotInfo.number == 5 ? minGreen = 15 : minGreen = 19;
#else
    (theContextCoordination.robotRole == ContextCoordination::striker) ? minGreen = 15 : minGreen = 17;
#endif
    SuperPixel a, b, c, d, e;

    // Top -------------------
    a.initialize(Vector2i(ballPoints[0].point.x(), ballPoints[0].point.y()+ratio), theCameraInfo.width, theCameraInfo.height);
    b.initialize(Vector2i(ballPoints[0].point.x()+ratio, ballPoints[0].point.y()+ratio), theCameraInfo.width, theCameraInfo.height);
    c.initialize(Vector2i(ballPoints[0].point.x()-ratio, ballPoints[0].point.y()+ratio), theCameraInfo.width, theCameraInfo.height);
    d.initialize(Vector2i(ballPoints[0].point.x()-ratio*2, ballPoints[0].point.y()+ratio), theCameraInfo.width, theCameraInfo.height);
    e.initialize(Vector2i(ballPoints[0].point.x()+ratio*2, ballPoints[0].point.y()+ratio), theCameraInfo.width, theCameraInfo.height);

    if(!a.atBorder && !d.atBorder && !e.atBorder){
        int clippedBottomA = a.center.y();
        int clippedBottomB = b.center.y();
        int clippedBottomC = c.center.y();
        int clippedBottomD = d.center.y();
        int clippedBottomE = e.center.y();
        theBodyContour.clipBottom(a.center.x(), clippedBottomA, theCameraInfo.height);
        theBodyContour.clipBottom(b.center.x(), clippedBottomB, theCameraInfo.height);
        theBodyContour.clipBottom(c.center.x(), clippedBottomC, theCameraInfo.height);
        theBodyContour.clipBottom(d.center.x(), clippedBottomD, theCameraInfo.height);
        theBodyContour.clipBottom(e.center.x(), clippedBottomE, theCameraInfo.height);
        if(a.center.y() > clippedBottomA || b.center.y() > clippedBottomB || c.center.y() > clippedBottomC || d.center.y() > clippedBottomD || e.center.y() > clippedBottomE){
            a.isGreen = true;
            b.isGreen = true;
            c.isGreen = true;
            d.isGreen = true;
            e.isGreen = true;
            green+=5;
        }else{
            for(int i = 0; i < 5; i++){
                a.isSubPixelGreen(theColorTable[getPixel(a.subPixels[i].y(), a.subPixels[i].x())].is(ColorClasses::green));
                b.isSubPixelGreen(theColorTable[getPixel(b.subPixels[i].y(), b.subPixels[i].x())].is(ColorClasses::green));
                c.isSubPixelGreen(theColorTable[getPixel(c.subPixels[i].y(), c.subPixels[i].x())].is(ColorClasses::green));
                d.isSubPixelGreen(theColorTable[getPixel(d.subPixels[i].y(), d.subPixels[i].x())].is(ColorClasses::green));
                e.isSubPixelGreen(theColorTable[getPixel(e.subPixels[i].y(), e.subPixels[i].x())].is(ColorClasses::green));
            }
            green += (a.isGreen + b.isGreen + c.isGreen + d.isGreen + e.isGreen);
        }
    }
#ifdef WANNA_SEE_UP
    else
    {
        green += 5;
        a.isGreen = true;
        b.isGreen = true;
        c.isGreen = true;
        d.isGreen = true;
        e.isGreen = true;
    }
#endif
//    CROSS2("representation:BallPercept:Image", a.center.x(), a.center.y(), 3, 3, Drawings::solidPen, a.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", b.center.x(), b.center.y(), 3, 3, Drawings::solidPen, b.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", c.center.x(), c.center.y(), 3, 3, Drawings::solidPen, c.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", d.center.x(), d.center.y(), 3, 3, Drawings::solidPen, d.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", e.center.x(), e.center.y(), 3, 3, Drawings::solidPen, e.isGreen? ColorRGBA::green : ColorRGBA::yellow);

    // Bottom -------------------
    a.initialize(Vector2i(ballPoints[4].point.x(), ballPoints[4].point.y()-ratio), theCameraInfo.width, theCameraInfo.height);
    b.initialize(Vector2i(ballPoints[4].point.x()+ratio, ballPoints[4].point.y()-ratio), theCameraInfo.width, theCameraInfo.height);
    c.initialize(Vector2i(ballPoints[4].point.x()-ratio, ballPoints[4].point.y()-ratio), theCameraInfo.width, theCameraInfo.height);
    d.initialize(Vector2i(ballPoints[4].point.x()+ratio*2, ballPoints[4].point.y()-ratio), theCameraInfo.width, theCameraInfo.height);
    e.initialize(Vector2i(ballPoints[4].point.x()-ratio*2, ballPoints[4].point.y()-ratio), theCameraInfo.width, theCameraInfo.height);

    if(!a.atBorder && !d.atBorder && !e.atBorder){
        for(int i = 0; i < 5; i++){
            a.isSubPixelGreen(theColorTable[getPixel(a.subPixels[i].y(), a.subPixels[i].x())].is(ColorClasses::green));
            b.isSubPixelGreen(theColorTable[getPixel(b.subPixels[i].y(), b.subPixels[i].x())].is(ColorClasses::green));
            c.isSubPixelGreen(theColorTable[getPixel(c.subPixels[i].y(), c.subPixels[i].x())].is(ColorClasses::green));
            d.isSubPixelGreen(theColorTable[getPixel(d.subPixels[i].y(), d.subPixels[i].x())].is(ColorClasses::green));
            e.isSubPixelGreen(theColorTable[getPixel(e.subPixels[i].y(), e.subPixels[i].x())].is(ColorClasses::green));
        }
        green += (a.isGreen + b.isGreen + c.isGreen + d.isGreen + e.isGreen);
    }else{
        green += 5;
    }

    CROSS2("representation:BallPercept:Image", a.center.x(), a.center.y(), 3, 3, Drawings::solidPen, a.isGreen? ColorRGBA::green : ColorRGBA::yellow);
    CROSS2("representation:BallPercept:Image", b.center.x(), b.center.y(), 3, 3, Drawings::solidPen, b.isGreen? ColorRGBA::green : ColorRGBA::yellow);
    CROSS2("representation:BallPercept:Image", c.center.x(), c.center.y(), 3, 3, Drawings::solidPen, c.isGreen? ColorRGBA::green : ColorRGBA::yellow);
    CROSS2("representation:BallPercept:Image", d.center.x(), d.center.y(), 3, 3, Drawings::solidPen, d.isGreen? ColorRGBA::green : ColorRGBA::yellow);
    CROSS2("representation:BallPercept:Image", e.center.x(), e.center.y(), 3, 3, Drawings::solidPen, e.isGreen? ColorRGBA::green : ColorRGBA::yellow);

    // Right -------------------
    a.initialize(Vector2i(ballPoints[2].point.x()+ratio, ballPoints[2].point.y()), theCameraInfo.width, theCameraInfo.height);
    b.initialize(Vector2i(ballPoints[2].point.x()+ratio, ballPoints[2].point.y()+ratio), theCameraInfo.width, theCameraInfo.height);
    c.initialize(Vector2i(ballPoints[2].point.x()+ratio, ballPoints[2].point.y()-ratio), theCameraInfo.width, theCameraInfo.height);
    d.initialize(Vector2i(ballPoints[2].point.x()+ratio, ballPoints[2].point.y()+ratio*2), theCameraInfo.width, theCameraInfo.height);
    e.initialize(Vector2i(ballPoints[2].point.x()+ratio, ballPoints[2].point.y()-ratio*2), theCameraInfo.width, theCameraInfo.height);

    if(!a.atBorder && !d.atBorder && !e.atBorder){
        int clippedBottomA = a.center.y();
        int clippedBottomB = b.center.y();
        int clippedBottomC = c.center.y();
        int clippedBottomD = d.center.y();
        int clippedBottomE = e.center.y();

        theBodyContour.clipBottom(a.center.x(), clippedBottomA, theCameraInfo.height);
        theBodyContour.clipBottom(b.center.x(), clippedBottomB, theCameraInfo.height);
        theBodyContour.clipBottom(c.center.x(), clippedBottomC, theCameraInfo.height);
        theBodyContour.clipBottom(d.center.x(), clippedBottomD, theCameraInfo.height);
        theBodyContour.clipBottom(e.center.x(), clippedBottomE, theCameraInfo.height);

        sideInbody = ((a.center.y() > clippedBottomA) +
                      (b.center.y() > clippedBottomB) +
                      (c.center.y() > clippedBottomC) +
                      (d.center.y() > clippedBottomD) +
                      (e.center.y() > clippedBottomE)) > 2;

        if(a.center.y() > clippedBottomA){a.isGreen = true;}
        if(b.center.y() > clippedBottomB){b.isGreen = true;}
        if(c.center.y() > clippedBottomC){c.isGreen = true;}
        if(d.center.y() > clippedBottomD){d.isGreen = true;}
        if(e.center.y() > clippedBottomE){e.isGreen = true;}


        for(int i = 0; i < 5; i++)
        {
            a.isSubPixelGreen(theColorTable[getPixel(a.subPixels[i].y(), a.subPixels[i].x())].is(ColorClasses::green));
            b.isSubPixelGreen(theColorTable[getPixel(b.subPixels[i].y(), b.subPixels[i].x())].is(ColorClasses::green));
            c.isSubPixelGreen(theColorTable[getPixel(c.subPixels[i].y(), c.subPixels[i].x())].is(ColorClasses::green));
            d.isSubPixelGreen(theColorTable[getPixel(d.subPixels[i].y(), d.subPixels[i].x())].is(ColorClasses::green));
            e.isSubPixelGreen(theColorTable[getPixel(e.subPixels[i].y(), e.subPixels[i].x())].is(ColorClasses::green));
        }
        green += (a.isGreen + b.isGreen + c.isGreen + d.isGreen + e.isGreen);
    }

//    CROSS2("representation:BallPercept:Image", a.center.x(), a.center.y(), 3, 3, Drawings::solidPen, a.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", b.center.x(), b.center.y(), 3, 3, Drawings::solidPen, b.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", c.center.x(), c.center.y(), 3, 3, Drawings::solidPen, c.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", d.center.x(), d.center.y(), 3, 3, Drawings::solidPen, d.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", e.center.x(), e.center.y(), 3, 3, Drawings::solidPen, e.isGreen? ColorRGBA::green : ColorRGBA::yellow);

    // Left -------------------
    a.initialize(Vector2i(ballPoints[6].point.x()-ratio, ballPoints[6].point.y()), theCameraInfo.width, theCameraInfo.height);
    b.initialize(Vector2i(ballPoints[6].point.x()-ratio, ballPoints[6].point.y()+ratio), theCameraInfo.width, theCameraInfo.height);
    c.initialize(Vector2i(ballPoints[6].point.x()-ratio, ballPoints[6].point.y()-ratio), theCameraInfo.width, theCameraInfo.height);
    d.initialize(Vector2i(ballPoints[6].point.x()-ratio, ballPoints[6].point.y()+ratio*2), theCameraInfo.width, theCameraInfo.height);
    e.initialize(Vector2i(ballPoints[6].point.x()-ratio, ballPoints[6].point.y()-ratio*2), theCameraInfo.width, theCameraInfo.height);

    if(!a.atBorder && !d.atBorder && !e.atBorder){
        int clippedBottomA = a.center.y();
        int clippedBottomB = b.center.y();
        int clippedBottomC = c.center.y();
        int clippedBottomD = d.center.y();
        int clippedBottomE = e.center.y();

        theBodyContour.clipBottom(a.center.x(), clippedBottomA, theCameraInfo.height);
        theBodyContour.clipBottom(b.center.x(), clippedBottomB, theCameraInfo.height);
        theBodyContour.clipBottom(c.center.x(), clippedBottomC, theCameraInfo.height);
        theBodyContour.clipBottom(d.center.x(), clippedBottomD, theCameraInfo.height);
        theBodyContour.clipBottom(e.center.x(), clippedBottomE, theCameraInfo.height);

        if(sideInbody){
            if(((a.center.y() > clippedBottomA) +
                (b.center.y() > clippedBottomB) +
                (c.center.y() > clippedBottomC) +
                (d.center.y() > clippedBottomD) +
                (e.center.y() > clippedBottomE)) > 2)
                return false;
        }

        if(a.center.y() > clippedBottomA){a.isGreen = true;}
        if(b.center.y() > clippedBottomB){b.isGreen = true;}
        if(c.center.y() > clippedBottomC){c.isGreen = true;}
        if(d.center.y() > clippedBottomD){d.isGreen = true;}
        if(e.center.y() > clippedBottomE){e.isGreen = true;}

        for(int i = 0; i < 5; i++)
        {
            a.isSubPixelGreen(theColorTable[getPixel(a.subPixels[i].y(), a.subPixels[i].x())].is(ColorClasses::green));
            b.isSubPixelGreen(theColorTable[getPixel(b.subPixels[i].y(), b.subPixels[i].x())].is(ColorClasses::green));
            c.isSubPixelGreen(theColorTable[getPixel(c.subPixels[i].y(), c.subPixels[i].x())].is(ColorClasses::green));
            d.isSubPixelGreen(theColorTable[getPixel(d.subPixels[i].y(), d.subPixels[i].x())].is(ColorClasses::green));
            e.isSubPixelGreen(theColorTable[getPixel(e.subPixels[i].y(), e.subPixels[i].x())].is(ColorClasses::green));
        }
        green += (a.isGreen + b.isGreen + c.isGreen + d.isGreen + e.isGreen);
    }

//    CROSS2("representation:BallPercept:Image", a.center.x(), a.center.y(), 3, 3, Drawings::solidPen, a.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", b.center.x(), b.center.y(), 3, 3, Drawings::solidPen, b.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", c.center.x(), c.center.y(), 3, 3, Drawings::solidPen, c.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", d.center.x(), d.center.y(), 3, 3, Drawings::solidPen, d.isGreen? ColorRGBA::green : ColorRGBA::yellow);
//    CROSS2("representation:BallPercept:Image", e.center.x(), e.center.y(), 3, 3, Drawings::solidPen, e.isGreen? ColorRGBA::green : ColorRGBA::yellow);

    return green > minGreen;
}
