/**
* @file LEDHandler.cpp
* This file implements a module that generates the LEDRequest from certain representations.
* @author jeff
*/

#include "LEDHandler.h"
#include "Representations/BehaviorControl/Role.h"

#include <algorithm>

void LEDHandler::update(LEDRequest& ledRequest)
{
    //reset
    for(int i = 0; i < ledRequest.numOfLEDs; ++i)
        ledRequest.ledStates[i] = LEDRequest::off;

    //update
    setRightEar(ledRequest);
    setLeftEar(ledRequest);
    setRightEye(ledRequest);
    setLeftEye(ledRequest);
    setChestButton(ledRequest);
}

void LEDHandler::setRightEar(LEDRequest& ledRequest)
{
    //right ear -> battery
    LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEar];

    int onLEDs = std::min(static_cast<int>(theSystemSensorData.batteryLevel / 0.1f), 9);

    for(int i = 0; i <= onLEDs; i++)
        ledRequest.ledStates[LEDRequest::earsRight0Deg + i] = state;
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest)
{
    //left ear -> connected players
    //          + GameController connection lost -> freaky blinking
    if(theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) > 2000)
    {
        ledRequest.ledStates[LEDRequest::earsLeft324Deg] = LEDRequest::blinking;
        ledRequest.ledStates[LEDRequest::earsLeft144Deg] = LEDRequest::blinking;
    }

    int numberOfConnectedTeammates = static_cast<int>(theTeammateData.teammates.size());
    if(numberOfConnectedTeammates > 0)
    {
        ledRequest.ledStates[LEDRequest::earsLeft0Deg] = LEDRequest::on;
        ledRequest.ledStates[LEDRequest::earsLeft36Deg] = LEDRequest::on;
    }
    if(numberOfConnectedTeammates > 1)
    {
        ledRequest.ledStates[LEDRequest::earsLeft72Deg] = LEDRequest::on;
        ledRequest.ledStates[LEDRequest::earsLeft108Deg] = LEDRequest::on;
    }
    if(numberOfConnectedTeammates > 2)
    {
        ledRequest.ledStates[LEDRequest::earsLeft180Deg] = LEDRequest::on;
        ledRequest.ledStates[LEDRequest::earsLeft216Deg] = LEDRequest::on;
    }
    if(numberOfConnectedTeammates > 3)
    {
        ledRequest.ledStates[LEDRequest::earsLeft252Deg] = LEDRequest::on;
        ledRequest.ledStates[LEDRequest::earsLeft288Deg] = LEDRequest::on;
    }
}

void LEDHandler::setEyeColor(LEDRequest& ledRequest,
                             bool left,
                             BehaviorLEDRequest::EyeColor col,
                             LEDRequest::LEDState s)
{
    LEDRequest::LED first = left ? LEDRequest::faceLeftRed0Deg : LEDRequest::faceRightRed0Deg;

    static const int redOffset = 0,
            greenOffset = LEDRequest::faceLeftGreen0Deg - LEDRequest::faceLeftRed0Deg,
            blueOffset = LEDRequest::faceLeftBlue0Deg - LEDRequest::faceLeftRed0Deg,
            numOfLEDsPerColor = LEDRequest::faceLeftRed315Deg - LEDRequest::faceLeftRed0Deg;

    LEDRequest::LEDState halfState = s == LEDRequest::off ? LEDRequest::off : LEDRequest::half;

    switch(col)
    {
    case BehaviorLEDRequest::defaultColor:
        ASSERT(false);
        break;
    case BehaviorLEDRequest::red:
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = s;
        break;
    case BehaviorLEDRequest::green:
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = s;
        break;
    case BehaviorLEDRequest::blue:
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    case BehaviorLEDRequest::white:
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = s;
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = s;
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    case BehaviorLEDRequest::magenta:
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = halfState;
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    case BehaviorLEDRequest::yellow:
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = halfState;
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + redOffset + i] = s;
        break;
    case BehaviorLEDRequest::cyan:
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + greenOffset + i] = halfState;
        for(int i = 0; i <= numOfLEDsPerColor; i++)
            ledRequest.ledStates[first + blueOffset + i] = s;
        break;
    default:
        ASSERT(false);
        break;
    }
}

void LEDHandler::setLeftEye(LEDRequest& ledRequest)
{
    //    //left eye -> groundContact ? ballSeen and GoalSeen : blue
    //    LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye];

    //    //no groundContact
    //    if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    //        setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
    //    //overwrite
    //    else if(theBehaviorLEDRequest.leftEyeColor != BehaviorLEDRequest::defaultColor)
    //        //blue
    //        setEyeColor(ledRequest, true, theBehaviorLEDRequest.leftEyeColor, state);
    //    //default
    //    else
    //    {
    //        bool ballSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;
    //        bool goalSeen = theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen) < 250;

    //        if(ballSeen && goalSeen)
    //            //red
    //            setEyeColor(ledRequest, true, BehaviorLEDRequest::red, state);
    //        else if(ballSeen)
    //            //white
    //            setEyeColor(ledRequest, true, BehaviorLEDRequest::white, state);
    //        else if(goalSeen)
    //            //green
    //            setEyeColor(ledRequest, true, BehaviorLEDRequest::green, state);
    //    }
    //left eye -> groundContact ? ballSeen and GoalSeen : blue

    LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::leftEye];

    //    no groundContact
    //    if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    //        setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
    //    overwrite
    //    if(theBehaviorLEDRequest.leftEyeColor != BehaviorLEDRequest::defaultColor)
    //blue
    //    setEyeColor(ledRequest, true, theBehaviorLEDRequest.leftEyeColor, state);
    //default


    bool ballSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;

    // TODO: this used to be seeing a whole goal, now it's only at least one post
    bool goalSeen = theFrameInfo.getTimeSince(theGoalPercept.timeWhenCompleteGoalLastSeen) < 250;

    bool globalBall = theGlobalBallEstimation.isMultiRobotValid;

    if(globalBall)
    {
        if(ballSeen)
        {
            if(goalSeen) // global && goal && ball
            {
                setEyeColor(ledRequest, true, BehaviorLEDRequest::magenta, state);
            }
            else // global && ball
            {
                setEyeColor(ledRequest, true, BehaviorLEDRequest::red, state);
            }
        }
        else if(goalSeen) //global && goal
        {
            setEyeColor(ledRequest, true, BehaviorLEDRequest::green, state);
        }
        else // solo global
        {
            setEyeColor(ledRequest, true, BehaviorLEDRequest::yellow, state);
        }
    }
    else if(ballSeen)
    {
        if(goalSeen) // ball && goal
        {
            setEyeColor(ledRequest, true, BehaviorLEDRequest::blue, state);
        }
        else // only ball
        {
            setEyeColor(ledRequest, true, BehaviorLEDRequest::cyan, state);
        }
    }
    else if(goalSeen) // solo goal
    {
        setEyeColor(ledRequest, true, BehaviorLEDRequest::white, state);
    }
//    else // nothing
//    {
//        setEyeColor(ledRequest, true, BehaviorLEDRequest::defaultColor, state);
//    }

    //    if(ballSeen && goalSeen)
    //      //red
    //      setEyeColor(ledRequest, true, BehaviorLEDRequest::red, state);
    //    else if(ballSeen)
    //      //white
    //      setEyeColor(ledRequest, true, BehaviorLEDRequest::white, state);
    //    else if(goalSeen)
    //      //green
    //      setEyeColor(ledRequest, true, BehaviorLEDRequest::green, state);

}

void LEDHandler::setRightEye(LEDRequest& ledRequest)
{
    //    //right eye -> groundContact ? role : role -> blinking
    //    //           + penalty shootout: native_{striker,keeper} ? {striker,keeper} : off
    //    LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye];

    //    //no groundContact
    //    if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    //        setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
    //    //overwrite
    //    else if(theBehaviorLEDRequest.rightEyeColor != BehaviorLEDRequest::defaultColor)
    //        setEyeColor(ledRequest, false, theBehaviorLEDRequest.rightEyeColor, state);
    //    else
    //    {
    //        switch(theBehaviorStatus.role)
    //        {
    //        case Role::goalie:
    //            setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
    //            break;
    //        case Role::defender:
    //            setEyeColor(ledRequest, false, BehaviorLEDRequest::white, state);
    //            break;
    //        case Role::striker:
    //            setEyeColor(ledRequest, false, BehaviorLEDRequest::red, state);
    //            break;
    //        case Role::supporter:
    //            setEyeColor(ledRequest, false, BehaviorLEDRequest::green, state);
    //            break;
    //        case Role::jolly:
    //            setEyeColor(ledRequest, false, BehaviorLEDRequest::magenta, state);
    //            break;
    //        case Role::no_role:
    //            //off
    //            break;
    //        default:
    //            ASSERT(false);
    //        }
    //    }

    //right eye -> groundContact ? role : role -> blinking
    //           + penalty shootout: native_{striker,keeper} ? {striker,keeper} : off

    LEDRequest::LEDState state = theBehaviorLEDRequest.modifiers[BehaviorLEDRequest::rightEye];

    //no groundContact
    if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
        setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
    //overwrite
    //else if(theBehaviorLEDRequest.rightEyeColor != BehaviorLEDRequest::defaultColor)
    //setEyeColor(ledRequest, false, theBehaviorLEDRequest.rightEyeColor, state);

    switch( theContextCoordination.robotRole )
    {

    case ContextCoordination::defender:
        setEyeColor(ledRequest, false, BehaviorLEDRequest::green, state);
        break;

    case ContextCoordination::supporter:
        setEyeColor(ledRequest, false, BehaviorLEDRequest::yellow, state);
        break;

    case ContextCoordination::striker:
        setEyeColor(ledRequest, false, BehaviorLEDRequest::red, state);
        break;

    case ContextCoordination::jolly:
        setEyeColor(ledRequest, false, BehaviorLEDRequest::blue, state);
        break;

    case ContextCoordination::goalie:
        setEyeColor(ledRequest, false, BehaviorLEDRequest::white, state);
        break;

    case ContextCoordination::guard:
        setEyeColor(ledRequest, false, BehaviorLEDRequest::cyan, state);
        break;

//    default:
//        setEyeColor(ledRequest, false, BehaviorLEDRequest::defaultColor, state);
//        break;

    }
}

void LEDHandler::setHead(LEDRequest& ledRequest)
{
    //not used yet
    for(unsigned i = LEDRequest::headLedRearLeft0; i <= LEDRequest::headLedMiddleLeft0; i++)
        ledRequest.ledStates[i] = LEDRequest::on;
}

void LEDHandler::setChestButton(LEDRequest& ledRequest)
{
    // Since libbhuman only sets an led if its state was changed we "override"
    // the on status of the red led so it can be deactivated if a whistle
    // was recognized. This is to override the yellow (red + green) chestbutton
    // that is set by the libGameCtrl in SET state. In addition we have to check
    // for the penalty status of the robot to not deactivate the red penalty light.
    if(theGameInfo.state == STATE_SET || theRobotInfo.penalty != PENALTY_NONE)
        ledRequest.ledStates[LEDRequest::chestRed] = LEDRequest::on;
}

MAKE_MODULE(LEDHandler, behaviorControl)
