/**
 * Modification of the 2015 BallPerceptor
 * @file BallPerceptor.h
 * This file declares a module that provides the ball percept.
 * If possible, it uses the full YCbCr422 image.
 * @author Dario Albani
 * @author Vincenzo Suriani
 * @author Ali Youssef
  */
#pragma once

//#define PANIC
#define MINWHITE 7

#define SPQR_ERR(x) std::cerr << "\033[22;31;1m" << x << "\033[0m"<< std::endl;
#define SPQR_INFO(x) std::cerr << "\033[22;34;1m" << x << "\033[0m" << std::endl;
//#define SPQR_SUCC(x) std::cerr << "\033[0;32;1m" << x << "\033[0m" << std::endl;
//#define SPQR_WARN(x) std::cerr << "\033[0;33;1m"  << x << "\033[0m" << std::endl;
//#define SPQR_SPECIAL(x) std::cerr << "\033[0;35;1m" << x << "\033[0m" << std::endl;

#ifdef TARGET_ROBOT
#define IS_FULL_SIZE true
#else
#define IS_FULL_SIZE theImage.isFullSize
#endif

#include "Representations/spqr_representations/OurDefinitions.h"

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/PlayersPercept.h"
#include "Representations/Perception/BodyContour.h"

#ifdef NO_COORDINATED_ROLES
#include "Representations/Infrastructure/RobotInfo.h"
#else
#include "Modules/spqr_modules/ContextCoordinator/ContextCoordinator.h"
#endif

#include <iostream>

MODULE(BallPerceptor,
{,
 REQUIRES(FieldDimensions),
 REQUIRES(Image),
 REQUIRES(CameraMatrix),
 REQUIRES(ImageCoordinateSystem),
 REQUIRES(CameraInfo),
 REQUIRES(ColorTable),
 REQUIRES(Odometer),
 REQUIRES(BallSpots),
 REQUIRES(BodyContour),
 USES(RobotPose),
 USES(PlayersPercept),
 #ifdef NO_COORDINATED_ROLES
 USES(RobotInfo),
 #else
 USES(ContextCoordination),
 #endif
 PROVIDES(BallPercept),
 DEFINES_PARAMETERS(
 {,
  (float)(2.f) clippingApproxRadiusScale,
  (float)(2.5f) clippingApproxRadiusPixelBonus,
  (float)(1.3f) checkMaxRadiusDifference,
  (float)(0.9f) checkMinRadiusDifference,
  (float)(2.f) checkMaxRadiusPixelBonus,
  (float)(6.f) checkMinRadiusPixelBonus,
  (float)(1.1f) checkOutlineRadiusScale,
  (float)(2.f) checkOutlineRadiusPixelBonus,
  (bool)(false) wasOnLower, //If the ball on the lower camera put the prior on it
 }),
       });

/**
 * The class scales the input and output data if full size images
 * are avalable.
 */
class BallPerceptorScaler : public BallPerceptorBase
{
private:
    using BallPerceptorBase::theImage; // prevent direct access to image

protected:
    CameraInfo theCameraInfo;
    BallSpots theBallSpots;
    ImageCoordinateSystem theImageCoordinateSystem;

    /**
   * The only access to image pixels.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param The pixel as a temporary object. Do not use yCbCrPadding of that pixel.
   */
    const Image::Pixel getPixel(int y, int x) const
    {
        if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
            return theImage.getFullSizePixel(y, x);
        else
            return theImage[y][x];
    }

    /**
   * Update the copies of input representations that contain data
   * that might have to be scaled up.
   * Must be called in each cycle before any computations are performed.
   */
    void scaleInput();

    /**
   * Scale down the output if required.
   * @param ballPercept The ball percept the fields of which might be scaled.
   */
    void scaleOutput(BallPercept& ballPercept) const;

    /**
   * Scale down a single value if required.
   * This is only supposed to be used in debug drawings.
   * @param value The value that might be scaled.
   * @return The scaled value.
   */
    template<typename T> T scale(T value) const
    {
        if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
            return value / (T) 2;
        else
            return value;
    }
};

/**
 * The actual ball perceptor.
 * It is basically identical to the module "BallPerceptor".
 */
class BallPerceptor : public BallPerceptorScaler
{
private:
    using BallPerceptorBase::theImage; // prevent direct access to image

    struct BallPoint
    {
        Vector2i step;
        Vector2i start;
        Vector2i point;
        Vector2f pointf;
        bool atBorder = false;
        bool isValid = false;
    };


    struct SuperPixel{
    public:
        int numberOfGreens;
        bool isGreen;
        bool atBorder;
        Vector2i center;

        // 0-------1
        // ----c----
        // 3-------2
        Vector2i subPixels[5];

        void initialize(const Vector2i& centerPixel, const int width, const int height)
        {
            center = centerPixel;
            setAtBorder(width,height);
            setSubPixels( );
            numberOfGreens = 0;
            isGreen = false;
            atBorder = false;
        }

        void isSubPixelGreen(bool isSubPixelGreen){
            if(isSubPixelGreen){
                numberOfGreens++;
                if(numberOfGreens > 3)
                {
                    isGreen = true;
                }
            }
        }

    private:
        void setAtBorder(const int height, const int width)
        {
            atBorder = (center.x() < 3 || center.x() >= width-3 || center.y() < 3 || center.y() >= height-3);
        }

        void setSubPixels()
        {
            subPixels[0] = Vector2i(center.x()-1,center.y()-1);
            subPixels[1] = Vector2i(center.x()+1,center.y()-1);
            subPixels[2] = Vector2i(center.x()+1,center.y()+1);
            subPixels[3] = Vector2i(center.x()-1,center.y()+1);
            subPixels[4] = Vector2i(center.x(),center.y());
        }
    };

    float sqrMaxBallDistance; /**< The square of the maximal allowed ball distance. */

    void update(BallPercept& ballPercept);

    void fromBallSpots(BallPercept&);
    int totalBlackPixels;
    int totalWhitePixels;
    /** ########## begin: analyzing chain for possible balls. ########## */

    // limits of the image where a ball should be found
    static const int left = 2;
    int right; //right, horizon and height are set in update()
    int horizon;
    int height;

    int whiteCounter = 0;
    int blackCounter = 0;

    BallPercept::Status analyzeBallSpot(BallSpot& ballspot, BallPercept& ballPercept);
    bool checkBallSpot(const BallSpot& ballSpot);
    bool isRegionCircular(BallSpot &ballSpot);
    bool isRegionCircularAux(Vector2i &position, Vector2i increment, bool isWhite);

    float approxRadius1; /**< Bearing based approximation of the radius. */

    bool searchBallPoints(const BallSpot& ballSpot);

    Image::Pixel startPixel; /**< The ball spot pixel. */
    BallPoint ballPoints[8]; /**< Points on the outer edge of the ball. */
    Vector2i approxCenter2;

    void searchBallPointFar(const Vector2i& start, const Vector2i& step, const int maxLength, BallPoint& ballPoint);
    bool checkBallPoints();
    bool getBallFromBallPoints(Vector2f& center, float& radius) const;
    bool isOnObstacle(const BallSpot &ballSpot);
    bool isGreenAround() const;
    bool isGreenAroundFarBallSpots() const;

    unsigned int validBallPoints; /**< Count of usable points on the outer edge of the ball. */

    bool calculateBallInImage(BallPercept& ballPercept) const;
    bool checkBallInImage(BallPercept& ballPercept) const;
    bool calculateBallOnField(BallPercept& ballPercept) const;
    bool checkBallOnField(BallPercept& ballPercept) const;

public:
    BallPerceptor();
};
