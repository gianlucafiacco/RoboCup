#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldBoundary.h"

MODULE(BallSpotProvider,
{,
 REQUIRES(ColorTable),
 REQUIRES(Image),
 REQUIRES(ImageCoordinateSystem),
 REQUIRES(ScanlineRegionsClipped),
 REQUIRES(CameraInfo),
 REQUIRES(FieldBoundary),
 PROVIDES(BallSpots),
       });

/**
* @class BallSpotProvider
* A module that provides spots that might be inside balls
*/
class BallSpotProvider: public BallSpotProviderBase
{
private:
    void update(BallSpots& ballSpots);
    using Scanline = ScanlineRegionsClipped::Scanline;
    using Region = ScanlineRegionsClipped::Region;

    /**searches the scanlines for ball spots*/
    bool searchScanLines(BallSpots& ballSpots) const;

    /**Tries to find the center of a Ball*/
    bool getBallSpot(BallSpot& ballSpot, bool isWhite) const;
};
