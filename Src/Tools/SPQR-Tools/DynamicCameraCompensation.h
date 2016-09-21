#pragma once

#include "Tools/Math/Pose2f.h"
#include "Representations/Infrastructure/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Platform/SystemCall.h"
#include "Platform/Camera.h"
#include "Platform/Linux/NaoCamera.h"
#include <vector>

#define VALID_THRESHOLD 0.5
#define BAD_SITUATION 8 //Number of consecutive bad frames accepted before enabling the auto compensation

class DynamicCameraCompensation
{  
private:
    static int badSituation;
    static float time_stamp;
public:
    static bool enabled;
    static void switchCameraSettings(CameraSettings& upperCameraSettings, CameraSettings& lowerCameraSettings);
    static void checkGrid(const Image image, const ColorTable colorTable, int horizon, CameraSettings& upperCameraSettings, CameraSettings& lowerCameraSettings);
};
