#include "DynamicCameraCompensation.h"
//#define DEBUG

#ifdef DEBUG
#include <iostream>
#endif

bool DynamicCameraCompensation::enabled = false;
int DynamicCameraCompensation::badSituation = 0;
float DynamicCameraCompensation::time_stamp = SystemCall::getCurrentSystemTime();

void DynamicCameraCompensation::switchCameraSettings(CameraSettings &upperCameraSettings, CameraSettings &lowerCameraSettings){
    if(upperCameraSettings.settings[CameraSettings::AutoWhiteBalance].value == 1)
        upperCameraSettings.settings[CameraSettings::AutoWhiteBalance].value = 0;
    else
        upperCameraSettings.settings[CameraSettings::AutoWhiteBalance].value = 1;

    if(upperCameraSettings.settings[CameraSettings::AutoExposure].value == 1)
        upperCameraSettings.settings[CameraSettings::AutoExposure].value = 0;
    else
        upperCameraSettings.settings[CameraSettings::AutoExposure].value = 1;

    if(lowerCameraSettings.settings[CameraSettings::AutoWhiteBalance].value == 1)
        lowerCameraSettings.settings[CameraSettings::AutoWhiteBalance].value = 0;
    else
        lowerCameraSettings.settings[CameraSettings::AutoWhiteBalance].value = 1;

    if(lowerCameraSettings.settings[CameraSettings::AutoExposure].value == 1)
        lowerCameraSettings.settings[CameraSettings::AutoExposure].value = 0;
    else
        lowerCameraSettings.settings[CameraSettings::AutoExposure].value = 1;

    enabled= !enabled;
#ifdef DEBUG
    std::cout << "--------------------  DYNAMIC CAMERA switchCameraSettings enabled "<< enabled<<std::endl;
#endif
}

void DynamicCameraCompensation::checkGrid(const Image image, const ColorTable colorTable, int horizon, CameraSettings &upperCameraSettings, CameraSettings &lowerCameraSettings){
    const int width = image.width - 20;
    const int height = image.height - 3;

    //Get the horizon
    horizon = std::min(horizon, height);

    if(height - horizon < 64){
        badSituation = 0;
        return; // The robot is looking too high
    }

    unsigned int validPixels, badPixels = 0;

    /*
     * Scan a grid on the below-the-horizon part of the image . Check for ?white (naos, ball, line)? or green(field) pixels
     */
    for(int w = 20; w<width; w+= width/8){
        for(int h = horizon; h< height; h+= (height - horizon)/8){
            if(colorTable[image[h][w]].is(ColorClasses::green) /*|| colorTable[image[h][w]].is(ColorClasses::white)*/){
                validPixels ++;
            }else{
                badPixels ++;
            }
        }
    }

#ifdef DEBUG
        std::cout << "DYNAMIC CAMERA bad and valid pixels "<< badPixels << " " << validPixels <<std::endl;
        std::cout << "DYNAMIC CAMERA pixels average "<< (validPixels/(float)(validPixels+badPixels)) <<std::endl;
#endif
    /*
     * Check if the percentage of valid pixels is above the threshold.
     * If any, check if too many consecutive frames (BAD_SITUATION) had an high percentage of bad pixels.
     */
    if((validPixels/(float)(validPixels+badPixels)) <  VALID_THRESHOLD){
        if(badSituation > BAD_SITUATION){
            switchCameraSettings(upperCameraSettings, lowerCameraSettings);
            badSituation = 0;
        }else{
            badSituation++;
        }
    }else{
        badSituation = 0;
    }
}
