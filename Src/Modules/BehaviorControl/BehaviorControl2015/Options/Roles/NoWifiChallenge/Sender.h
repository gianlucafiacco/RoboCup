#include <iostream>

option(Sender)
{
    initial_state(left)
    {
        transition
        {
			//~ std::cerr << "asdsadsa2"<<std::endl;
            if (state_time > 3500) goto play;
        }

        action
        {
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(90);
            theHeadMotionRequest.tilt = Angle::fromDegrees(0);  //25
            theHeadMotionRequest.speed = pi/5; //fromDegrees(200);
        }
    }

    state(play)
    {
        transition
        {
			// std::cerr << "avail " << theNoWifiPacketToSend.isPacketAvailable << std::endl;
			// std::cerr << "header " << theNoWifiPacketToSend.packet.header.type << std::endl;
            if(theNoWifiPacketToSend.isPacketAvailable) {
                if(theNoWifiPacketToSend.packet.header.type==1) {
					std::cerr << "goto pl location" << std::endl;
					goto playLocation;
				} else if (theNoWifiPacketToSend.packet.header.type==2) {
					std::cerr << "goto pl data" << std::endl;
					goto playData;
				}
            // if(state_time >8000) goto wait;
			}
        }

        action
        {
			Stand();
        }
    }

	state(playLocation) {
		transition {
			if (state_time > 1000) {
				std::cerr << "goto play" << std::endl;
				goto play;
			}
		}
		
		action {
			std::cerr << "action playlocation" << std::endl;
			if (!libCodeRelease.playLocation())
				std::cerr << "Receiver location NOT correctly played" << std::endl;
			else
				std::cerr << "Receiver location correctly played" << std::endl;
		}
	}
	
	state(playData) {
		transition {
			if (state_time > 1000) {
				std::cerr << "goto play" << std::endl;
				goto play;
			}
		}
		
		action {
			std::cerr << "action playdata" << std::endl;
			if (!libCodeRelease.playData())
				std::cerr << "Receiver data NOT correctly played" << std::endl;
			else
				std::cerr << "Receiver data correctly played" << std::endl;
			
		}
	}

    //~ state(wait){
        //~ transition{
            //~ //if(state_time >20000) goto play;
        //~ }
        //~ action{
            //~ //std::cout<<"cinqueeeeee"<< std::endl;
            //~ Stand();
            //~ //std::cout<<"seiiiiiiiii"<<a;
        //~ }
    //~ }
}

