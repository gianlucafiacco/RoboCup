#ifndef BUFFERCORRELATIONINTIME_H
#define BUFFERCORRELATIONINTIME_H

#include <vector>
#include "Tools/Math/Pose2f.h"
#include <iostream>

class BufferCorrelationInTime // mettere un set reference
{
public:

    class Entry{
    public:
        int id;
        unsigned ts;
        float validity;
        Pose2f pose;
        Entry(){id=0; ts=0; validity=0;}
        Entry(int r, unsigned timeStamp, float val, float x, float y){id=r; ts=timeStamp; validity=val; pose = Pose2f(x,y);}
    };

    /** Constructor */
    BufferCorrelationInTime() {init();}

    /**
   * initializes the BufferCorrelationInTime
   */
    void init() {
        for (int i=0; i<10; ++i)
        {
            min_in_time[i] = std::make_pair(0,i);
            max_in_time[i] = std::make_pair(0,i);
            references[i] = 0;
        }
    }

    bool add(unsigned ts, int r, float val, float x, float y, int time, bool owner = false) // val passed in [0,...,255]
    {
        bool return_value = false;
//        std::cerr << "Validity trying to insert: " << val << std::endl;
        if( (/*owner &&*/ val >= 0.4) /*|| val >= (0.4 * 255.f)*/ ) // consider only poses with validity more or equals to 40%
        {
//            if(r==2) std::cerr << "[" << r << "] " << "val major" << std::endl;
            for( int t=0; t<10; ++t ) // time 't'
            {
                if( owner || ((references[t] - 2000 <= ts) && (references[t] + 2000 >= ts)) )
                {
//                    if(r==2) std::cerr << "[" << r << "] " << "for [" << t << "] in bound" << std::endl;
                    if( informationMatrix[r-1][t].ts < ts )
                    {
//                        if(r==2) std::cerr << "[" << r << "] " << "for " << t << " info old" << std::endl;
//                        if(r==2) std::cerr << "[" << r << "] " << "owner? " << owner << std::endl;
                        if( ts > max_in_time[t].first ) // divento il nuovo massimo
                        {
                            max_in_time[t] = std::make_pair(ts,r);
                        }
                        if(owner)
                        {
                            if(t==time)
                            {
                                references[time] = ts;
                                informationMatrix[r-1][time] = Entry(r, ts, val, x, y); // place the new entry in the matrix
                                return_value = true;
                            }
                        }
                        else
                        {
//                            if(r==2) std::cerr << "[" << r << "] " << "inserted" << std::endl;
                            informationMatrix[r-1][t] = Entry(r, ts, val, x, y); // place the new entry in the matrix
                            return_value = true;
                        }

                        if(min_in_time[t].second == r || min_in_time[t].first == 0) // r was the lower
                        {
                            unsigned tmp_min = max_in_time[t].first;
                            int pos = 1, pos_tmp = 0;
                            for( int r_tmp = 0; r_tmp < 5; ++r_tmp, ++pos )
                            {
                                if( informationMatrix[r_tmp][t].ts != 0 && informationMatrix[r_tmp][t].ts <= tmp_min)
                                {
                                    tmp_min = informationMatrix[r_tmp][t].ts;
                                    pos_tmp = pos;
                                }
                            }
                            min_in_time[t] = std::make_pair(tmp_min, pos_tmp);
                        }
                    }
                }
            }
        }
        return return_value;
    }

public:
    Entry informationMatrix[5][10];
    unsigned references[10];

    bool fallHere(int t, unsigned range) const
    {
        return getMax(t) - getMin(t) >= range;
    }

    unsigned getMin(int t) const
    {
        return min_in_time[t].first;
    }

    unsigned getMax(int t) const
    {
        return max_in_time[t].first;
    }

private:
    std::pair<unsigned,int> min_in_time[10], max_in_time[10];
};


#endif // BUFFERCORRELATIONINTIME_H
