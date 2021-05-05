#ifndef HEARTBEAT_H
#define HEARTBEAT_H

class TimeSlice
{
    private:
        long _previous_time = 0;
        long _interval = 0;
    public:
        TimeSlice(long t)
        {
            _interval = t;
        }        
        void Interval(long ct)
        {
            _interval = ct;
            _previous_time = 0;
        }
        bool Triggered(long ct)
        {
            bool ret = false;
            long delta = ct - _previous_time;
            if ( delta < 0 )
            {
                delta = 0;
            }
            if ( !(delta < _interval) )
            {
                ret = true;
                _previous_time = ct;
            }
            return ret;
        }
};

#endif