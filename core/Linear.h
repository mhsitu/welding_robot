#ifndef STRAIGHT_LINE
#define STRAIGHT_LINE

//Unit: time -> ms, point -> mm
class StraightLine
{
public:
    /*
        @param ts: total time
        @param init_pt & end_pt: points  
    */
    StraightLine()
    {
        this->ts = 0;
        this->init_pt = 0;
        this->end_py = 0;
    }

    bool setParam(int _ts, int _init_pt, int _end_pt)
    {
        ts = _ts;
        init_pt = _init_pt;
        end_pt = _end_pt;
    }

    bool getLinePoint(int u, int* ret)
    {
        if(ts == 0)
            return false;

        ret = float(u)/float(ts) * (init_pt - end_pt) + init_pt;

        return true;
    }
private:
    int ts;
    int init_pt;
    int end_pt;
};

#endif