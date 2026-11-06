#include "chain.hpp"

namespace KDL {
    Chain::Chain():
            nrOfJoints(0),
            nrOfSegments(0),
            segments(0)
    {
    }

    Chain::Chain(const Chain& in):
            nrOfJoints(0),
            nrOfSegments(0),
            segments(0)
    {
        for(unsigned int i=0;i<in.getNrOfSegments();i++)
            this->addSegment(in.getSegment(i));
    }

    Chain& Chain::operator=(const Chain& arg)
    {
        nrOfJoints=0;
        nrOfSegments=0;
        segments.resize(0);
        for(unsigned int i=0;i<arg.nrOfSegments;i++)
            addSegment(arg.getSegment(i));
        return *this;
    }

    void Chain::addSegment(const Segment& segment)
    {
        segments.push_back(segment);
        nrOfSegments++;
        if(segment.getJoint().getType()!=Joint::Fixed)
            nrOfJoints++;
    }

    void Chain::addChain(const Chain& chain)
    {
        for(unsigned int i=0;i<chain.getNrOfSegments();i++)
            this->addSegment(chain.getSegment(i));
    }

    const Segment& Chain::getSegment(unsigned int nr)const
    {
        return segments[nr];
    }

    Segment& Chain::getSegment(unsigned int nr)
    {
        return segments[nr];
    }

    Chain::~Chain()
    {
    }
}
