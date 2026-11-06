#ifndef KDL_CHAIN_HPP
#define KDL_CHAIN_HPP

#include "segment.hpp"
#include <string>

namespace KDL {
    /* 连杆数包含固定关节，关节数仅统计活动关节。 */
    class Chain {
    private:
        unsigned int nrOfJoints;
        unsigned int nrOfSegments;
    public:
        std::vector<Segment> segments;

        Chain();
        Chain(const Chain& in);
        Chain& operator = (const Chain& arg);

        void addSegment(const Segment& segment);

        void addChain(const Chain& chain);

        unsigned int getNrOfJoints()const {return nrOfJoints;};

        unsigned int getNrOfSegments()const {return nrOfSegments;};

        const Segment& getSegment(unsigned int nr)const;

        Segment& getSegment(unsigned int nr);

        virtual ~Chain();
    };
}

#endif
