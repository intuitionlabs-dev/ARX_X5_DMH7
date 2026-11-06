#ifndef KDL_CHAIN_IKSOLVER_RECURSIVE_NEWTON_EULER_HPP
#define KDL_CHAIN_IKSOLVER_RECURSIVE_NEWTON_EULER_HPP

#include "chainidsolver.hpp"

namespace KDL{
    class ChainIdSolver_RNE : public ChainIdSolver{
    public:

        ChainIdSolver_RNE(const Chain& chain,Vector grav);
        ~ChainIdSolver_RNE(){};

        int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques);

        virtual void updateInternalDataStructures();

    private:
        const Chain& chain;
        unsigned int nj;
        unsigned int ns;
        std::vector<Frame> X;
        std::vector<Twist> S;
        std::vector<Twist> v;
        std::vector<Twist> a;
        std::vector<Wrench> f;
        Twist ag;
    };
}

#endif
