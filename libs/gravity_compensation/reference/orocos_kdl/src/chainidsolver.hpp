#ifndef KDL_CHAIN_IDSOLVER_HPP
#define KDL_CHAIN_IDSOLVER_HPP

#include "chain.hpp"
#include "frames.hpp"
#include "jntarray.hpp"
#include "solveri.hpp"

namespace KDL
{
    typedef std::vector<Wrench> Wrenches;

	class ChainIdSolver : public KDL::SolverI
	{
		public:

        virtual int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)=0;
	};
}

#endif
