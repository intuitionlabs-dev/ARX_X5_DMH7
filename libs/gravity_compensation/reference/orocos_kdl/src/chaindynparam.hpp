#ifndef KDLCHAINDYNPARAM_HPP
#define KDLCHAINDYNPARAM_HPP

#include "chainidsolver_recursive_newton_euler.hpp"
#include "articulatedbodyinertia.hpp"
#include "jntspaceinertiamatrix.hpp"
#include <Eigen/StdVector>

namespace KDL {
    class ChainDynParam : public SolverI
    {
    public:
        ChainDynParam(const Chain& chain, Vector _grav);
        virtual ~ChainDynParam();

        virtual int JntToCoriolis(const JntArray &q, const JntArray &q_dot, JntArray &coriolis);
	virtual int JntToMass(const JntArray &q, JntSpaceInertiaMatrix& H);
	virtual int JntToGravity(const JntArray &q,JntArray &gravity);

    virtual void updateInternalDataStructures();

    private:
        const Chain& chain;
	int nr;
	unsigned int nj;
        unsigned int ns;
	Vector grav;
	Vector vectornull;
	JntArray jntarraynull;
	ChainIdSolver_RNE chainidsolver_coriolis;
	ChainIdSolver_RNE chainidsolver_gravity;
	std::vector<Wrench> wrenchnull;
        std::vector<Frame> X;
        std::vector<Twist> S;

        std::vector<ArticulatedBodyInertia, Eigen::aligned_allocator<ArticulatedBodyInertia> > Ic;
        Wrench F;
        Twist ag;
    };
}

#endif
