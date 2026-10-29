#include "chaindynparam.hpp"
#include "frames_io.hpp"
#include <iostream>

namespace KDL {
    ChainDynParam::ChainDynParam(const Chain& _chain, Vector _grav):
            chain(_chain),
            nr(0),
            nj(chain.getNrOfJoints()),
            ns(chain.getNrOfSegments()),
            grav(_grav),
            jntarraynull(nj),
            chainidsolver_coriolis( chain, Vector::Zero()),
            chainidsolver_gravity( chain, grav),
            wrenchnull(ns,Wrench::Zero()),
            X(ns),
            S(ns),
            Ic(ns)
    {
        ag=-Twist(grav,Vector::Zero());
    }

    void ChainDynParam::updateInternalDataStructures() {
        nj = chain.getNrOfJoints();
        ns = chain.getNrOfSegments();
        jntarraynull.resize(nj);
        chainidsolver_coriolis.updateInternalDataStructures();
        chainidsolver_gravity.updateInternalDataStructures();
        wrenchnull.resize(ns,Wrench::Zero());
        X.resize(ns);
        S.resize(ns);
        Ic.resize(ns);
    }

    int ChainDynParam::JntToMass(const JntArray &q, JntSpaceInertiaMatrix& H)
    {
        if(nj != chain.getNrOfJoints() || ns != chain.getNrOfSegments())
            return (error = E_NOT_UP_TO_DATE);

        if(q.rows()!=nj || H.rows()!=nj || H.columns()!=nj )
            return (error = E_SIZE_MISMATCH);
        unsigned int k=0;
	double q_;

        for(unsigned int i=0;i<ns;i++)
	{
          Ic[i]=chain.getSegment(i).getInertia();
          if(chain.getSegment(i).getJoint().getType()!=Joint::Fixed)
	  {
	      q_=q(k);
	      k++;
	  }
	  else
	  {
	    q_=0.0;
	  }
	  X[i]=chain.getSegment(i).pose(q_);
	  S[i]=X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));
        }

        int j,l;
	k=nj-1;
        for(int i=ns-1;i>=0;i--)
	{
	  if(i!=0)
	    {
	      /* 串联链中前一个连杆作为当前连杆的父节点。 */
	      Ic[i-1]=Ic[i-1]+X[i]*Ic[i];
	    }

	  F=Ic[i]*S[i];
      if(chain.getSegment(i).getJoint().getType()!=Joint::Fixed)
	  {
          H(k,k)=dot(S[i],F);
          H(k,k)+=chain.getSegment(i).getJoint().getInertia();
	      j=k;
	      l=i;
	      while(l!=0)
		{
		  /* 串联链中前一个连杆作为当前连杆的父节点。 */
		  F=X[l]*F;
		  l--;

          if(chain.getSegment(l).getJoint().getType()!=Joint::Fixed)
		  {
		    j--;
		    H(k,j)=dot(F,S[l]);
		    H(j,k)=H(k,j);
		  }
		}
	      k--;
	  }
	}
	return (error = E_NOERROR);
    }

    int ChainDynParam::JntToCoriolis(const JntArray &q, const JntArray &q_dot, JntArray &coriolis)
    {
	SetToZero(jntarraynull);

	return chainidsolver_coriolis.CartToJnt(q, q_dot, jntarraynull, wrenchnull, coriolis);
    }

    int ChainDynParam::JntToGravity(const JntArray &q,JntArray &gravity)
    {
	SetToZero(jntarraynull);

	return chainidsolver_gravity.CartToJnt(q, jntarraynull, jntarraynull, wrenchnull, gravity);
    }

    ChainDynParam::~ChainDynParam()
    {
    }
}
