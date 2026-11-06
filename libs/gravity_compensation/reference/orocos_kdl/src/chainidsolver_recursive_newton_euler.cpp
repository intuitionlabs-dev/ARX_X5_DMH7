#include "chainidsolver_recursive_newton_euler.hpp"
#include "frames_io.hpp"

namespace KDL{
    ChainIdSolver_RNE::ChainIdSolver_RNE(const Chain& chain_,Vector grav):
        chain(chain_),nj(chain.getNrOfJoints()),ns(chain.getNrOfSegments()),
        X(ns),S(ns),v(ns),a(ns),f(ns)
    {
        ag=-Twist(grav,Vector::Zero());
    }

    void ChainIdSolver_RNE::updateInternalDataStructures() {
        nj = chain.getNrOfJoints();
        ns = chain.getNrOfSegments();
        X.resize(ns);
        S.resize(ns);
        v.resize(ns);
        a.resize(ns);
        f.resize(ns);
    }

    int ChainIdSolver_RNE::CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques)
    {
        if(nj != chain.getNrOfJoints() || ns != chain.getNrOfSegments())
            return (error = E_NOT_UP_TO_DATE);

        if(q.rows()!=nj || q_dot.rows()!=nj || q_dotdot.rows()!=nj || torques.rows()!=nj || f_ext.size()!=ns)
            return (error = E_SIZE_MISMATCH);
        unsigned int j=0;

        for(unsigned int i=0;i<ns;i++){
            double q_,qdot_,qdotdot_;
            if(chain.getSegment(i).getJoint().getType()!=Joint::Fixed) {
                q_=q(j);
                qdot_=q_dot(j);
                qdotdot_=q_dotdot(j);
                j++;
            }else
                q_=qdot_=qdotdot_=0.0;

            X[i]=chain.getSegment(i).pose(q_);

            Twist vj=X[i].M.Inverse(chain.getSegment(i).twist(q_,qdot_));
            S[i]=X[i].M.Inverse(chain.getSegment(i).twist(q_,1.0));

            /* 在连杆坐标系中递推速度和加速度。 */
            if(i==0){
                v[i]=vj;
                a[i]=X[i].Inverse(ag)+S[i]*qdotdot_+v[i]*vj;
            }else{
                v[i]=X[i].Inverse(v[i-1])+vj;
                a[i]=X[i].Inverse(a[i-1])+S[i]*qdotdot_+v[i]*vj;
            }

            RigidBodyInertia Ii=chain.getSegment(i).getInertia();
            f[i]=Ii*a[i]+v[i]*(Ii*v[i])-f_ext[i];
        }

        j=nj-1;
        for(int i=ns-1;i>=0;i--){
            if(chain.getSegment(i).getJoint().getType()!=Joint::Fixed) {
                torques(j)=dot(S[i],f[i]);
                torques(j)+=chain.getSegment(i).getJoint().getInertia()*q_dotdot(j);
                --j;
            }
            if(i!=0)
                f[i-1]=f[i-1]+X[i]*f[i];
        }
	return (error = E_NOERROR);
    }
}
