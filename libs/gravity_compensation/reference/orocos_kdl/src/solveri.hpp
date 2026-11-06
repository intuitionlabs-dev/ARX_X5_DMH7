#ifndef	__SOLVERI_HPP
#define	__SOLVERI_HPP

namespace KDL {
/* 保存最近一次求解错误；0 为成功，正值为降级警告，负值为失败。 */
class SolverI
{
public:
    enum {
        E_DEGRADED         = +1,

        E_NOERROR          =  0,

        E_NO_CONVERGE      = -1,

        E_UNDEFINED        = -2,

        E_NOT_UP_TO_DATE = -3,

        E_SIZE_MISMATCH = -4,

        E_MAX_ITERATIONS_EXCEEDED = -5,

        E_OUT_OF_RANGE = -6,

        E_NOT_IMPLEMENTED = -7,

        E_SVD_FAILED = -8
    };

	SolverI() :
			error(E_NOERROR)
	{}

	virtual ~SolverI()
	{}

	virtual int getError() const { return error; }

	virtual const char* strError(const int error) const
	{
		if (E_NOERROR == error) return "No error";
		else if (E_NO_CONVERGE == error) return "Failed to converge";
		else if (E_UNDEFINED == error) return "Undefined value";
		else if (E_DEGRADED == error) return "Converged but degraded solution";
		else if (E_NOT_UP_TO_DATE == error) return "Internal data structures not up to date with Chain";
		else if (E_SIZE_MISMATCH == error) return "The size of the input does not match the internal state";
		else if (E_MAX_ITERATIONS_EXCEEDED == error) return "The maximum number of iterations is exceeded";
		else if (E_OUT_OF_RANGE == error) return "The requested index is out of range";
		else if (E_NOT_IMPLEMENTED == error) return "The requested function is not yet implemented";
		else  if (E_SVD_FAILED == error) return "SVD failed";
		else return "UNKNOWN ERROR";
	}

	virtual void updateInternalDataStructures() = 0;

protected:

	int		error;
};
}

#endif
