#ifndef __ND_HPP__
#define __ND_HPP__

template<typename FLT_TYPE = float> class normal_likelihood
{
public:
	normal_likelihood(const FLT_TYPE sigma)
	{
		a = 1.0 / sqrtf(2.0 * sigma * sigma);
		sq2 = sigma * sigma * 2.0;
	}
	FLT_TYPE operator()(const FLT_TYPE x)
	{
		return a * expf(-x * x / sq2);
	}
private:
	FLT_TYPE a;
	FLT_TYPE sq2;
};

#endif

