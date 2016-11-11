#ifndef __FILTER_H__
#define __FILTER_H__

class filter
{
public:
	enum type_t {
		FILTER_HPF,
		FILTER_LPF
	};
private:
	type_t type;
	float time_const;
	float x;
	float out;
	float k[4];
	bool angle;
public:
	filter(const enum type_t type, const float tc, const float out0, const bool angle = false)
	{
		this->angle = angle;
		time_const = tc;
		switch(type)
		{
		case FILTER_LPF:
			k[3] = - 1 / (1.0 + 2 * time_const);
			k[2] = - k[3];
			k[1] = (1.0 - 2 * time_const) * k[3];
			k[0] = - k[1] - 1.0;
			x = (1 - k[2]) * out0 / k[3];
			break;
		case FILTER_HPF:
			k[3] = - 1 / (1.0 + 2 * time_const);
			k[2] = - k[3] * 2 * time_const ;
			k[1] = (1.0 - 2 * time_const) * k[3];
			k[0] = 2 * time_const * (- k[1] + 1.0);
			x = (1 - k[2]) * out0 / k[3];
			break;
		}
	}
	void set(const float &out0)
	{
		if(angle)
		{
			x = (1 - k[2]) * remainder(out0, M_PI * 2.0) / k[3];
		}
		else
		{
			x = (1 - k[2]) * out0 / k[3];
		}
		out = out0;
	}
	float in(const float &i)
	{
		float in = i;
		if(angle)
		{
			in = out + remainder(in - out, M_PI * 2.0);
		}
		x = k[0] * in + k[1] * x;
		out = k[2] * in + k[3] * x;
		return out;
	}
};

#endif

