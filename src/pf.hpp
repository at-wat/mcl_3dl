#include <random>
#include <vector>
#include <algorithm>
#include <functional>

namespace pf
{
	template<typename FLT_TYPE = float> class particleBase
	{
	public:
		virtual FLT_TYPE &operator[](const size_t i) = 0;
		virtual const size_t size() = 0;
		virtual void normalize() = 0;
		template <typename T> T operator+(const T &a)
		{
			T in = a;
			T ret;
			for(size_t i = 0; i < size(); i ++)
			{
				ret[i] = (*this)[i] + in[i];
			}
			return ret;
		}
		void weight(const FLT_TYPE &s)
		{
			for(size_t i = 0; i < size(); i ++)
			{
				(*this)[i] = (*this)[i] * s;
			}
		}
		template <typename T> FLT_TYPE cov_element(
				const T &e, const size_t &j, const size_t &k)
		{
			T exp = e;
			return ((*this)[k] - exp[k]) * ((*this)[j] - exp[j]);
		}
		template <typename T> T generate_noise(
				std::default_random_engine &engine,
				T mean, T sigma)
		{
			T noise;
			for(size_t i = 0; i < size(); i ++)
			{
				std::normal_distribution<FLT_TYPE> nd(mean[i], sigma[i]);
				noise[i] = nd(engine);
			}
			return noise;
		}
	private:
	};

	template<typename T, typename FLT_TYPE = float> class particle_filter
	{
	public:
		particle_filter(const int nParticles):
			engine(seed_gen())
		{
			particles.resize(nParticles);
		}
		void init(T mean, T sigma)
		{
			for(auto &p: particles)
			{
				p.state = p.state.generate_noise(engine, mean, sigma);
				p.probability = 1.0 / particles.size();
			}
		}
		void resample(T sigma)
		{
			FLT_TYPE accum = 0;
			for(auto &p: particles)
			{
				accum += p.probability;
				p.accum_probability = accum;
			}

			particles_dup = particles;
			std::sort(particles_dup.begin(), particles_dup.end());

			FLT_TYPE pstep = accum / particles.size();
			FLT_TYPE pscan = 0;
			auto it = particles_dup.begin();
			auto it_prev = particles_dup.end();

			FLT_TYPE prob = 1.0 / particles.size();
			for(auto &p: particles)
			{
				pscan += pstep;
				it = std::lower_bound(it, particles_dup.end(), particle(pscan));
				particle p0 = *it;
				if(it == it_prev)
				{
					p.state = p0.state + p0.state.generate_noise(engine, T(), sigma);
					p.state.normalize();
				}
				else if(it == particles_dup.end())
				{
					p.state = it_prev->state;
				}
				else
				{
					p.state = p0.state;
				}
				it_prev = it;
				p.probability = prob;
			}
		}
		void noise(T sigma)
		{
			for(auto &p: particles)
			{
				p.state = p.state + p.state.generate_noise(engine, T(), sigma);
			}
		}
		void predict(std::function<void(T&)> model)
		{
			for(auto &p: particles)
			{
				model(p.state);
			}
		}
		void measure(std::function<FLT_TYPE(const T&)> likelihood)
		{
			FLT_TYPE sum = 0;
			for(auto &p: particles)
			{
				p.probability *= likelihood(p.state);
				sum += p.probability;
			}
			for(auto &p: particles)
			{
				p.probability /= sum;
			}
		}
		T expectation(const FLT_TYPE pass_ratio = 1.0)
		{
			T e;
			FLT_TYPE p_sum = 0;

			if(pass_ratio < 1.0)
				std::sort(particles.rbegin(), particles.rend());
			for(auto &p: particles)
			{
				T e1 = p.state;
				e1.weight(p.probability);
				e = e1 + e;
				p_sum += p.probability;
				if(p_sum > pass_ratio) break;
			}
			e.weight(1.0f / p_sum);
			return e;
		}
		std::vector<T> covariance(const FLT_TYPE pass_ratio = 1.0)
		{
			T e = expectation(pass_ratio);
			FLT_TYPE p_sum = 0;
			size_t p_num = 0;
			std::vector<T> cov;
			cov.resize(e.size());

			for(auto &p: particles)
			{
				p_num ++;
				p_sum += p.probability;
				if(p_sum > pass_ratio) break;
			}
			p_sum = 0;
			for(auto &p: particles)
			{
				for(size_t j = 0; j < ie.size(); j ++)
				{
					for(size_t k = j; k < ie.size(); k ++)
					{
						cov[k][j] = cov[j][k] += p.state.cov_element(e, j, k) * p.probability;
					}
				}

				p_sum += p.probability;
				if(p_sum > pass_ratio) break;
			}
			for(size_t j = 0; j < ie.size(); j ++)
			{
				for(size_t k = j; k < ie.size(); k ++)
				{
					cov[k][j] /= p_sum;
					cov[j][k] /= p_sum;
				}
			}

			return cov;
		}
		T max()
		{
			T *m = &particles[0].state;
			FLT_TYPE max_probability = particles[0].probability;
			for(auto &p: particles)
			{
				if(max_probability < p.probability)
				{
					max_probability = p.probability;
					m = &p.state;
				}
			}
			return *m;
		}
		const T get_particle(const size_t i)
		{
			return particles[i].state;
		}
		const size_t get_particle_size()
		{
			return particles.size();
		}
	private:
		class particle
		{
		public:
			particle()
			{
				probability = 0.0;
			}
			particle(FLT_TYPE prob)
			{
				accum_probability = prob;
			}
			T state;
			FLT_TYPE probability;
			FLT_TYPE accum_probability;
			const bool operator<(const particle &p2) const
			{
				return this->accum_probability < p2.accum_probability;
			}
		};
		std::vector<particle> particles;
		std::vector<particle> particles_dup;
		std::random_device seed_gen;
		std::default_random_engine engine;
		T ie;
	};
}


