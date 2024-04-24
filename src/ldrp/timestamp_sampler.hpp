#pragma once

#include <utility>
#include <vector>
#include <algorithm>


/** TimestampSampler stores an ordered list of time-element pairs, and can efficiently look up the closes element given a target timestamp. */
template<typename T, typename Time_t>
class TimestampSampler {
public:
	using This_T = TimestampSampler<T, Time_t>;
	using Type = T;
	using TimeT = Time_t;
	using ElemT = std::pair<TimeT, Type>;

private:
	static bool t_less__(TimeT t, const ElemT& elem) {
		return t < elem.first;
	}
	static bool t_greater__(const ElemT& elem, TimeT t) {
		return t > elem.first;
	}

public:
	TimestampSampler() = default;
	~TimestampSampler() = default;


	/** Update the minimum bound timestamp such as to limit the number of entities stored in the buffer */
	void updateMin(TimeT min) {
		this->absolute_bound = min;
		this->enforceBound();
	}

	/** Add a new sample to the buffer */
	inline void insert(TimeT time, const Type& sample)
		{ this->_insert(time, Type{ sample }); }
	/** Add a new sample to the buffer */
	inline void insert(TimeT time, Type&& sample)
		{ this->_insert(time, sample); }

	/** Erase all the elements */
	inline void clear() { this->samples.clear(); }

	/** Sample the element with a timestamp closest to that given as a parameter */
	inline const Type* sample(TimeT time) const {
		const ElemT* elem = this->sampleTimestamped(time);
		return elem ? elem->second : nullptr;
	}
	/** Sample the closest element, but returns the full pair so that the element's timestamp can be accessed */
	const ElemT* sampleTimestamped(TimeT time) const {
		if( this->samples.empty() )					return nullptr;
		if( time <= this->samples.front().first )	return &this->samples.front();
		if( time >= this->samples.back().first )	return &this->samples.back();
		if( this->samples.size() == 1 )				return &this->samples[0];

		auto greater = std::lower_bound(this->samples.begin(), this->samples.end(), time, &This_T::t_greater__);	// "lower bound" of times greater than t
		if( greater == this->samples.begin() )		return &*greater;

		auto less = greater - 1;
		return abs(time - less->first) < abs(greater->first - time) ? &*less : &*greater;		// return closer sample
	}

	/** Return a const reference to the internal element buffer */
	inline const std::vector<ElemT>& getSamples() const { return this->samples; }


protected:
	void _insert(TimeT time, Type&& sample) {
		if(this->samples.size() <= 0 || time > this->samples.back().first) {
			this->samples.emplace_back(time, std::forward<Type>(sample));
		} else {
			auto after = std::upper_bound(this->samples.begin(), this->samples.end(), time, &This_T::t_less__);
			auto before = after - 1;
			if(after == this->samples.begin()) {
				this->samples.insert(after, std::pair{time, std::forward<Type>(sample)});
			} else {
				if(before == this->samples.begin() || before->first < time) {
					this->samples.insert(after, std::pair{time, std::forward<Type>(sample)});
				} else {
					before->second = sample;
				}
			}
		}
	}

	void enforceBound() {
		auto last = this->samples.begin();
		for(; last < this->samples.end() && last->first < this->absolute_bound; last++);
		this->samples.erase(this->samples.begin(), last);
	}

protected:
	TimeT absolute_bound = static_cast<TimeT>(0);	// the absolute minimum bounding time
	std::vector<ElemT> samples{};	// use a tree for better search performance


};
