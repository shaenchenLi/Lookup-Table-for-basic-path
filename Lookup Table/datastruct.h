#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <algorithm>
#include <unordered_map>
#include <vector>
using std::make_pair;
using std::unordered_map;
using std::vector;

#include "range.h"

using PATH = unordered_map<vector<float>, vector<float>>;

namespace std
{
	template<>
	struct hash<vector<float>>
	{
		std::size_t operator()(const vector<float> &key) const
		{
			std::size_t result = hash<float>()(*key.begin());
			for (auto i = key.begin() + 1; i != key.end(); i++)
				result = result ^ (hash<float>()(*key.begin()));
			return result;
		}
	};
}

struct path_library
{
	path_library()
	{
		path_lib = new PATH;
		L_min = new vector<float>;
		_L_min();
	}
	virtual ~path_library()
	{
		delete path_lib;
	}

	virtual bool _control(const vector<float> &bound_condition, vector<float> *control) { return true;  }
	virtual void add(vector<float> *bound_condition, vector<float> *control)
	{
			path_lib->insert(std::make_pair(*bound_condition, *control));
	}
	
	virtual void create_lib() {}
	float _L_min(const float &theta, const float err = 0)
	{
		return (sinf(theta)*powf((1 - cosf(theta)) / 8, -1.5) / 6.f / std::min(KMAX, std::abs(KMIN)) + err);
	}
	void _L_min()
	{
		L_min->push_back(8 * PI / 9);
		L_min->push_back(_L_min(*L_min->rbegin()));
		for (int i = 0; i < 6; i++) 
		{
			L_min->push_back(5 * PI / 6 - i*0.1f*PI);
			L_min->push_back(_L_min(*L_min->rbegin()));
		}
	}

	virtual PATH* _lib() const { return path_lib; }

protected:
	PATH* path_lib;
	vector<float>* L_min; 
};

#endif