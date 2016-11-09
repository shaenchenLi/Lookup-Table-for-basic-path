#include "straight.h"

void Straight_Path::straight_path_library::add(vector<float> *bound_condition, vector<float> *control)
{
	_control(*bound_condition, control);
	straight_path_lib->insert(make_pair(*bound_condition, *control));
}

bool Straight_Path::straight_path_library::_control(const vector<float> &bound_condition, vector<float> *control)
{
	for (int i = 0; i < 4; i++)
	{
		control->push_back(i / 3.f*bound_condition[0]);
		control->push_back(0);
	}
	return true;
}

void Straight_Path::straight_path_library::create_lib()
{	
	int N = (int)std::ceil((X_max_S - X_min_S) / discrete_space);
	float space = (X_max_S - X_min_S) / N;

	float l;
	for (int i = 1; i <= N; i++)
	{
		l = space*i + X_min_S;
		vector<float> bound_condition = { l, 0, 0 };
		vector<float> *control = new vector<float>;
		add(&bound_condition, control);
		delete control;
	}
}