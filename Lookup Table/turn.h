#ifndef TURN_H
#define TURN_H

#include <iostream>
using std::endl; using std::cout;

#include "range.h"
#include "datastruct.h"

namespace Turn_Path
{
	struct turn_path_library : public path_library
	{
		turn_path_library(const int &f) :flag(f)
		{
			turn_path_lib = new PATH;			
		}
		turn_path_library() :turn_path_library(1) {}
		~turn_path_library()
		{
			delete turn_path_lib;
		}

		bool _turn(const vector<float> &bound_condition, vector<float> *control);
		bool _left_turn_center(const vector<float> &bound_condition, vector<float> *control);
		bool _control(const vector<float> &bound_condition, vector<float> *control);
		void add(vector<float> *bound_condition, vector<float> *control) override;
		void create_lib() override;

		PATH* _lib() const override { return turn_path_lib; }

	private:
		PATH* turn_path_lib;
		int flag = 1; //flag=1 equals to left-turning; flag=2 equals to left-turning with center; flag=-1 equals to right turning
	};
}

#endif