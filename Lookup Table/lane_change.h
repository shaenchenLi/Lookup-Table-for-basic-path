#ifndef LANE_CHANGE_H
#define LANE_CHANGE_H

#include "datastruct.h"
#include "range.h"

namespace Lane_Change_Path
{
	struct lane_change_path_library : public path_library
	{
		lane_change_path_library()
		{
			lane_change_path_lib = new PATH;
			failed_bound = new vector<vector<float>>;
		}
		lane_change_path_library(const lane_change_path_library &l)
		{
			lane_change_path_lib = l._lib();
			failed_bound = l._failed_bound();
		}
		~lane_change_path_library()
		{
			delete lane_change_path_lib;
		}

		void left_right(lane_change_path_library* left_lib);
		bool _control(const vector<float> &bound_condition, vector<float> *control/*, void *data*/);
		void add(vector<float> *bound_condition, vector<float> *control/*, void *data*/) override;
		void create_lib() override;

		PATH* _lib() const override { return lane_change_path_lib; }
		vector<vector<float>>* _failed_bound() const { return failed_bound; }

	private:
		PATH* lane_change_path_lib;
		vector<vector<float>>* failed_bound;
	};
}

#endif