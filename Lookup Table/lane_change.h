#ifndef LANE_CHANGE_H
#define LANE_CHANGE_H

#include "datastruct.h"
#include "range.h"

namespace Lane_Change_Path
{
	struct lane_change_path_library : public path_library
	{
		lane_change_path_library(const int &f) :flag(f)
		{
			lane_change_path_lib = new PATH;
			failed_bound = new vector<vector<float>>;
		}
		lane_change_path_library(const lane_change_path_library &l)
		{
			lane_change_path_lib = l._lib();
			failed_bound = l._failed_bound();
			flag = l.flag;
		}

		lane_change_path_library() :lane_change_path_library(0) {}
		~lane_change_path_library()
		{
			delete lane_change_path_lib;
			delete failed_bound;
		}

		void left_right(lane_change_path_library* left_lib);
		bool _obs_control(const vector<float> &bound_condition, vector<float> *control);
		bool _safe_control(const vector<float> &bound_condition, vector<float> *control);
		bool _control(const vector<float> &bound_condition, vector<float> *control);
		void add(vector<float> *bound_condition, vector<float> *control) override;
		void create_lib() override;

		PATH* _lib() const override { return lane_change_path_lib; }
		vector<vector<float>>* _failed_bound() const { return failed_bound; }

	private:
		PATH* lane_change_path_lib;
		vector<vector<float>>* failed_bound;
		int flag; //flag=1 means that there is a obstacle or lane ,flag=0 means there is nothing
	};
}

#endif