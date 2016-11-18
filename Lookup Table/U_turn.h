#ifndef U_TURN_H
#define U_TURN_H

#include "datastruct.h"
#include "range.h"

namespace U_Turn_Path
{
	struct U_turn_path_library : public path_library
	{
		U_turn_path_library(const int &f) :flag(f)
		{
			U_turn_path_lib = new PATH;
		}

		U_turn_path_library() :U_turn_path_library(0) {}
		~U_turn_path_library()
		{
			delete U_turn_path_lib;
		}

		bool _lane_control(const vector<float> &bound_condition, vector<float> *control);
		bool _no_lane_control(const vector<float> &bound_condition, vector<float> *control);
		bool _control(const vector<float> &bound_condition, vector<float> *control);
		void add(vector<float> *bound_condition, vector<float> *control) override;
		void create_lib() override;

		PATH* _lib() const override { return U_turn_path_lib; }

	private:
		PATH* U_turn_path_lib;
		int flag; //flag=1 means there is lanes needed to obey; flag=0 means nothing
	};
}

#endif