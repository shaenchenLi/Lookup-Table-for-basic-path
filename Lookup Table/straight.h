#ifndef STRAIGHT_H
#define STRAIGHT_H

#include "datastruct.h"
#include "range.h"

namespace Straight_Path
{
	struct straight_path_library : public path_library
	{
		straight_path_library()
		{
			vector<float> bound_condition = { X_min_S, 0, 0 };
			vector<float>* control = new vector<float>;
			_control(bound_condition, control);
			straight_path_lib = new PATH;
			straight_path_lib->insert({ bound_condition, *control });
		}
		~straight_path_library()
		{
			delete straight_path_lib;
		}

		bool _control(const vector<float> &bound_condition, vector<float> *control) override;
		void add(vector<float> *bound_condition, vector<float> *control) override;
		void create_lib() override;
		PATH* _lib() const override { return straight_path_lib; }

	private:
		PATH *straight_path_lib;
	};
}

#endif