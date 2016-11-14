#include "lane_change.h"

bool Lane_Change_Path::lane_change_path_library::_obs_control(const vector<float> &bound_condition, vector<float> *control)
{
	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];
	float l = bound_condition[3];
	float w = bound_condition[4];

	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	bool result = false;
	for (auto a = L_min->begin(); a != L_min->end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = std::min(_L_min(alfa_tmp, ERROR_2), (l + (w - ERROR_1) / tanf(alfa_tmp) - W / 2 / sinf(alfa_tmp)));
		if (l1_tmp > l - L_F_BA + ERROR_1 || l1_tmp >= xg)
			break;

		// calculate l2&l3
		l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
		l3_tmp = (yg - l2_tmp*sinf(alfa_tmp)) / sinf(thetag);

		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < _L_min((alfa_tmp + thetag) > PI ? (2.f*PI - alfa_tmp - thetag) : (alfa_tmp + thetag)))
			continue;

		if (l2_tmp*sinf(alfa_tmp) < w)
		{
			if (w < ((l - l1_tmp - l2_tmp*cosf(alfa_tmp) - W / 2 * sinf(thetag))*tanf(thetag) + l2_tmp*sinf(alfa_tmp) + ERROR_1))
				continue;
		}

		x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
		y2_tmp = l2_tmp*sinf(alfa_tmp);
		if (((y2_tmp + 0.5f*W*cosf(alfa_tmp))>yg + W / 2 + 0.2) || (x2_tmp + W / 2 * sinf(alfa_tmp) - L_F_BA*cosf(alfa_tmp)) > xg)
			continue;

		result = true;
		alfa = alfa_tmp;
		l1 = l1_tmp;
		l2 = l2_tmp;
		l3 = l3_tmp;
		x2 = x2_tmp;
		y2 = y2_tmp;
	}
	if (result == false)
		failed_bound->push_back(bound_condition);
	else
	{
		control->push_back(0); control->push_back(0);
		control->push_back(l1 / 2); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(0.5f*(l1 + x2)); control->push_back(0.5f*y2);
		control->push_back(x2); control->push_back(y2);
		control->push_back(0.5f*(xg + x2)); control->push_back(0.5f*(yg + y2));
		control->push_back(xg); control->push_back(yg);
	}
	return result;
}

bool Lane_Change_Path::lane_change_path_library::_safe_control(const vector<float> &bound_condition, vector<float> *control)
{
	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];

	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	bool result = false;
	for (auto a = L_min->begin(); a != L_min->end(); a += 2)
	{
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = _L_min(alfa_tmp, ERROR_2);
		if (l1_tmp > xg)
			break;

		// calculate l2&l3
		l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
		l3_tmp = (xg - l1_tmp + l2_tmp*cosf(alfa_tmp)) / cosf(thetag);
		
		if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < _L_min((alfa_tmp + thetag) > PI ? (2.f*PI - alfa_tmp - thetag) : (alfa_tmp + thetag)))
			continue;

		x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
		y2_tmp = l2_tmp*sinf(alfa_tmp);
		if (((y2_tmp + 0.5f*W*cosf(alfa_tmp))>yg + W / 2 + 0.2) || (x2_tmp + W / 2 * sinf(alfa_tmp) - L_F_BA*cosf(alfa_tmp)) > xg)// path can't exceed the allowed lane
			continue;

		result = true;
		alfa = alfa_tmp;
		l1 = l1_tmp;
		l2 = l2_tmp;
		l3 = l3_tmp;
		x2 = x2_tmp;
		y2 = y2_tmp;
	}
	if (result == false)
		failed_bound->push_back(bound_condition);
	else
	{
		control->push_back(0); control->push_back(0);
		control->push_back(l1 / 2); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(0.5f*(l1 + x2)); control->push_back(0.5f*y2);
		control->push_back(x2); control->push_back(y2);
		control->push_back(0.5f*(xg + x2)); control->push_back(0.5f*(yg + y2));
		control->push_back(xg); control->push_back(yg);
	}
	return result;
}

bool Lane_Change_Path::lane_change_path_library::_control(const vector<float> &bound_condition, vector<float> *control)
{
	if (flag == 1)
		return _obs_control(bound_condition, control);
	else
		return _safe_control(bound_condition, control);
}

void Lane_Change_Path::lane_change_path_library::add(vector<float> *bound_condition, vector<float> *control)
{
	if (_control(*bound_condition, control))
		lane_change_path_lib->insert(make_pair(*bound_condition, *control));
}

void Lane_Change_Path::lane_change_path_library::create_lib()
{
	int N_x = (int)std::ceil((X_max_L - X_min_L) / discrete_space);
	float space_x = (X_max_L - X_min_L) / N_x;
	int N_y = (int)std::ceil((Y_max_L - Y_min_L) / discrete_space);
	float space_y = (Y_max_L - Y_min_L) / N_y;
	int N_theta = (int)std::ceil((THETA_max_L - THETA_min_L / theta_space));
	float space_theta = (THETA_max_L - THETA_min_L) / N_theta;

	if (flag==1)
	{
		int N_l = (int)std::ceil((L_max_L - L_min_L / discrete_space));
		float space_l = (L_max_L - L_min_L) / N_l;
		int N_w = (int)std::ceil((W_max_L - W_min_L / discrete_space));
		float space_w = (W_max_L - W_min_L) / N_w;
		for (int i = 0; i <= N_x; i++)
		{
			for (int j = 0; j <= N_y; j++)
			{
				for (int k = 0; k <= N_theta; k++)
				{
					for (int p = 0; p <= N_l; p++)
					{
						for (int q = 0; q <= N_w; q++)
						{
							vector<float> bound_condition = { space_x*i + X_min_L, space_y*j + Y_min_L, space_theta*k + THETA_min_L, p*space_l + L_min_L, std::min(space_y*j + Y_min_L, q*space_w + W_min_L) };
							vector<float> *control = new vector<float>;
							add(&bound_condition, control);
							delete control;
						}
					}
				}
			}
		}
	}
	else
	{
		for (int i = 0; i <= N_x; i++)
		{
			for (int j = 0; j <= N_y; j++)
			{
				for (int k = 0; k <= N_theta; k++)
				{
					vector<float> bound_condition = { space_x*i + X_min_L, space_y*j + Y_min_L, space_theta*k + THETA_min_L };
					vector<float> *control = new vector<float>;
					add(&bound_condition, control);
					delete control;
				}
			}
		}
	}
}

void Lane_Change_Path::lane_change_path_library::left_right(lane_change_path_library* left_lib)
{
	for (auto &i : *left_lib->_lib())
		lane_change_path_lib->insert({ { i.first[0], i.first[1] * (-1), i.first[2] }, { i.second[0], i.second[2] * (-1) } });
	for (auto &i : *left_lib->_failed_bound())
		failed_bound->push_back({ i[0], i[1] * (-1), i[2] });
}