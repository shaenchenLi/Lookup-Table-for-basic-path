#include "turn.h"

bool Turn_Path::turn_path_library::_left_turn_center(const vector<float> &bound_condition, vector<float> *control)
{
	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];
	float l = bound_condition[3];
	float w = bound_condition[4];
	float r = bound_condition[5];
	cout << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
	cout << "l:" << l << " w:" << w << "r:" << r << endl;

	bool result = false;
	
	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	float step = 0.1;
	float exceed_allow_x = 3;
	float exceed_allow_y = 5;
	for (auto a = L_min->begin(); a != L_min->end() - 4; a += 2)
	{
		if (thetag == PI / 4)
			break;
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = _L_min(alfa_tmp) - step;
		while (result == false)
		{
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;

			l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
			l3_tmp = (yg - l2_tmp*sinf(alfa_tmp)) / sinf(thetag);
			if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < _L_min(2.f*PI - alfa_tmp - thetag))
				continue;

			x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
			y2_tmp = l2_tmp*sinf(alfa_tmp);
			if (x2_tmp > (xg + exceed_allow_x) || y2_tmp >= yg - exceed_allow_y)
				continue;

			float beita = PI - atan2f(w, l + r - l1_tmp) - alfa_tmp;
			float d = sqrtf(powf((l + r - l1_tmp), 2) + powf(w, 2))*sinf(beita);
			float x_v = l1_tmp - d / tanf(beita)*cosf(alfa_tmp);
			float y_v = d / tanf(beita)*sinf(alfa_tmp);
			if (x_v <= x2_tmp && y_v <= y2_tmp)
			{
				if (d < (r + 0.5f*W + ERROR_3))
					continue;
				if (d > r + 0.5f*W + 1.f)
					continue;
			}
			else
			{
				beita = alfa_tmp + thetag - atan2f(l + r - x2_tmp, w - y2_tmp) - PI;
				d = sqrtf(powf((l + r - x2_tmp), 2) + powf((w - y2_tmp), 2))*sinf(beita);
				if (d < (r + 0.5f*W + ERROR_3))
					continue;
				if (d > r + 0.5f*W + 1.f)
					continue;
			}

			result = true;
			alfa = alfa_tmp;
			l1 = l1_tmp;
			l2 = l2_tmp;
			l3 = l3_tmp;
			x2 = x2_tmp;
			y2 = y2_tmp;
			break;
		}
	}
	std::fstream outfile;
	if (result == false)
	{
		outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn_center_failed.txt", std::ios::app);
		outfile << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
		outfile << endl;
		outfile.close();
	}
	else
	{
		cout << "l1:" << l1 << " l2:" << l2 << " l3:" << l3 << " alfa:" << alfa << endl;
		control->push_back(0); control->push_back(0);
		control->push_back(l1 / 2); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(0.5f*(l1 + x2)); control->push_back(flag*0.5f*y2);
		control->push_back(x2); control->push_back(flag*y2);
		control->push_back(0.5f*(xg + x2)); control->push_back(flag*0.5f*(yg + y2));
		control->push_back(xg); control->push_back(flag*yg);
		outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn_center.txt", std::ios::app);
		outfile << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << " ";
		for (auto &i : *control)
			outfile << i << " ";
		outfile.close();
	}
	cout << endl;
	return result;
}

bool Turn_Path::turn_path_library::_turn(const vector<float> &bound_condition, vector<float> *control)
{
	float xg = bound_condition[0];
	float yg = flag*bound_condition[1];
	float thetag = flag*bound_condition[2];
	cout << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;

	bool result = false;

	if (thetag == PI / 4)
		return result;

	float alfa, l1, l2, l3, x2, y2, alfa_tmp, l1_tmp, l2_tmp, l3_tmp, x2_tmp, y2_tmp;
	float step = 0.1;
	float exceed_allow_x = 3;
	float exceed_allow_y = 5;
	for (auto a = L_min->begin(); a != L_min->end() - 4; a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = _L_min(alfa_tmp) - step;
		while (result == false)
		{
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;

			l2_tmp = (xg*sinf(thetag) - yg*cosf(thetag) - l1_tmp*sinf(thetag)) / (-cosf(alfa_tmp)*sinf(thetag) - sinf(alfa_tmp)*cosf(thetag));
			l3_tmp = (yg - l2_tmp*sinf(alfa_tmp)) / sinf(thetag);

			if (std::min(l2_tmp, l1_tmp) < *(a + 1) || std::min(l2_tmp, l3_tmp) < _L_min(2.f*PI - alfa_tmp - thetag))
				continue;

			x2_tmp = l1_tmp - l2_tmp*cosf(alfa_tmp);
			y2_tmp = l2_tmp*sinf(alfa_tmp);
			if (y2_tmp >= (yg - exceed_allow_y) || x2_tmp > (xg + exceed_allow_x))
				continue;

			result = true;
			alfa = alfa_tmp;
			l1 = l1_tmp;
			l2 = l2_tmp;
			l3 = l3_tmp;
			x2 = x2_tmp;
			y2 = y2_tmp;
			break;
		}
	}
	std::fstream outfile;
	if (result == false)
	{
		if (flag == -1)
		{
			outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_rightturn_failed.txt", std::ios::app);
			outfile << bound_condition[0] << " " << bound_condition[1] * (-1) << " " << bound_condition[2] * (-1) << endl;
		}
		else
		{
			outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn_failed.txt", std::ios::app);
			outfile << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
		}
		outfile.close();
	}
	else
	{
		cout << "l1:" << l1 << " l2:" << l2 << " l3:" << l3 << " alfa:" << alfa << endl;
		control->push_back(0); control->push_back(0);
		control->push_back(l1 / 2); control->push_back(0);
		control->push_back(l1); control->push_back(0);
		control->push_back(0.5f*(l1 + x2)); control->push_back(flag*0.5f*y2);
		control->push_back(x2); control->push_back(flag*y2);
		control->push_back(0.5f*(xg + x2)); control->push_back(flag*0.5f*(yg + y2));
		control->push_back(xg); control->push_back(flag*yg);
		if (flag == -1)
		{
			outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_rightturn.txt", std::ios::app);
			outfile << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << " ";
		}
		else
		{
			outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn.txt", std::ios::app);
			outfile << bound_condition[0] << " " << bound_condition[1] * (-1) << " " << bound_condition[2] * (-1) << " ";
		}
		for (auto &i : *control)
			outfile << i << " ";
		outfile << endl;
		outfile.close();
	}
	cout << endl;
	return result;
}

bool Turn_Path::turn_path_library::_control(const vector<float> &bound_condition, vector<float> *control)
{
	if (flag == 2)
		return _left_turn_center(bound_condition, control);
	else
		return _turn(bound_condition, control);
}

void Turn_Path::turn_path_library::add(vector<float> *bound_condition, vector<float> *control)
{
	if (_control(*bound_condition, control))
		turn_path_lib->insert(make_pair(*bound_condition, *control));
}

void Turn_Path::turn_path_library::create_lib()
{
	float X_min, X_max, Y_min, Y_max, THETA_min, THETA_max;
	if (flag == -1)
	{
		X_min = X_min_RT;
		X_max = X_max_RT;
		Y_min = Y_min_RT;
		Y_max = Y_max_RT;
		THETA_min = THETA_min_RT;
		THETA_max = THETA_max_RT;
	}
	else
	{
		X_min = X_min_LT;
		X_max = X_max_LT;
		Y_min = Y_min_LT;
		Y_max = Y_max_LT;
		THETA_min = THETA_min_LT;
		THETA_max = THETA_max_LT;
	}

	int N_x = (int)std::ceil(std::abs(X_max - X_min) / discrete_space);
	float space_x = (X_max - X_min) / N_x;
	int N_y = (int)std::ceil(std::abs(Y_max - Y_min) / discrete_space);
	float space_y = (Y_max - Y_min) / N_y;
	int N_theta = (int)std::ceil(std::abs(THETA_max - THETA_min / theta_space));
	float space_theta = (THETA_max - THETA_min) / N_theta;

	if (flag != 2)
	{
		for (int i = 0; i <= N_x; i++)
		{
			for (int j = 0; j <= N_y; j++)
			{
				for (int k = 0; k <= N_theta; k++)
				{
					vector<float> bound_condition = { space_x*i + X_min, space_y*j + Y_min, space_theta*k + THETA_min };
					vector<float> *control = new vector<float>;
					add(&bound_condition, control);
					delete control;
				}
			}
		}
	}
	else
	{
		int N_l = (int)std::ceil((L_max_LT - L_min_LT / discrete_space));
		float space_l = (L_max_LT - L_min_LT) / N_l;
		int N_w = (int)std::ceil((W_max_LT - W_min_LT / discrete_space));
		float space_w = (W_max_LT - W_min_LT) / N_w;
		int N_r = (int)std::ceil((R_max_LT - R_min_LT / discrete_space));
		float space_r = (R_max_LT - R_min_LT) / N_r;
		for (int i = 0; i <= N_x; i++)
		{
			for (int j = 0; j <= N_y; j++)
			{
				for (int k = 0; k <= N_theta; k++)
				{
					for (int l = 0; l <= N_l; l++)
					{
						for (int w = 0; w <= N_w; w++)
						{
							for (int r = 0; r <= N_r; r++)
							{
								vector<float> bound_condition = { space_x*i + X_min, space_y*j + Y_min, space_theta*k + THETA_min, l*space_l + L_min_LT, w*space_w + W_min_LT, r*space_r + R_min_LT };
								vector<float> *control = new vector<float>;
								add(&bound_condition, control);
								delete control;
							}
						}
					}
				}
			}
		}
	}
}