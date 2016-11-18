#include "U_turn.h"

bool U_Turn_Path::U_turn_path_library::_lane_control(const vector<float> &bound_condition, vector<float> *control)
{
	bool result = false;

	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];
	float l = bound_condition[3];
	float w = bound_condition[4];
	cout << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
	cout << "l:" << l << " w:" << w << endl;
	float alfa, beita, l1, l2, l3, l4, alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	float P1[2], P2[2], P3[2], P1_tmp[2], P2_tmp[2], P3_tmp[2];

	float exceed_allow_x = 8;
	float exceed_allow_y = 1;
	float step = 0.2f;
	for (auto a = L_min->begin(); a != L_min->end(); a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;
		while (result == false)
		{
			if (result == true)
				break;
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;

			P1_tmp[0] = l1_tmp;
			P1_tmp[1] = 0;

			for (auto b = L_min->begin(); b != L_min->end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = _L_min(beita_tmp) - step;
				while (1)
				{
					l4_tmp += step;
					P3_tmp[0] = xg - l4_tmp*cosf(thetag);
					P3_tmp[1] = yg - l4_tmp*sinf(thetag);
					if (P3_tmp[0] > xg + exceed_allow_x || P3_tmp[1] > exceed_allow_y + yg)
						break;

					l3_tmp = (xg*sinf(alfa_tmp) + yg*cosf(alfa_tmp) - l4_tmp*(cosf(thetag)*sinf(alfa_tmp) + sinf(thetag)*cosf(alfa_tmp) - l1_tmp*sinf(alfa_tmp)));
					l3_tmp = l3_tmp / (cosf(beita_tmp + thetag)*sinf(alfa_tmp) + sinf(beita_tmp + thetag)*cosf(alfa_tmp));
					l2_tmp = (yg - l4_tmp*sinf(thetag) - l3_tmp*sinf(beita_tmp + thetag)) / sinf(alfa_tmp);

					P2_tmp[0] = l1_tmp - l2_tmp*cosf(alfa_tmp);
					P2_tmp[1] = l2_tmp*sinf(alfa_tmp);
					if (P2_tmp[0] > xg + exceed_allow_x || P2_tmp[1] > exceed_allow_y + yg)
						continue;

					if (std::min(l1_tmp, l2_tmp) < _L_min(alfa_tmp))
						continue;

					if (std::min(l3_tmp, l4_tmp) < _L_min(beita_tmp))
						continue;

					float gama = (P1_tmp[0] - P2_tmp[0])*(P3_tmp[0] - P2_tmp[0]) + (P1_tmp[1] - P2_tmp[1])*(P3_tmp[1] - P2_tmp[1]);
					gama = gama / ((sqrtf(powf((P1_tmp[0] - P2_tmp[0]), 2) + powf((P1_tmp[1] - P2_tmp[1]), 2)))*(sqrtf(powf((P3_tmp[0] - P2_tmp[0]), 2) + powf((P3_tmp[1] - P2_tmp[1]), 2))));
					gama = acosf(gama);
					if (std::min(l2_tmp, l3_tmp) < _L_min(gama))
						continue;

					if (w < P2_tmp[2])
					{
						if ((w*tanf(alfa_tmp - PI / 2) + l1_tmp) < (l + w*0.5f / cosf(alfa_tmp - PI / 2)))
							continue;
					}
					else if (w < P3_tmp[2])
					{
						if ((P2_tmp[0] - (w - P2_tmp[1]) / tanf(gama + alfa_tmp - PI)) < (l + w*0.5f / sinf(gama + alfa_tmp - PI)))
							continue;
					}
					else if (w<yg&&w>P3_tmp[2])
					{
						if ((xg - (yg - w)*tanf(thetag)) < (l - w*0.5f / sinf(thetag)))
							continue;
					}

					result = true;
					alfa = alfa_tmp;
					beita = beita_tmp;
					l1 = l1_tmp;
					l2 = l2_tmp;
					l3 = l3_tmp;
					l4 = l4_tmp;
					P1[0] = P1_tmp[0]; P1[1] = P1_tmp[1];
					P2[0] = P2_tmp[0]; P2[1] = P2_tmp[1];
					P3[0] = P3_tmp[0]; P3[1] = P3_tmp[1];
					break;
				}
			}
		}
	}
	std::fstream outfile;
	if (result == false)
	{
		outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn_constraint_failed.txt", std::ios::app);
		for (auto &i : bound_condition)
			outfile << i << " ";
		outfile << endl;
		outfile.close();
	}
	else
	{
		cout << "l1:" << l1 << " l2:" << l2 << " l3:" << l3 << " l4:" << l4 << " alfa:" << alfa << endl;
		control->push_back(0); control->push_back(0);
		control->push_back(P1[0] / 2); control->push_back(P1[1] / 2);
		control->push_back(P1[0]); control->push_back(P1[1]);
		control->push_back(0.5f*(P1[0] + P2[0])); control->push_back(0.5f*(P1[1] + P2[1]));
		control->push_back(P2[0]); control->push_back(P2[1]);
		control->push_back(0.5f*(P2[0] + P3[0])); control->push_back(0.5f*(P2[1] + P3[1]));
		control->push_back(P3[0]); control->push_back(P3[1]);
		control->push_back(0.5f*(P3[0] + xg)); control->push_back(0.5f*(P3[1] + yg));
		control->push_back(xg); control->push_back(yg);
		outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn_constraint.txt", std::ios::app);
		for (auto &i : bound_condition)
			outfile << i << " ";
		for (auto &i : *control)
			outfile << i << " ";
		outfile << endl;
		outfile.close();
	}
	cout << endl;
	return result;
}

bool U_Turn_Path::U_turn_path_library::_no_lane_control(const vector<float> &bound_condition, vector<float> *control/*, void *data*/)
{
	float xg = bound_condition[0];
	float yg = bound_condition[1];
	float thetag = bound_condition[2];
	cout << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
	float alfa, beita, l1, l2, l3, l4, alfa_tmp, beita_tmp, l1_tmp, l2_tmp, l3_tmp, l4_tmp;
	float P1[2], P2[2], P3[2], P1_tmp[2], P2_tmp[2], P3_tmp[2];
	bool result = false;

	float exceed_allow_x = 8;
	float exceed_allow_y = 1;
	float step = 0.2f;
	for (auto a = L_min->begin(); a != L_min->end(); a += 2)
	{
		if (result == true)
			break;
		// calculate l1
		alfa_tmp = *a;
		l1_tmp = *(a + 1) - step;
		while (result == false)
		{
			if (result == true)
				break;
			l1_tmp += step;
			if (l1_tmp > xg + exceed_allow_x)
				break;

			P1_tmp[0] = l1_tmp;
			P1_tmp[1] = 0;

			for (auto b = L_min->begin(); b != L_min->end(); b += 2)
			{
				if (result == true)
					break;
				beita_tmp = *b;
				l4_tmp = _L_min(beita_tmp) - step;
				while (1)
				{
					l4_tmp += step;
					P3_tmp[0] = xg - l4_tmp*cosf(thetag);
					P3_tmp[1] = yg - l4_tmp*sinf(thetag);
					if (P3_tmp[0] > xg + exceed_allow_x || P3_tmp[1] > exceed_allow_y + yg)
						break;

					l3_tmp = (xg*sinf(alfa_tmp) + yg*cosf(alfa_tmp) - l4_tmp*(cosf(thetag)*sinf(alfa_tmp) + sinf(thetag)*cosf(alfa_tmp) - l1_tmp*sinf(alfa_tmp)));
					l3_tmp = l3_tmp / (cosf(beita_tmp + thetag)*sinf(alfa_tmp) + sinf(beita_tmp + thetag)*cosf(alfa_tmp));
					l2_tmp = (yg - l4_tmp*sinf(thetag) - l3_tmp*sinf(beita_tmp + thetag)) / sinf(alfa_tmp);
					
					P2_tmp[0] = l1_tmp - l2_tmp*cosf(alfa_tmp);
					P2_tmp[1] = l2_tmp*sinf(alfa_tmp);
					if (P2_tmp[0] > xg + exceed_allow_x || P2_tmp[1] > exceed_allow_y + yg)
						continue;
					
					if (std::min(l1_tmp, l2_tmp) < _L_min(alfa_tmp))
						continue;
					
					if (std::min(l3_tmp, l4_tmp) < _L_min(beita_tmp))
						continue;

					float gama = (P1_tmp[0] - P2_tmp[0])*(P3_tmp[0] - P2_tmp[0]) + (P1_tmp[1] - P2_tmp[1])*(P3_tmp[1] - P2_tmp[1]);
					gama = gama / ((sqrtf(powf((P1_tmp[0] - P2_tmp[0]), 2) + powf((P1_tmp[1] - P2_tmp[1]), 2)))*(sqrtf(powf((P3_tmp[0] - P2_tmp[0]), 2) + powf((P3_tmp[1] - P2_tmp[1]), 2))));
					gama = acosf(gama);
					if (std::min(l2_tmp, l3_tmp) < _L_min(gama))
						continue;

					result = true;
					alfa = alfa_tmp;
					beita = beita_tmp;
					l1 = l1_tmp;
					l2 = l2_tmp;
					l3 = l3_tmp;
					l4 = l4_tmp;
					P1[0] = P1_tmp[0]; P1[1] = P1_tmp[1];
					P2[0] = P2_tmp[0]; P2[1] = P2_tmp[1];
					P3[0] = P3_tmp[0]; P3[1] = P3_tmp[1];
					break;
				}
			}
		}
	}
	std::fstream outfile;
	if (result == false)
	{
		outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn_failed.txt", std::ios::app);
		outfile << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << endl;
		outfile.close();
	}
	else
	{
		cout << "l1:" << l1 << " l2:" << l2 << " l3:" << l3 << " l4:" << l4 << " alfa:" << alfa << endl;
		control->push_back(0); control->push_back(0);
		control->push_back(P1[0] / 2); control->push_back(P1[1] / 2);
		control->push_back(P1[0]); control->push_back(P1[1]);
		control->push_back(0.5f*(P1[0] + P2[0])); control->push_back(0.5f*(P1[1] + P2[1]));
		control->push_back(P2[0]); control->push_back(P2[1]);
		control->push_back(0.5f*(P2[0] + P3[0])); control->push_back(0.5f*(P2[1] + P3[1]));
		control->push_back(P3[0]); control->push_back(P3[1]);
		control->push_back(0.5f*(P3[0] + xg)); control->push_back(0.5f*(P3[1] + yg));
		control->push_back(xg); control->push_back(yg);
		outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn.txt", std::ios::app);
		outfile << bound_condition[0] << " " << bound_condition[1] << " " << bound_condition[2] << " ";
		for (auto &i : *control)
			outfile << i << " ";
		outfile << endl;
		outfile.close();
	}
	cout << endl;
	return result;
}

bool U_Turn_Path::U_turn_path_library::_control(const vector<float> &bound_condition, vector<float> *control)
{
	if (flag == 1)
		return _lane_control(bound_condition, control);
	else
		return _no_lane_control(bound_condition, control);
}

void U_Turn_Path::U_turn_path_library::add(vector<float> *bound_condition, vector<float> *control)
{
	if (_control(*bound_condition, control))
		U_turn_path_lib->insert(make_pair(*bound_condition, *control));
}

void U_Turn_Path::U_turn_path_library::create_lib()
{
	int N_x = (int)std::ceil((X_max_U - X_min_U) / discrete_space);
	float space_x = (X_max_U - X_min_U) / N_x;
	int N_y = (int)std::ceil((Y_max_U - Y_min_U) / discrete_space);
	float space_y = (Y_max_U - Y_min_U) / N_y;
	int N_theta = (int)std::ceil((THETA_max_U - THETA_min_U )/ theta_space);
	float space_theta = (THETA_max_U - THETA_min_U) / N_theta;

	if (flag == 1)
	{
		int N_l = (int)std::ceil((L_max_U - L_min_U / discrete_space));
		float space_l = (L_max_U - L_min_U) / N_l;
		int N_w = (int)std::ceil((W_max_U - W_min_U / discrete_space));
		float space_w = (W_max_U - W_min_U) / N_w;
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
							vector<float> bound_condition = { space_x*i + X_min_U, space_y*j + Y_min_U, space_theta*k + THETA_min_U, p*space_l + L_min_U, std::min(space_y*j + Y_min_U, q*space_w + W_min_U) };
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
					vector<float> bound_condition = { space_x*i + X_min_U, space_y*j + Y_min_U, space_theta*k + THETA_min_U };
					vector<float> *control = new vector<float>;
					add(&bound_condition, control);
					delete control;
				}
			}
		}
	}
}