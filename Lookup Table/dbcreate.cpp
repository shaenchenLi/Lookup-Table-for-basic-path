#include <iostream>
#include <fstream>
using std::cout;
using std::endl;

#include "lane_change.h"
#include "straight.h"
#include "tinysplinecpp.h"
#include "turn.h"
#include "U_turn.h"

int main()
{
	std::fstream outfile;
	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_leftturn.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_rightturn.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_leftturn_failed.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_rightturn_failed.txt", std::ios::out);
	outfile.close();
	Lane_Change_Path::lane_change_path_library lane_change_lib;
	lane_change_lib.create_lib();
	lane_change_lib.~lane_change_path_library();

	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_leftturn_constraint.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_rightturn_constraint.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_leftturn_constraint_failed.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\lane_change_rightturn_constraint_failed.txt", std::ios::out);
	outfile.close();
	Lane_Change_Path::lane_change_path_library lane_change_constraint_lib(1);
	lane_change_constraint_lib.create_lib();
	lane_change_constraint_lib.~lane_change_path_library();

	outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn_center.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn_center_failed.txt", std::ios::out);
	outfile.close();
	Turn_Path::turn_path_library turn_right_center_lib(2);
	turn_right_center_lib.create_lib();
	turn_right_center_lib.~turn_path_library();

	outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_rightturn.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_rightturn_failed.txt", std::ios::out);
	outfile.close();
	Turn_Path::turn_path_library turn_right_lib(-1);
	turn_right_lib.create_lib();
	turn_right_lib.~turn_path_library();

	outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\turn_leftturn_failed.txt", std::ios::out);
	outfile.close();
	Turn_Path::turn_path_library turn_left_lib(1);
	turn_left_lib.create_lib();
	turn_left_lib.~turn_path_library();

	outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn_failed.txt", std::ios::out);
	outfile.close();
	U_Turn_Path::U_turn_path_library U_turn_lib(0);
	U_turn_lib.create_lib();
	U_turn_lib.~U_turn_path_library();

	outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn_constraint.txt", std::ios::out);
	outfile.close();
	outfile.open("E:\\postgraduate\\codes\\database\\database\\U_turn_constraint_failed.txt", std::ios::out);
	outfile.close();
	U_Turn_Path::U_turn_path_library U_turn_constraint_lib;
	U_turn_constraint_lib.create_lib();
	U_turn_constraint_lib.~U_turn_path_library();
}