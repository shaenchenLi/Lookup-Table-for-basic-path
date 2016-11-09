#include <iostream>
#include <fstream>
using std::cout;
using std::endl;

#include "lane_change.h"
#include "straight.h"
#include "tinysplinecpp.h"

int main()
{
	Straight_Path::straight_path_library straight_path_lib;
	straight_path_lib.create_lib();

	std::ofstream outfile;
	outfile.open("E:\\postgraduate\\codes\\database\\database\\straight.txt");
	for (auto &path : (*straight_path_lib._lib()))
	{
		auto key = path.first;
		auto value = path.second;
		for (auto &k : key)
			outfile << k << " ";
		for (auto &v : value)
			outfile << v << " ";
		outfile << endl;
	}
	outfile.close();

	Lane_Change_Path::lane_change_path_library lane_change_path_lib_leftturn;
	lane_change_path_lib_leftturn.create_lib();

	Lane_Change_Path::lane_change_path_library lane_change_path_lib_rightturn;
	lane_change_path_lib_rightturn.left_right(&lane_change_path_lib_leftturn);

	outfile.open("E:\\postgraduate\\codes\\database\\database\\leftturn.txt");
	for (auto &path : (*lane_change_path_lib_leftturn._lib()))
	{
		auto key = path.first;
		auto value = path.second;
		for (auto &k : key)
			outfile << k << " ";
		for (auto &v : value)
			outfile << v << " ";
		outfile << endl;
	}
	outfile.close();

	outfile.open("E:\\postgraduate\\codes\\database\\database\\rightturn.txt");
	for (auto &path : (*lane_change_path_lib_rightturn._lib()))
	{
		auto key = path.first;
		auto value = path.second;
		for (auto &k : key)
			outfile << k << " ";
		for (auto &v : value)
			outfile << v << " ";
		outfile << endl;
	}
	outfile.close();

	//system("pause");
}