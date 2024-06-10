#pragma once
#include <vector>
#include <set>
class UAV;
class Path_node;
class Working_node
{
public:
	using size_count = float;    
	float ef=0;
	float t_ll = 0;
	float pl_err = 0;
	float size_err = 0;
	float ls = 0;
	int sub_path_id=0;
	size_count node_size=0.0;
	float need=0;
	std::vector<float> coord;
	using node_type = std::set<int>;
	node_type type;
	int col_UAV_num = 0;
	int id = 0;
	int no = 0;
	Working_node() = default;
	Working_node(float x, float y, size_count s, node_type t, int col_n) :coord{ x,y }, node_size(s), type(t), col_UAV_num(col_n){}
	using time_count = float;
	time_count t_end = 0.0;
	std::vector<int> UAV_id_list;
	std::vector<int> Path_node_no_list;
	time_count cal_time_end(std::vector<UAV>&);
	time_count cal_time_end_with_complete_Path(std::vector<UAV>&);
};

class Path_node:public Working_node {
public:
	time_count t_start = 0.0;
	int node_no = 0;
	Path_node(const Working_node& wn):Working_node(wn){}
	//Path_node() = default;
};

