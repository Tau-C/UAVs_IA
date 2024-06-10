#pragma once
#include "Working_node.h"
#include <vector>

class UAV
{public:
	int id = 0;
	int Path_node_count=0;
	static constexpr float eff = 3;
	float speed = 10;
	float t_used = 0;
	std::vector<float> Path_node_t_list;
	int tmp_node_no;
	float tmp_t_to_node;
	double Path_change_idx=0;
	double col_length_idx= FLT_MAX;
	double con_index = 0;
	float ret_PathLength_to_node_insert(Working_node);
	float ret_PathLength_from_node_insert(Working_node);
	float ret_PathLength_from_node_j();
	using UAV_type = int;
	UAV_type type = 0;
	std::vector<Path_node> Path;
	UAV() = default;
	UAV(int t, int i) :type(t), id(i) {};
	float cal_t_go_to(const Working_node& node) const;
	float cal_t_gone(const Working_node& node) const;
	void Path_node_no_assign();
	float cal_t_to_node_no_j(int j);
	void clear_t_used();
	void erase_node_i_in_path(int);
	float cal_t_to_node_to_insert(Working_node, int insert_place);
};

