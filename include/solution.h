#pragma once
#include "UAV.h"
#include "Working_node.h"
#include <vector>
class Solution
{public:
	std::vector<Working_node> node_list;
	std::vector<std::vector<UAV>> UAV_list;
	float fitness=0;
	int sol_id;
	Solution() = default;
	Solution(const std::vector<Working_node>& nl,const std::vector<std::vector<UAV>>& Ul):node_list(nl),UAV_list(Ul){}
	float cal_fitness();
	UAV& id_to_UAV(int);
	void assign_node_no();
	bool compare_node(Working_node &,Working_node&);
	Working_node& id_to_node(int);
	void repair(int );
	void repair_ig(int);
	bool need_repair();
	void node_Path_node_connect();
	void UAVs_t_used_clear();
	void UAVs_Path_clear();
	void clear();
	void mutate(const std::vector<Working_node>&, int, int);
	void ls_mutate(const std::vector<Working_node>&, int, const std::vector<float*>&, int);
	bool check_node_id(int );
	bool check_Path();
	void local_search(int );
	void insert_rnd_nodes(const std::vector<Working_node>&, int);
	std::vector<int> make_uavs_assigned(Working_node, int insert_place, int col_num);
	void cal_uav_path_t();
	void clear_ext_path_node(float);
	void ls_node_exchange(std::vector<Working_node>,  int);
	void ls_path_intersect();
	int tmp_no=0;
	void ls_node_swap();
	void ls_node_move();
	std::vector<std::vector<Working_node>> sub_path_generation();
	std::vector<int> make_uavs_assigned_m(Working_node node_to_insert, int insert_place, int col_num);
	void ls_node_exchange_m(std::vector<Working_node> wn_list, int unchange_counter);
	void ls_node_del();
	bool need_repair_m();
	std::vector<int> make_uavs_assigned_r(Working_node node_to_insert, int insert_place, int col_num);
};

