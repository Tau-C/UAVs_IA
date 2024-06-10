#include "solution.h"
#include "UAV.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include "map_config.h"
//#define rand_u
const double pi = 3.1415926535897932384626;
using namespace std;
extern float T;
extern int opt_size;
typedef float V_vec[MAP_NODE_NUM];
extern V_vec* V_matrix;
extern float MAX_MUTATE;

extern int mode;
typedef float Q_vec_1[MAP_NODE_NUM][UAV_NUM];
extern Q_vec_1* Q_table_1;
extern int mode_ch_counter;
extern int iteration_time;
extern int* min_node_col;
extern int IT;
int MAX_ACO_IT = IT;
float alpha = 3;
float beta = 1;
extern float eps;
extern int TRY_T_MAX;
int ret_GN(int N);
extern float dis_matric[MAP_NODE_NUM + 1][MAP_NODE_NUM];
float Solution::cal_fitness() {
	float all_size = 0;
	for (const auto& node : node_list) {
		all_size += node.node_size;
	}
	fitness = all_size;
	return all_size;
}

UAV& Solution::id_to_UAV(int idx) {
	for (auto& UAV_sub_list : UAV_list) {
		for (auto& uav : UAV_sub_list) {
			if (uav.id == idx) {
				return uav;
			}
		}
	}
}

void Solution::assign_node_no() {
	for (int i = 0; i < size(node_list); i++) {
		node_list[i].no = i;
	}
}

bool Solution::compare_node(Working_node& a, Working_node& b) {
	return a.no < b.no;
}

Working_node& Solution::id_to_node(int idx) {
	int flag = 0;
	for (auto& node_i : node_list) {
		if (node_i.id == idx) {
			flag = 1;
			return node_i; 
		}
	}
	if (flag == 0) {
		cout << "can't find the node" << endl;
	}
}

void Solution::node_Path_node_connect() {
	for (auto &node : node_list) {
		node.Path_node_no_list.clear();
	}
	for (Working_node& node: node_list) {
		for (auto idx : node.UAV_id_list) {
			UAV& uav = id_to_UAV(idx);
			uav.Path_node_no_assign();
			for (auto& path_node : uav.Path) {
				if (path_node.id == node.id) {
					node.Path_node_no_list.push_back(path_node.node_no);
					break;
				}
			}
		}
	}
}

bool Solution::check_node_id(int idx) {
	for (auto& node : node_list) {
		if (node.id == idx) {
			return true;
		}
	}
	return false;
}
bool Solution::check_Path() {
	for (auto& uav_l : UAV_list) {
		for (auto& uav : uav_l) {
			for (auto& p : uav.Path) {
				if (!check_node_id(p.id)) {
					return false;
				}
			}
		}
	}
	return true;
}

void Solution::cal_uav_path_t() {
	UAVs_t_used_clear();
	node_Path_node_connect();
	for (auto& node : node_list) {
		vector<UAV> UAVs_col;
		for (int idx : node.UAV_id_list) {
			UAVs_col.push_back(id_to_UAV(idx));
		}
		float t_end_node = node.cal_time_end_with_complete_Path(UAVs_col);
		node.t_end = t_end_node;
		for (auto uav_n : UAVs_col) {
			auto& uav_c = id_to_UAV(uav_n.id);
			uav_c.t_used = uav_n.t_used;
			uav_c.Path = uav_n.Path;
			uav_c.Path_node_t_list.push_back(uav_c.t_used);
		}
	}
}

//vector<int> Solution::make_uavs_assigned(Working_node node_to_insert, int insert_place, int col_num) {
//	float theta_1 = (float)rand()/ (float)RAND_MAX;
//	float theta_2 = 1-theta_1;
//
//	//float theta_1 = 0.5;
//	//float theta_2 = 0.5;
//	cal_uav_path_t();
//	vector<int> final_uav_col_id;
//	Solution sol_tmp = *this;
//	auto& node_list_to_insert = sol_tmp.node_list;
//	node_list_to_insert.insert(node_list_to_insert.begin() + insert_place, node_to_insert);
//	sol_tmp.assign_node_no();
//	auto& node_inserted = sol_tmp.node_list[insert_place];
//	auto type_allowed = node_to_insert.type;
//	vector<int> uav_allowed_id_list;
//	for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
//		for (auto& uav : UAV_list[*iter]) {
//			uav_allowed_id_list.push_back(uav.id);
//		}
//	}
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_ch = sol_tmp.id_to_UAV(idx);
//		//determine uav's insert_place in path
//		if (size(uav_ch.Path) == 0) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int flag = 0;
//		for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
//			int node_id_first = uav_ch.Path[i].id;
//			int node_id_second = uav_ch.Path[i + 1].id;
//			auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
//			auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
//			if (sol_tmp.compare_node(node_ch_first, node_inserted) && sol_tmp.compare_node(node_inserted, node_ch_second)) {
//				uav_ch.tmp_node_no = i + 1;
//				flag = 1;
//			}
//		}
//		if (flag == 1) {
//			continue;
//		}
//
//		int node_first_id = uav_ch.Path[0].id;
//		auto& node_first = sol_tmp.id_to_node(node_first_id);
//		if (sol_tmp.compare_node(node_inserted, node_first)) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
//		auto& node_last = sol_tmp.id_to_node(node_last_id);
//		if (sol_tmp.compare_node(node_last, node_inserted)) {
//			uav_ch.tmp_node_no = size(uav_ch.Path);
//			continue;
//		}
//	}
//	//cal Path_change_index, delete some uavs
//	vector<int> uav_allowed_id_list_1;
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_idx = sol_tmp.id_to_UAV(idx);
//		float P_1 = uav_idx.ret_PathLength_to_node_insert(node_inserted);
//		float P_2 = uav_idx.ret_PathLength_from_node_insert(node_inserted);
//		float P_p = uav_idx.ret_PathLength_from_node_j();
//		if ((((P_2 + P_1 - P_p) / uav_idx.speed) > T - uav_idx.t_used)) {
//			continue;
//		}
//		else {
//			uav_allowed_id_list_1.push_back(uav_idx.id);
//			if (P_p == 0) {
//				uav_idx.Path_change_idx = 1;
//			}
//			else {
//				uav_idx.Path_change_idx = (P_1 + P_2) / (P_p);
//			}
//			
//			//uav_idx.Path_change_idx = P_1;
//			if (uav_idx.tmp_node_no == 0) {
//				uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
//			}
//			else {
//				uav_idx.tmp_t_to_node = uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed;
//			}
//		}
//	}
//	if (size(uav_allowed_id_list_1) == 0)
//		return final_uav_col_id;
//	vector<UAV> uav_allowed_list;
//	for (int idx_1 : uav_allowed_id_list_1) {
//		auto& uav_1 = sol_tmp.id_to_UAV(idx_1);
//		uav_allowed_list.push_back(uav_1);
//	}
//	//sort the list according to the time to arrive node_to_insert
//	sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& uav_x, UAV& uav_y) {return uav_x.tmp_t_to_node < uav_y.tmp_t_to_node; });
//	
//	//split the uavs_list to constract the possible_col_uavs_list
//	vector<vector<int>> uavs_split_id_list;
//	for (int i = 0; i < size(uav_allowed_list); i++) {
//		vector<int> uavs_sub_id_list;
//		float first_t = uav_allowed_list[i].tmp_t_to_node;
//		uavs_sub_id_list.push_back(uav_allowed_list[i].id);
//		for (int j = i + 1; j < size(uav_allowed_list); j++) {
//			float succeed_t = uav_allowed_list[j].tmp_t_to_node;
//			if ((succeed_t - first_t) >= node_inserted.node_size / uav_allowed_list[i].speed) {
//				continue;
//			}
//			else {
//				uavs_sub_id_list.push_back(uav_allowed_list[j].id);
//			}
//		}
//		uavs_split_id_list.push_back(uavs_sub_id_list);
//	}
//	vector<vector<UAV>> final_ch_uavs_split_list;
//	vector<vector<int>> final_ch_uavs_split_id1_list;
//	int max_sub_list_size = 0;
//	int flag = 0;
//	for (auto& sub_list : uavs_split_id_list) {
//		if (size(sub_list) > col_num) {
//			flag = 1;
//			vector<UAV> tmp_uav_list;
//			for (int id : sub_list) {
//				tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//			}
//			final_ch_uavs_split_list.push_back(tmp_uav_list);
//		}
//		else {
//			max_sub_list_size = max(max_sub_list_size,(int)size(sub_list));
//		}
//	}
//	if (size(final_ch_uavs_split_list) == 0) {
//		for (auto& sub_list : uavs_split_id_list) {
//			if (size(sub_list) == max_sub_list_size) {
//				final_ch_uavs_split_id1_list.push_back(sub_list);
//			}
//		}
//		int rnd_no = rand() % size(final_ch_uavs_split_id1_list);
//		final_uav_col_id = final_ch_uavs_split_id1_list[rnd_no];
//	}
//	else {
//		for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//			for (int i = 0; i < size(final_ch_uavs_split_list[k]) - col_num + 1; i++) {
//				float first_t = final_ch_uavs_split_list[k][i].tmp_t_to_node;
//				auto& uav_f = final_ch_uavs_split_list[k][i];
//				float succeed_t = final_ch_uavs_split_list[k][i + col_num - 1].tmp_t_to_node;
//				float tmp_length = succeed_t - first_t;
//				if (tmp_length < uav_f.col_length_idx) {
//					for (int j = i; j < i + col_num; j++) {
//						final_ch_uavs_split_list[k][j].col_length_idx = tmp_length;
//					}
//				}
//				else {
//					final_ch_uavs_split_list[k][i+col_num-1].col_length_idx = tmp_length;
//				}
//			}
//		}
//		double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//		double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//				max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx,max_length_index[i]);
//				max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//			}
//		}
//		double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//				auto& uav_ij = final_ch_uavs_split_list[i][j];
//				uav_ij.con_index = theta_1 * max_Path_change_index[i] / uav_ij.Path_change_idx + theta_2 * (1+max_length_index[i]) / (1+uav_ij.col_length_idx);
//				sum_con_index[i] += uav_ij.con_index;
//			}
//		}
//		delete[] max_length_index;
//		delete[] max_Path_change_index;
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//				final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//			}
//		}
//		delete[] sum_con_index;
//		vector<vector<int>> uavs_id_list_f;
//		double sum_sum = 0;
//		double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			vector<int> uavs_id_list_f_sub;
//			set<int> rnd_no_chd_set;
//			int sum = 0;
//			while (size(rnd_no_chd_set) < col_num) {
//				double rnd_num = (double)rand() / (double)RAND_MAX;
//				double acc_pro=0;
//				int chd_j = INT_MAX;
//				for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//					acc_pro += final_ch_uavs_split_list[i][j].con_index;
//					if (acc_pro >= rnd_num) {
//						chd_j = j;
//						break;
//					}
//				}
//				if (chd_j >= size(final_ch_uavs_split_list[i])) {
//					chd_j = size(final_ch_uavs_split_list[i]) - 1;
//				}
//				if (rnd_no_chd_set.find(chd_j) == rnd_no_chd_set.end()) {
//					rnd_no_chd_set.insert(chd_j);
//					uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][chd_j].id);
//					tmp_sum_ptr[i] += final_ch_uavs_split_list[i][chd_j].con_index;
//					sum_sum += final_ch_uavs_split_list[i][chd_j].con_index;
//				}
//			}
//			uavs_id_list_f.push_back(uavs_id_list_f_sub);
//		}
//		for (int i = 0; i < size(uavs_id_list_f); i++) {
//			(*(tmp_sum_ptr + i)) /= sum_sum;
//			//cout << tmp_sum_ptr[i]<<" ";
//		}
//		//cout << endl;
//		double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//		double acc_pro = 0;
//		for (int i = 0; i < size(uavs_id_list_f); i++) {
//			acc_pro += *(tmp_sum_ptr+i);
//			if (acc_pro > rnd_pro_f) {
//				final_uav_col_id = uavs_id_list_f[i];
//				break;
//			}
//		}
//		delete[] tmp_sum_ptr;
//	}
//	return final_uav_col_id;
//}


//vector<int> Solution::make_uavs_assigned(Working_node node_to_insert, int insert_place, int col_num) {
//	//float theta_1 = (float)rand() / (float)RAND_MAX;
//	float theta_1 = 0;
//	float theta_2 = 1 - theta_1;
//
//	//float theta_1 = 0.5;
//	//float theta_2 = 0.5;
//	cal_uav_path_t();
//	vector<int> final_uav_col_id;
//	Solution sol_tmp = *this;
//	auto& node_list_to_insert = sol_tmp.node_list;
//	node_list_to_insert.insert(node_list_to_insert.begin() + insert_place, node_to_insert);
//	sol_tmp.assign_node_no();
//	auto& node_inserted = sol_tmp.node_list[insert_place];
//	auto type_allowed = node_to_insert.type;
//	vector<int> uav_allowed_id_list;
//	for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
//		for (auto& uav : UAV_list[*iter]) {
//			uav_allowed_id_list.push_back(uav.id);
//		}
//	}
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_ch = sol_tmp.id_to_UAV(idx);
//		//determine uav's insert_place in path
//		if (size(uav_ch.Path) == 0) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int flag = 0;
//		for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
//			int node_id_first = uav_ch.Path[i].id;
//			int node_id_second = uav_ch.Path[i + 1].id;
//			auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
//			auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
//			if (sol_tmp.compare_node(node_ch_first, node_inserted) && sol_tmp.compare_node(node_inserted, node_ch_second)) {
//				uav_ch.tmp_node_no = i + 1;
//				flag = 1;
//			}
//		}
//		if (flag == 1) {
//			continue;
//		}
//
//		int node_first_id = uav_ch.Path[0].id;
//		auto& node_first = sol_tmp.id_to_node(node_first_id);
//		if (sol_tmp.compare_node(node_inserted, node_first)) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
//		auto& node_last = sol_tmp.id_to_node(node_last_id);
//		if (sol_tmp.compare_node(node_last, node_inserted)) {
//			uav_ch.tmp_node_no = size(uav_ch.Path);
//			continue;
//		}
//	}
//	//cal Path_change_index, delete some uavs
//	vector<int> uav_allowed_id_list_1;
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_idx = sol_tmp.id_to_UAV(idx);
//		float P_1 = uav_idx.ret_PathLength_to_node_insert(node_inserted);
//		float P_2 = uav_idx.ret_PathLength_from_node_insert(node_inserted);
//		float P_p = uav_idx.ret_PathLength_from_node_j();
//		if ((((P_2 + P_1 - P_p) / uav_idx.speed) > T - uav_idx.t_used)) {
//			continue;
//		}
//		else {
//			uav_allowed_id_list_1.push_back(uav_idx.id);
//			if (P_p == 0) {
//				uav_idx.Path_change_idx = 1;
//			}
//			else {
//				uav_idx.Path_change_idx = (P_1 + P_2) / (P_p);
//			}
//
//			//uav_idx.Path_change_idx = P_1;
//			if (uav_idx.tmp_node_no == 0) {
//				uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
//			}
//			else {
//				uav_idx.tmp_t_to_node = uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed;
//			}
//		}
//	}
//	if (size(uav_allowed_id_list_1) == 0)
//		return final_uav_col_id;
//	vector<UAV> uav_allowed_list;
//	for (int idx_1 : uav_allowed_id_list_1) {
//		auto& uav_1 = sol_tmp.id_to_UAV(idx_1);
//		uav_allowed_list.push_back(uav_1);
//	}
//	//sort the list according to the time to arrive node_to_insert
//	sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& uav_x, UAV& uav_y) {return uav_x.tmp_t_to_node < uav_y.tmp_t_to_node; });
//
//	//split the uavs_list to constract the possible_col_uavs_list
//	vector<vector<int>> uavs_split_id_list;
//	for (int i = 0; i < size(uav_allowed_list); i++) {
//		vector<int> uavs_sub_id_list;
//		float first_t = uav_allowed_list[i].tmp_t_to_node;
//		uavs_sub_id_list.push_back(uav_allowed_list[i].id);
//		for (int j = i + 1; j < size(uav_allowed_list); j++) {
//			float succeed_t = uav_allowed_list[j].tmp_t_to_node;
//			if ((succeed_t - first_t) >= node_inserted.node_size / uav_allowed_list[i].speed) {
//				continue;
//			}
//			else {
//				uavs_sub_id_list.push_back(uav_allowed_list[j].id);
//			}
//		}
//		uavs_split_id_list.push_back(uavs_sub_id_list);
//	}
//	vector<vector<UAV>> final_ch_uavs_split_list;
//	vector<vector<int>> final_ch_uavs_split_id1_list;
//	int max_sub_list_size = 0;
//	int flag = 0;
//	for (auto& sub_list : uavs_split_id_list) {
//		if (size(sub_list) > col_num) {
//			flag = 1;
//			vector<UAV> tmp_uav_list;
//			for (int id : sub_list) {
//				tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//			}
//			final_ch_uavs_split_list.push_back(tmp_uav_list);
//		}
//		else {
//			max_sub_list_size = max(max_sub_list_size, (int)size(sub_list));
//		}
//	}
//	if (size(final_ch_uavs_split_list) == 0) {
//		for (auto& sub_list : uavs_split_id_list) {
//			if (size(sub_list) == max_sub_list_size) {
//				final_ch_uavs_split_id1_list.push_back(sub_list);
//			}
//		}
//		int rnd_no = rand() % size(final_ch_uavs_split_id1_list);
//		final_uav_col_id = final_ch_uavs_split_id1_list[rnd_no];
//	}
//	else {
//		for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//			for (int i = 0; i < size(final_ch_uavs_split_list[k]); i++) {
//				float sum_l = 0;
//				for (int j = 0; j < size(final_ch_uavs_split_list[k]); j++) {
//					sum_l += sqrt(pow((final_ch_uavs_split_list[k][i].tmp_t_to_node - final_ch_uavs_split_list[k][j].tmp_t_to_node),2));
//				}
//				final_ch_uavs_split_list[k][i].col_length_idx = sum_l / size(final_ch_uavs_split_list);
//			}
//		}
//		double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//		double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//				max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx, max_length_index[i]);
//				max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//			}
//		}
//		double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//				auto& uav_ij = final_ch_uavs_split_list[i][j];
//				uav_ij.con_index = theta_1 * max_Path_change_index[i] / uav_ij.Path_change_idx + theta_2 * (max_length_index[i]+1) / (uav_ij.col_length_idx+1);
//				sum_con_index[i] += uav_ij.con_index;
//			}
//		}
//		delete[] max_length_index;
//		delete[] max_Path_change_index;
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//				final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//			}
//		}
//		delete[] sum_con_index;
//		vector<vector<int>> uavs_id_list_f;
//		double sum_sum = 0;
//		double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//		for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//			vector<int> uavs_id_list_f_sub;
//			set<int> rnd_no_chd_set;
//			int sum = 0;
//			sort(final_ch_uavs_split_list[i].begin(), final_ch_uavs_split_list[i].end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
//			for (int con_i = 0; con_i < col_num; con_i++) {
//				uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][con_i].id);
//			}
//			/*while (size(rnd_no_chd_set) < col_num) {
//				double rnd_num = (double)rand() / (double)RAND_MAX;
//				double acc_pro = 0;
//				int chd_j = INT_MAX;
//				for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//					acc_pro += final_ch_uavs_split_list[i][j].con_index;
//					if (acc_pro >= rnd_num) {
//						chd_j = j;
//						break;
//					}
//				}
//				if (chd_j >= size(final_ch_uavs_split_list[i])) {
//					chd_j = size(final_ch_uavs_split_list[i]) - 1;
//				}
//				if (rnd_no_chd_set.find(chd_j) == rnd_no_chd_set.end()) {
//					rnd_no_chd_set.insert(chd_j);
//					uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][chd_j].id);
//					tmp_sum_ptr[i] += final_ch_uavs_split_list[i][chd_j].con_index;
//					sum_sum += final_ch_uavs_split_list[i][chd_j].con_index;
//				}
//			}*/
//			uavs_id_list_f.push_back(uavs_id_list_f_sub);
//		}
//		float min_err = FLT_MAX;
//		vector<int> uav_list_min_err;
//		float min_t = FLT_MAX;
//		for (auto uav_list_id : uavs_id_list_f) {
//			float tmp_value = node_inserted.node_size / UAV_list[0][0].eff;
//			float E_tmp=FLT_MAX;
//			vector<UAV> uav_tmp_list;
//			for (int uav_idx_i : uav_list_id) {
//				uav_tmp_list.push_back(sol_tmp.id_to_UAV(uav_idx_i));
//			}
//			sort(uav_tmp_list.begin(), uav_tmp_list.end(), [](UAV& a, UAV& b) {return a.tmp_t_to_node < b.tmp_t_to_node; });
//			int ff = 0;
//			min_t = uav_tmp_list[0].tmp_t_to_node;
//			for (auto& uav_cc : uav_tmp_list) {
//				ff++;
//				tmp_value += uav_cc.tmp_t_to_node;
//				if (E_tmp > tmp_value / ff) {
//					E_tmp = tmp_value / ff;
//				}
//				else {
//					break;
//				}
//			}
//			if (min_err > (E_tmp-min_t)) {
//				min_err = (E_tmp - min_t);
//				uav_list_min_err = uav_list_id;
//			}
//		}
//		final_uav_col_id = uav_list_min_err;
//
//
//		//for (int i = 0; i < size(uavs_id_list_f); i++) {
//		//	(*(tmp_sum_ptr + i)) /= sum_sum;
//		//	//cout << tmp_sum_ptr[i]<<" ";
//		//}
//		////cout << endl;
//		//double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//		//double acc_pro = 0;
//		//for (int i = 0; i < size(uavs_id_list_f); i++) {
//		//	acc_pro += *(tmp_sum_ptr + i);
//		//	if (acc_pro > rnd_pro_f) {
//		//		final_uav_col_id = uavs_id_list_f[i];
//		//		break;
//		//	}
//		//}
//		delete[] tmp_sum_ptr;
//	}
//	return final_uav_col_id;
//}


std::vector<std::vector<Working_node>> Solution::sub_path_generation() {
	vector<vector<Working_node>> sub_path;
	for (int i = 0; i < size(UAV_list); i++) {
		vector<Working_node> path_i;
		for (int j = 0; j < size(node_list); j++) {
			auto node_j = node_list[j];
			auto type_j = node_j.type;
			for (auto iter = type_j.begin(); iter != type_j.end(); iter++) {
				if (*iter == i) {
					float size_modify = 0;
					for (int uav_id_no = 0; uav_id_no < size(node_j.UAV_id_list); uav_id_no++) {
						auto uav_id = id_to_UAV(node_j.UAV_id_list[uav_id_no]);
						if (uav_id.type == i) {
							auto path_no = node_j.Path_node_no_list[uav_id_no];
							size_modify += uav_id.Path[path_no].t_end - uav_id.Path[path_no].t_start;
						}
					}
					size_modify *= UAV::eff;
					if (size_modify > 0) {
						path_i.push_back(node_j);
						path_i[size(path_i) - 1].node_size = size_modify;
						path_i[size(path_i) - 1].sub_path_id = i;
					}
				}
			}
		}
		sub_path.push_back(path_i);
	}
	return sub_path;
}
//vector<int> Solution::make_uavs_assigned_m(Working_node node_to_insert, int insert_place, int col_num) {
//	float theta_1 = 1;
//	//float theta_1 = 1;
//	float theta_2 = 1 - theta_1;
//
//	//float theta_1 = 0.5;
//	//float theta_2 = 0.5;
//	cal_uav_path_t();
//	vector<int> final_uav_col_id;
//	Solution sol_tmp = *this;
//	auto& node_list_to_insert = sol_tmp.node_list;
//	node_list_to_insert.insert(node_list_to_insert.begin() + insert_place, node_to_insert);
//	sol_tmp.assign_node_no();
//	auto& node_inserted = sol_tmp.node_list[insert_place];
//	auto type_allowed = node_to_insert.type;
//	vector<int> uav_allowed_id_list;
//	vector<UAV> uav_lll;
//	for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
//		for (auto& uav : UAV_list[*iter]) {
//			uav_allowed_id_list.push_back(uav.id);
//			uav_lll.push_back(uav);
//		}
//	}
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_ch = sol_tmp.id_to_UAV(idx);
//		//determine uav's insert_place in path
//		if (size(uav_ch.Path) == 0) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int flag = 0;
//		for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
//			int node_id_first = uav_ch.Path[i].id;
//			int node_id_second = uav_ch.Path[i + 1].id;
//			auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
//			auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
//			if (sol_tmp.compare_node(node_ch_first, node_inserted) && sol_tmp.compare_node(node_inserted, node_ch_second)) {
//				uav_ch.tmp_node_no = i + 1;
//				flag = 1;
//			}
//		}
//		if (flag == 1) {
//			continue;
//		}
//
//		int node_first_id = uav_ch.Path[0].id;
//		auto& node_first = sol_tmp.id_to_node(node_first_id);
//		if (sol_tmp.compare_node(node_inserted, node_first)) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
//		auto& node_last = sol_tmp.id_to_node(node_last_id);
//		if (sol_tmp.compare_node(node_last, node_inserted)) {
//			uav_ch.tmp_node_no = size(uav_ch.Path);
//			continue;
//		}
//	}
//	//cal Path_change_index, delete some uavs
//	vector<int> uav_allowed_id_list_1;
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_idx = sol_tmp.id_to_UAV(idx);
//		float P_1 = uav_idx.ret_PathLength_to_node_insert(node_inserted);
//		float P_2 = uav_idx.ret_PathLength_from_node_insert(node_inserted);
//		float P_p = uav_idx.ret_PathLength_from_node_j();
//		//if ((((P_2 + P_1 - P_p) / uav_idx.speed) > T - uav_idx.t_used)) {
//		//	continue;
//		//}
//		//else {
//		uav_allowed_id_list_1.push_back(uav_idx.id);
//
//		uav_idx.Path_change_idx = (T - uav_idx.t_used);
//
//		//uav_idx.Path_change_idx = P_1;
//		if (uav_idx.tmp_node_no == 0) {
//			uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
//		}
//		else {
//			uav_idx.tmp_t_to_node = uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed;
//		}
//		//}
//	}
//	if (size(uav_allowed_id_list_1) == 0)
//		return final_uav_col_id;
//	vector<UAV> uav_allowed_list;
//	if (size(uav_allowed_id_list_1) < col_num) {
//		set<int> uav_id_set;
//		sort(uav_lll.begin(), uav_lll.end(), [](UAV& a, UAV& b) {return (T - a.t_used) > (T - b.t_used); });
//		for (auto& uav_de : uav_lll) {
//			if (size(final_uav_col_id) == col_num) {
//				break;
//			}
//			if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
//				uav_id_set.insert(uav_de.id);
//				final_uav_col_id.push_back(uav_de.id);
//			}
//		}
//		return final_uav_col_id;
//	}
//	for (int idx_1 : uav_allowed_id_list_1) {
//		auto& uav_1 = sol_tmp.id_to_UAV(idx_1);
//		uav_allowed_list.push_back(uav_1);
//	}
//	int col_re = min(col_num, (int)size(uav_allowed_id_list_1));
//	double* max_Path_change_index = new double();
//	for (int u_i = 0; u_i < size(uav_allowed_id_list_1); u_i++) {
//		*max_Path_change_index = max(*max_Path_change_index, uav_allowed_list[u_i].Path_change_idx);
//	}
//	for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
//		uav_allowed_list[u_i].con_index = 0;
//		for (int j_i = 0; j_i < size(uav_allowed_list); j_i++) {
//			uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0];
//		}
//	}
//	delete max_Path_change_index;
//	set<int> uav_id_set;
//	sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
//	//random_shuffle(uav_allowed_list.begin(), uav_allowed_list.end());
//	for (auto& uav_de : uav_allowed_list) {
//		if (size(final_uav_col_id) == col_re) {
//			break;
//		}
//		if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
//			uav_id_set.insert(uav_de.id);
//			final_uav_col_id.push_back(uav_de.id);
//		}
//	}
//	//sort the list according to the time to arrive node_to_insert
//	//sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& uav_x, UAV& uav_y) {return uav_x.tmp_t_to_node < uav_y.tmp_t_to_node; });
//
//	////split the uavs_list to constract the possible_col_uavs_list
//	//vector<vector<int>> uavs_split_id_list;
//	//for (int i = 0; i < size(uav_allowed_list); i++) {
//	//	vector<int> uavs_sub_id_list;
//	//	float first_t = uav_allowed_list[i].tmp_t_to_node;
//	//	uavs_sub_id_list.push_back(uav_allowed_list[i].id);
//	//	for (int j = i + 1; j < size(uav_allowed_list); j++) {
//	//		float succeed_t = uav_allowed_list[j].tmp_t_to_node;
//	//		if ((succeed_t - first_t) >= node_inserted.node_size / uav_allowed_list[i].speed) {
//	//			continue;
//	//		}
//	//		else {
//	//			uavs_sub_id_list.push_back(uav_allowed_list[j].id);
//	//		}
//	//	}
//	//	uavs_split_id_list.push_back(uavs_sub_id_list);
//	//}
//	//vector<vector<UAV>> final_ch_uavs_split_list;
//	//vector<vector<int>> final_ch_uavs_split_id1_list;
//	//int max_sub_list_size = 0;
//	//int flag = 0;
//	//for (auto& sub_list : uavs_split_id_list) {
//	//	if (size(sub_list) > col_num) {
//	//		flag = 1;
//	//		vector<UAV> tmp_uav_list;
//	//		for (int id : sub_list) {
//	//			tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//	//		}
//	//		final_ch_uavs_split_list.push_back(tmp_uav_list);
//	//	}
//	//	else {
//
//	//		max_sub_list_size = max(max_sub_list_size, (int)size(sub_list));
//	//	}
//	//}
//	//if (size(final_ch_uavs_split_list) == 0) {
//	//	for (auto& sub_list : uavs_split_id_list) {
//	//		if (size(sub_list) == max_sub_list_size) {
//	//			final_ch_uavs_split_id1_list.push_back(sub_list);
//	//			vector<UAV> tmp_uav_list;
//	//			for (int id : sub_list) {
//	//				tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//	//			}
//	//			final_ch_uavs_split_list.push_back(tmp_uav_list);
//	//		}
//	//	}
//	//	for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//	//		for (int i = 0; i < size(final_ch_uavs_split_list[k]); i++) {
//	//			float sum_l = 0;
//	//			for (int j = 0; j < size(final_ch_uavs_split_list[k]); j++) {
//	//				sum_l += sqrt(pow((final_ch_uavs_split_list[k][i].tmp_t_to_node - final_ch_uavs_split_list[k][j].tmp_t_to_node), 2));
//	//			}
//	//			final_ch_uavs_split_list[k][i].col_length_idx = sum_l / size(final_ch_uavs_split_list);
//	//			if (final_ch_uavs_split_list[k][i].Path_change_idx < 0) {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = -1;
//	//			}
//	//			else {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = 1/(1+ final_ch_uavs_split_list[k][i].col_length_idx);
//	//			}
//	//		}
//	//	}
//	//	double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//	//	double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx, max_length_index[i]);
//	//			max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//	//		}
//	//	}
//	//	double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			auto& uav_ij = final_ch_uavs_split_list[i][j];
//	//			uav_ij.con_index = theta_1 * uav_ij.Path_change_idx/max_Path_change_index[i]   + theta_2 *(uav_ij.col_length_idx) /  (max_length_index[i]);
//	//			sum_con_index[i] += uav_ij.con_index;
//	//		}
//	//	}
//	//	delete[] max_length_index;
//	//	delete[] max_Path_change_index;
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//	//		}
//	//	}
//	//	delete[] sum_con_index;
//	//	vector<vector<int>> uavs_id_list_f;
//	//	double sum_sum = 0;
//	//	double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//
//	//	uavs_id_list_f= final_ch_uavs_split_id1_list;
//	//	float min_err = FLT_MAX;
//	//	vector<int> uav_list_min_err;
//	//	float min_t = FLT_MAX;
//	//	for (auto uav_list_id : uavs_id_list_f) {
//	//		//float t_delta = 0;
//	//		//float tmp_value = node_inserted.node_size / UAV_list[0][0].eff;
//	//		//float E_tmp = 0;
//	//		vector<UAV> uav_tmp_list;
//	//		for (int uav_idx_i : uav_list_id) {
//	//			uav_tmp_list.push_back(sol_tmp.id_to_UAV(uav_idx_i));
//	//		}
//	//		/*sort(uav_tmp_list.begin(), uav_tmp_list.end(), [](UAV& a, UAV& b) {return a.tmp_t_to_node < b.tmp_t_to_node; });*/
//	//		//int ff = 0;
//	//		//min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		//for (auto& uav_cc : uav_tmp_list) {
//	//		//	ff++;
//	//		//	t_delta += uav_cc.Path_change_idx;
//	//		//	tmp_value += uav_cc.tmp_t_to_node;
//	//		//	if (E_tmp > tmp_value / ff) {
//	//		//		E_tmp = tmp_value / ff;
//	//		//	}
//	//		//	else {
//	//		//		break;
//	//		//	}
//	//		//}
//	//		////if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		////	continue;
//	//		////}
//	//		//if (min_err > (E_tmp - min_t)) {
//	//		//	min_err = (E_tmp - min_t);
//	//		//	final_uav_col_id = uav_list_id;
//	//		//}
//	//	    float ff = 0;
//	//		min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		for (auto& uav_cc : uav_tmp_list) {
//	//			ff += uav_cc.con_index;
//	//		}
//	//		//if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		//	continue;
//	//		//}
//	//		if (min_err < ff) {
//	//			min_err = ff;
//	//			final_uav_col_id = uav_list_id;
//	//		}
//	//	}
//
//
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	(*(tmp_sum_ptr + i)) /= sum_sum;
//	//	//	//cout << tmp_sum_ptr[i]<<" ";
//	//	//}
//	//	////cout << endl;
//	//	//double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//	//	//double acc_pro = 0;
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	acc_pro += *(tmp_sum_ptr + i);
//	//	//	if (acc_pro > rnd_pro_f) {
//	//	//		final_uav_col_id = uavs_id_list_f[i];
//	//	//		break;
//	//	//	}
//	//	//}
//	//	delete[] tmp_sum_ptr;
//	//}
//	//else {
//	//	for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//	//		for (int i = 0; i < size(final_ch_uavs_split_list[k]); i++) {
//	//			float sum_l = 0;
//	//			for (int j = 0; j < size(final_ch_uavs_split_list[k]); j++) {
//	//				sum_l += sqrt(pow((final_ch_uavs_split_list[k][i].tmp_t_to_node - final_ch_uavs_split_list[k][j].tmp_t_to_node), 2));
//	//			}
//	//			final_ch_uavs_split_list[k][i].col_length_idx = sum_l / size(final_ch_uavs_split_list);
//	//			if (final_ch_uavs_split_list[k][i].Path_change_idx < 0) {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = -1;
//	//			}
//	//			else {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = 1 / (1 + final_ch_uavs_split_list[k][i].col_length_idx);
//	//			}
//	//		}
//	//	
//	//	}
//	//	double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//	//	double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx, max_length_index[i]);
//	//			max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//	//		}
//
//	//	}
//	//	double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			auto& uav_ij = final_ch_uavs_split_list[i][j];
//	//			uav_ij.con_index = theta_1 *  uav_ij.Path_change_idx/max_Path_change_index[i]  + theta_2 * (uav_ij.col_length_idx ) /(max_length_index[i] ) ;
//	//			sum_con_index[i] += uav_ij.con_index;
//	//		}
//
//	//	}
//	//	delete[] max_length_index;
//	//	delete[] max_Path_change_index;
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//	//		}
//	//	}
//	//	delete[] sum_con_index;
//	//	vector<vector<int>> uavs_id_list_f;
//	//	double sum_sum = 0;
//	//	double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		vector<int> uavs_id_list_f_sub;
//	//		set<int> rnd_no_chd_set;
//	//		int sum = 0;
//	//		sort(final_ch_uavs_split_list[i].begin(), final_ch_uavs_split_list[i].end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
//	//		for (int con_i = 0; con_i < col_num; con_i++) {
//	//			uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][con_i].id);
//	//		}
//	//		//while (size(rnd_no_chd_set) < col_num) {
//	//		//	double rnd_num = (double)rand() / (double)RAND_MAX;
//	//		//	double acc_pro = 0;
//	//		//	int chd_j = INT_MAX;
//	//		//	for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//		//		acc_pro += final_ch_uavs_split_list[i][j].con_index;
//	//		//		if (acc_pro >= rnd_num) {
//	//		//			chd_j = j;
//	//		//			break;
//	//		//		}
//	//		//	}
//	//		//	if (chd_j >= size(final_ch_uavs_split_list[i])) {
//	//		//		chd_j = size(final_ch_uavs_split_list[i]) - 1;
//	//		//	}
//	//		//	if (rnd_no_chd_set.find(chd_j) == rnd_no_chd_set.end()) {
//	//		//		rnd_no_chd_set.insert(chd_j);
//	//		//		uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][chd_j].id);
//	//		//		tmp_sum_ptr[i] += final_ch_uavs_split_list[i][chd_j].con_index;
//	//		//		sum_sum += final_ch_uavs_split_list[i][chd_j].con_index;
//	//		//	}
//	//		//}
//	//		uavs_id_list_f.push_back(uavs_id_list_f_sub);
//	//	}
//	//	float min_err = FLT_MAX;
//	//	vector<int> uav_list_min_err;
//	//	float min_t = FLT_MAX;
//	//	for (auto uav_list_id : uavs_id_list_f) {
//	//		float t_delta = 0;
//	//		float tmp_value = node_inserted.node_size / UAV_list[0][0].eff;
//	//		float E_tmp = FLT_MAX;
//	//		vector<UAV> uav_tmp_list;
//	//		for (int uav_idx_i : uav_list_id) {
//	//			uav_tmp_list.push_back(sol_tmp.id_to_UAV(uav_idx_i));
//	//		}
//	//		sort(uav_tmp_list.begin(), uav_tmp_list.end(), [](UAV& a, UAV& b) {return a.tmp_t_to_node < b.tmp_t_to_node; });
//	//		int ff = 0;
//	//		min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		for (auto& uav_cc : uav_tmp_list) {
//	//			ff++;
//	//			t_delta += uav_cc.Path_change_idx;
//	//			tmp_value += uav_cc.tmp_t_to_node;
//	//			if (E_tmp > tmp_value / ff) {
//	//				E_tmp = tmp_value / ff;
//	//			}
//	//			else {
//	//				break;
//	//			}
//	//		}
//	//		//if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		//	continue;
//	//		//}
//	//		if (min_err > (E_tmp - min_t)) {
//	//			min_err = (E_tmp - min_t);
//	//			final_uav_col_id = uav_list_id;
//	//		}
//	//	}
//
//
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	(*(tmp_sum_ptr + i)) /= sum_sum;
//	//	//	//cout << tmp_sum_ptr[i]<<" ";
//	//	//}
//	//	////cout << endl;
//	//	//double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//	//	//double acc_pro = 0;
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	acc_pro += *(tmp_sum_ptr + i);
//	//	//	if (acc_pro > rnd_pro_f) {
//	//	//		final_uav_col_id = uavs_id_list_f[i];
//	//	//		break;
//	//	//	}
//	//	//}
//	//	delete[] tmp_sum_ptr;
//	//}
//	return final_uav_col_id;
//}
//vector<int> Solution::make_uavs_assigned_m(Working_node node_to_insert, int insert_place, int col_num) {
//	float theta_1 = (float)rand() / (float)RAND_MAX;
//	//float theta_1 = 1;
//	float theta_2 = 1 - theta_1;
//
//	//float theta_1 = 0.5;
//	//float theta_2 = 0.5;
//	cal_uav_path_t();
//	vector<int> final_uav_col_id;
//	Solution sol_tmp = *this;
//	auto& node_list_to_insert = sol_tmp.node_list;
//	node_list_to_insert.insert(node_list_to_insert.begin() + insert_place, node_to_insert);
//	sol_tmp.assign_node_no();
//	auto& node_inserted = sol_tmp.node_list[insert_place];
//	auto type_allowed = node_to_insert.type;
//	vector<int> uav_allowed_id_list;
//	for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
//		for (auto& uav : UAV_list[*iter]) {
//			uav_allowed_id_list.push_back(uav.id);
//		}
//	}
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_ch = sol_tmp.id_to_UAV(idx);
//		//determine uav's insert_place in path
//		if (size(uav_ch.Path) == 0) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int flag = 0;
//		for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
//			int node_id_first = uav_ch.Path[i].id;
//			int node_id_second = uav_ch.Path[i + 1].id;
//			auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
//			auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
//			if (sol_tmp.compare_node(node_ch_first, node_inserted) && sol_tmp.compare_node(node_inserted, node_ch_second)) {
//				uav_ch.tmp_node_no = i + 1;
//				flag = 1;
//			}
//		}
//		if (flag == 1) {
//			continue;
//		}
//
//		int node_first_id = uav_ch.Path[0].id;
//		auto& node_first = sol_tmp.id_to_node(node_first_id);
//		if (sol_tmp.compare_node(node_inserted, node_first)) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
//		auto& node_last = sol_tmp.id_to_node(node_last_id);
//		if (sol_tmp.compare_node(node_last, node_inserted)) {
//			uav_ch.tmp_node_no = size(uav_ch.Path);
//			continue;
//		}
//	}
//	//cal Path_change_index, delete some uavs
//	vector<int> uav_allowed_id_list_1;
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_idx = sol_tmp.id_to_UAV(idx);
//		float P_1 = uav_idx.ret_PathLength_to_node_insert(node_inserted);
//		float P_2 = uav_idx.ret_PathLength_from_node_insert(node_inserted);
//		float P_p = uav_idx.ret_PathLength_from_node_j();
//		//if ((((P_2 + P_1 - P_p) / uav_idx.speed) > T - uav_idx.t_used)) {
//		//	continue;
//		//}
//		//else {
//		uav_allowed_id_list_1.push_back(uav_idx.id);
//
//		//uav_idx.Path_change_idx =( T-uav_idx.t_used);
//		/*uav_idx.col_length_idx = P_2 + P_1 - P_p;*/
//		uav_idx.Path_change_idx = P_2 + P_1 - P_p;
//		if (uav_idx.tmp_node_no == 0) {
//			uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
//		}
//		else {
//			uav_idx.tmp_t_to_node = uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed;
//		}
//		//}
//	}
//	if (size(uav_allowed_id_list_1) == 0)
//		return final_uav_col_id;
//	vector<UAV> uav_allowed_list;
//	for (int idx_1 : uav_allowed_id_list_1) {
//		auto& uav_1 = sol_tmp.id_to_UAV(idx_1);
//		uav_allowed_list.push_back(uav_1);
//	}
//	int col_re = min(col_num, (int)size(uav_allowed_id_list_1));
//	double* max_Path_change_index = new double();
//	double* max_length_index = new double();
//	double* max_t_left = new double();
//	*max_Path_change_index = FLT_MAX;
//	*max_length_index = FLT_MAX;
//	for (int u_i = 0; u_i < size(uav_allowed_id_list_1); u_i++) {
//		uav_allowed_list[u_i].col_length_idx = 0;
//		for (int j_i = 0; j_i < size(uav_allowed_list); j_i++) {
//			uav_allowed_list[u_i].col_length_idx += abs(uav_allowed_list[u_i].tmp_t_to_node - uav_allowed_list[j_i].tmp_t_to_node);
//		}
//		//if (uav_allowed_list[u_i].Path_change_idx < 0) {
//		//	uav_allowed_list[u_i].col_length_idx = -1;
//		//}
//
//		//else {
//		//	uav_allowed_list[u_i].col_length_idx = 1 / (1 + uav_allowed_list[u_i].col_length_idx);
//		//}
//		*max_Path_change_index = min(*max_Path_change_index, uav_allowed_list[u_i].Path_change_idx);
//		*max_length_index = min(*max_length_index, uav_allowed_list[u_i].col_length_idx);
//		*max_t_left = max(*max_length_index, double(T - uav_allowed_list[u_i].t_used));
//	}
//	if (col_re == 1) {
//		if (max_t_left[0] < 0) {
//			max_t_left[0] = 1;
//		}
//		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
//			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
//			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (max_length_index[0] + 1) / ((uav_allowed_list[u_i].col_length_idx) + 1);
//		}
//	}
//	else {
//		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
//			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
//			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (T - uav_allowed_list[u_i].t_used) / ((max_t_left[0]));
//		}
//	}
//	delete max_Path_change_index;
//	delete max_length_index;
//	delete max_t_left;
//	set<int> uav_id_set;
//	sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
//	for (auto& uav_de : uav_allowed_list) {
//		if (size(final_uav_col_id) == col_re) {
//			break;
//		}
//		if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
//			uav_id_set.insert(uav_de.id);
//			final_uav_col_id.push_back(uav_de.id);
//		}
//	}
//	//sort the list according to the time to arrive node_to_insert
//	//sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& uav_x, UAV& uav_y) {return uav_x.tmp_t_to_node < uav_y.tmp_t_to_node; });
//
//	////split the uavs_list to constract the possible_col_uavs_list
//	//vector<vector<int>> uavs_split_id_list;
//	//for (int i = 0; i < size(uav_allowed_list); i++) {
//	//	vector<int> uavs_sub_id_list;
//	//	float first_t = uav_allowed_list[i].tmp_t_to_node;
//	//	uavs_sub_id_list.push_back(uav_allowed_list[i].id);
//	//	for (int j = i + 1; j < size(uav_allowed_list); j++) {
//	//		float succeed_t = uav_allowed_list[j].tmp_t_to_node;
//	//		if ((succeed_t - first_t) >= node_inserted.node_size / uav_allowed_list[i].speed) {
//	//			continue;
//	//		}
//	//		else {
//	//			uavs_sub_id_list.push_back(uav_allowed_list[j].id);
//	//		}
//	//	}
//	//	uavs_split_id_list.push_back(uavs_sub_id_list);
//	//}
//	//vector<vector<UAV>> final_ch_uavs_split_list;
//	//vector<vector<int>> final_ch_uavs_split_id1_list;
//	//int max_sub_list_size = 0;
//	//int flag = 0;
//	//for (auto& sub_list : uavs_split_id_list) {
//	//	if (size(sub_list) > col_num) {
//	//		flag = 1;
//	//		vector<UAV> tmp_uav_list;
//	//		for (int id : sub_list) {
//	//			tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//	//		}
//	//		final_ch_uavs_split_list.push_back(tmp_uav_list);
//	//	}
//	//	else {
//
//	//		max_sub_list_size = max(max_sub_list_size, (int)size(sub_list));
//	//	}
//	//}
//	//if (size(final_ch_uavs_split_list) == 0) {
//	//	for (auto& sub_list : uavs_split_id_list) {
//	//		if (size(sub_list) == max_sub_list_size) {
//	//			final_ch_uavs_split_id1_list.push_back(sub_list);
//	//			vector<UAV> tmp_uav_list;
//	//			for (int id : sub_list) {
//	//				tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//	//			}
//	//			final_ch_uavs_split_list.push_back(tmp_uav_list);
//	//		}
//	//	}
//	//	for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//	//		for (int i = 0; i < size(final_ch_uavs_split_list[k]); i++) {
//	//			float sum_l = 0;
//	//			for (int j = 0; j < size(final_ch_uavs_split_list[k]); j++) {
//	//				sum_l += sqrt(pow((final_ch_uavs_split_list[k][i].tmp_t_to_node - final_ch_uavs_split_list[k][j].tmp_t_to_node), 2));
//	//			}
//	//			final_ch_uavs_split_list[k][i].col_length_idx = sum_l / size(final_ch_uavs_split_list);
//	//			if (final_ch_uavs_split_list[k][i].Path_change_idx < 0) {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = -1;
//	//			}
//	//			else {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = 1/(1+ final_ch_uavs_split_list[k][i].col_length_idx);
//	//			}
//	//		}
//	//	}
//	//	double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//	//	double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx, max_length_index[i]);
//	//			max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//	//		}
//	//	}
//	//	double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			auto& uav_ij = final_ch_uavs_split_list[i][j];
//	//			uav_ij.con_index = theta_1 * uav_ij.Path_change_idx/max_Path_change_index[i]   + theta_2 *(uav_ij.col_length_idx) /  (max_length_index[i]);
//	//			sum_con_index[i] += uav_ij.con_index;
//	//		}
//	//	}
//	//	delete[] max_length_index;
//	//	delete[] max_Path_change_index;
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//	//		}
//	//	}
//	//	delete[] sum_con_index;
//	//	vector<vector<int>> uavs_id_list_f;
//	//	double sum_sum = 0;
//	//	double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//
//	//	uavs_id_list_f= final_ch_uavs_split_id1_list;
//	//	float min_err = FLT_MAX;
//	//	vector<int> uav_list_min_err;
//	//	float min_t = FLT_MAX;
//	//	for (auto uav_list_id : uavs_id_list_f) {
//	//		//float t_delta = 0;
//	//		//float tmp_value = node_inserted.node_size / UAV_list[0][0].eff;
//	//		//float E_tmp = 0;
//	//		vector<UAV> uav_tmp_list;
//	//		for (int uav_idx_i : uav_list_id) {
//	//			uav_tmp_list.push_back(sol_tmp.id_to_UAV(uav_idx_i));
//	//		}
//	//		/*sort(uav_tmp_list.begin(), uav_tmp_list.end(), [](UAV& a, UAV& b) {return a.tmp_t_to_node < b.tmp_t_to_node; });*/
//	//		//int ff = 0;
//	//		//min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		//for (auto& uav_cc : uav_tmp_list) {
//	//		//	ff++;
//	//		//	t_delta += uav_cc.Path_change_idx;
//	//		//	tmp_value += uav_cc.tmp_t_to_node;
//	//		//	if (E_tmp > tmp_value / ff) {
//	//		//		E_tmp = tmp_value / ff;
//	//		//	}
//	//		//	else {
//	//		//		break;
//	//		//	}
//	//		//}
//	//		////if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		////	continue;
//	//		////}
//	//		//if (min_err > (E_tmp - min_t)) {
//	//		//	min_err = (E_tmp - min_t);
//	//		//	final_uav_col_id = uav_list_id;
//	//		//}
//	//	    float ff = 0;
//	//		min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		for (auto& uav_cc : uav_tmp_list) {
//	//			ff += uav_cc.con_index;
//	//		}
//	//		//if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		//	continue;
//	//		//}
//	//		if (min_err < ff) {
//	//			min_err = ff;
//	//			final_uav_col_id = uav_list_id;
//	//		}
//	//	}
//
//
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	(*(tmp_sum_ptr + i)) /= sum_sum;
//	//	//	//cout << tmp_sum_ptr[i]<<" ";
//	//	//}
//	//	////cout << endl;
//	//	//double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//	//	//double acc_pro = 0;
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	acc_pro += *(tmp_sum_ptr + i);
//	//	//	if (acc_pro > rnd_pro_f) {
//	//	//		final_uav_col_id = uavs_id_list_f[i];
//	//	//		break;
//	//	//	}
//	//	//}
//	//	delete[] tmp_sum_ptr;
//	//}
//	//else {
//	//	for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//	//		for (int i = 0; i < size(final_ch_uavs_split_list[k]); i++) {
//	//			float sum_l = 0;
//	//			for (int j = 0; j < size(final_ch_uavs_split_list[k]); j++) {
//	//				sum_l += sqrt(pow((final_ch_uavs_split_list[k][i].tmp_t_to_node - final_ch_uavs_split_list[k][j].tmp_t_to_node), 2));
//	//			}
//	//			final_ch_uavs_split_list[k][i].col_length_idx = sum_l / size(final_ch_uavs_split_list);
//	//			if (final_ch_uavs_split_list[k][i].Path_change_idx < 0) {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = -1;
//	//			}
//	//			else {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = 1 / (1 + final_ch_uavs_split_list[k][i].col_length_idx);
//	//			}
//	//		}
//	//	
//	//	}
//	//	double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//	//	double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx, max_length_index[i]);
//	//			max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//	//		}
//
//	//	}
//	//	double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			auto& uav_ij = final_ch_uavs_split_list[i][j];
//	//			uav_ij.con_index = theta_1 *  uav_ij.Path_change_idx/max_Path_change_index[i]  + theta_2 * (uav_ij.col_length_idx ) /(max_length_index[i] ) ;
//	//			sum_con_index[i] += uav_ij.con_index;
//	//		}
//
//	//	}
//	//	delete[] max_length_index;
//	//	delete[] max_Path_change_index;
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//	//		}
//	//	}
//	//	delete[] sum_con_index;
//	//	vector<vector<int>> uavs_id_list_f;
//	//	double sum_sum = 0;
//	//	double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		vector<int> uavs_id_list_f_sub;
//	//		set<int> rnd_no_chd_set;
//	//		int sum = 0;
//	//		sort(final_ch_uavs_split_list[i].begin(), final_ch_uavs_split_list[i].end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
//	//		for (int con_i = 0; con_i < col_num; con_i++) {
//	//			uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][con_i].id);
//	//		}
//	//		//while (size(rnd_no_chd_set) < col_num) {
//	//		//	double rnd_num = (double)rand() / (double)RAND_MAX;
//	//		//	double acc_pro = 0;
//	//		//	int chd_j = INT_MAX;
//	//		//	for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//		//		acc_pro += final_ch_uavs_split_list[i][j].con_index;
//	//		//		if (acc_pro >= rnd_num) {
//	//		//			chd_j = j;
//	//		//			break;
//	//		//		}
//	//		//	}
//	//		//	if (chd_j >= size(final_ch_uavs_split_list[i])) {
//	//		//		chd_j = size(final_ch_uavs_split_list[i]) - 1;
//	//		//	}
//	//		//	if (rnd_no_chd_set.find(chd_j) == rnd_no_chd_set.end()) {
//	//		//		rnd_no_chd_set.insert(chd_j);
//	//		//		uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][chd_j].id);
//	//		//		tmp_sum_ptr[i] += final_ch_uavs_split_list[i][chd_j].con_index;
//	//		//		sum_sum += final_ch_uavs_split_list[i][chd_j].con_index;
//	//		//	}
//	//		//}
//	//		uavs_id_list_f.push_back(uavs_id_list_f_sub);
//	//	}
//	//	float min_err = FLT_MAX;
//	//	vector<int> uav_list_min_err;
//	//	float min_t = FLT_MAX;
//	//	for (auto uav_list_id : uavs_id_list_f) {
//	//		float t_delta = 0;
//	//		float tmp_value = node_inserted.node_size / UAV_list[0][0].eff;
//	//		float E_tmp = FLT_MAX;
//	//		vector<UAV> uav_tmp_list;
//	//		for (int uav_idx_i : uav_list_id) {
//	//			uav_tmp_list.push_back(sol_tmp.id_to_UAV(uav_idx_i));
//	//		}
//	//		sort(uav_tmp_list.begin(), uav_tmp_list.end(), [](UAV& a, UAV& b) {return a.tmp_t_to_node < b.tmp_t_to_node; });
//	//		int ff = 0;
//	//		min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		for (auto& uav_cc : uav_tmp_list) {
//	//			ff++;
//	//			t_delta += uav_cc.Path_change_idx;
//	//			tmp_value += uav_cc.tmp_t_to_node;
//	//			if (E_tmp > tmp_value / ff) {
//	//				E_tmp = tmp_value / ff;
//	//			}
//	//			else {
//	//				break;
//	//			}
//	//		}
//	//		//if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		//	continue;
//	//		//}
//	//		if (min_err > (E_tmp - min_t)) {
//	//			min_err = (E_tmp - min_t);
//	//			final_uav_col_id = uav_list_id;
//	//		}
//	//	}
//
//
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	(*(tmp_sum_ptr + i)) /= sum_sum;
//	//	//	//cout << tmp_sum_ptr[i]<<" ";
//	//	//}
//	//	////cout << endl;
//	//	//double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//	//	//double acc_pro = 0;
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	acc_pro += *(tmp_sum_ptr + i);
//	//	//	if (acc_pro > rnd_pro_f) {
//	//	//		final_uav_col_id = uavs_id_list_f[i];
//	//	//		break;
//	//	//	}
//	//	//}
//	//	delete[] tmp_sum_ptr;
//	//}
//	return final_uav_col_id;
//}
vector<int> Solution::make_uavs_assigned_m(Working_node node_to_insert, int insert_place, int col_num) {
	float theta_1 = (float)rand() / (float)RAND_MAX;
	//float theta_1 = 1;
	float theta_2 = 1 - theta_1;

	//float theta_1 = 0.5;
	//float theta_2 = 0.5;
	cal_uav_path_t();
	vector<int> final_uav_col_id;
	Solution sol_tmp = *this;
	auto& node_list_to_insert = sol_tmp.node_list;
	node_list_to_insert.insert(node_list_to_insert.begin() + insert_place, node_to_insert);
	sol_tmp.assign_node_no();
	auto& node_inserted = sol_tmp.node_list[insert_place];
	auto type_allowed = node_to_insert.type;
	vector<int> uav_allowed_id_list;
	for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
		for (auto& uav : UAV_list[*iter]) {
			uav_allowed_id_list.push_back(uav.id);
		}
	}
	for (int idx : uav_allowed_id_list) {
		auto& uav_ch = sol_tmp.id_to_UAV(idx);
		//determine uav's insert_place in path
		if (size(uav_ch.Path) == 0) {
			uav_ch.tmp_node_no = 0;
			continue;
		}
		int flag = 0;
		for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
			int node_id_first = uav_ch.Path[i].id;
			int node_id_second = uav_ch.Path[i + 1].id;
			auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
			auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
			if (sol_tmp.compare_node(node_ch_first, node_inserted) && sol_tmp.compare_node(node_inserted, node_ch_second)) {
				uav_ch.tmp_node_no = i + 1;
				flag = 1;
			}
		}
		if (flag == 1) {
			continue;
		}

		int node_first_id = uav_ch.Path[0].id;
		auto& node_first = sol_tmp.id_to_node(node_first_id);
		if (sol_tmp.compare_node(node_inserted, node_first)) {
			uav_ch.tmp_node_no = 0;
			continue;
		}
		int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
		auto& node_last = sol_tmp.id_to_node(node_last_id);
		if (sol_tmp.compare_node(node_last, node_inserted)) {
			uav_ch.tmp_node_no = size(uav_ch.Path);
			continue;
		}
	}
	//cal Path_change_index, delete some uavs
	vector<int> uav_allowed_id_list_1;
	for (int idx : uav_allowed_id_list) {
		auto& uav_idx = sol_tmp.id_to_UAV(idx);
		float P_1 = uav_idx.ret_PathLength_to_node_insert(node_inserted);
		float P_2 = uav_idx.ret_PathLength_from_node_insert(node_inserted);
		float P_p = uav_idx.ret_PathLength_from_node_j();
		//if ((((P_2 + P_1 - P_p) / uav_idx.speed) > T - uav_idx.t_used)) {
		//	continue;
		//}
		//else {
		uav_allowed_id_list_1.push_back(uav_idx.id);

		//uav_idx.Path_change_idx =( T-uav_idx.t_used);
		/*uav_idx.col_length_idx = P_2 + P_1 - P_p;*/
		uav_idx.Path_change_idx = P_2 + P_1 - P_p;
		if (uav_idx.tmp_node_no == 0) {
			uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
		}
		else {
			uav_idx.tmp_t_to_node = uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed;
		}
		//}
	}
	if (size(uav_allowed_id_list_1) == 0)
		return final_uav_col_id;
	vector<UAV> uav_allowed_list;
	for (int idx_1 : uav_allowed_id_list_1) {
		auto& uav_1 = sol_tmp.id_to_UAV(idx_1);
		uav_allowed_list.push_back(uav_1);
	}
	int col_re = min(col_num, (int)size(uav_allowed_id_list_1));
	double* max_Path_change_index = new double();
	double* max_length_index = new double();
	double* max_t_left = new double();
	*max_Path_change_index = FLT_MAX;
	*max_length_index = FLT_MAX;
	for (int u_i = 0; u_i < size(uav_allowed_id_list_1); u_i++) {
		uav_allowed_list[u_i].col_length_idx = 0;
		for (int j_i = 0; j_i < size(uav_allowed_list); j_i++) {
			uav_allowed_list[u_i].col_length_idx += abs(uav_allowed_list[u_i].tmp_t_to_node - uav_allowed_list[j_i].tmp_t_to_node);
		}
		//if (uav_allowed_list[u_i].Path_change_idx < 0) {
		//	uav_allowed_list[u_i].col_length_idx = -1;
		//}

		//else {
		//	uav_allowed_list[u_i].col_length_idx = 1 / (1 + uav_allowed_list[u_i].col_length_idx);
		//}
		*max_Path_change_index = min(*max_Path_change_index, uav_allowed_list[u_i].Path_change_idx);
		*max_length_index = min(*max_length_index, uav_allowed_list[u_i].col_length_idx);
		*max_t_left = max(*max_length_index, double(T - uav_allowed_list[u_i].t_used));
	}
	if (col_re != 1) {
		if (max_t_left[0] < 0) {
			max_t_left[0] = 1;
		}
		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (max_length_index[0] + 1) / ((uav_allowed_list[u_i].col_length_idx) + 1);
		}
	}
	else {
		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (T - uav_allowed_list[u_i].t_used) / ((max_t_left[0]));
		}
	}
	delete max_Path_change_index;
	delete max_length_index;
	delete max_t_left;
	set<int> uav_id_set;
	sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
	for (auto& uav_de : uav_allowed_list) {
		if (size(final_uav_col_id) == col_re) {
			break;
		}
		if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
			uav_id_set.insert(uav_de.id);
			final_uav_col_id.push_back(uav_de.id);
		}
	}
	return final_uav_col_id;
}
//vector<int> Solution::make_uavs_assigned(Working_node node_to_insert, int insert_place, int col_num) {
//	float theta_1 = (float)rand() / (float)RAND_MAX;
//	//float theta_1 = 1;
//	float theta_2 = 1 - theta_1;
//
//	//float theta_1 = 0.5;
//	//float theta_2 = 0.5;
//	cal_uav_path_t();
//	vector<int> final_uav_col_id;
//	Solution sol_tmp = *this;
//	auto& node_list_to_insert = sol_tmp.node_list;
//	node_list_to_insert.insert(node_list_to_insert.begin() + insert_place, node_to_insert);
//	sol_tmp.assign_node_no();
//	auto& node_inserted = sol_tmp.node_list[insert_place];
//	auto type_allowed = node_to_insert.type;
//	vector<int> uav_allowed_id_list;
//	for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
//		for (auto& uav : UAV_list[*iter]) {
//			uav_allowed_id_list.push_back(uav.id);
//		}
//	}
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_ch = sol_tmp.id_to_UAV(idx);
//		//determine uav's insert_place in path
//		if (size(uav_ch.Path) == 0) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int flag = 0;
//		for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
//			int node_id_first = uav_ch.Path[i].id;
//			int node_id_second = uav_ch.Path[i + 1].id;
//			auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
//			auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
//			if (sol_tmp.compare_node(node_ch_first, node_inserted) && sol_tmp.compare_node(node_inserted, node_ch_second)) {
//				uav_ch.tmp_node_no = i + 1;
//				flag = 1;
//			}
//		}
//		if (flag == 1) {
//			continue;
//		}
//
//		int node_first_id = uav_ch.Path[0].id;
//		auto& node_first = sol_tmp.id_to_node(node_first_id);
//		if (sol_tmp.compare_node(node_inserted, node_first)) {
//			uav_ch.tmp_node_no = 0;
//			continue;
//		}
//		int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
//		auto& node_last = sol_tmp.id_to_node(node_last_id);
//		if (sol_tmp.compare_node(node_last, node_inserted)) {
//			uav_ch.tmp_node_no = size(uav_ch.Path);
//			continue;
//		}
//	}
//	//cal Path_change_index, delete some uavs
//	vector<int> uav_allowed_id_list_1;
//	for (int idx : uav_allowed_id_list) {
//		auto& uav_idx = sol_tmp.id_to_UAV(idx);
//		float P_1 = uav_idx.ret_PathLength_to_node_insert(node_inserted);
//		float P_2 = uav_idx.ret_PathLength_from_node_insert(node_inserted);
//		float P_p = uav_idx.ret_PathLength_from_node_j();
//		if ((((P_2 + P_1 - P_p) / uav_idx.speed) > T - uav_idx.t_used)) {
//			continue;
//		}
//		else {
//			uav_allowed_id_list_1.push_back(uav_idx.id);
//		
//			//uav_idx.Path_change_idx =( T-uav_idx.t_used);
//			/*uav_idx.col_length_idx = P_2 + P_1 - P_p;*/
//			uav_idx.Path_change_idx = P_2 + P_1 - P_p;
//			if (uav_idx.tmp_node_no == 0) {
//				uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
//			}
//			else {
//				uav_idx.tmp_t_to_node = uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed;
//			}		
//		}
//	}
//	if (size(uav_allowed_id_list_1) == 0)
//		return final_uav_col_id;
//	vector<UAV> uav_allowed_list;
//	for (int idx_1 : uav_allowed_id_list_1) {
//		auto& uav_1 = sol_tmp.id_to_UAV(idx_1);
//		uav_allowed_list.push_back(uav_1);
//	}
//	int col_re = min(col_num, (int)size(uav_allowed_id_list_1));
//	double* max_Path_change_index = new double();
//	double* max_length_index = new double();
//	double* max_t_left = new double();
//	*max_Path_change_index = FLT_MAX;
//	*max_length_index = FLT_MAX;
//	for (int u_i = 0; u_i < size(uav_allowed_id_list_1); u_i++) {
//		uav_allowed_list[u_i].col_length_idx = 0;
//		for (int j_i = 0; j_i < size(uav_allowed_list); j_i++) {
//			uav_allowed_list[u_i].col_length_idx += abs(uav_allowed_list[u_i].tmp_t_to_node - uav_allowed_list[j_i].tmp_t_to_node);
//		}
//		//if (uav_allowed_list[u_i].Path_change_idx < 0) {
//		//	uav_allowed_list[u_i].col_length_idx = -1;
//		//}
//
//		//else {
//		//	uav_allowed_list[u_i].col_length_idx = 1 / (1 + uav_allowed_list[u_i].col_length_idx);
//		//}
//		*max_Path_change_index = min(*max_Path_change_index, uav_allowed_list[u_i].Path_change_idx);
//		*max_length_index = min(*max_length_index, uav_allowed_list[u_i].col_length_idx);
//		*max_t_left = max(*max_length_index, double(T - uav_allowed_list[u_i].t_used));
//	}
//	if (col_re == 1) {
//		if (max_t_left[0] < 0) {
//			max_t_left[0] = 1;
//		}
//		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
//			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
//			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (max_length_index[0]+1) / ((uav_allowed_list[u_i].col_length_idx)+1);
//		}
//	}
//	else {
//		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
//			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
//			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (T-uav_allowed_list[u_i].t_used) / ((max_t_left[0]));
//		}
//	}
//	delete max_Path_change_index;
//	delete max_length_index;
//	delete max_t_left;
//	set<int> uav_id_set;
//	sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
//	for (auto& uav_de : uav_allowed_list) {
//		if (size(final_uav_col_id) == col_re) {
//			break;
//		}
//		if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
//			uav_id_set.insert(uav_de.id);
//			final_uav_col_id.push_back(uav_de.id);
//		}
//	}
//	//sort the list according to the time to arrive node_to_insert
//	//sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& uav_x, UAV& uav_y) {return uav_x.tmp_t_to_node < uav_y.tmp_t_to_node; });
//
//	////split the uavs_list to constract the possible_col_uavs_list
//	//vector<vector<int>> uavs_split_id_list;
//	//for (int i = 0; i < size(uav_allowed_list); i++) {
//	//	vector<int> uavs_sub_id_list;
//	//	float first_t = uav_allowed_list[i].tmp_t_to_node;
//	//	uavs_sub_id_list.push_back(uav_allowed_list[i].id);
//	//	for (int j = i + 1; j < size(uav_allowed_list); j++) {
//	//		float succeed_t = uav_allowed_list[j].tmp_t_to_node;
//	//		if ((succeed_t - first_t) >= node_inserted.node_size / uav_allowed_list[i].speed) {
//	//			continue;
//	//		}
//	//		else {
//	//			uavs_sub_id_list.push_back(uav_allowed_list[j].id);
//	//		}
//	//	}
//	//	uavs_split_id_list.push_back(uavs_sub_id_list);
//	//}
//	//vector<vector<UAV>> final_ch_uavs_split_list;
//	//vector<vector<int>> final_ch_uavs_split_id1_list;
//	//int max_sub_list_size = 0;
//	//int flag = 0;
//	//for (auto& sub_list : uavs_split_id_list) {
//	//	if (size(sub_list) > col_num) {
//	//		flag = 1;
//	//		vector<UAV> tmp_uav_list;
//	//		for (int id : sub_list) {
//	//			tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//	//		}
//	//		final_ch_uavs_split_list.push_back(tmp_uav_list);
//	//	}
//	//	else {
//
//	//		max_sub_list_size = max(max_sub_list_size, (int)size(sub_list));
//	//	}
//	//}
//	//if (size(final_ch_uavs_split_list) == 0) {
//	//	for (auto& sub_list : uavs_split_id_list) {
//	//		if (size(sub_list) == max_sub_list_size) {
//	//			final_ch_uavs_split_id1_list.push_back(sub_list);
//	//			vector<UAV> tmp_uav_list;
//	//			for (int id : sub_list) {
//	//				tmp_uav_list.push_back(sol_tmp.id_to_UAV(id));
//	//			}
//	//			final_ch_uavs_split_list.push_back(tmp_uav_list);
//	//		}
//	//	}
//	//	for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//	//		for (int i = 0; i < size(final_ch_uavs_split_list[k]); i++) {
//	//			float sum_l = 0;
//	//			for (int j = 0; j < size(final_ch_uavs_split_list[k]); j++) {
//	//				sum_l += sqrt(pow((final_ch_uavs_split_list[k][i].tmp_t_to_node - final_ch_uavs_split_list[k][j].tmp_t_to_node), 2));
//	//			}
//	//			final_ch_uavs_split_list[k][i].col_length_idx = sum_l / size(final_ch_uavs_split_list);
//	//			if (final_ch_uavs_split_list[k][i].Path_change_idx < 0) {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = -1;
//	//			}
//	//			else {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = 1/(1+ final_ch_uavs_split_list[k][i].col_length_idx);
//	//			}
//	//		}
//	//	}
//	//	double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//	//	double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx, max_length_index[i]);
//	//			max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//	//		}
//	//	}
//	//	double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			auto& uav_ij = final_ch_uavs_split_list[i][j];
//	//			uav_ij.con_index = theta_1 * uav_ij.Path_change_idx/max_Path_change_index[i]   + theta_2 *(uav_ij.col_length_idx) /  (max_length_index[i]);
//	//			sum_con_index[i] += uav_ij.con_index;
//	//		}
//	//	}
//	//	delete[] max_length_index;
//	//	delete[] max_Path_change_index;
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//	//		}
//	//	}
//	//	delete[] sum_con_index;
//	//	vector<vector<int>> uavs_id_list_f;
//	//	double sum_sum = 0;
//	//	double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//
//	//	uavs_id_list_f= final_ch_uavs_split_id1_list;
//	//	float min_err = FLT_MAX;
//	//	vector<int> uav_list_min_err;
//	//	float min_t = FLT_MAX;
//	//	for (auto uav_list_id : uavs_id_list_f) {
//	//		//float t_delta = 0;
//	//		//float tmp_value = node_inserted.node_size / UAV_list[0][0].eff;
//	//		//float E_tmp = 0;
//	//		vector<UAV> uav_tmp_list;
//	//		for (int uav_idx_i : uav_list_id) {
//	//			uav_tmp_list.push_back(sol_tmp.id_to_UAV(uav_idx_i));
//	//		}
//	//		/*sort(uav_tmp_list.begin(), uav_tmp_list.end(), [](UAV& a, UAV& b) {return a.tmp_t_to_node < b.tmp_t_to_node; });*/
//	//		//int ff = 0;
//	//		//min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		//for (auto& uav_cc : uav_tmp_list) {
//	//		//	ff++;
//	//		//	t_delta += uav_cc.Path_change_idx;
//	//		//	tmp_value += uav_cc.tmp_t_to_node;
//	//		//	if (E_tmp > tmp_value / ff) {
//	//		//		E_tmp = tmp_value / ff;
//	//		//	}
//	//		//	else {
//	//		//		break;
//	//		//	}
//	//		//}
//	//		////if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		////	continue;
//	//		////}
//	//		//if (min_err > (E_tmp - min_t)) {
//	//		//	min_err = (E_tmp - min_t);
//	//		//	final_uav_col_id = uav_list_id;
//	//		//}
//	//	    float ff = 0;
//	//		min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		for (auto& uav_cc : uav_tmp_list) {
//	//			ff += uav_cc.con_index;
//	//		}
//	//		//if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		//	continue;
//	//		//}
//	//		if (min_err < ff) {
//	//			min_err = ff;
//	//			final_uav_col_id = uav_list_id;
//	//		}
//	//	}
//
//
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	(*(tmp_sum_ptr + i)) /= sum_sum;
//	//	//	//cout << tmp_sum_ptr[i]<<" ";
//	//	//}
//	//	////cout << endl;
//	//	//double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//	//	//double acc_pro = 0;
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	acc_pro += *(tmp_sum_ptr + i);
//	//	//	if (acc_pro > rnd_pro_f) {
//	//	//		final_uav_col_id = uavs_id_list_f[i];
//	//	//		break;
//	//	//	}
//	//	//}
//	//	delete[] tmp_sum_ptr;
//	//}
//	//else {
//	//	for (int k = 0; k < size(final_ch_uavs_split_list); k++) {
//	//		for (int i = 0; i < size(final_ch_uavs_split_list[k]); i++) {
//	//			float sum_l = 0;
//	//			for (int j = 0; j < size(final_ch_uavs_split_list[k]); j++) {
//	//				sum_l += sqrt(pow((final_ch_uavs_split_list[k][i].tmp_t_to_node - final_ch_uavs_split_list[k][j].tmp_t_to_node), 2));
//	//			}
//	//			final_ch_uavs_split_list[k][i].col_length_idx = sum_l / size(final_ch_uavs_split_list);
//	//			if (final_ch_uavs_split_list[k][i].Path_change_idx < 0) {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = -1;
//	//			}
//	//			else {
//	//				final_ch_uavs_split_list[k][i].col_length_idx = 1 / (1 + final_ch_uavs_split_list[k][i].col_length_idx);
//	//			}
//	//		}
//	//	
//	//	}
//	//	double* max_length_index = new double[size(final_ch_uavs_split_list)]();
//	//	double* max_Path_change_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			max_length_index[i] = max(final_ch_uavs_split_list[i][j].col_length_idx, max_length_index[i]);
//	//			max_Path_change_index[i] = max(final_ch_uavs_split_list[i][j].Path_change_idx, max_Path_change_index[i]);
//	//		}
//
//	//	}
//	//	double* sum_con_index = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			auto& uav_ij = final_ch_uavs_split_list[i][j];
//	//			uav_ij.con_index = theta_1 *  uav_ij.Path_change_idx/max_Path_change_index[i]  + theta_2 * (uav_ij.col_length_idx ) /(max_length_index[i] ) ;
//	//			sum_con_index[i] += uav_ij.con_index;
//	//		}
//
//	//	}
//	//	delete[] max_length_index;
//	//	delete[] max_Path_change_index;
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//			final_ch_uavs_split_list[i][j].con_index /= sum_con_index[i];
//	//		}
//	//	}
//	//	delete[] sum_con_index;
//	//	vector<vector<int>> uavs_id_list_f;
//	//	double sum_sum = 0;
//	//	double* tmp_sum_ptr = new double[size(final_ch_uavs_split_list)]();
//	//	for (int i = 0; i < size(final_ch_uavs_split_list); i++) {
//	//		vector<int> uavs_id_list_f_sub;
//	//		set<int> rnd_no_chd_set;
//	//		int sum = 0;
//	//		sort(final_ch_uavs_split_list[i].begin(), final_ch_uavs_split_list[i].end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
//	//		for (int con_i = 0; con_i < col_num; con_i++) {
//	//			uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][con_i].id);
//	//		}
//	//		//while (size(rnd_no_chd_set) < col_num) {
//	//		//	double rnd_num = (double)rand() / (double)RAND_MAX;
//	//		//	double acc_pro = 0;
//	//		//	int chd_j = INT_MAX;
//	//		//	for (int j = 0; j < size(final_ch_uavs_split_list[i]); j++) {
//	//		//		acc_pro += final_ch_uavs_split_list[i][j].con_index;
//	//		//		if (acc_pro >= rnd_num) {
//	//		//			chd_j = j;
//	//		//			break;
//	//		//		}
//	//		//	}
//	//		//	if (chd_j >= size(final_ch_uavs_split_list[i])) {
//	//		//		chd_j = size(final_ch_uavs_split_list[i]) - 1;
//	//		//	}
//	//		//	if (rnd_no_chd_set.find(chd_j) == rnd_no_chd_set.end()) {
//	//		//		rnd_no_chd_set.insert(chd_j);
//	//		//		uavs_id_list_f_sub.push_back(final_ch_uavs_split_list[i][chd_j].id);
//	//		//		tmp_sum_ptr[i] += final_ch_uavs_split_list[i][chd_j].con_index;
//	//		//		sum_sum += final_ch_uavs_split_list[i][chd_j].con_index;
//	//		//	}
//	//		//}
//	//		uavs_id_list_f.push_back(uavs_id_list_f_sub);
//	//	}
//	//	float min_err = FLT_MAX;
//	//	vector<int> uav_list_min_err;
//	//	float min_t = FLT_MAX;
//	//	for (auto uav_list_id : uavs_id_list_f) {
//	//		float t_delta = 0;
//	//		float tmp_value = node_inserted.node_size / UAV_list[0][0].eff;
//	//		float E_tmp = FLT_MAX;
//	//		vector<UAV> uav_tmp_list;
//	//		for (int uav_idx_i : uav_list_id) {
//	//			uav_tmp_list.push_back(sol_tmp.id_to_UAV(uav_idx_i));
//	//		}
//	//		sort(uav_tmp_list.begin(), uav_tmp_list.end(), [](UAV& a, UAV& b) {return a.tmp_t_to_node < b.tmp_t_to_node; });
//	//		int ff = 0;
//	//		min_t = uav_tmp_list[0].tmp_t_to_node;
//	//		for (auto& uav_cc : uav_tmp_list) {
//	//			ff++;
//	//			t_delta += uav_cc.Path_change_idx;
//	//			tmp_value += uav_cc.tmp_t_to_node;
//	//			if (E_tmp > tmp_value / ff) {
//	//				E_tmp = tmp_value / ff;
//	//			}
//	//			else {
//	//				break;
//	//			}
//	//		}
//	//		//if (t_delta < node_inserted.node_size / UAV::eff) {
//	//		//	continue;
//	//		//}
//	//		if (min_err > (E_tmp - min_t)) {
//	//			min_err = (E_tmp - min_t);
//	//			final_uav_col_id = uav_list_id;
//	//		}
//	//	}
//
//
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	(*(tmp_sum_ptr + i)) /= sum_sum;
//	//	//	//cout << tmp_sum_ptr[i]<<" ";
//	//	//}
//	//	////cout << endl;
//	//	//double rnd_pro_f = (double)rand() / (double)RAND_MAX;
//	//	//double acc_pro = 0;
//	//	//for (int i = 0; i < size(uavs_id_list_f); i++) {
//	//	//	acc_pro += *(tmp_sum_ptr + i);
//	//	//	if (acc_pro > rnd_pro_f) {
//	//	//		final_uav_col_id = uavs_id_list_f[i];
//	//	//		break;
//	//	//	}
//	//	//}
//	//	delete[] tmp_sum_ptr;
//	//}
//	return final_uav_col_id;
//}
vector<int> Solution::make_uavs_assigned(Working_node node_to_insert, int insert_place, int col_num) {
	float theta_1 = (float)rand() / (float)RAND_MAX;
	//float theta_1 = 1;
	float theta_2 = 1 - theta_1;

	//float theta_1 = 0.5;
	//float theta_2 = 0.5;
	cal_uav_path_t();
	vector<int> final_uav_col_id;
	Solution sol_tmp = *this;
	auto& node_list_to_insert = sol_tmp.node_list;
	node_list_to_insert.insert(node_list_to_insert.begin() + insert_place, node_to_insert);
	sol_tmp.assign_node_no();
	auto& node_inserted = sol_tmp.node_list[insert_place];
	auto type_allowed = node_to_insert.type;
	vector<int> uav_allowed_id_list;
	for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
		for (auto& uav : UAV_list[*iter]) {
			uav_allowed_id_list.push_back(uav.id);
		}
	}
	for (int idx : uav_allowed_id_list) {
		auto& uav_ch = sol_tmp.id_to_UAV(idx);
		//determine uav's insert_place in path
		if (size(uav_ch.Path) == 0) {
			uav_ch.tmp_node_no = 0;
			continue;
		}
		int flag = 0;
		for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
			int node_id_first = uav_ch.Path[i].id;
			int node_id_second = uav_ch.Path[i + 1].id;
			auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
			auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
			if (sol_tmp.compare_node(node_ch_first, node_inserted) && sol_tmp.compare_node(node_inserted, node_ch_second)) {
				uav_ch.tmp_node_no = i + 1;
				flag = 1;
			}
		}
		if (flag == 1) {
			continue;
		}

		int node_first_id = uav_ch.Path[0].id;
		auto& node_first = sol_tmp.id_to_node(node_first_id);
		if (sol_tmp.compare_node(node_inserted, node_first)) {
			uav_ch.tmp_node_no = 0;
			continue;
		}
		int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
		auto& node_last = sol_tmp.id_to_node(node_last_id);
		if (sol_tmp.compare_node(node_last, node_inserted)) {
			uav_ch.tmp_node_no = size(uav_ch.Path);
			continue;
		}
	}
	//cal Path_change_index, delete some uavs
	vector<int> uav_allowed_id_list_1;
	for (int idx : uav_allowed_id_list) {
		auto& uav_idx = sol_tmp.id_to_UAV(idx);
		float P_1 = uav_idx.ret_PathLength_to_node_insert(node_inserted);
		float P_2 = uav_idx.ret_PathLength_from_node_insert(node_inserted);
		float P_p = uav_idx.ret_PathLength_from_node_j();
		if ((((P_2 + P_1 - P_p) / uav_idx.speed) > T - uav_idx.t_used)) {
			continue;
		}
		else {
			uav_allowed_id_list_1.push_back(uav_idx.id);

			//uav_idx.Path_change_idx =( T-uav_idx.t_used);
			/*uav_idx.col_length_idx = P_2 + P_1 - P_p;*/
			uav_idx.Path_change_idx = P_2 + P_1 - P_p;
			if (uav_idx.tmp_node_no == 0) {
				uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
			}
			else {
				uav_idx.tmp_t_to_node = uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed;
			}
		}
	}
	if (size(uav_allowed_id_list_1) == 0)
		return final_uav_col_id;
	vector<UAV> uav_allowed_list;
	for (int idx_1 : uav_allowed_id_list_1) {
		auto& uav_1 = sol_tmp.id_to_UAV(idx_1);
		uav_allowed_list.push_back(uav_1);
	}
	int col_re = min(col_num, (int)size(uav_allowed_id_list_1));
	double* max_Path_change_index = new double();
	double* max_length_index = new double();
	double* max_t_left = new double();
	*max_Path_change_index = FLT_MAX;
	*max_length_index = FLT_MAX;
	for (int u_i = 0; u_i < size(uav_allowed_id_list_1); u_i++) {
		uav_allowed_list[u_i].col_length_idx = 0;
		for (int j_i = 0; j_i < size(uav_allowed_list); j_i++) {
			uav_allowed_list[u_i].col_length_idx += abs(uav_allowed_list[u_i].tmp_t_to_node - uav_allowed_list[j_i].tmp_t_to_node);
		}
		//if (uav_allowed_list[u_i].Path_change_idx < 0) {
		//	uav_allowed_list[u_i].col_length_idx = -1;
		//}

		//else {
		//	uav_allowed_list[u_i].col_length_idx = 1 / (1 + uav_allowed_list[u_i].col_length_idx);
		//}
		*max_Path_change_index = min(*max_Path_change_index, uav_allowed_list[u_i].Path_change_idx);
		*max_length_index = min(*max_length_index, uav_allowed_list[u_i].col_length_idx);
		*max_t_left = max(*max_length_index, double(T - uav_allowed_list[u_i].t_used));
	}
	if (col_re != 1) {
		if (max_t_left[0] < 0) {
			max_t_left[0] = 1;
		}
		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (max_length_index[0] + 1) / ((uav_allowed_list[u_i].col_length_idx) + 1);
		}
	}
	else {
		for (int u_i = 0; u_i < size(uav_allowed_list); u_i++) {
			//uav_allowed_list[u_i].con_index = theta_1 * uav_allowed_list[u_i].Path_change_idx / max_Path_change_index[0] + theta_2 * ((uav_allowed_list[u_i].col_length_idx)) / (max_length_index[0]);
			uav_allowed_list[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list[u_i].Path_change_idx + theta_2 * (T - uav_allowed_list[u_i].t_used) / ((max_t_left[0]));
		}
	}
	delete max_Path_change_index;
	delete max_length_index;
	delete max_t_left;
	set<int> uav_id_set;
	sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
	for (auto& uav_de : uav_allowed_list) {
		if (size(final_uav_col_id) == col_re) {
			break;
		}
		if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
			uav_id_set.insert(uav_de.id);
			final_uav_col_id.push_back(uav_de.id);
		}
	}
	return final_uav_col_id;
}
vector<int> Solution::make_uavs_assigned_r(Working_node node_to_insert, int insert_place, int col_num) {
	vector<int> final_uav_col_id;
	set<int> f_uav_set;
	while (size(f_uav_set) < col_num) {
		int uav_id = rand() % UAV_NUM;
		if (f_uav_set.find(uav_id) == f_uav_set.end()) {
			f_uav_set.insert(uav_id);
			final_uav_col_id.push_back(uav_id);
		}
	}
	return final_uav_col_id;
}
void Solution::mutate(const vector<Working_node>& wn_list, int flag_mode, int unchange_counter) {
	//int MAX_MUTATE;
	//if (flag_mode == 0) {
	//	MAX_MUTATE = rand()%(size(node_list) / 5+1)+1;
	//}
	//if (flag_mode == 1) {
	//	MAX_MUTATE = rand() % (size(node_list) / 4 + 1) + 1;
	//}
	int mutate_i = 0;
	int m_i = 0;

	while (mutate_i < MAX_MUTATE) {
		int rnd_i = rand() % (size(wn_list));
		mutate_i++;
		cal_uav_path_t();
		if (check_node_id(rnd_i)) {//&& m_i<round(MAX_MUTATE/2)
			//m_i++;
			//cout << "col_0" << endl;
			assign_node_no();
			auto node = id_to_node(rnd_i);
			/*int rnd_col_num = rand() % (node.col_UAV_num + 1) ;*/
			//float* col_num_pro = new float[node.col_UAV_num]();
			//float sum_sum = 0;
			//int choosed_col_num;
			//for (int i = 0; i < node.col_UAV_num; i++) {
			//	col_num_pro[i] = pow(col_num_choosed_pro_list[node.id][i],beta);

			//}
			//for (int i = 0; i < node.col_UAV_num; i++) {
			//	col_num_pro[i] *= pow(((i + 1) / node.node_size),alpha);
			//	sum_sum += col_num_pro[i];
			//}
			//for (int i = 0; i < node.col_UAV_num; i++) {
			//	col_num_pro[i] /= sum_sum;
			//}
			//float rnd_col_p = (float)rand() / (float)RAND_MAX;
			//float pro_base = 0;
			//int flag_find = 0;
			//for (int i = 0; i < node.col_UAV_num; i++) {
			//	pro_base += col_num_pro[i];
			//	if (pro_base >= rnd_col_p) {
			//		flag_find = 1;
			//		choosed_col_num = i + 1;
			//		break;
			//	}
			//}
			//if (flag_find == 0) {
			//	choosed_col_num = node.col_UAV_num;
			//}
			//delete[] col_num_pro;

			vector<UAV>  uav_allowed;
			set<int> type_allowed = node.type;
			for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
				for (auto& uav : UAV_list[*iter]) {
					uav_allowed.push_back(id_to_UAV(uav.id));
				}
			}
			//int rnd_col_num = 0;

			//int max_col = min(node.col_UAV_num, UAV_NUM);
			//int choosed_col_num = 1;
			//float max_col_q = Q_table[node.id][0];
			//for (int col_i = 0; col_i < max_col; col_i++) {
			//	if (Q_table[node.id][col_i] > max_col_q) {
			//		choosed_col_num = col_i + 1;
			//		max_col_q = Q_table[node.id][col_i];
			//	}
			//}
			vector<int> vec_rel_num;
			for (int insert_p = 0; insert_p <= node.no; insert_p++) {
				int rel_num = 0;
				set<int> type_i = node.type;
				if (insert_p > 0) {
					for (int node_rel = 0; node_rel < insert_p; node_rel++) {
						auto& node_ii = node_list[node_rel];
						set<int> type_ii = node_ii.type;
						int re_flag = 0;
						for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
							for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
								if (*iter == *iter_i) {
									re_flag = 1;
								}
							}
						}
						if (re_flag == 1) {
							rel_num++;
						}
					}
				}
				vec_rel_num.push_back(rel_num);
			}
			int rnd_col_num = 0;
			int max_col = min(node.col_UAV_num, UAV_NUM);
			int choosed_col_num = 1;
			float max_col_q = 0;
			//vector<float> max_col_i;
			//for (int node_no = 0; node_no <= vec_rel_num[node.no]; node_no++) {
			//	float col_i_s = 0;
			//	for (int col_i = 0; col_i < max_col; col_i++) {
			//		if (col_i_s < Q_table_1[node.id][node_no][col_i]) {
			//			col_i_s = Q_table_1[node.id][node_no][col_i];
			//		}
			//	}
			//	max_col_i.push_back(col_i_s);
			//}
			//for (int col_i = 0; col_i < max_col; col_i++) {
			//	float col_i_s = 0;
			//	for (int node_no = 0; node_no <= vec_rel_num[node.no]; node_no++) {
			//		if (max_col_i[node_no] != 0) {
			//			col_i_s += Q_table_1[node.id][node_no][col_i]/ max_col_i[node_no];
			//		}
			//		
			//	}
			//	if (col_i_s > max_col_q) {
			//		choosed_col_num = col_i + 1;
			//		max_col_q = col_i_s;
			//	}
			//}
			vector<int> poss_act;
			for (int node_no = 0; node_no <= vec_rel_num[node.no]; node_no++) {
				float col_i_s=0;
				for (int col_i = 0; col_i < max_col; col_i++){
					if (Q_table_1[node.id][node_no][col_i] > col_i_s) {
						choosed_col_num = col_i + 1;
						col_i_s = Q_table_1[node.id][node_no][col_i];
					}
				}
				poss_act.push_back(choosed_col_num);
			}
			choosed_col_num = poss_act[max(rand() % (vec_rel_num[node.no]+1), rand() % (vec_rel_num[node.no] + 1))];
			//float col_i_s = 0;
			//for (int col_i = 0; col_i < max_col; col_i++) {
			//	col_i_s = Q_table_1[node.id][node.no][col_i];
			//	if (col_i_s > max_col_q) {
			//		choosed_col_num = col_i + 1;
			//		max_col_q = col_i_s;
			//	}
			//}
			float rand_p_col = (float)rand() / RAND_MAX;
			if (rand_p_col < eps || max_col_q == 0) {
				choosed_col_num = rand() % (max_col)+1;
				rnd_col_num = min((int)choosed_col_num, (int)size(uav_allowed));
				//rnd_col_num = max(min_node_col[node.id], rnd_col_num);
			}
			//int rnd_col_num = 1+rand() % (min(node_to_insert.col_UAV_num, (int)size(uavs_allowed_id_list)));
			else {
				//double U1 = (float)rand() / (float)RAND_MAX;
				//double U2 = (float)rand() / (float)RAND_MAX;
				//double Z = sqrt(-2 * log(U1)) * cos(2 * pi * U2);
				//double min_std = (float)2 / (float)3;
				//double max_std = (float)max(4, node.col_UAV_num) / (float)6;
				////double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(IT), 1.6);
			
				//unchange_counter = min(MAX_ACO_IT, unchange_counter);
				//double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(MAX_ACO_IT), 1);

				/*double Y = choosed_col_num + Z * std;*/
				double Y = ret_GN(choosed_col_num);
				//cout << choosed_col_num << " " << std << " ";
				if (Y > node.col_UAV_num) {
					Y = node.col_UAV_num;
				}
				if (Y < 1) {
					Y = 1;
				}
				//cout << Y << endl;
				rnd_col_num = min((int)Y, (int)size(uav_allowed));
	/*			rnd_col_num = max(min_node_col[node.id], rnd_col_num);*/
			}
			
			//int rnd_col_num = min((int)choosed_col_num, (int)size(uav_allowed));
			//if (rnd_col_num < 0) {
			//	rnd_col_num = 1;
			//}
			if (rnd_col_num == 0) {
				int i = 0;
				for (; i < size(node_list); i++) {
					if (node_list[i].id == rnd_i) {
						break;
					}
				}
				//auto UAV_list_copy = UAV_list;
				for (int id_u : node.UAV_id_list) {
					auto& uav = id_to_UAV(id_u);
					//cout <<"Path_size_before: "<< size(uav.Path) << endl;
					uav.erase_node_i_in_path(rnd_i);
					//cout << "Path_size_after: " << size(uav.Path) << endl;
				} 
				node_list.erase(node_list.begin() + i);
				//if (!check_Path()) {
				//	cout << "some_thing_wrong" << endl;
				//}
			}
			else {
				set<int> choosed_uav_no;
				vector<int> choosed_uav_id;
				/*vector<UAV>  uav_allowed;
				set<int> type_allowed = node.type;
				for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
					for (auto& uav : UAV_list[*iter]) {
						uav_allowed.push_back(uav);
					}
				}*/
				rnd_col_num = min((int)size(uav_allowed), rnd_col_num);

				//while (size(choosed_uav_no) < rnd_col_num) {
				//	int rnd_uav_no = rand() % size(uav_allowed);
				//	if (choosed_uav_no.find(rnd_uav_no) == choosed_uav_no.end()) {
				//		choosed_uav_no.insert(rnd_uav_no);
				//		choosed_uav_id.push_back(uav_allowed[rnd_uav_no].id);
				//	}
			    //}
				//Solution tmp_sol = (*this);
				//tmp_sol.node_list.erase(tmp_sol.node_list.begin() + node.no);
				//for (auto uav_tmp_id : node.UAV_id_list) {
				//	tmp_sol.id_to_UAV(uav_tmp_id).erase_node_i_in_path(node.id);
				//}
				//choosed_uav_id = tmp_sol.make_uavs_assigned_m(node, node.no, rnd_col_num);
				if (rnd_col_num = size(node.UAV_id_list)) {
					continue;
				}
				if (rnd_col_num < size(node.UAV_id_list)) {
					set<int> ch_set;
					while (size(ch_set) < rnd_col_num) {
						int rnd_ua = rand() % size(node.UAV_id_list);
						if (ch_set.find(rnd_ua) == ch_set.end()) {
							ch_set.insert(rnd_ua);
							choosed_uav_id.push_back(node.UAV_id_list[rnd_ua]);
						}
					}
				}
				if (rnd_col_num > size(node.UAV_id_list)) {
					choosed_uav_id = node.UAV_id_list;
					set<int> ch_set(choosed_uav_id.begin(),choosed_uav_id.end());
					sort(uav_allowed.begin(), uav_allowed.end(), [](UAV& a, UAV& b) {return (T - a.t_used) > (T - b.t_used); });
					for (auto uu : uav_allowed) {
						if (size(ch_set) == rnd_col_num) {
							break;
						}
						if (ch_set.find(uu.id) == ch_set.end()) {
							ch_set.insert(uu.id);
							choosed_uav_id.push_back(uu.id);
						}
					}
				}
				if (size(choosed_uav_id) == 0) {
					mutate_i--;
					continue;
				}
				vector<int> new_in_id;
				vector<int> move_out_id;
#ifdef rand_u
				node.UAV_id_list = make_uavs_assigned_r(node, node.no, rnd_col_num);
#endif // rand_u
				for (int id_1 : node.UAV_id_list) {
					int fl_1 = 0;
					for (int id_2 : choosed_uav_id) {
						if (id_1 == id_2) {
							fl_1 = 1;
							break;
						}
					}
					if (fl_1 == 0) {
						move_out_id.push_back(id_1);
					}
				}
				for (int id_1 : choosed_uav_id) {
					int fl_1 = 0;
					for (int id_2 : node.UAV_id_list) {
						if (id_1 == id_2) {
							fl_1 = 1;
							break;
						}
					}
					if (fl_1 == 0) {
						new_in_id.push_back(id_1);
					}
				}
				node.UAV_id_list = choosed_uav_id;
				id_to_node(rnd_i).UAV_id_list = choosed_uav_id;
				for (int id_out : move_out_id) {
					UAV& uav_out = id_to_UAV(id_out);
					uav_out.erase_node_i_in_path(node.id);
				}
				for (int id_in : new_in_id) {
					UAV& uav_out = id_to_UAV(id_in);
					int fl = 0;
					if (size(uav_out.Path) == 0) {
						uav_out.Path.push_back(node);
						continue;
					}
					for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
						int idx_first = uav_out.Path[i_tt].id;
						int idx_second = uav_out.Path[i_tt + 1].id;
						auto& node_first = id_to_node(idx_first);
						auto& node_second = id_to_node(idx_second);
						if (compare_node(node_first, node) && (compare_node(node, node_second))) {
							uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node);
							fl = 1;
							break;
						}
					}
					if (fl == 0) {
						int node_first_id = uav_out.Path[0].id;
						int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
						auto& node_first_0 = id_to_node(node_first_id);
						auto& node_last_0 = id_to_node(node_last_id);
						if (compare_node(node, node_first_0)) {
							uav_out.Path.insert(uav_out.Path.begin(), node);
							continue;
						}
						if (compare_node(node_last_0, node)) {
							uav_out.Path.insert(uav_out.Path.end(), node);
							continue;
						}
					}
				}
			}
		}
		else {
			//if (check_node_id(rnd_i)) {
			//	continue;
			//}
			int rnd_insert_p = rand() % (size(node_list) + 1);
			auto node = wn_list[rnd_i];
			set<int> type_allowed = node.type;
			set<int> choosed_uav_no;
			vector<int> choosed_uav_id;
			vector<UAV>  uav_allowed;
			for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
				for (auto& uav : UAV_list[*iter]) {
					uav_allowed.push_back(uav);
				}
			}

			//float* col_num_pro = new float[node_to_insert.col_UAV_num]();
			//float sum_sum = 0;
			//int choosed_col_num;
			//for (int i = 0; i < node_to_insert.col_UAV_num; i++) {
			//	col_num_pro[i] = col_num_choosed_pro_list[node_to_insert.id][i];

			//}
			//for (int i = 0; i < node_to_insert.col_UAV_num; i++) {
			//	col_num_pro[i] *= ((i + 1) / node_to_insert.node_size);
			//	sum_sum += col_num_pro[i];
			//}
			//for (int i = 0; i < node_to_insert.col_UAV_num; i++) {
			//	col_num_pro[i] /= sum_sum;
			//}
			//float rnd_col_p = (float)rand() / (float)RAND_MAX;
			//float pro_base = 0;
			//int flag_find = 0;
			//for (int i = 0; i < node_to_insert.col_UAV_num; i++) {
			//	pro_base += col_num_pro[i];
			//	if (pro_base >= rnd_col_p) {
			//		flag_find = 1;
			//		choosed_col_num = i + 1;
			//		break;
			//	}
			//}
			//if (flag_find == 0) {
			//	choosed_col_num = node_to_insert.col_UAV_num;
			//}
			//delete[] col_num_pro;


			//int rnd_col_num=0;
			//int max_col = min(node_to_insert.col_UAV_num, UAV_NUM);
			//int choosed_col_num = 1;
			//float max_col_q = Q_table[node_to_insert.id][0];
			//for (int col_i = 0; col_i < max_col; col_i++) {
			//	if (Q_table[node_to_insert.id][col_i] > max_col_q) {
			//		choosed_col_num = col_i + 1;
			//		max_col_q = Q_table[node_to_insert.id][col_i];
			//	}
			//}

			int rnd_col_num = 0;
			int max_col = min(node.col_UAV_num, UAV_NUM);
			int choosed_col_num = 1;
			float max_col_q = 0;
			vector<int> vec_rel_num;
			for (int insert_p = 0; insert_p <= rnd_insert_p; insert_p++) {
				int rel_num = 0;
				set<int> type_i = node.type;
				if (insert_p > 0) {
					for (int node_rel = 0; node_rel < insert_p; node_rel++) {
						auto& node_ii = node_list[node_rel];
						set<int> type_ii = node_ii.type;
						int re_flag = 0;
						for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
							for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
								if (*iter == *iter_i) {
									re_flag = 1;
								}
							}
						}
						if (re_flag == 1) {
							rel_num++;
						}
					}
				}
				vec_rel_num.push_back(rel_num);
			}
			//vector<int> poss_act;
			//for (int node_no = 0; node_no <= min(vec_rel_num[rnd_insert_p],1); node_no++) {
			//	float col_i_s = 0;
			//	for (int col_i = 0; col_i < max_col; col_i++) {
			//		if (Q_table_1[node.id][node_no][col_i] > col_i_s) {
			//			choosed_col_num = col_i + 1;
			//			col_i_s = Q_table_1[node.id][node_no][col_i];
			//		}
			//	}
			//	poss_act.push_back(choosed_col_num);
			//}
			//choosed_col_num = poss_act[max(rand() % (vec_rel_num[rnd_insert_p]+1), rand() % (vec_rel_num[rnd_insert_p])+1)];
			choosed_col_num = rand() % max_col+1;
			//vector<float> max_col_i;
			//for (int node_no = 0; node_no <= vec_rel_num[rnd_insert_p]; node_no++) {
			//	 float col_i_s = 0;
			//	 for (int col_i = 0; col_i < max_col; col_i++){
			//		if (col_i_s < Q_table_1[node.id][node_no][col_i]) {
			//			col_i_s = Q_table_1[node.id][node_no][col_i];
			//		}				
			//	 }
			//	 max_col_i.push_back(col_i_s);
			//}
			//for (int col_i = 0; col_i < max_col; col_i++) {
			//	float col_i_s = 0;
			//	for (int node_no = 0; node_no <= vec_rel_num[rnd_insert_p]; node_no++) {
			//		if (max_col_i[node_no] != 0) {
			//			col_i_s += Q_table_1[node.id][node_no][col_i]/ max_col_i[node_no];
			//		}
			//	}
			//	if (col_i_s > max_col_q) {
			//		choosed_col_num = col_i + 1;
			//		max_col_q = col_i_s;
			//	}
			//}
			//for (int col_i = 0; col_i < max_col; col_i++){
			//	col_i_s = Q_table_1[node.id][rnd_insert_p][col_i];
			//	if (col_i_s > max_col_q) {
			//		choosed_col_num = col_i + 1;
			//		max_col_q = col_i_s;
			//	}
			//}

			float rand_p_col = (float)rand() / RAND_MAX;
			if (rand_p_col < eps || max_col_q == 0) {
				choosed_col_num = rand() % max_col + 1;
				rnd_col_num = min((int)choosed_col_num, (int)size(uav_allowed));
				//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);
			}
			//int rnd_col_num = 1+rand() % (min(node_to_insert.col_UAV_num, (int)size(uavs_allowed_id_list)));
			else {
				//double U1 = (float)rand() / (float)RAND_MAX;
				//double U2 = (float)rand() / (float)RAND_MAX;
				//double Z = sqrt(-2 * log(U1)) * cos(2 * pi * U2);

				//double min_std = (float)2 / (float)3;
				//double max_std = (float)max(4, node_to_insert.col_UAV_num) / (float)6;
				////double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(IT), 1.6);

				//unchange_counter = min(MAX_ACO_IT, unchange_counter);
				//double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(MAX_ACO_IT), 1);

				/*double Y = choosed_col_num + Z*std;*/
				double Y = ret_GN(choosed_col_num);
				//cout << choosed_col_num << " " << std << " ";
				if (Y > node.col_UAV_num) {
					Y = node.col_UAV_num;
				}
				if (Y < 1) {
					Y = 1;
				}
				//cout << Y << endl;
				rnd_col_num = min((int)Y, (int)size(uav_allowed));
				//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);
			}
			
			if (rnd_col_num < 0) {
				rnd_col_num = 1;
			}
		
			//int rnd_col_num = 1+rand() % (min(node_to_insert.col_UAV_num, (int)size(uavs_allowed_id_list)));
			
			//int rnd_col_num = min(choosed_col_num, (int)size(uav_allowed));
		
			//int max_col_num = min(node_to_insert.col_UAV_num, int(size(uav_allowed)));
			//int rnd_col_num =1+rand() % (max_col_num);
			//while (size(choosed_uav_no) < rnd_col_num) {
			//	int rnd_uav_no = rand() % size(uav_allowed);
			//	if (choosed_uav_no.find(rnd_uav_no) == choosed_uav_no.end()) {
			//		choosed_uav_no.insert(rnd_uav_no);
			//		choosed_uav_id.push_back(uav_allowed[rnd_uav_no].id);
			//	}
			//}
			choosed_uav_id = make_uavs_assigned_m(node, rnd_insert_p, rnd_col_num);
#ifdef rand_u
			choosed_uav_id = make_uavs_assigned_r(node, rnd_insert_p, rnd_col_num);
#endif // rand_u
			if (size(choosed_uav_id) == 0){
				mutate_i--;
				continue;
			}
			node_list.insert(node_list.begin() + rnd_insert_p, wn_list[rnd_i]);
			assign_node_no();
			auto& node_to_insert = node_list[rnd_insert_p];
			node_to_insert.UAV_id_list = choosed_uav_id;
			for (int id_in : node_to_insert.UAV_id_list) {
				UAV& uav_out = id_to_UAV(id_in);
				int fl = 0;
				if (size(uav_out.Path) == 0) {
					uav_out.Path.push_back(node_to_insert);
					continue;
				}
				for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
					int idx_first = uav_out.Path[i_tt].id;
					int idx_second = uav_out.Path[i_tt + 1].id;
					auto& node_first = id_to_node(idx_first);
					auto& node_second = id_to_node(idx_second);
					if (compare_node(node_first, node_to_insert) && (compare_node(node_to_insert, node_second))) {
						uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node_to_insert);
						fl = 1;
						break;
					}
				}
				if (fl == 0) {
					int node_first_id = uav_out.Path[0].id;
					int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
					auto& node_first_0 = id_to_node(node_first_id);
					auto& node_last_0 = id_to_node(node_last_id);
					if (compare_node(node_to_insert, node_first_0)) {
						uav_out.Path.insert(uav_out.Path.begin(), node_to_insert);
						continue;
					}
					if (compare_node(node_last_0, node_to_insert)) {
						uav_out.Path.insert(uav_out.Path.end(), node_to_insert);
						continue;
					}
				}
			}
		}
	}
}

void Solution::ls_mutate(const vector<Working_node>& wn_list, int flag_mode, const vector<float*>& col_num_choosed_pro_list, int unchange_counter) {
	int MAX_MUTATE=size(node_list)/3;

	int mutate_i = 0;

	while (mutate_i < MAX_MUTATE) {
		int rnd_i = rand() % (size(wn_list));
		if (check_node_id(rnd_i)) {
			mutate_i++;
			//cout << "col_0" << endl;
			assign_node_no();
			auto node = id_to_node(rnd_i);
			/*int rnd_col_num = rand() % (node.col_UAV_num + 1) ;*/
			float* col_num_pro = new float[node.col_UAV_num]();
			float sum_sum = 0;
			int choosed_col_num;
			for (int i = 0; i < node.col_UAV_num; i++) {
				col_num_pro[i] = col_num_choosed_pro_list[node.id][i];

			}
			for (int i = 0; i < node.col_UAV_num; i++) {
				col_num_pro[i] *= ((i + 1) / node.node_size);
				sum_sum += col_num_pro[i];
			}
			for (int i = 0; i < node.col_UAV_num; i++) {
				col_num_pro[i] /= sum_sum;
			}
			float rnd_col_p = (float)rand() / (float)RAND_MAX;
			float pro_base = 0;
			int flag_find = 0;
			for (int i = 0; i < node.col_UAV_num; i++) {
				pro_base += col_num_pro[i];
				if (pro_base >= rnd_col_p) {
					flag_find = 1;
					choosed_col_num = i + 1;
					break;
				}
			}
			if (flag_find == 0) {
				choosed_col_num = node.col_UAV_num;
			}
			delete[] col_num_pro;

			//int rnd_col_num = 1+rand() % (min(node_to_insert.col_UAV_num, (int)size(uavs_allowed_id_list)));

			//double U1 = (float)rand() / (float)RAND_MAX;
			//double U2 = (float)rand() / (float)RAND_MAX;
			//double Z = sqrt(-2 * log(U1)) * cos(2 * pi * U2);
			//double min_std = (float)1 / (float)3;
			//double max_std = (float)max(2, node.col_UAV_num) / (float)6;
			////double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(IT), 1.6);

			//unchange_counter = min(MAX_ACO_IT, unchange_counter);
			//double std = min_std + (max_std - min_std) * pow((float)(unchange_counter) / (float)(MAX_ACO_IT), 1.6);

			//double Y = choosed_col_num + Z * std;
			double Y = choosed_col_num;
			//cout << choosed_col_num << " " << std << " ";
			if ((1 <= Y) && (Y <= node.col_UAV_num)) {
				Y = round(Y);
			}
			if (Y > node.col_UAV_num) {
				Y = node.col_UAV_num;
			}
			if (Y < 1) {
				Y = 1;
			}
			//cout << Y << endl;
			vector<UAV>  uav_allowed;
			set<int> type_allowed = node.type;
			for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
				for (auto& uav : UAV_list[*iter]) {
					uav_allowed.push_back(uav);
				}
			}
			int rnd_col_num = min((int)Y, (int)size(uav_allowed));
			//int rnd_col_num = min((int)choosed_col_num, (int)size(uav_allowed));
			if (rnd_col_num < 0) {
				rnd_col_num = 1;
			}
			if (rnd_col_num == 0) {
				int i = 0;
				for (; i < size(node_list); i++) {
					if (node_list[i].id == rnd_i) {
						break;
					}
				}
				//auto UAV_list_copy = UAV_list;
				for (int id_u : node.UAV_id_list) {
					auto& uav = id_to_UAV(id_u);
					//cout <<"Path_size_before: "<< size(uav.Path) << endl;
					uav.erase_node_i_in_path(rnd_i);
					//cout << "Path_size_after: " << size(uav.Path) << endl;
				}
				node_list.erase(node_list.begin() + i);
				//if (!check_Path()) {
				//	cout << "some_thing_wrong" << endl;
				//}
			}
			else {
				set<int> choosed_uav_no;
				vector<int> choosed_uav_id;
				/*vector<UAV>  uav_allowed;
				set<int> type_allowed = node.type;
				for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
					for (auto& uav : UAV_list[*iter]) {
						uav_allowed.push_back(uav);
					}
				}*/
				rnd_col_num = min((int)size(uav_allowed), rnd_col_num);

				while (size(choosed_uav_no) < rnd_col_num) {
					int rnd_uav_no = rand() % size(uav_allowed);
					if (choosed_uav_no.find(rnd_uav_no) == choosed_uav_no.end()) {
						choosed_uav_no.insert(rnd_uav_no);
						choosed_uav_id.push_back(uav_allowed[rnd_uav_no].id);
					}
				}
				vector<int> new_in_id;
				vector<int> move_out_id;
				for (int id_1 : node.UAV_id_list) {
					int fl_1 = 0;
					for (int id_2 : choosed_uav_id) {
						if (id_1 == id_2) {
							fl_1 = 1;
							break;
						}
					}
					if (fl_1 == 0) {
						move_out_id.push_back(id_1);
					}
				}
				for (int id_1 : choosed_uav_id) {
					int fl_1 = 0;
					for (int id_2 : node.UAV_id_list) {
						if (id_1 == id_2) {
							fl_1 = 1;
							break;
						}
					}
					if (fl_1 == 0) {
						new_in_id.push_back(id_1);
					}
				}
				node.UAV_id_list = choosed_uav_id;
				id_to_node(rnd_i).UAV_id_list = choosed_uav_id;
				for (int id_out : move_out_id) {
					UAV& uav_out = id_to_UAV(id_out);
					uav_out.erase_node_i_in_path(node.id);
				}
				for (int id_in : new_in_id) {
					UAV& uav_out = id_to_UAV(id_in);
					int fl = 0;
					if (size(uav_out.Path) == 0) {
						uav_out.Path.push_back(node);
						continue;
					}
					for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
						int idx_first = uav_out.Path[i_tt].id;
						int idx_second = uav_out.Path[i_tt + 1].id;
						auto& node_first = id_to_node(idx_first);
						auto& node_second = id_to_node(idx_second);
						if (compare_node(node_first, node) && (compare_node(node, node_second))) {
							uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node);
							fl = 1;
							break;
						}
					}
					if (fl == 0) {
						int node_first_id = uav_out.Path[0].id;
						int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
						auto& node_first_0 = id_to_node(node_first_id);
						auto& node_last_0 = id_to_node(node_last_id);
						if (compare_node(node, node_first_0)) {
							uav_out.Path.insert(uav_out.Path.begin(), node);
							continue;
						}
						if (compare_node(node_last_0, node)) {
							uav_out.Path.insert(uav_out.Path.end(), node);
							continue;
						}
					}
				}
			}
		}
	}
}

bool Solution::need_repair_m() {
	node_Path_node_connect();
	UAVs_t_used_clear();
	bool ret = false;
	for (auto& node : node_list) {
		vector<UAV> col_uav_list;
		for (int i = 0; i < size(node.UAV_id_list); i++) {
			int idx = node.UAV_id_list[i];
			UAV& uav = id_to_UAV(idx);
			if (uav.cal_t_to_node_no_j(node.Path_node_no_list[i]) > T) {
				ret= true;
			}
			col_uav_list.push_back(uav);
		}
		float t_end = node.cal_time_end_with_complete_Path(col_uav_list);
		if (t_end > T) {
			ret = true;
			node.t_end = t_end;
			for (auto& uav_r : col_uav_list) {
				int flag = 0;
				for (auto& uav_l : UAV_list) {
					for (auto& uav : uav_l) {
						if (uav.id == uav_r.id) {
							uav.t_used = t_end;
							uav.Path = uav_r.Path;
							flag = 1;
							break;
						}
					}
					if (flag == 1) {
						break;
					}
				}
			}
		}
		else {
			node.t_end = t_end;
			for (auto& uav_r : col_uav_list) {
				int flag = 0;
				for (auto& uav_l : UAV_list) {
					for (auto& uav : uav_l) {
						if (uav.id == uav_r.id) {
							uav.t_used = uav_r.t_used;
							uav.Path = uav_r.Path;
							flag = 1;
							break;
						}
					}
					if (flag == 1) {
						break;
					}
				}
			}
		}
	}
	return ret;
}


bool Solution::need_repair() {
	node_Path_node_connect();
	UAVs_t_used_clear();
	for (auto& node : node_list) {
		vector<UAV> col_uav_list;
		for (int i = 0; i < size(node.UAV_id_list); i++) {
			int idx = node.UAV_id_list[i];
			UAV& uav = id_to_UAV(idx);
			if (uav.cal_t_to_node_no_j(node.Path_node_no_list[i]) > T) {
				return true;
			}
			col_uav_list.push_back(uav);
		}
		float t_end = node.cal_time_end_with_complete_Path(col_uav_list);
		if (t_end > T) {
			return true;
		}
		else {
			node.t_end = t_end;
			for (auto& uav_r : col_uav_list) {
				int flag = 0;
				for (auto& uav_l : UAV_list) {
					for (auto& uav : uav_l) {
						if (uav.id == uav_r.id) {
							uav.t_used = uav_r.t_used;
							uav.Path = uav_r.Path;
							flag = 1;
							break;
						}
					}
					if (flag == 1) {
						break;
					}
				}
			}
		}
	}
	return false;
}
void Solution::UAVs_Path_clear() {
	for (auto& uav_l : UAV_list) {
		for (auto& uav : uav_l) {
			uav.Path.clear();
		}
	}
}

void Solution::UAVs_t_used_clear() {
	for (vector<UAV>& uav_list : UAV_list) {
		for (UAV& uav : uav_list) {
			uav.clear_t_used();
			uav.Path_node_t_list.clear();
		}
	}
}

void Solution::clear() {
	UAVs_Path_clear();
	UAVs_t_used_clear();
	node_list.clear();
}

void Solution::repair(int flag_mode) {
	//确定每个任务点与UAV路径中路径点的关联；
	node_Path_node_connect();
	int size_ini = size(node_list);
	if (need_repair()) {
		auto sol_tmp = *this;
		sol_tmp.clear();
		int count = 0;
		int MAX_COUNT = 20;
		set<int> node_no_set;
		int node_no_tmp = -1;
		//cout << size(node_list)<<endl;
		for (auto node : node_list) {
			node_no_tmp++;
			node.Path_node_no_list.clear();
			float in_pro = (float)rand() / (float)RAND_MAX;
			float MAX_pro;

			float index_n;
			//index_cal
			int no_id = size(sol_tmp.node_list);
			float V_node = V_matrix[no_id][node.id];
			auto M_tmp = V_matrix[no_id];
			for (int i = 0; i < MAP_NODE_NUM; i++) {
				for (int j = 0; j < MAP_NODE_NUM-1; j++) {
					if (M_tmp[j] > M_tmp[j + 1]) {
						float tmp = M_tmp[j + 1];
						M_tmp[j + 1] = M_tmp[j];
						M_tmp[j] = tmp;
					}
				}
			}
			int high = (MAP_NODE_NUM-1);/*
			int h_m = (MAP_NODE_NUM - 1) *0.75;*/
			int medium = 0.8 * (MAP_NODE_NUM - 1);
			int low = 0.2 * (MAP_NODE_NUM - 1);
			//cout << low << " " << medium << " " << h_m << " " << high << endl;
			// 
			if (V_node <= M_tmp[low]) {
				index_n = 0.3 - 0.1 * pow((float)iteration_time / IT, 1.5);
			}//+ 0.2 * pow((float)iteration_time / IT, 1.5)
			if (M_tmp[low]<V_node&& V_node <=M_tmp[medium]) {
				index_n = 0.6- 0.4 * pow((float)iteration_time / IT, 1.5);
			}
			//if (M_tmp[medium]<V_node&& V_node <= M_tmp[h_m]) {
			//	index_n = 0.7 + 0.3 * pow((float)mode_ch_counter / 60, 1);
			//}
			if (M_tmp[medium] < V_node&& V_node <= M_tmp[high]) {
				index_n = 0.9 - 0.7 * pow((float)iteration_time / IT, 1.5);
			}
			//// 
			// 
			// 
			

			//if (V_node <= M_tmp[low]) {
			//	index_n = 0.3+ 0.3 * pow((float)iteration_time / IT, 1.5);
			//}
			//if (M_tmp[low] < V_node && V_node <= M_tmp[medium]) {
			//	index_n = 0.6+ 0.3 * pow((float)iteration_time / IT, 1.5);
			//}
			////if (M_tmp[medium]<V_node&& V_node <= M_tmp[h_m]) {
			////	index_n = 0.7 + 0.3 * pow((float)mode_ch_counter / 60, 1);
			////}
			//if (M_tmp[medium] < V_node && V_node <= M_tmp[high]) {
			//	index_n = 0.9 ;
			//}

			//cout << M_tmp[low] << " " << M_tmp[medium] << " " << M_tmp[h_m] << " " << M_tmp[high] << endl;
			//cout << endl;
			//cout << sum_v << " ";
			//cout << index_n <<endl;

			//if (V_node <= M_tmp[high]/4) {
			//	index_n = 0.2 + 0.1 * pow((float)mode_ch_counter / 60, 1);
			//}
			//if (M_tmp[high] *0.25 <= V_node && V_node <= M_tmp[high] * 0.5) {
			//	index_n = 0.3 + 0.2 * pow((float)mode_ch_counter / 60, 1);
			//}
			//if (M_tmp[high] * 0.5 <= V_node && V_node <= M_tmp[high] * 0.75) {
			//	index_n = 0.5 + 0.3 * pow((float)mode_ch_counter / 60, 1);
			//}
			//if (M_tmp[high] * 0.75 <= V_node && V_node <= M_tmp[high]) {
			//	index_n = 0.8;
			//}


			//if (V_node <= M_tmp[high]/3) {
			//	index_n = 0.5;
			//}
			//if (M_tmp[high] *1/3 <= V_node && V_node <= M_tmp[high] * 2/3) {
			//	index_n = 0.75;
			//}
			//if (M_tmp[high] * 2/3 <= V_node && V_node <= M_tmp[high] ) {
			//	index_n = 1;
			//}

			//
			//cout << index_n << endl;
			//
			//opt_size ? MAX_pro = (float)opt_size / size(node_list)* 0.8 : MAX_pro = 0.6;
			// 
			MAX_pro = index_n;
			//cout << MAX_pro << endl;
			//if (flag_mode == 0){
			//	MAX_pro = 0.8;
			//}
			//if (flag_mode == 1) {
			//	MAX_pro = 0.6;
			//}
			//加入任务点序列中
			if (in_pro < MAX_pro) {
				vector<UAV> col_UAV_list;
				vector<int> tmp_uav_id_list = node.UAV_id_list;
				int j_tmp = -1;
				int break_flag = 0;
				for (int i = 0; i < size(tmp_uav_id_list); i++) {
					j_tmp++;
					int idx = node.UAV_id_list[j_tmp];
					UAV& uav_i = sol_tmp.id_to_UAV(idx);
					if (uav_i.cal_t_go_to(node) >= T) {
						//node.UAV_id_list.erase(node.UAV_id_list.begin() + j_tmp);
						//j_tmp--;
						//continue;
						break_flag = 1;
						break;
					}
					else {
						col_UAV_list.push_back(uav_i);
					}
				}

				if ((size(col_UAV_list) == 0 )||(break_flag==1)) {
					count++;
					continue;
				}
				for (auto& uav_col : col_UAV_list) {
					uav_col.Path.push_back(node);
					node.Path_node_no_list.push_back(size(uav_col.Path) - 1);
				}
				float t_tmp_end=node.cal_time_end(col_UAV_list);
				if (t_tmp_end > T) {
					count++;
					continue;
				}
				else {
					node_no_set.insert(node_no_tmp);
					count = 0;
					node.t_end = t_tmp_end;
					sol_tmp.node_list.push_back(node);
					//cout << node_no_tmp << " ";
					for (auto& uav_l_tmp : sol_tmp.UAV_list) {
						for (auto& uav_tmp : uav_l_tmp) {
							for (auto& uav_r : col_UAV_list) {
								if (uav_tmp.id == uav_r.id) {
									uav_tmp.Path = uav_r.Path;
									uav_tmp.t_used = uav_r.t_used;
									break;
								}
							}
						}
					}
				}
			}
			//if (count == MAX_COUNT) {
			//	break;
			//}
		}
		//cout <<size(node_no_set)<< endl;
		//sol_tmp.clear_ext_path_node(0.5);
		count = 0;			
		float t_limi = (float)2000/3;
		while (size(node_no_set)<size_ini) {
			int break_flag1 = 1;
			for (auto uav_sub : sol_tmp.UAV_list) {
				for (auto uav_s : uav_sub) {
					if (T - uav_s.t_used > t_limi) {
						break_flag1 = 0;
					}
				}
			}
			if (break_flag1 == 1) {
				break;
			}
			int rnd_node_no = rand() % size(node_list);
			if (node_no_set.find(rnd_node_no) == node_no_set.end()) {
				
				node_no_set.insert(rnd_node_no);
				auto node_to_insert = node_list[rnd_node_no];
				vector<int> allowed_insert_p_list;
				for (int insert_p = 0; insert_p <= size(sol_tmp.node_list); insert_p++) {
					Solution sol_tt = sol_tmp;
					sol_tt.node_list.insert(sol_tt.node_list.begin() + insert_p, node_to_insert);
					sol_tt.assign_node_no();
					node_to_insert.no = sol_tt.node_list[insert_p].no;
					for (auto uav_tt_idx : node_to_insert.UAV_id_list) {
						int fl = 0;
						UAV& uav_tt = sol_tt.id_to_UAV(uav_tt_idx);
						if (size(uav_tt.Path) == 0) {
							uav_tt.Path.push_back(node_to_insert);
							continue;
						}
						for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
							int idx_first = uav_tt.Path[i_tt].id;
							int idx_second = uav_tt.Path[i_tt + 1].id;
							auto& node_first = sol_tt.id_to_node(idx_first);
							auto& node_second = sol_tt.id_to_node(idx_second);
							if (sol_tt.compare_node(node_first, node_to_insert) && (sol_tt.compare_node(node_to_insert, node_second))) {
								uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_to_insert);
								fl = 1;
								break;
							}
						}
						if (fl == 0) {
							int node_first_id = uav_tt.Path[0].id;
							int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
							Working_node& node_first_0 = sol_tt.id_to_node(node_first_id);
							auto& node_last_0 = sol_tt.id_to_node(node_last_id);
							if (sol_tt.compare_node(node_to_insert, node_first_0)) {
								uav_tt.Path.insert(uav_tt.Path.begin(), node_to_insert);
								continue;
							}
							if (sol_tt.compare_node(node_last_0, node_to_insert)) {
								uav_tt.Path.insert(uav_tt.Path.end(), node_to_insert);
								continue;
							}
						}
					}
					if (!sol_tt.need_repair()) {
						allowed_insert_p_list.push_back(insert_p);
					}
				}
				if (size(allowed_insert_p_list) == 0) {
					continue;
				}
				int rnd_insert_p_no = rand() % size(allowed_insert_p_list);
				int insert_p_f = allowed_insert_p_list[rnd_insert_p_no];
				sol_tmp.node_list.insert(sol_tmp.node_list.begin() + insert_p_f, node_to_insert);
				sol_tmp.assign_node_no();
				node_to_insert.no = sol_tmp.node_list[insert_p_f].no;
				for (auto uav_tt_idx : node_to_insert.UAV_id_list) {
					int fl = 0;
					UAV& uav_tt = sol_tmp.id_to_UAV(uav_tt_idx);
					if (size(uav_tt.Path) == 0) {
						uav_tt.Path.push_back(node_to_insert);
						continue;
					}
					for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
						int idx_first = uav_tt.Path[i_tt].id;
						int idx_second = uav_tt.Path[i_tt + 1].id;
						auto& node_first = sol_tmp.id_to_node(idx_first);
						auto& node_second = sol_tmp.id_to_node(idx_second);
						if (sol_tmp.compare_node(node_first, node_to_insert) && (sol_tmp.compare_node(node_to_insert, node_second))) {
							uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_to_insert);
							fl = 1;
							break;
						}
					}
					if (fl == 0) {
						int node_first_id = uav_tt.Path[0].id;
						int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
						Working_node& node_first_0 = sol_tmp.id_to_node(node_first_id);
						auto& node_last_0 = sol_tmp.id_to_node(node_last_id);
						if (sol_tmp.compare_node(node_to_insert, node_first_0)) {
							uav_tt.Path.insert(uav_tt.Path.begin(), node_to_insert);
							continue;
						}
						if (sol_tmp.compare_node(node_last_0, node_to_insert)) {
							uav_tt.Path.insert(uav_tt.Path.end(), node_to_insert);
							continue;
						}
					}
				}
				if (sol_tmp.need_repair()) {
					cout << "construct a invalid solution" << endl;
				}
			}
		}
		*this = sol_tmp;
		if (this->need_repair()) {
			cout << "construct a invalid solution" << endl;
		}
	} 
}


//void Solution::repair_ig(int flag_mode) {
//	//确定每个任务点与UAV路径中路径点的关联；
//	node_Path_node_connect();
//	int size_ini = size(node_list);
//	if (need_repair()) {
//		auto sol_tmp = *this;
//		sol_tmp.clear();
//		int count = 0;
//		int MAX_COUNT = 20;
//		set<int> node_no_set;
//		int node_no_tmp = -1;
//		//cout << size(node_list)<<endl;
//		for (auto node : node_list) {
//			node_no_tmp++;
//			node.Path_node_no_list.clear();
//			float in_pro = (float)rand() / (float)RAND_MAX;
//			float MAX_pro;
//
//			float index_n;
//			//index_cal
//			int no_id = size(sol_tmp.node_list);
//			float V_node = V_matrix[no_id][node.id];
//			auto M_tmp = V_matrix[no_id];
//			for (int i = 0; i < MAP_NODE_NUM; i++) {
//				for (int j = 0; j < MAP_NODE_NUM - 1; j++) {
//					if (M_tmp[j] > M_tmp[j + 1]) {
//						float tmp = M_tmp[j + 1];
//						M_tmp[j + 1] = M_tmp[j];
//						M_tmp[j] = tmp;
//					}
//				}
//			}
//			int high = (MAP_NODE_NUM - 1);/*
//			int h_m = (MAP_NODE_NUM - 1) *0.75;*/
//			int medium = 0.8 * (MAP_NODE_NUM - 1);
//			int low = 0.3 * (MAP_NODE_NUM - 1);
//			//cout << low << " " << medium << " " << h_m << " " << high << endl;
//			// 
//			if (V_node <= M_tmp[low]) {
//				index_n = 0.3 - 0.1 * pow((float)iteration_time / IT, 1);
//			}//+ 0.2 * pow((float)iteration_time / IT, 1.5)
//			if (M_tmp[low] < V_node && V_node <= M_tmp[medium]) {
//				index_n = 0.5 - 0.3 * pow((float)iteration_time / IT, 1);
//			}
//			//if (M_tmp[medium]<V_node&& V_node <= M_tmp[h_m]) {
//			//	index_n = 0.7 + 0.3 * pow((float)mode_ch_counter / 60, 1);
//			//}
//			if (M_tmp[medium] < V_node && V_node <= M_tmp[high]) {
//				index_n = 0.8 - 0.6 * pow((float)iteration_time / IT, 1);
//			}
//			//// 
//			// 
//			// 
//
//
//			//if (V_node <= M_tmp[low]) {
//			//	index_n = 0.3+ 0.3 * pow((float)iteration_time / IT, 1.5);
//			//}
//			//if (M_tmp[low] < V_node && V_node <= M_tmp[medium]) {
//			//	index_n = 0.6+ 0.3 * pow((float)iteration_time / IT, 1.5);
//			//}
//			////if (M_tmp[medium]<V_node&& V_node <= M_tmp[h_m]) {
//			////	index_n = 0.7 + 0.3 * pow((float)mode_ch_counter / 60, 1);
//			////}
//			//if (M_tmp[medium] < V_node && V_node <= M_tmp[high]) {
//			//	index_n = 0.9 ;
//			//}
//
//			//cout << M_tmp[low] << " " << M_tmp[medium] << " " << M_tmp[h_m] << " " << M_tmp[high] << endl;
//			//cout << endl;
//			//cout << sum_v << " ";
//			//cout << index_n <<endl;
//
//			//if (V_node <= M_tmp[high]/4) {
//			//	index_n = 0.2 + 0.1 * pow((float)mode_ch_counter / 60, 1);
//			//}
//			//if (M_tmp[high] *0.25 <= V_node && V_node <= M_tmp[high] * 0.5) {
//			//	index_n = 0.3 + 0.2 * pow((float)mode_ch_counter / 60, 1);
//			//}
//			//if (M_tmp[high] * 0.5 <= V_node && V_node <= M_tmp[high] * 0.75) {
//			//	index_n = 0.5 + 0.3 * pow((float)mode_ch_counter / 60, 1);
//			//}
//			//if (M_tmp[high] * 0.75 <= V_node && V_node <= M_tmp[high]) {
//			//	index_n = 0.8;
//			//}
//
//
//			//if (V_node <= M_tmp[high]/3) {
//			//	index_n = 0.5;
//			//}
//			//if (M_tmp[high] *1/3 <= V_node && V_node <= M_tmp[high] * 2/3) {
//			//	index_n = 0.75;
//			//}
//			//if (M_tmp[high] * 2/3 <= V_node && V_node <= M_tmp[high] ) {
//			//	index_n = 1;
//			//}
//
//			//
//			//cout << index_n << endl;
//			//
//			//opt_size ? MAX_pro = (float)opt_size / size(node_list)* 0.8 : MAX_pro = 0.6;
//			// 
//			MAX_pro = index_n;
//#ifdef IG
//			MAX_PRO = 0.3
//#endif
//
//#ifdef ABC
//				MAX_PRO = 0.3
//#endif
//			//cout << MAX_pro << endl;
//			//if (flag_mode == 0){
//			//	MAX_pro = 0.8;
//			//}
//			//if (flag_mode == 1) {
//			//	MAX_pro = 0.6;
//			//}
//			//加入任务点序列中
//			if (in_pro < MAX_pro) {
//				vector<UAV> col_UAV_list;
//				vector<int> tmp_uav_id_list = node.UAV_id_list;
//				int j_tmp = -1;
//				int break_flag = 0;
//				for (int i = 0; i < size(tmp_uav_id_list); i++) {
//					j_tmp++;
//					int idx = node.UAV_id_list[j_tmp];
//					UAV& uav_i = sol_tmp.id_to_UAV(idx);
//					if (uav_i.cal_t_go_to(node) >= T) {
//						//node.UAV_id_list.erase(node.UAV_id_list.begin() + j_tmp);
//						//j_tmp--;
//						//continue;
//						break_flag = 1;
//						break;
//					}
//					else {
//						col_UAV_list.push_back(uav_i);
//					}
//				}
//
//				if ((size(col_UAV_list) == 0) || (break_flag == 1)) {
//					count++;
//					continue;
//				}
//				for (auto& uav_col : col_UAV_list) {
//					uav_col.Path.push_back(node);
//					node.Path_node_no_list.push_back(size(uav_col.Path) - 1);
//				}
//				float t_tmp_end = node.cal_time_end(col_UAV_list);
//				if (t_tmp_end > T) {
//					count++;
//					continue;
//				}
//				else {
//					node_no_set.insert(node_no_tmp);
//					count = 0;
//					node.t_end = t_tmp_end;
//					sol_tmp.node_list.push_back(node);
//					//cout << node_no_tmp << " ";
//					for (auto& uav_l_tmp : sol_tmp.UAV_list) {
//						for (auto& uav_tmp : uav_l_tmp) {
//							for (auto& uav_r : col_UAV_list) {
//								if (uav_tmp.id == uav_r.id) {
//									uav_tmp.Path = uav_r.Path;
//									uav_tmp.t_used = uav_r.t_used;
//									break;
//								}
//							}
//						}
//					}
//				}
//			}
//			//if (count == MAX_COUNT) {
//			//	break;
//			//}
//		}
//		//cout <<size(node_no_set)<< endl;
//		//sol_tmp.clear_ext_path_node(0.5);
//		count = 0;
//		float t_limi = (float)2000 / 3;
//		while (size(node_no_set) < size_ini) {
//			int break_flag1 = 1;
//			for (auto uav_sub : sol_tmp.UAV_list) {
//				for (auto uav_s : uav_sub) {
//					if (T - uav_s.t_used > t_limi) {
//						break_flag1 = 0;
//					}
//				}
//			}
//			if (break_flag1 == 1) {
//				break;
//			}
//			int rnd_node_no = rand() % size(node_list);
//			if (node_no_set.find(rnd_node_no) == node_no_set.end()) {
//
//				node_no_set.insert(rnd_node_no);
//				auto node_to_insert = node_list[rnd_node_no];
//				vector<int> allowed_insert_p_list;
//				vector<float> p_t_end_list;
//				for (int insert_p = 0; insert_p <= size(sol_tmp.node_list); insert_p++) {
//					Solution sol_tt = sol_tmp;
//					sol_tt.node_list.insert(sol_tt.node_list.begin() + insert_p, node_to_insert);
//					sol_tt.assign_node_no();
//					node_to_insert.no = sol_tt.node_list[insert_p].no;
//					for (auto uav_tt_idx : node_to_insert.UAV_id_list) {
//						int fl = 0;
//						UAV& uav_tt = sol_tt.id_to_UAV(uav_tt_idx);
//						if (size(uav_tt.Path) == 0) {
//							uav_tt.Path.push_back(node_to_insert);
//							continue;
//						}
//						for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
//							int idx_first = uav_tt.Path[i_tt].id;
//							int idx_second = uav_tt.Path[i_tt + 1].id;
//							auto& node_first = sol_tt.id_to_node(idx_first);
//							auto& node_second = sol_tt.id_to_node(idx_second);
//							if (sol_tt.compare_node(node_first, node_to_insert) && (sol_tt.compare_node(node_to_insert, node_second))) {
//								uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_to_insert);
//								fl = 1;
//								break;
//							}
//						}
//						if (fl == 0) {
//							int node_first_id = uav_tt.Path[0].id;
//							int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
//							Working_node& node_first_0 = sol_tt.id_to_node(node_first_id);
//							auto& node_last_0 = sol_tt.id_to_node(node_last_id);
//							if (sol_tt.compare_node(node_to_insert, node_first_0)) {
//								uav_tt.Path.insert(uav_tt.Path.begin(), node_to_insert);
//								continue;
//							}
//							if (sol_tt.compare_node(node_last_0, node_to_insert)) {
//								uav_tt.Path.insert(uav_tt.Path.end(), node_to_insert);
//								continue;
//							}
//						}
//					}
//					if (!sol_tt.need_repair()) {
//						allowed_insert_p_list.push_back(insert_p);
//						float t_end_max = 0;
//						for (int k = 0; k < size(sol_tt.node_list); k++) {
//							t_end_max = max(t_end_max, sol_tt.node_list[k].t_end);
//						}
//						p_t_end_list.push_back(t_end_max);
//					}
//				}
//				if (size(allowed_insert_p_list) == 0) {
//					continue;
//				}
//				int max_p_idx = 0;
//				float tmp_t_end = LONG_MAX;
//				for (int k = 0; k < size(allowed_insert_p_list); k++) {
//					if (tmp_t_end > p_t_end_list[k]) {
//						tmp_t_end = p_t_end_list[k];
//						max_p_idx = k;
//					}
//				}
//				int insert_p_f = allowed_insert_p_list[max_p_idx];
//				sol_tmp.node_list.insert(sol_tmp.node_list.begin() + insert_p_f, node_to_insert);
//				sol_tmp.assign_node_no();
//				node_to_insert.no = sol_tmp.node_list[insert_p_f].no;
//				for (auto uav_tt_idx : node_to_insert.UAV_id_list) {
//					int fl = 0;
//					UAV& uav_tt = sol_tmp.id_to_UAV(uav_tt_idx);
//					if (size(uav_tt.Path) == 0) {
//						uav_tt.Path.push_back(node_to_insert);
//						continue;
//					}
//					for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
//						int idx_first = uav_tt.Path[i_tt].id;
//						int idx_second = uav_tt.Path[i_tt + 1].id;
//						auto& node_first = sol_tmp.id_to_node(idx_first);
//						auto& node_second = sol_tmp.id_to_node(idx_second);
//						if (sol_tmp.compare_node(node_first, node_to_insert) && (sol_tmp.compare_node(node_to_insert, node_second))) {
//							uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_to_insert);
//							fl = 1;
//							break;
//						}
//					}
//					if (fl == 0) {
//						int node_first_id = uav_tt.Path[0].id;
//						int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
//						Working_node& node_first_0 = sol_tmp.id_to_node(node_first_id);
//						auto& node_last_0 = sol_tmp.id_to_node(node_last_id);
//						if (sol_tmp.compare_node(node_to_insert, node_first_0)) {
//							uav_tt.Path.insert(uav_tt.Path.begin(), node_to_insert);
//							continue;
//						}
//						if (sol_tmp.compare_node(node_last_0, node_to_insert)) {
//							uav_tt.Path.insert(uav_tt.Path.end(), node_to_insert);
//							continue;
//						}
//					}
//				}
//				if (sol_tmp.need_repair()) {
//					cout << "construct a invalid solution" << endl;
//				}
//			}
//		}
//		*this = sol_tmp;
//		if (this->need_repair()) {
//			cout << "construct a invalid solution" << endl;
//		}
//	}
//}


void Solution::repair_ig(int flag_mode) {
	//确定每个任务点与UAV路径中路径点的关联；
	node_Path_node_connect();
	int size_ini = size(node_list);
	auto sol_tmp = *this;
	sol_tmp.clear();
	int count = 0;
	int MAX_COUNT = 20;
	set<int> node_no_set;
	int node_no_tmp = -1;
	//cout << size(node_list)<<endl;
	int node_id_i = -1;
	for (auto node : node_list) {
		node_id_i++;
		node_no_tmp++;
		node.Path_node_no_list.clear();
		float in_pro = (float)rand() / (float)RAND_MAX;
		float MAX_pro;

		float index_n;
		//index_cal
		int no_id = size(sol_tmp.node_list);

		float V_node=0;
		float t_sum = 0;
		for (auto uav_sub : sol_tmp.UAV_list) {
			for (auto uav_s : uav_sub) {
				t_sum += (T - uav_s.t_used);
			}
		}
		if (t_sum < node.node_size / UAV::eff) {
			continue;
		}
		vector<float> v_value;
		vector<int> rel_vec;
		for (int node_no = node_id_i; node_no < size(node_list); node_no++) {
			int rel_num = 0;
			auto& node_i =node_list[node_no];
			set<int> type_i = node_i.type;
			if (node_no > 0) {
				for (int node_rel = 0; node_rel < no_id; node_rel++) {
					auto& node_ii = sol_tmp.node_list[node_rel];
					set<int> type_ii = node_ii.type;
					int re_flag = 0;
					for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
						for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
							if (*iter == *iter_i) {
								re_flag = 1;
							}
						}
					}
					if (re_flag == 1) {
						rel_num++;
					}
				}
			}
			rel_vec.push_back(rel_num);
		}
		for (int v_i = node_id_i; v_i < size(node_list); v_i++) {
			int id_tmp = node_list[v_i].id;
			float v_tmp = 0;
			int kk = -1;
			int s_size = 0;
			//for (int v_i_i = no_id; v_i_i < MAP_NODE_NUM; v_i_i++) {
			//	kk++;
			//	v_tmp += Q_table_1[id_tmp][rel_vec[v_i- node_id_i]+kk][size(node_list[v_i].UAV_id_list) - 1]/(kk+1);////( kk +1)(v_i_i+1)
			//	//if (Q_table_1[id_tmp][rel_vec[v_i - node_id_i] + kk][size(node_list[v_i].UAV_id_list) - 1] > 0) {
			//	//	s_size += 1;
			//	//}
			//}
			////if (s_size > 0) {
			////	v_tmp /= s_size;
			////}
			//
			////v_tmp = Q_table_1[id_tmp][rel_vec[v_i - node_id_i]][size(node_list[v_i].UAV_id_list) - 1];
			////cout << v_tmp << endl;
			v_tmp = Q_table_1[id_tmp][rel_vec[v_i - node_id_i]][size(node_list[v_i].UAV_id_list) - 1];
			v_value.push_back(v_tmp);///(v_i+1)
		}
		int num_in = MAP_NODE_NUM;
		V_node = v_value[0];
		sort(v_value.begin(), v_value.end(), [](float a, float b) {return a < b; });
		for (int i_no = 0; i_no < size(v_value); i_no++) {
			if (V_node == v_value[i_no]) {
				num_in = i_no;
				break;
			}
		}

		//for (int v_i = no_id; v_i < MAP_NODE_NUM; v_i++) {
		//	V_node += Q_table_1[node.id][v_i][size(node.UAV_id_list)-1];
		//}
		
		//auto M_tmp = V_matrix[no_id];
		//V_node = M_tmp[node.id];
		//float max_M=0;
		//for (int i = 0; i < MAP_NODE_NUM; i++) {
		//	max_M = max(max_M, M_tmp[i]);
		//	for (int j = 0; j < MAP_NODE_NUM - 1; j++) {
		//		if (M_tmp[j] > M_tmp[j + 1]) {
		//			float tmp = M_tmp[j + 1];
		//			M_tmp[j + 1] = M_tmp[j];
		//			M_tmp[j] = tmp;
		//			
		//		}
		//	}
		//}
		//for (int i = 0; i < MAP_NODE_NUM; i++) {
		//	if (M_tmp[i] > V_node) {
		//		num_in = i;
		//		break;
		//	}
		//}
		index_n = (float)(num_in + 1)/ (size(v_value));
		//cout << index_n << endl;

		MAX_pro = index_n;
		//cout << MAX_pro << endl;
		//cout << MAX_pro << endl;
		if (mode == 2) {
			MAX_pro = (float)rand() / RAND_MAX;
		}
#ifdef IG
		MAX_PRO = 0.3
#endif

#ifdef ABC
			MAX_PRO = 0.3
#endif

#ifdef HS
			MAX_PRO = 0.8
#endif


#ifdef gwo
			MAX_PRO = 0.3
#endif
			//cout << MAX_pro << endl;
			//if (flag_mode == 0){
			//	MAX_pro = 0.8;
			//}
			//if (flag_mode == 1) {
			//	MAX_pro = 0.6;
			//}
			//加入任务点序列中
		if (in_pro < MAX_pro) {
			vector<UAV> col_UAV_list;
			vector<int> tmp_uav_id_list = node.UAV_id_list;
			int j_tmp = -1;
			int break_flag = 0;
			for (int i = 0; i < size(tmp_uav_id_list); i++) {
				j_tmp++;
				int idx = node.UAV_id_list[j_tmp];
				UAV& uav_i = sol_tmp.id_to_UAV(idx);
				if (uav_i.cal_t_go_to(node) >= T) {
					//node.UAV_id_list.erase(node.UAV_id_list.begin() + j_tmp);
					//j_tmp--;
					//continue;
					break_flag = 1;
					break;
				}
				else {
					col_UAV_list.push_back(uav_i);
				}
			}

			if ((size(col_UAV_list) == 0) || (break_flag == 1)) {
				count++;
				continue;
			}
			for (auto& uav_col : col_UAV_list) {
				uav_col.Path.push_back(node);
				node.Path_node_no_list.push_back(size(uav_col.Path) - 1);
			}
			float t_tmp_end = node.cal_time_end(col_UAV_list);
			if (t_tmp_end > T) {
				count++;
				continue;
			}
			else {
				node_no_set.insert(node_no_tmp);
				count = 0;
				node.t_end = t_tmp_end;
				sol_tmp.node_list.push_back(node);
				//cout << node_no_tmp << " ";
				for (auto& uav_l_tmp : sol_tmp.UAV_list) {
					for (auto& uav_tmp : uav_l_tmp) {
						for (auto& uav_r : col_UAV_list) {
							if (uav_tmp.id == uav_r.id) {
								uav_tmp.Path = uav_r.Path;
								uav_tmp.t_used = uav_r.t_used;
								break;
							}
						}
					}
				}
			}
		}
		//if (count == MAX_COUNT) {
		//	break;
		//}
	}
	//cout <<size(node_no_set)<< endl;
	//sol_tmp.clear_ext_path_node(1);
	count = 0;
	set<int> node_ids;
	for (auto node : sol_tmp.node_list) {
		node_ids.insert(node.id);
	}
	while (size(node_ids) < size_ini) {
		//int break_flag1 = 1;
		float t_sum = 0;
		for (auto uav_sub : sol_tmp.UAV_list) {
			for (auto uav_s : uav_sub) {
				t_sum += (T - uav_s.t_used);
			}
		}
		vector<Working_node> vector_al;
		for (auto node : node_list) {
			if (node_ids.find(node.id) == node_ids.end() && t_sum > node.node_size / UAV::eff) {
				vector_al.push_back(node);
			}
			else {
				node_ids.insert(node.id);
			}
		}
		//int rnd_node_no = rand() % size(vector_al);
		//
		//node_no_set.insert(rnd_node_no);			
		//if (t_sum < node_list[rnd_node_no].node_size / UAV::eff) {
		//	continue;
		//}
		if (size(vector_al) == 0) {
			break;
		}
		sort(vector_al.begin(), vector_al.end(), [](Working_node& a, Working_node& b) {return a.node_size > b.node_size; });
		//random_shuffle(vector_al.begin(), vector_al.end());
		auto node_to_insert = vector_al[0];
		node_ids.insert(vector_al[0].id);
		//float u_t=0;
		//for (auto uav_tt_idx : node_to_insert.UAV_id_list) {
		//	u_t += (T - sol_tmp.id_to_UAV(uav_tt_idx).t_used);
		//}
		//if (u_t < node_to_insert.node_size / UAV::eff) {
		//	continue;
		//}
		vector<int> allowed_insert_p_list;
		vector<float> p_t_end_list;
		int insert_col = size(node_to_insert.UAV_id_list);
		vector<vector<int>> UAV_ID_tmp;
		//int rnd_col_num = 0;

		for (int insert_p = 0; insert_p <= size(sol_tmp.node_list); insert_p++) {
			int rel_num = 0;
			set<int> type_i = node_to_insert.type;
			if (insert_p > 0) {
				for (int node_rel = 0; node_rel < insert_p; node_rel++) {
					auto& node_ii = sol_tmp.node_list[node_rel];
					set<int> type_ii = node_ii.type;
					int re_flag = 0;
					for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
						for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
							if (*iter == *iter_i) {
								re_flag = 1;
							}
						}
					}
					if (re_flag == 1) {
						rel_num++;
					}
				}
			}
			int max_col = min(node_to_insert.col_UAV_num, UAV_NUM);
			float max_col_q = 0;
			for (int col_i = 0; col_i < max_col; col_i++) {
				if (Q_table_1[node_to_insert.id][rel_num][col_i] > max_col_q) {
					insert_col = col_i + 1;
					max_col_q = Q_table_1[node_to_insert.id][rel_num][col_i];
				}
			}
			node_to_insert.UAV_id_list = sol_tmp.make_uavs_assigned(node_to_insert, insert_p, insert_col);
#ifdef rand_u
			node_to_insert.UAV_id_list = sol_tmp.make_uavs_assigned_r(node_to_insert, insert_p, insert_col);
#endif // rand_u

			if (size(node_to_insert.UAV_id_list) == 0) {
				continue;
			}
			Solution sol_tt = sol_tmp;
			sol_tt.node_list.insert(sol_tt.node_list.begin() + insert_p, node_to_insert);
			sol_tt.assign_node_no();
			node_to_insert.no = sol_tt.node_list[insert_p].no;
				
			for (auto uav_tt_idx : node_to_insert.UAV_id_list) {
				int fl = 0;
				UAV& uav_tt = sol_tt.id_to_UAV(uav_tt_idx);
				if (size(uav_tt.Path) == 0) {
					uav_tt.Path.push_back(node_to_insert);
					continue;
				}
				for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
					int idx_first = uav_tt.Path[i_tt].id;
					int idx_second = uav_tt.Path[i_tt + 1].id;
					auto& node_first = sol_tt.id_to_node(idx_first);
					auto& node_second = sol_tt.id_to_node(idx_second);
					if (sol_tt.compare_node(node_first, node_to_insert) && (sol_tt.compare_node(node_to_insert, node_second))) {
						uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_to_insert);
						fl = 1;
						break;
					}
				}
				if (fl == 0) {
					int node_first_id = uav_tt.Path[0].id;
					int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
					Working_node& node_first_0 = sol_tt.id_to_node(node_first_id);
					auto& node_last_0 = sol_tt.id_to_node(node_last_id);
					if (sol_tt.compare_node(node_to_insert, node_first_0)) {
						uav_tt.Path.insert(uav_tt.Path.begin(), node_to_insert);
						continue;
					}
					if (sol_tt.compare_node(node_last_0, node_to_insert)) {
						uav_tt.Path.insert(uav_tt.Path.end(), node_to_insert);
						continue;
					}
				}
			}
			if (!sol_tt.need_repair()) {
				allowed_insert_p_list.push_back(insert_p);
				UAV_ID_tmp.push_back(node_to_insert.UAV_id_list);
				float t_end_max = 0;
				//for (int k = 0; k < size(sol_tt.node_list); k++) {
				//	t_end_max = max(t_end_max, sol_tt.node_list[k].t_end);
				//}
				for (auto uav_l : sol_tt.UAV_list) {
					for (auto uav_ll : uav_l) {
						t_end_max += uav_ll.t_used;
					}
				}
				p_t_end_list.push_back(t_end_max);
			}
		}
		if (size(allowed_insert_p_list) == 0) {
			continue;
		}
		int max_p_idx = 0;
		float tmp_t_end = FLT_MAX;
		for (int k = 0; k < size(allowed_insert_p_list); k++) {
			if (tmp_t_end > p_t_end_list[k]) {
				tmp_t_end = p_t_end_list[k];
				max_p_idx = k;
			}
		}
		//max_p_idx = rand() % size(allowed_insert_p_list);
		int insert_p_f = allowed_insert_p_list[max_p_idx];
		node_to_insert.UAV_id_list = UAV_ID_tmp[max_p_idx];
		sol_tmp.node_list.insert(sol_tmp.node_list.begin() + insert_p_f, node_to_insert);
		sol_tmp.assign_node_no();
		node_to_insert.no = sol_tmp.node_list[insert_p_f].no;
		for (auto uav_tt_idx : node_to_insert.UAV_id_list) {
			int fl = 0;
			UAV& uav_tt = sol_tmp.id_to_UAV(uav_tt_idx);
			if (size(uav_tt.Path) == 0) {
				uav_tt.Path.push_back(node_to_insert);
				continue;
			}
			for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
				int idx_first = uav_tt.Path[i_tt].id;
				int idx_second = uav_tt.Path[i_tt + 1].id;
				auto& node_first = sol_tmp.id_to_node(idx_first);
				auto& node_second = sol_tmp.id_to_node(idx_second);
				if (sol_tmp.compare_node(node_first, node_to_insert) && (sol_tmp.compare_node(node_to_insert, node_second))) {
					uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_to_insert);
					fl = 1;
					break;
				}
			}
			if (fl == 0) {
				int node_first_id = uav_tt.Path[0].id;
				int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
				Working_node& node_first_0 = sol_tmp.id_to_node(node_first_id);
				auto& node_last_0 = sol_tmp.id_to_node(node_last_id);
				if (sol_tmp.compare_node(node_to_insert, node_first_0)) {
					uav_tt.Path.insert(uav_tt.Path.begin(), node_to_insert);
					continue;
				}
				if (sol_tmp.compare_node(node_last_0, node_to_insert)) {
					uav_tt.Path.insert(uav_tt.Path.end(), node_to_insert);
					continue;
				}
			}
		}
		if (sol_tmp.need_repair()) {
			cout << "construct a invalid solution" << endl;
		}
	}
	*this = sol_tmp;
	if (this->need_repair()) {
		cout << "construct a invalid solution" << endl;
	}
}

void Solution::local_search(int it) {
	int it_i = 0;
	int rnd_ls_t = 1;
	if (size(node_list) < 2) {
		return;
	}
	int max_path_num = 0;
	for (int i = 0; i < size(UAV_list);i++) {
		for (int ii = 0; ii < size(UAV_list[i]); ii++) {
			max_path_num = max(max_path_num, (int)size(UAV_list[i][ii].Path));
		}
	}
	if (max_path_num < 2) {
		return;
	}
	while (it_i < rnd_ls_t) {
		assign_node_no();
		set<int> switch_nums;
		int rnd_UAV_set = rand() % (size(UAV_list));
		int rnd_UAV_no = rand() % (size(UAV_list[rnd_UAV_set]));
		if (size(UAV_list[rnd_UAV_set][rnd_UAV_no].Path) < 2) {
			continue;
		}
		else {
			while (size(switch_nums) < 2) {
				int rnd_num = rand() % (size(UAV_list[rnd_UAV_set][rnd_UAV_no].Path));
				if (switch_nums.find(rnd_num) == switch_nums.end()) {
					switch_nums.insert(rnd_num);
				}
			}
		}
		auto iter = switch_nums.begin();
		int first_p = id_to_node(UAV_list[rnd_UAV_set][rnd_UAV_no].Path[*iter].id).no;
	
		auto node_in_first = node_list[first_p];
		iter++;
		int second_p = id_to_node(UAV_list[rnd_UAV_set][rnd_UAV_no].Path[*iter].id).no;
		auto node_in_second = node_list[second_p];
		node_list.erase(node_list.begin() + second_p);
		node_list.erase(node_list.begin() + first_p);
		for (auto uav_id : node_in_second.UAV_id_list) {
			auto& uav_c = id_to_UAV(uav_id);
			uav_c.erase_node_i_in_path(node_in_second.id);
		}
		for (auto uav_id : node_in_first.UAV_id_list) {
			auto& uav_c = id_to_UAV(uav_id);
			uav_c.erase_node_i_in_path(node_in_first.id);
		}
		node_list.insert(node_list.begin() + first_p, node_in_second);
		node_list.insert(node_list.begin() + second_p, node_in_first);
		assign_node_no();
		node_in_second.no = node_list[first_p].no;
		node_in_first.no = node_list[second_p].no;
		//int ss_flag = 0;
		//for (auto iter = node_in_first.type.begin(); iter != node_in_first.type.end(); iter++) {
		//	for (auto iter_i = node_in_second.type.begin(); iter_i != node_in_second.type.end(); iter++) {
		//		if (*iter != *iter_i) {
		//			ss_flag = 1;
		//			break;
		//		}
		//	}
		//	if (ss_flag == 1) {
		//		break;
		//	}
		//}
		//if (ss_flag==0) {
		//	auto tmp_set = node_in_first.UAV_id_list;
		//	node_in_first.UAV_id_list = node_in_second.UAV_id_list;
		//	node_in_second.UAV_id_list = tmp_set;
		//}
		for (int id_in : node_in_second.UAV_id_list) {
			UAV& uav_out = id_to_UAV(id_in);
			int fl = 0;
			if (size(uav_out.Path) == 0) {
				uav_out.Path.push_back(node_in_second);
				continue;
			}
			for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
				int idx_first = uav_out.Path[i_tt].id;
				int idx_second = uav_out.Path[i_tt + 1].id;
				auto& node_first = id_to_node(idx_first);
				auto& node_second = id_to_node(idx_second);
				if (compare_node(node_first, node_in_second) && (compare_node(node_in_second, node_second))) {
					uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node_in_second);
					fl = 1;
					break;
				}
			}
			if (fl == 1) {
				continue;
			}
			if (fl == 0) {
				int node_first_id = uav_out.Path[0].id;
				int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
				auto& node_first_0 = id_to_node(node_first_id);
				auto& node_last_0 = id_to_node(node_last_id);
				if (compare_node(node_in_second, node_first_0)) {
					uav_out.Path.insert(uav_out.Path.begin(), node_in_second);
					continue;
				}
				if (compare_node(node_last_0, node_in_second)) {
					uav_out.Path.insert(uav_out.Path.end(), node_in_second);
					continue;
				}
			}
			cout << "fail_insert" << endl;
		}
		for (int id_in : node_in_first.UAV_id_list) {
			UAV& uav_out = id_to_UAV(id_in);
			int fl = 0;
			if (size(uav_out.Path) == 0) {
				uav_out.Path.push_back(node_in_first);
				continue;
			}
			for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
				int idx_first = uav_out.Path[i_tt].id;
				int idx_second = uav_out.Path[i_tt + 1].id;
				auto& node_first = id_to_node(idx_first);
				auto& node_second = id_to_node(idx_second);
				if (compare_node(node_first, node_in_first) && (compare_node(node_in_first, node_second))) {
					uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node_in_first);
					fl = 1;
					break;
				}

			}
			if (fl == 1) {
				continue;
			}
			if (fl == 0) {
				int node_first_id = uav_out.Path[0].id;
				int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
				auto& node_first_0 = id_to_node(node_first_id);
				auto& node_last_0 = id_to_node(node_last_id);
				if (compare_node(node_in_first, node_first_0)) {
					uav_out.Path.insert(uav_out.Path.begin(), node_in_first);
					continue;
				}
				if (compare_node(node_last_0, node_in_first)) {
					uav_out.Path.insert(uav_out.Path.end(), node_in_first);
					continue;
				}
			}
			cout << "fail_insert" << endl;
		}
		it_i++;
	}
}


void Solution::insert_rnd_nodes(const vector<Working_node>& wn_list, int unchange_counter){
	set<int> node_id;
	for (auto& node_i :node_list) {
		node_id.insert(node_i.id);
	}
	vector<Working_node> nodes_left;
	vector<float> t_lefts;
	vector<vector<UAV>>& uav_list = UAV_list;
	Solution sol_gen = (*this);
	t_lefts.resize(size(uav_list));
	float t_max = 0;
	vector<vector<int>> uav_l_l;
	for (int i = 0; i < size(uav_list); i++) {
		vector<int> uav_l;
		for (int j = 0; j < size(uav_list[i]); j++) {
			t_lefts[i] += T - uav_list[i][j].t_used;
			uav_l.push_back(uav_list[i][j].id);
		}
		t_max += t_lefts[i];
		uav_l_l.push_back(uav_l);
	}

	for (auto& node_i : wn_list) {
		if (node_id.find(node_i.id) == node_id.end() && node_i.node_size / UAV::eff < t_max) {
			nodes_left.push_back(node_i);
		}
		else {
			node_id.insert(node_i.id);
		}
	}
	if (size(nodes_left) == 0) {
		return;
	}
	sort(nodes_left.begin(), nodes_left.end(), [](Working_node& a, Working_node& b) {return a.node_size > b.node_size; });
	//random_shuffle(nodes_left.begin(), nodes_left.end());
	for (auto& node_i : nodes_left) {
		//auto& node_i = nodes_left[0];
		node_id.insert(node_i.id);
		int insert_p = INT_MAX;
		//vector<int> insert_ps;
		//vector<vector<int>> UAV_id_listsl;
		float min_t_max = LONG_MAX;
		set<int> type_node = node_i.type;
		float t_al = 0;
		vector<int> uav_id_tmp;
		for (auto iter = type_node.begin(); iter != type_node.end(); iter++) {
			t_al += t_lefts[*iter];
		}
		if (t_al < node_i.node_size / UAV::eff) {
			continue;
		}
		else {
			vector<int> uavs_allowed_id_list;
			set<int> type_allowed = node_i.type;
			for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
				auto uav_sub_list = UAV_list[*iter];
				for (auto& uav : uav_sub_list) {
					uavs_allowed_id_list.push_back(uav.id);
				}
			}
			for (int i = 0; i <= size(sol_gen.node_list); i++) {
				Solution sol_tt = sol_gen;
				int rnd_col_num = 0;
				int max_col = min(node_i.col_UAV_num, UAV_NUM);
				int choosed_col_num = 1;
				float max_col_q = 0;
				for (int col_i = 0; col_i < max_col; col_i++) {
					if (Q_table_1[node_i.id][i][col_i] > max_col_q) {
						choosed_col_num = col_i + 1;
						max_col_q = Q_table_1[node_i.id][i][col_i];
					}
				}

				float rand_p_col = (float)rand() / RAND_MAX;
				if (rand_p_col < eps || max_col_q == 0) {
					choosed_col_num = rand() % max_col + 1;
					rnd_col_num = min((int)choosed_col_num, (int)size(uavs_allowed_id_list));
					//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);
				}
				else {

					rnd_col_num = min((int)choosed_col_num, (int)size(uavs_allowed_id_list));
					//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);

				}
				if (rnd_col_num < 0) {
					rnd_col_num = 1;
				}
				//int rnd_col_num = min(choosed_col_num, (int)size(uavs_allowed_id_list));
				vector<int> uav_col_id_list = make_uavs_assigned(node_i, i, rnd_col_num);
				if (size(uav_col_id_list) == 0) {
					continue;
				}
				//float t_again = 0;
				//for (auto uav_id_k : uav_col_id_list) {
				//	t_again += T - id_to_UAV(uav_id_k).t_used;
				//}
				//if (t_again < node_i.node_size / UAV::eff) {
				//	continue;
				//}
				node_i.UAV_id_list = uav_col_id_list;
				sol_tt.node_list.insert(sol_tt.node_list.begin() + i, node_i);
				sol_tt.assign_node_no();
				node_i.no = i;


				for (int uav_tt_idx : sol_tt.node_list[i].UAV_id_list) {
					int fl = 0;
					UAV& uav_tt = sol_tt.id_to_UAV(uav_tt_idx);
					if (size(uav_tt.Path) == 0) {
						uav_tt.Path.push_back(node_i);
						continue;
					}
					for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
						int idx_first = uav_tt.Path[i_tt].id;
						int idx_second = uav_tt.Path[i_tt + 1].id;
						auto& node_first = sol_tt.id_to_node(idx_first);
						auto& node_second = sol_tt.id_to_node(idx_second);
						if (sol_tt.compare_node(node_first, node_i) && (sol_tt.compare_node(node_i, node_second))) {
							uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_i);
							fl = 1;
							break;
						}
					}
					if (fl == 0) {
						int node_first_id = uav_tt.Path[0].id;
						int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
						Working_node& node_first_0 = sol_tt.id_to_node(node_first_id);
						auto& node_last_0 = sol_tt.id_to_node(node_last_id);
						if (sol_tt.compare_node(node_i, node_first_0)) {
							uav_tt.Path.insert(uav_tt.Path.begin(), node_i);
							continue;
						}
						if (sol_tt.compare_node(node_last_0, node_i)) {
							uav_tt.Path.insert(uav_tt.Path.end(), node_i);
							continue;
						}
					}
				}
				if (!sol_tt.need_repair()) {
					//insert_ps.push_back(i);
					//UAV_id_listsl.push_back(uav_col_id_list);
					float max_node_t = 0;
					for (auto node_ii : sol_tt.node_list) {
						max_node_t = max(max_node_t, node_ii.t_end);
					}
					if (min_t_max > max_node_t) {
						min_t_max = max_node_t;
						insert_p = i;
						uav_id_tmp = node_i.UAV_id_list;
					}
				}
			}
			//if (size(insert_ps) == 0) {
			//	continue;
			//}
			//int kk = rand() % size(insert_ps);
			//insert_p = insert_ps[kk];
			//node_i.UAV_id_list = UAV_id_listsl[kk];
			if (insert_p != INT_MAX) {
				node_i.UAV_id_list = uav_id_tmp;
				sol_gen.node_list.insert(sol_gen.node_list.begin() + insert_p, node_i);
				sol_gen.assign_node_no();
				node_i.no = sol_gen.node_list[insert_p].no;
				for (int uav_tt_idx : node_i.UAV_id_list) {
					int fl = 0;
					UAV& uav_tt = sol_gen.id_to_UAV(uav_tt_idx);
					if (size(uav_tt.Path) == 0) {
						uav_tt.Path.push_back(node_i);
						continue;
					}
					for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
						int idx_first = uav_tt.Path[i_tt].id;
						int idx_second = uav_tt.Path[i_tt + 1].id;
						auto& node_first = sol_gen.id_to_node(idx_first);
						auto& node_second = sol_gen.id_to_node(idx_second);
						if (sol_gen.compare_node(node_first, node_i) && (sol_gen.compare_node(node_i, node_second))) {
							uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_i);
							fl = 1;
							break;
						}
					}
					if (fl == 0) {
						int node_first_id = uav_tt.Path[0].id;
						int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
						Working_node& node_first_0 = sol_gen.id_to_node(node_first_id);
						auto& node_last_0 = sol_gen.id_to_node(node_last_id);
						if (sol_gen.compare_node(node_i, node_first_0)) {
							uav_tt.Path.insert(uav_tt.Path.begin(), node_i);
							continue;
						}
						if (sol_gen.compare_node(node_last_0, node_i)) {
							uav_tt.Path.insert(uav_tt.Path.end(), node_i);
							continue;
						}
					}
				}
				*this = sol_gen;
				((*this).need_repair());
				Solution sol_tmp = (*this);
				(*this).clear_ext_path_node(1);

			}
			//else {
			//	break;
			//}
		}
	}
}

//void Solution::insert_rnd_nodes(const vector<Working_node>& wn_list, int unchange_counter) {
//	set<int> node_id;
//	for (auto& node_i : node_list) {
//		node_id.insert(node_i.id);
//	}
//	vector<vector<UAV>>& uav_list = UAV_list;
//	while (size(node_id) < size(wn_list)) {
//		vector<Working_node> nodes_left;
//		vector<float> t_lefts;
//		t_lefts.resize(size(uav_list));
//		float t_max = 0;
//		vector<vector<int>> uav_l_l;
//		for (int i = 0; i < size(uav_list); i++) {
//			vector<int> uav_l;
//			for (int j = 0; j < size(uav_list[i]); j++) {
//				t_lefts[i] += T - uav_list[i][j].t_used;
//				uav_l.push_back(uav_list[i][j].id);
//			}
//			t_max += t_lefts[i];
//			uav_l_l.push_back(uav_l);
//		}
//
//		for (auto& node_i : wn_list) {
//			if (node_id.find(node_i.id) == node_id.end() && node_i.node_size / UAV::eff < t_max) {
//				nodes_left.push_back(node_i);
//			}
//			else {
//				node_id.insert(node_i.id);
//			}
//		}
//		if (size(nodes_left) == 0) {
//			break;
//		}
//		sort(nodes_left.begin(), nodes_left.end(), [](Working_node& a, Working_node& b) {return a.node_size > b.node_size; });
//		//random_shuffle(nodes_left.begin(), nodes_left.end());
//		auto& node_i = nodes_left[0];
//		node_id.insert(node_i.id);
//		set<int> type_node = node_i.type;
//		float t_al = 0;
//		vector<int> uav_id_tmp;
//		for (auto iter = type_node.begin(); iter != type_node.end(); iter++) {
//			t_al += t_lefts[*iter];
//		}
//		if (t_al < node_i.node_size / UAV::eff) {
//			continue;
//		}
//		else {
//			vector<int> uavs_allowed_id_list;
//			set<int> type_allowed = node_i.type;
//			for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
//				auto uav_sub_list = UAV_list[*iter];
//				for (auto& uav : uav_sub_list) {
//					uavs_allowed_id_list.push_back(uav.id);
//				}
//			}
//			int i = rand() % (size((*this).node_list)+1);
//			Solution sol_tt = (*this);
//			int rnd_col_num = 0;
//			int max_col = min(node_i.col_UAV_num, UAV_NUM);
//			int choosed_col_num = 1;
//			float max_col_q = 0;
//			for (int col_i = 0; col_i < max_col; col_i++) {
//				if (Q_table_1[node_i.id][i][col_i] > max_col_q) {
//					choosed_col_num = col_i + 1;
//					max_col_q = Q_table_1[node_i.id][i][col_i];
//				}
//			}
//
//			float rand_p_col = (float)rand() / RAND_MAX;
//			if (rand_p_col < eps || max_col_q == 0) {
//				choosed_col_num = rand() % max_col + 1;
//				rnd_col_num = min((int)choosed_col_num, (int)size(uavs_allowed_id_list));
//				//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);
//			}
//			else {
//
//				rnd_col_num = min((int)choosed_col_num, (int)size(uavs_allowed_id_list));
//				//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);
//
//			}
//			if (rnd_col_num < 0) {
//				rnd_col_num = 1;
//			}
//			//int rnd_col_num = min(choosed_col_num, (int)size(uavs_allowed_id_list));
//			//cout << i << endl;
//			vector<int> uav_col_id_list = make_uavs_assigned(node_i, i, rnd_col_num);
//			if (size(uav_col_id_list) == 0) {
//				continue;
//			}
//			//float t_again = 0;
//			//for (auto uav_id_k : uav_col_id_list) {
//			//	t_again += T - id_to_UAV(uav_id_k).t_used;
//			//}
//			//if (t_again < node_i.node_size / UAV::eff) {
//			//	continue;
//			//}
//			node_i.UAV_id_list = uav_col_id_list;
//			sol_tt.node_list.insert(sol_tt.node_list.begin() + i, node_i);
//			sol_tt.assign_node_no();
//			node_i.no = i;
//			for (int uav_tt_idx : sol_tt.node_list[i].UAV_id_list) {
//				int fl = 0;
//				UAV& uav_tt = sol_tt.id_to_UAV(uav_tt_idx);
//				if (size(uav_tt.Path) == 0) {
//					uav_tt.Path.push_back(node_i);
//					continue;
//				}
//				for (int i_tt = 0; i_tt < size(uav_tt.Path) - 1; i_tt++) {
//					int idx_first = uav_tt.Path[i_tt].id;
//					int idx_second = uav_tt.Path[i_tt + 1].id;
//					auto& node_first = sol_tt.id_to_node(idx_first);
//					auto& node_second = sol_tt.id_to_node(idx_second);
//					if (sol_tt.compare_node(node_first, node_i) && (sol_tt.compare_node(node_i, node_second))) {
//						uav_tt.Path.insert(uav_tt.Path.begin() + i_tt + 1, node_i);
//						fl = 1;
//						break;
//					}
//				}
//				if (fl == 0) {
//					int node_first_id = uav_tt.Path[0].id;
//					int node_last_id = uav_tt.Path[size(uav_tt.Path) - 1].id;
//					Working_node& node_first_0 = sol_tt.id_to_node(node_first_id);
//					auto& node_last_0 = sol_tt.id_to_node(node_last_id);
//					if (sol_tt.compare_node(node_i, node_first_0)) {
//						uav_tt.Path.insert(uav_tt.Path.begin(), node_i);
//						continue;
//					}
//					if (sol_tt.compare_node(node_last_0, node_i)) {
//						uav_tt.Path.insert(uav_tt.Path.end(), node_i);
//						continue;
//					}
//				}
//			}
//			if (!sol_tt.need_repair()) {
//				(*this) = sol_tt;
//			}
//			//else {
//			//	break;
//			//}
//		}
//	}
//}

//void Solution::insert_rnd_nodes(const vector<Working_node>& wn_list, int unchange_counter) {
//#ifdef IG
//	TRY_T_MAX = 15;
//#endif // IG
//
//#ifdef HS
//	TRY_T_MAX = 15;
//#endif // HS
//	int try_i = 0;
//	set<int> id_a;
//	for (auto node_i : node_list) {
//		id_a.insert(node_i.id);
//	}
//	while (try_i < TRY_T_MAX && size(id_a)<size(wn_list) ) {
//		float t_limi=0;
//		for (auto uav_sub : UAV_list) {
//			for (auto uav_s : uav_sub) {
//				t_limi += (T - uav_s.t_used);
//			}
//		}
//		vector<int> idsl_left;
//		for (auto node_chh : node_list) {
//			if (id_a.find(node_chh.id) == id_a.end()&& t_limi<node_chh.node_size/UAV::eff) {
//				idsl_left.push_back(node_chh.id);
//			}
//			else {
//				id_a.insert(node_chh.id);
//			}
//		}
//		if(size(idsl_left)==0){
//			break;
//		}
//		int rnd_idx = idsl_left[rand() % (size(idsl_left))];
//		if (!check_node_id(rnd_idx)) {
//			id_a.insert(rnd_idx);
//			try_i++;
//			int try_col = 0;
//			int MAX_TRY_COL = 1;
//			auto sol_tmp = *this;
//			int rnd_insert_p = rand() % (size(node_list) + 1);
//			auto node_to_insert = wn_list[rnd_idx];
//			auto& node_list_tmp = sol_tmp.node_list;
//			node_list_tmp.insert(node_list_tmp.begin() + rnd_insert_p, node_to_insert);
//			sol_tmp.assign_node_no();
//			node_to_insert.no = node_list_tmp[rnd_insert_p].no;
//			vector<int> uavs_allowed_id_list;
//			set<int> type_allowed = node_to_insert.type;
//			for (auto iter = type_allowed.begin(); iter != type_allowed.end(); iter++) {
//				auto uav_sub_list = sol_tmp.UAV_list[*iter];
//				for (auto& uav : uav_sub_list) {
//					uavs_allowed_id_list.push_back(uav.id);
//				}
//			}
//			while (try_col < MAX_TRY_COL) {
//				int rnd_col_num = 0;
//				int max_col = min(node_to_insert.col_UAV_num, UAV_NUM);
//				int choosed_col_num = 1;
//				float max_col_q = Q_table_1[node_to_insert.id][node_to_insert.no][0];
//				for (int col_i = 0; col_i < max_col; col_i++) {
//					if (Q_table_1[node_to_insert.id][node_to_insert.no][col_i] > max_col_q) {
//						choosed_col_num = col_i + 1;
//						max_col_q = Q_table_1[node_to_insert.id][node_to_insert.no][col_i];
//					}
//				}
//
//				float rand_p_col = (float)rand() / RAND_MAX;
//				if (rand_p_col < eps || max_col_q == 0) {
//					choosed_col_num = rand() % max_col + 1;
//					rnd_col_num = min((int)choosed_col_num, (int)size(uavs_allowed_id_list));
//					//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);
//				}
//				else {
//					//double U1 = (float)rand() / (float)RAND_MAX;
//					//double U2 = (float)rand() / (float)RAND_MAX;
//					//double Z = sqrt(-2 * log(U1)) * cos(2 * pi * U2);
//
//					//double min_std = (float)2 / (float)3;
//					//double max_std = (float)max(4, node_to_insert.col_UAV_num) / (float)6;
//					////double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(IT), 1.6);
//
//					//unchange_counter = min(MAX_ACO_IT, unchange_counter);
//					//double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(MAX_ACO_IT), 1);
//
//					//double Y = choosed_col_num + Z * std;
//					////cout << choosed_col_num<<" "<<std << " ";
//					//if ((1 <= Y) && (Y <= node_to_insert.col_UAV_num)) {
//					//	Y = round(Y);
//					//}
//					//if (Y > node_to_insert.col_UAV_num) {
//					//	Y = node_to_insert.col_UAV_num;
//					//}
//					//if (Y < 1) {
//					//	Y = 1;
//					//}
//					//cout << Y << endl;
//					rnd_col_num = min((int)choosed_col_num, (int)size(uavs_allowed_id_list));
//					//rnd_col_num = max(min_node_col[node_to_insert.id], rnd_col_num);
//				
//				}
//				if (rnd_col_num < 0) {
//					rnd_col_num = 1;
//				}
//				//int rnd_col_num = min(choosed_col_num, (int)size(uavs_allowed_id_list));
//				if (min_node_col[node_to_insert.id] > rnd_col_num) {
//					continue;
//				}
//				vector<int> uav_col_id_list = make_uavs_assigned(node_to_insert, rnd_insert_p, rnd_col_num);
//				//if (mode == 3) {
//				//	uav_col_id_list.clear();
//				//	set<int> rnd_col_id_set;
//				//	while (size(rnd_col_id_set) < rnd_col_num) {
//				//		int rnd_id_c = rand() % size(uavs_allowed_id_list);
//				//		if (rnd_col_id_set.find(rnd_id_c) == rnd_col_id_set.end()) {
//				//			rnd_col_id_set.insert(rnd_id_c);
//				//			uav_col_id_list.push_back(uavs_allowed_id_list[rnd_id_c]);
//				//		}
//				//	}
//				//}
//				if (size(uav_col_id_list) == 0) {
//					try_col++;
//					continue;
//				}
//				//float t_all=0;
//				//for (int id_x : uav_col_id_list) {
//				//	t_all += T-sol_tmp.id_to_UAV(id_x).t_used;
//				//}
//				//if (t_all < node_to_insert.node_size / UAV::eff) {
//				//	try_col++;
//				//	continue;
//				//}
//				auto sol_tt = sol_tmp;
//				auto& node_list_tt = sol_tt.node_list;
//				node_list_tt[rnd_insert_p].UAV_id_list = uav_col_id_list;
//				node_to_insert.UAV_id_list = uav_col_id_list;
//				for (int uav_id : uav_col_id_list) {
//					auto& uav_allowed = sol_tt.id_to_UAV(uav_id);
//					if (size(uav_allowed.Path) == 0) {
//						uav_allowed.Path.push_back(node_to_insert);
//						continue;
//					}
//					int fl = 0;
//					for (int i = 0; i < size(uav_allowed.Path) - 1; i++) {
//						auto& node_f = sol_tt.id_to_node(uav_allowed.Path[i].id);
//						auto& node_s =sol_tt.id_to_node( uav_allowed.Path[i + 1].id);
//						if (sol_tt.compare_node(node_f, node_to_insert) && sol_tt.compare_node(node_to_insert, node_s)) {
//							uav_allowed.Path.insert(uav_allowed.Path.begin() + i + 1, node_to_insert);
//							fl = 1;
//						}
//					}
//					if (fl == 1) {
//						continue;
//					}
//					else {
//						auto& node_first = sol_tt.id_to_node(uav_allowed.Path[0].id);
//						auto& node_second = sol_tt.id_to_node(uav_allowed.Path[size(uav_allowed.Path) - 1].id);
//						if (sol_tt.compare_node(node_to_insert, node_first)) {
//							uav_allowed.Path.insert(uav_allowed.Path.begin(), node_to_insert);
//							continue;
//						}
//						if (sol_tt.compare_node(node_second, node_to_insert)) {
//							uav_allowed.Path.push_back(node_to_insert);
//							continue;
//						}
//					}
//				}
//				if (!sol_tt.need_repair()) {
//					sol_tmp = sol_tt;
//					*this = sol_tmp;
//					try_col = 0;
//					break;
//				}
//				else {
//					try_col++;
//				}
//			}
//		}
//	}
//}

void Solution::clear_ext_path_node(float th_pro) {
	for (auto& uav_sub_list : UAV_list) {
		for (auto& uav_c : uav_sub_list) {
			if (size(uav_c.Path) > 0) {
				int iter_n = -1;
				vector<Path_node> path_tmp = uav_c.Path;
				for (int i = 0; i<size(path_tmp);i++) {
					iter_n++;
					auto path_node = uav_c.Path[iter_n];
					if (path_node.t_end == path_node.t_start) {
						float o_pro = (float)rand() / (float)RAND_MAX;
						if (o_pro > th_pro) {
							continue;
						}
						auto& node_tmp = id_to_node(path_node.id);
						uav_c.Path.erase(uav_c.Path.begin() + iter_n);
						iter_n--;
						vector<int> uav_tmp_col_list;
						for (int ii = 0; ii< size(node_tmp.UAV_id_list); ii++) {
							if (node_tmp.UAV_id_list[ii] == uav_c.id) {
								continue;
							}
							else {
								uav_tmp_col_list.push_back(node_tmp.UAV_id_list[ii]);
							}
						}
						node_tmp.UAV_id_list = uav_tmp_col_list;
						//if (size(node_tmp.UAV_id_list) == 0) {
						//	node_list.erase(node_list.begin() + node_tmp.no);
						//}
	/*					node_Path_node_connect();*/
						//if (need_repair()) {
						//	cout << "something go wrong!" << endl;
						//}
						cal_uav_path_t();
					}
				}
			}
		}
	}					
}

void Solution::ls_node_exchange_m(vector<Working_node> wn_list, int unchange_counter) {
	vector<Working_node> node_ex_list;
	for (auto node_i : wn_list) {
		if (!check_node_id(node_i.id)) {
			node_ex_list.push_back(node_i);
		}
	}
	if (size(node_ex_list) == 0) {
		return;
	}
	float roo = (float)rand() / RAND_MAX;
	float sum_s = 0;
	auto node_ex_tmp = node_ex_list;
	for (auto nodec : node_ex_list) {
		sum_s += nodec.node_size;
	}
	for (auto nodec : node_ex_tmp) {
		nodec.node_size/= sum_s;
	}
	float ro_acc = 0;
	int node_ro_idx = 0;
	int flag_ro = 0;
	for (int id_no = 0; id_no < size(node_ex_tmp); id_no++) {
		ro_acc += node_ex_tmp[id_no].node_size;
		if (ro_acc > roo) {
			node_ro_idx = id_no;
			flag_ro = 1;
		}
	}
	if (flag_ro == 0) {
		node_ro_idx = size(node_ex_tmp) - 1;
	}
	auto node_to_exchange = node_ex_list[node_ro_idx];
	//auto node_to_exchange = node_ex_list[rand()%size(node_ex_list)];
	int insert_p = rand() % size(node_list);
	float sum_sum = 0;
	int choosed_col_num = 0;
	int size_u_al=0;
	set<int> type_in = node_to_exchange.type;
	for (auto iter = type_in.begin(); iter != type_in.end(); iter++) {
		size_u_al += size(UAV_list[*iter]);
	}
	
	for (int i = 0; i < min(node_to_exchange.col_UAV_num, size_u_al); i++) {
		//col_num_pro[i] = Q_table_1[node_to_exchange.id][rnd_no][i];
		if (sum_sum < Q_table_1[node_to_exchange.id][insert_p][i]) {
			choosed_col_num = i + 1;
			sum_sum = Q_table_1[node_to_exchange.id][insert_p][i];
		}
	}
	float eps_pro = (float)rand() / RAND_MAX;
	if (eps_pro < eps||sum_sum==0) {
		choosed_col_num = rand() % (min(node_to_exchange.col_UAV_num, size_u_al)) + 1;
	}
	vector<int> uav_tmp_list = make_uavs_assigned_m(node_to_exchange, insert_p, choosed_col_num);
#ifdef rand_u
	uav_tmp_list = make_uavs_assigned_r(node_to_exchange, insert_p, choosed_col_num);
#endif // rand_u
	node_to_exchange.UAV_id_list = uav_tmp_list;
	set<int> type_h;
	for (int uav_c_id : node_to_exchange.UAV_id_list) {
		type_h.insert(id_to_UAV(uav_c_id).type);
	}
	node_list.insert(node_list.begin()+insert_p, node_to_exchange);
	assign_node_no();
	node_to_exchange.no = insert_p;
	for (int idx : uav_tmp_list) {
		UAV& uav_out = id_to_UAV(idx);
		int fl = 0;
		if (size(uav_out.Path) == 0) {
			uav_out.Path.push_back(node_to_exchange);
			continue;
		}
		for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
			int idx_first = uav_out.Path[i_tt].id;
			int idx_second = uav_out.Path[i_tt + 1].id;
			auto& node_first = id_to_node(idx_first);
			auto& node_second = id_to_node(idx_second);
			if (compare_node(node_first, node_to_exchange) && (compare_node(node_to_exchange, node_second))) {
				uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node_to_exchange);
				fl = 1;
				break;
			}
		}
		if (fl == 0) {
			int node_first_id = uav_out.Path[0].id;
			int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
			auto& node_first_0 = id_to_node(node_first_id);
			auto& node_last_0 = id_to_node(node_last_id);
			if (compare_node(node_to_exchange, node_first_0)) {
				uav_out.Path.insert(uav_out.Path.begin(), node_to_exchange);
				continue;
			}
			if (compare_node(node_last_0, node_to_exchange)) {
				uav_out.Path.insert(uav_out.Path.end(), node_to_exchange);
				continue;
			}
		}
	}
	//cal_uav_path_t();
	//cout << need_repair() << endl;
	float max_size = node_to_exchange.node_size;
	while (need_repair_m()) {
		//if (max_size < 0) {
		//	break;
		//}
		int uav_l_id = INT_MAX;
		float max_t = 0;
		float t_sum = 0;
		for (auto uav_l : UAV_list) {
			for (auto uav_i : uav_l) {
				if (uav_i.t_used > T) {
					if (max_t < uav_i.t_used) {
						max_t = uav_i.t_used;
						uav_l_id = uav_i.id;
						t_sum += T - uav_i.t_used;
					}
				}
			}
		}
		vector<float> eff_node;
		int node_id;
		auto UAV_ch = id_to_UAV(uav_l_id);
		for (auto node_ch : UAV_ch.Path) {
			if (node_ch.node_size > max_size) {
				eff_node.push_back(0);
				continue;
			}
			else {
				eff_node.push_back(1);
			}
		}
		for (auto node_ch : UAV_ch.Path) {
			//if (node_ch.node_size > max_size) {
			//	eff_node.push_back(0);
			//	continue;
			//}
			int ch_id = node_ch.id;
			auto node_ch_i = id_to_node(ch_id);
			float path_l= 0;
			for (int uav_no = 0;uav_no<size( node_ch_i.UAV_id_list);uav_no++) {
				//if (node_ch_i.UAV_id_list[uav_no] != UAV_ch.id) {
				//	continue;
				//}
				auto uav_n = id_to_UAV(node_ch_i.UAV_id_list[uav_no]);
				int path_no = node_ch_i.Path_node_no_list[uav_no];
				if (path_no == 0) {
					path_l += dis_matric[MAP_NODE_NUM][ch_id];
				}
				else {
					path_l+= dis_matric[uav_n.Path[path_no-1].id][ch_id];
				}
			}
			eff_node.push_back(path_l/ node_ch.node_size);//abs(node_ch.node_size / UAV::eff-t_sum)
		}
		//for (int path_ii = 0; path_ii < size(UAV_ch.Path); path_ii++) {
		//	//auto node_ccc = id_to_node(UAV_ch.Path[path_ii].id);
		//	//int cc_flag = 0;
		//	//for (int cc_u_id : node_ccc.UAV_id_list) {
		//	//	auto uav_ccc = id_to_UAV(cc_u_id);
		//	//	if (type_h.find(uav_ccc.type) != type_h.end()) {
		//	//		cc_flag = 1;
		//	//	}
		//	//}
		//	float path_l = 0;
		//	if (path_ii == 0) {
		//		path_l = dis_matric[MAP_NODE_NUM][UAV_ch.Path[path_ii].id];
		//	}
		//	else {
		//		path_l = dis_matric[UAV_ch.Path[path_ii-1].id][UAV_ch.Path[path_ii].id];
		//	}
		//	if (UAV_ch.Path[path_ii].t_end - UAV_ch.Path[path_ii].t_start == 0) {
		//		eff_node.push_back(FLT_MAX);
		//		continue;
		//	}
		//	//if (cc_flag == 0) {
		//	//	eff_node.push_back(0);
		//	//}
		//	//else{
		//	//}
		//	eff_node.push_back(path_l / ((UAV_ch.Path[path_ii].t_end- UAV_ch.Path[path_ii].t_start)*UAV::eff));//abs(node_ch.node_size / UAV::eff-t_sum)
		//}
		float eff_min = FLT_MAX;
		int node_no = 0;
		float eff_sum = 0;
		for (int eff_no = 0; eff_no < size(UAV_ch.Path); eff_no++) {
			eff_sum += eff_node[eff_no];
		}
		if (eff_sum == 0) {
			break;
		}
		for (int eff_no = 0; eff_no < size(UAV_ch.Path); eff_no++) {
			 eff_node[eff_no]/=eff_sum;
		}
		int ff = 0;
		float min_f = 0;
		for (int eff_no = 0; eff_no < size(UAV_ch.Path); eff_no++) {
			if (min_f < eff_node[eff_no]) {
				min_f = eff_node[eff_no];
				ff = eff_no;
			}
		}
		//float ro_pro = (float)rand() / RAND_MAX;
		//float eff_ac = 0;
		//int flag_oi = 0;
		//for (int eff_no = 0; eff_no < size(UAV_ch.Path); eff_no++) {
		//	eff_ac += eff_node[eff_no];
		//	if (eff_ac > ro_pro) {
		//		flag_oi = 1;
		//		node_no = eff_no;
		//	}
		//}
		//if (flag_oi = 0) {
		//	node_no = size(UAV_ch.Path) - 1;
		//}
		node_id = UAV_ch.Path[ff].id;
		//node_id = UAV_ch.Path[rand()%size(UAV_ch.Path)].id;
		auto& node_to_del = id_to_node(node_id);
		vector<int> uav_out_list;
		for (int uav_id_to_del : node_to_del.UAV_id_list) {
			if (uav_id_to_del != UAV_ch.id) {
				uav_out_list.push_back(uav_id_to_del);
				continue;
			}
			auto& uav_to_del = id_to_UAV(uav_id_to_del);
			uav_to_del.erase_node_i_in_path(node_id);
		}
		node_to_del.UAV_id_list = uav_out_list;
		if (size(node_to_del.UAV_id_list) == 0) {
			node_list.erase(node_list.begin() + node_to_del.no);
			max_size -= node_to_del.node_size;	
			assign_node_no();
		}
	}
}

void Solution::ls_node_del() {
	if (need_repair()) {
		cout << "bad happened" << endl;
	}
	vector<float> eff_node;
	for (auto node_ch_i : node_list) {
		float path_l = 0;
		for (int uav_no = 0; uav_no < size(node_ch_i.UAV_id_list); uav_no++) {
			//if (node_ch_i.UAV_id_list[uav_no] != UAV_ch.id) {
			//	continue;
			//}
			auto uav_n = id_to_UAV(node_ch_i.UAV_id_list[uav_no]);
			int path_no = node_ch_i.Path_node_no_list[uav_no];
			if (path_no == 0) {
				path_l += dis_matric[MAP_NODE_NUM][node_ch_i.id];
			}
			else {
				path_l += dis_matric[uav_n.Path[path_no - 1].id][node_ch_i.id];
			}
		}
		eff_node.push_back(path_l / node_ch_i.node_size);//abs(node_ch.node_size / UAV::eff-t_sum)
	}

	float eff_min = FLT_MAX;
	int node_no = 0;
	float eff_sum = 0;
	for (int eff_no = 0; eff_no < size(node_list); eff_no++) {
		eff_sum += eff_node[eff_no];
	}
	for (int eff_no = 0; eff_no < size(node_list); eff_no++) {
		eff_node[eff_no] /= eff_sum;
	}
	float ro_pro = (float)rand() / RAND_MAX;
	float eff_ac = 0;
	int flag_oi = 0;
	for (int eff_no = 0; eff_no < size(node_list); eff_no++) {
		eff_ac += eff_node[eff_no];
		if (eff_ac > ro_pro) {
			flag_oi = 1;
			node_no = eff_no;
		}
	}
	if (flag_oi = 0) {
		node_no = size(node_list) - 1;
	}
	auto node_to_del = node_list[node_no];
	for (int uav_no : node_to_del.UAV_id_list) {
		auto& uav_nm = id_to_UAV(uav_no);
		uav_nm.erase_node_i_in_path(node_to_del.id);
	}
	node_list.erase(node_list.begin() + node_no);
}
void Solution::ls_node_exchange(vector<Working_node> wn_list, int unchange_counter) {
	int rnd_no = rand() % size(node_list);
	if (size(node_list) == size(wn_list)) {
		return;
	}
	vector<int> id_ex;
	
	for (auto node : wn_list) {
		if (node.node_size < node_list[rnd_no].node_size) {
			continue;
		}
		int k = 0;
		for (auto node_r : node_list) {
			if (node_r.id == node.id) {
				k = 1;
				break;
			}
		}
		if (k == 0) {
			id_ex.push_back(node.id);
		}
	}
	if (size(id_ex) == 0) {
		return;
	}
	set<int> type_1;
	for (auto uav_in : node_list[rnd_no].UAV_id_list) {
		type_1.insert(id_to_UAV(uav_in).type);
	}
	//auto type_1 = node_list[rnd_no].type;
	int re = 1;
	vector<int> ex_id_f;
	for (auto ex_id : id_ex) {
		auto iter_i = type_1.begin();
		auto type_3 = wn_list[ex_id].type;
		for (int i = 0; i < size(type_1); i++) {
			if (type_3.find(*iter_i) == type_3.end()) {
				iter_i++;
				continue;
			}
			else {
				re=0;
				break;
			}
		}
		if (re==0) {
			ex_id_f.push_back(ex_id);
		}
		re = 1;
	}
	if (size(ex_id_f) == 0) {
		return;
	}
	int choosed_node_no = rand() % size(ex_id_f);

	auto node_to_exchange = wn_list[ex_id_f[choosed_node_no]];

	//float* col_num_pro = new float[node_to_exchange.col_UAV_num]();
	float sum_sum = 0;
	int choosed_col_num=0;
	int rel_num = 0;
	set<int> type_i = node_to_exchange.type;
	if (rnd_no > 0) {
		for (int node_rel = 0; node_rel < rnd_no; node_rel++) {
			auto& node_ii = node_list[node_rel];
			set<int> type_ii = node_ii.type;
			int re_flag = 0;
			for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
				for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
					if (*iter == *iter_i) {
						re_flag = 1;
					}
				}
			}
			if (re_flag == 1) {
				rel_num++;
			}
		}
	}
	for (int i = 0; i < node_to_exchange.col_UAV_num; i++) {
		//col_num_pro[i] = Q_table_1[node_to_exchange.id][rnd_no][i];
		if (sum_sum < Q_table_1[node_to_exchange.id][rel_num][i]) {
			choosed_col_num = i + 1;
			sum_sum = Q_table_1[node_to_exchange.id][rel_num][i];
		}
		//sum_sum += col_num_pro[i];
	}
	//for (int i = 0; i < node_to_exchange.col_UAV_num; i++) {
	//	/*col_num_pro[i] *= ((i + 1) / node_to_exchange.node_size);*/
	//	/*sum_sum += col_num_pro[i];*/
	//}
	//for (int i = 0; i < node_to_exchange.col_UAV_num; i++) {
	//	col_num_pro[i] /= sum_sum;
	//}
	//float rnd_col_p = (float)rand() / (float)RAND_MAX;
	//float pro_base = 0;
	//int flag_find = 0;
	//for (int i = 0; i < node_to_exchange.col_UAV_num; i++) {
	//	pro_base += col_num_pro[i];
	//	if (pro_base >= rnd_col_p) {
	//		flag_find = 1;
	//		choosed_col_num = i + 1;
	//		break;
	//	}
	//}
	//if (flag_find == 0) {
	//	choosed_col_num = node_to_exchange.col_UAV_num;
	//}
	//delete[] col_num_pro;
	//double U1 = (float)rand() / (float)RAND_MAX;
	//double U2 = (float)rand() / (float)RAND_MAX;
	//double Z = sqrt(-2 * log(U1)) * cos(2 * pi * U2);
	////double min_std = (float)1 / (float)3;
	////double max_std = (float)max(2, node_to_exchange.col_UAV_num) / (float)6;
	//double min_std = (float)2 / (float)3;
	//double max_std = (float)max(4, node_to_exchange.col_UAV_num) / (float)6;
	////double std = min_std + (max_std - min_std) * pow(((float)(iteration_time) / (float)(IT)), 1.6);

	//unchange_counter = min(MAX_ACO_IT, unchange_counter);
	//double std = min_std + (max_std - min_std) * pow((float)(iteration_time) / (float)(MAX_ACO_IT), 2);
	//double Y = choosed_col_num + Z * std;
	//double Y = choosed_col_num;
	////cout << choosed_col_num<<" "<<std << " ";
	//if ((1 <= Y) && (Y <= node_to_exchange.col_UAV_num)) {
	//	Y = round(Y);
	//}
	//if (Y > node_to_exchange.col_UAV_num) {
	//	Y = node_to_exchange.col_UAV_num;
	//}
	//if (Y < 1) {
	//	Y = 1;
	//}
	//cout << Y << endl;
	vector<int> allowed_uav_list;
	for (auto uav_id : node_list[rnd_no].UAV_id_list) {
		auto& uav_ty = id_to_UAV(uav_id);
		int type_i = uav_ty.type;
		if (node_to_exchange.type.find(type_i) == node_to_exchange.type.end()) {
			continue;
		}
		else {
			allowed_uav_list.push_back(uav_id);
		}
		//int flag_fi = 0;
		//auto iter_u = node_to_exchange.type.begin();
		//for (int i = 0; i < size(node_to_exchange.type); i++) {
		//	if (node_to_exchange.type.find(*iter_u) == node_to_exchange.type.end()) {
		//		iter_u++;
		//		continue;
		//	}
		//	else {
		//		flag_fi = 1;
		//		break;
		//	}
		//}
		//if (flag_fi == 1) {
		//	allowed_uav_list.push_back(uav_id);
		//}
	}
	
	float rnd_pro = (float)rand() / RAND_MAX;
	vector<int> UAV_id_all;
	for (auto iter_set = node_to_exchange.type.begin(); iter_set != node_to_exchange.type.end(); iter_set++) {
		for (auto uav_ii : UAV_list[*iter_set]) {
			UAV_id_all.push_back(uav_ii.id);
		}
	}
	if (rnd_pro < eps||sum_sum==0) {
		choosed_col_num = rand() % (min((int)size(UAV_id_all), node_to_exchange.col_UAV_num))+1;
	}
	choosed_col_num = ret_GN(choosed_col_num);
	int rnd_col_num = (int)min(choosed_col_num, (int)size(UAV_id_all));
	if (rnd_col_num < 1) {
		rnd_col_num = 1;
	}
	if (rnd_col_num >= size(allowed_uav_list)) {
		set<int> uav_id_set;
		for (int idx : allowed_uav_list) {
			uav_id_set.insert(idx);
		}
		/*while (size(uav_id_set) < rnd_col_num) {*/
			//int rnd_no = rand() % size(UAV_id_all);
		
		(*this).assign_node_no();
		Solution sol_tmp = *this;
		sol_tmp.cal_uav_path_t();
		//float t_first = FLT_MAX;
		for (int idx : UAV_id_all) {
			auto& uav_ch = sol_tmp.id_to_UAV(idx);
			int ii = 0;
			int flag_ff = 0;
			for (; ii < size(uav_ch.Path); ii++) {
				if (uav_ch.Path[ii].id == node_list[rnd_no].id) {
					flag_ff = 1;
					break;
				}
			}
			if (flag_ff == 1) {
				//cout << ii << endl;
				uav_ch.Path.erase(uav_ch.Path.begin() + ii);
			}
			//determine uav's insert_place in path
			if (size(uav_ch.Path) == 0) {
				uav_ch.tmp_node_no = 0;
				continue;
			}
			int flag = 0;
			for (int i = 0; i < size(uav_ch.Path) - 1; i++) {
				int node_id_first = uav_ch.Path[i].id;
				int node_id_second = uav_ch.Path[i + 1].id;
				auto& node_ch_first = sol_tmp.id_to_node(node_id_first);
				auto& node_ch_second = sol_tmp.id_to_node(node_id_second);
				if (sol_tmp.compare_node(node_ch_first, node_list[rnd_no]) && sol_tmp.compare_node(node_list[rnd_no], node_ch_second)) {
					uav_ch.tmp_node_no = i + 1;
					flag = 1;
				}
			}
			if (flag == 1) {
				continue;
			}

			int node_first_id = uav_ch.Path[0].id;
			auto& node_first = sol_tmp.id_to_node(node_first_id);
			if (sol_tmp.compare_node(node_list[rnd_no], node_first)) {
				uav_ch.tmp_node_no = 0;
				continue;
			}
			int node_last_id = uav_ch.Path[size(uav_ch.Path) - 1].id;
			auto& node_last = sol_tmp.id_to_node(node_last_id);
			if (sol_tmp.compare_node(node_last, node_list[rnd_no])) {
				uav_ch.tmp_node_no = size(uav_ch.Path);
				continue;
			}
		}
		vector<UAV> uav_allowed_list_1;		
		sol_tmp.node_list.erase(sol_tmp.node_list.begin()+rnd_no);
		sol_tmp.node_list.insert(sol_tmp.node_list.begin() + rnd_no, node_to_exchange);
		for (int idx : UAV_id_all) {
			auto& uav_idx = sol_tmp.id_to_UAV(idx);
			uav_idx.Path.insert(uav_idx.Path.begin() + uav_idx.tmp_node_no, node_to_exchange);
			float P_1 = uav_idx.ret_PathLength_to_node_insert(node_to_exchange);
			float P_2 = uav_idx.ret_PathLength_from_node_insert(node_to_exchange);
			float P_p = uav_idx.ret_PathLength_from_node_j();


			//uav_idx.Path_change_idx = (T-( uav_idx.t_used));

			uav_idx.Path_change_idx = P_1+ P_2- P_p;
			if (uav_idx.tmp_node_no == 0) {
				uav_idx.tmp_t_to_node = P_1 / uav_idx.speed;
			}
			else {
				uav_idx.tmp_t_to_node = (uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed);
			}
			uav_allowed_list_1.push_back(uav_idx);	

		}
		double* max_Path_change_index = new double();
		double* max_length_index = new double();
		double* max_t_left = new double();
		*max_Path_change_index = FLT_MAX;
		*max_length_index = FLT_MAX;
		for (int u_i = 0; u_i < size(allowed_uav_list); u_i++) {
			uav_allowed_list_1[u_i].col_length_idx = 0;
			for (int j_i = 0; j_i < size(allowed_uav_list); j_i++) {
				uav_allowed_list_1[u_i].col_length_idx += abs(uav_allowed_list_1[u_i].tmp_t_to_node - uav_allowed_list_1[j_i].tmp_t_to_node);
			}
			//if (uav_allowed_list_1[u_i].Path_change_idx < 0) {
			//	uav_allowed_list_1[u_i].col_length_idx = -1;
			//}

			//else {
			//	uav_allowed_list_1[u_i].col_length_idx = 1 / (1 + uav_allowed_list_1[u_i].col_length_idx);
			//}
			*max_Path_change_index = min(*max_Path_change_index, uav_allowed_list_1[u_i].Path_change_idx);
			*max_length_index = min(*max_length_index, uav_allowed_list_1[u_i].col_length_idx);
			*max_t_left = max(*max_t_left, (double)(T - uav_allowed_list_1[u_i].t_used));
		}
		float theta_1 = (float)rand() / RAND_MAX;
		float theta_2 = 1 - theta_1;

		if (rnd_col_num == 1) {
			for (int u_i = 0; u_i < size(allowed_uav_list); u_i++) {
				uav_allowed_list_1[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list_1[u_i].Path_change_idx + theta_2 * (T- uav_allowed_list_1[u_i].t_used) / (*max_t_left);
			}
		}
		else {
			for (int u_i = 0; u_i < size(allowed_uav_list); u_i++) {
				uav_allowed_list_1[u_i].con_index = theta_1 *  max_Path_change_index[0] /uav_allowed_list_1[u_i].Path_change_idx + theta_2 *  (max_length_index[0]+1)/((uav_allowed_list_1[u_i].col_length_idx)+1) ;
			}			
		}
		delete max_Path_change_index;
		delete max_length_index;
		delete max_t_left;
		//for (int uav_iid : allowed_uav_list) {
		//	auto& uav_a_i = sol_tmp.id_to_UAV(uav_iid);
		//	t_first = min(t_first, uav_a_i.tmp_t_to_node);
		//}
		//for (auto& uav_de : uav_allowed_list_1) {
		//	uav_de.con_index = abs(uav_de.tmp_t_to_node - t_first);
		//}
		sort(uav_allowed_list_1.begin(), uav_allowed_list_1.end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
		//sort(uav_allowed_list_1.begin(), uav_allowed_list_1.end(), [](UAV& a, UAV& b) {return a.con_index < b.con_index; });
		for (auto& uav_de : uav_allowed_list_1) {
			if (size(uav_id_set) == rnd_col_num) {
				break;
			}
			if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
				uav_id_set.insert(uav_de.id);
				allowed_uav_list.push_back(uav_de.id);
			}
		}

		//}
		node_to_exchange.UAV_id_list = allowed_uav_list;
	}
	else {
		//vector<int> uav_ex_id_list;
		//set<int> no_set;
		//while (size(uav_ex_id_list) < rnd_col_num) {
		//	int rand_no_c = rand() % size(allowed_uav_list);
		//	if (no_set.find(rand_no_c) == no_set.end()) {
		//		no_set.insert(rand_no_c);
		//		uav_ex_id_list.push_back(allowed_uav_list[rand_no_c]);
		//	}
		//}
		//node_to_exchange.UAV_id_list = uav_ex_id_list;
		set<int> uav_id_set;
		//vector<int> uav_f_list;
		//while (size(uav_id_set) < rnd_col_num) {
		//	int rnd_no = rand() % size(allowed_uav_list);

		Solution sol_tmp = *this;
		sol_tmp.cal_uav_path_t();
		for (int idx : allowed_uav_list) {
			auto& uav_ch = sol_tmp.id_to_UAV(idx);
			int ii = 0;
			int flag_ff = 0;
			for (; ii < size(uav_ch.Path); ii++) {
				if (uav_ch.Path[ii].id == node_list[rnd_no].id) {
					flag_ff = 1;
					uav_ch.tmp_node_no = ii;
					uav_ch.Path.erase(uav_ch.Path.begin() + ii);
					break;
				}
			}
		}
		vector<UAV> uav_allowed_list_1;
		sol_tmp.node_list.erase(sol_tmp.node_list.begin() + rnd_no);
		sol_tmp.node_list.insert(sol_tmp.node_list.begin() + rnd_no, node_to_exchange);
		for (int idx : allowed_uav_list) {
			auto& uav_idx = sol_tmp.id_to_UAV(idx);
			uav_idx.Path.insert(uav_idx.Path.begin() + uav_idx.tmp_node_no, node_to_exchange);
			float P_1 = uav_idx.ret_PathLength_to_node_insert(node_to_exchange);
			float P_2 = uav_idx.ret_PathLength_from_node_insert(node_to_exchange);
			float P_p = uav_idx.ret_PathLength_from_node_j();
			
			//uav_idx.Path_change_idx = P_1;
			//uav_idx.Path_change_idx = (T - uav_idx.t_used);
			uav_idx.Path_change_idx = P_1 + P_2 - P_p;
			//cout<< uav_idx.Path_change_idx<<" ";
			if (uav_idx.tmp_node_no == 0) {
				uav_idx.tmp_t_to_node =  P_1 / uav_idx.speed;
			}
			else {
				uav_idx.tmp_t_to_node = (uav_idx.Path_node_t_list[uav_idx.tmp_node_no - 1] + P_1 / uav_idx.speed);
			}
			uav_allowed_list_1.push_back(uav_idx);
		}
		//for (int u_i = 0; u_i < size(allowed_uav_list); u_i++) {
		//	uav_allowed_list_1[u_i].con_index = 0;
		//	for (int j_i = 0; j_i < size(allowed_uav_list); j_i++) {
		//		uav_allowed_list_1[u_i].con_index += abs(uav_allowed_list_1[u_i].tmp_t_to_node - uav_allowed_list_1[j_i].tmp_t_to_node);
		//	}
		//}
		double* max_Path_change_index = new double();
		double* max_length_index = new double();
		double* max_t_left = new double();
		*max_Path_change_index = FLT_MAX;
		*max_length_index = FLT_MAX;

		for (int u_i = 0; u_i < size(allowed_uav_list); u_i++) {
			uav_allowed_list_1[u_i].col_length_idx = 0;
			for (int j_i = 0; j_i < size(allowed_uav_list); j_i++) {
				uav_allowed_list_1[u_i].col_length_idx += abs(uav_allowed_list_1[u_i].tmp_t_to_node - uav_allowed_list_1[j_i].tmp_t_to_node);
			}
			//if (uav_allowed_list_1[u_i].Path_change_idx < 0) {
			//	uav_allowed_list_1[u_i].col_length_idx = -1;
			//}
			//else {
			//	uav_allowed_list_1[u_i].col_length_idx = 1 / (uav_allowed_list_1[u_i].col_length_idx+1);
			//}
			*max_Path_change_index = min(*max_Path_change_index, uav_allowed_list_1[u_i].Path_change_idx);
			*max_length_index = min(*max_length_index, uav_allowed_list_1[u_i].col_length_idx);
			*max_t_left = max(*max_t_left, (double)(T - uav_allowed_list_1[u_i].t_used));
		}
		float theta_1 = (float)rand() / RAND_MAX;
		float theta_2 = 1 - theta_1;
		if (rnd_col_num == 1) {
			for (int u_i = 0; u_i < size(allowed_uav_list); u_i++) {
				uav_allowed_list_1[u_i].con_index = theta_1 * max_Path_change_index[0] / uav_allowed_list_1[u_i].Path_change_idx + theta_2 * (T- uav_allowed_list_1[u_i].t_used) / (*max_t_left);
			}
		}
		else {
			for (int u_i = 0; u_i < size(allowed_uav_list); u_i++) {
				uav_allowed_list_1[u_i].con_index = theta_1 *  max_Path_change_index[0]/uav_allowed_list_1[u_i].Path_change_idx  + theta_2 *  (max_length_index[0]+1)/ (((uav_allowed_list_1[u_i].col_length_idx))+1);
			}		
		}

		delete max_Path_change_index;
		delete max_length_index;
		delete max_t_left;
		sort(uav_allowed_list_1.begin(), uav_allowed_list_1.end(), [](UAV& a, UAV& b) {return a.con_index > b.con_index; });
		allowed_uav_list.clear();
		for (auto& uav_de : uav_allowed_list_1) {
			if (size(uav_id_set) == rnd_col_num) {
				break;
			}
			if (uav_id_set.find(uav_de.id) == uav_id_set.end()) {
				uav_id_set.insert(uav_de.id);
				allowed_uav_list.push_back(uav_de.id);
			}
		}
		node_to_exchange.UAV_id_list = allowed_uav_list;
	}

	for (auto idx : node_list[rnd_no].UAV_id_list) {
		auto& uav_i = id_to_UAV(idx);
		uav_i.erase_node_i_in_path(node_list[rnd_no].id);
	}
	node_list.erase(node_list.begin() + rnd_no);
	node_list.insert(node_list.begin() + rnd_no, node_to_exchange);
	assign_node_no();
	node_to_exchange.no = rnd_no;
	for (auto idx_i : node_to_exchange.UAV_id_list) {
		auto& uav_out = id_to_UAV(idx_i);
		int fl = 0;
		if (size(uav_out.Path) == 0) {
			uav_out.Path.push_back(node_to_exchange);
			continue;
		}
		for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
			int idx_first = uav_out.Path[i_tt].id;
			int idx_second = uav_out.Path[i_tt + 1].id;
			auto& node_first = id_to_node(idx_first);
			auto& node_second = id_to_node(idx_second);
			if (compare_node(node_first, node_to_exchange) && (compare_node(node_to_exchange, node_second))) {
				uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node_to_exchange);
				fl = 1;
				break;
			}
		}
		if (fl == 0) {
			int node_first_id = uav_out.Path[0].id;
			int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
			auto& node_first_0 = id_to_node(node_first_id);
			auto& node_last_0 = id_to_node(node_last_id);
			if (compare_node(node_to_exchange, node_first_0)) {
				uav_out.Path.insert(uav_out.Path.begin(), node_to_exchange);
				continue;
			}
			if (compare_node(node_last_0, node_to_exchange)) {
				uav_out.Path.insert(uav_out.Path.end(), node_to_exchange);
				continue;
			}
		}
	}
}


void Solution::ls_path_intersect() {
	vector<int> t_allowed;
	for (int t_i = 0; t_i < size(UAV_list); t_i++) {
		vector<int> t_i_allowed;
		for (int t_ii = 0; t_ii < size(UAV_list[t_i]); t_ii++) {
			if (size(UAV_list[t_i][t_ii].Path) > 0) {
				t_i_allowed.push_back(t_ii);
			}
		}
		if (size(t_i_allowed) > 1) {
			t_allowed.push_back(t_i);
		}
	}
	if (size(t_allowed) < 1) {
		return;
	}
	int rnd_type = rand() % size(t_allowed);
	int uav_type = t_allowed[rnd_type];
	set<int> uav_no_set; 

	vector<UAV> UAV_g1;
	for (int tii = 0; tii < size(UAV_list[uav_type]); tii++) {
		if (size(UAV_list[uav_type][tii].Path) > 0) {
			UAV_g1.push_back(UAV_list[uav_type][tii]);
		}
	}
	if (size(UAV_g1) < 2) {
		return;
	}
	while (size(uav_no_set) < 2) {
		int rand_no = rand() % size(UAV_g1);
		if (uav_no_set.find(UAV_g1[rand_no].id) == uav_no_set.end()) {
			uav_no_set.insert(UAV_g1[rand_no].id);
		}
	}
	vector<int> first_uav_pn_no;
	vector<int> second_uav_pn_no;
	auto iter_u = uav_no_set.begin();
	auto& uav_1 = id_to_UAV(*iter_u);
	iter_u++;
	auto& uav_2 = id_to_UAV(*iter_u);
	int ii = -1;
	for (auto& pn_1 : uav_1.Path) {
		ii++;
		int f=0;
		for (auto& pn_2 : uav_2.Path) {
			if (pn_1.id == pn_2.id) {
				f = 1;
				break;
			}
		}
		if (f == 0) {
			first_uav_pn_no.push_back(ii);
		}
	}

	int iii = -1;
	for (auto& pn_1 : uav_2.Path) {
		iii++;
		int f = 0;
		for (auto& pn_2 : uav_1.Path) {
			if (pn_1.id == pn_2.id) {
				f = 1;
				break;
			}
		}
		if (f == 0) {
			second_uav_pn_no.push_back(iii);
		}
	}
	if (size(first_uav_pn_no) == 0 || size(second_uav_pn_no) == 0) {
		return;
	}
	int rand_1 = rand() % size(first_uav_pn_no);
	auto& node_1 = id_to_node(uav_1.Path[first_uav_pn_no[rand_1]].id);
	int rand_2 = rand() % size(second_uav_pn_no);
	
	auto& node_2 = id_to_node(uav_2.Path[second_uav_pn_no[rand_2]].id);
	uav_1.erase_node_i_in_path(node_1.id);
	uav_2.erase_node_i_in_path(node_2.id);
	int f_1 = 0;
	for (int i = 0; i < size(node_1.UAV_id_list); i++) {
		if(node_1.UAV_id_list[i] == uav_1.id) {
			f_1 = i;
			break;
		}
	}
	node_1.UAV_id_list.erase(node_1.UAV_id_list.begin() + f_1);
	node_1.UAV_id_list.push_back(uav_2.id);
	int f_2 = 0;
	for (int i = 0; i < size(node_2.UAV_id_list); i++) {
		if (node_2.UAV_id_list[i] == uav_2.id) {
			f_2 = i;
			break;
		}
	}
	node_2.UAV_id_list.erase(node_2.UAV_id_list.begin() + f_2);
	node_2.UAV_id_list.push_back(uav_1.id);
	assign_node_no();

	int idx_i = node_1.UAV_id_list[size(node_1.UAV_id_list) - 1];
	auto& uav_out = id_to_UAV(idx_i);
	int fl = 0;
	if (size(uav_out.Path) == 0) {
		uav_out.Path.push_back(node_1);
	}
	for (int i_tt = 0; i_tt < size(uav_out.Path) - 1; i_tt++) {
		int idx_first = uav_out.Path[i_tt].id;
		int idx_second = uav_out.Path[i_tt + 1].id;
		auto& node_first = id_to_node(idx_first);
		auto& node_second = id_to_node(idx_second);
		if (compare_node(node_first, node_1) && (compare_node(node_1, node_second))) {
			uav_out.Path.insert(uav_out.Path.begin() + i_tt + 1, node_1);
			fl = 1;
			break;
		}
	}
	if (fl == 0) {
		int node_first_id = uav_out.Path[0].id;
		int node_last_id = uav_out.Path[size(uav_out.Path) - 1].id;
		auto& node_first_0 = id_to_node(node_first_id);
		auto& node_last_0 = id_to_node(node_last_id);
		if (compare_node(node_1, node_first_0)) {
			uav_out.Path.insert(uav_out.Path.begin(), node_1);
		}
		if (compare_node(node_last_0, node_1)) {
			uav_out.Path.insert(uav_out.Path.end(), node_1);
		}
	}


	int idx_ii = node_2.UAV_id_list[size(node_2.UAV_id_list) - 1];
	auto& uav_outt = id_to_UAV(idx_ii);
	int fll = 0;
	if (size(uav_outt.Path) == 0) {
		uav_outt.Path.push_back(node_2);
	}
	for (int i_tt = 0; i_tt < size(uav_outt.Path) - 1; i_tt++) {
		int idx_first = uav_outt.Path[i_tt].id;
		int idx_second = uav_outt.Path[i_tt + 1].id;
		auto& node_first = id_to_node(idx_first);
		auto& node_second = id_to_node(idx_second);
		if (compare_node(node_first, node_2) && (compare_node(node_2, node_second))) {
			uav_outt.Path.insert(uav_outt.Path.begin() + i_tt + 1, node_2);
			fll = 1;
			break;
		}
	}
	if (fll == 0) {
		int node_first_id = uav_outt.Path[0].id;
		int node_last_id = uav_outt.Path[size(uav_outt.Path) - 1].id;
		auto& node_first_0 = id_to_node(node_first_id);
		auto& node_last_0 = id_to_node(node_last_id);
		if (compare_node(node_2, node_first_0)) {
			uav_outt.Path.insert(uav_outt.Path.begin(), node_2);
		}
		if (compare_node(node_last_0, node_2)) {
			uav_outt.Path.insert(uav_outt.Path.end(), node_2);
		}
	}
}


void Solution::ls_node_swap() {
	int rnd_uav_list = rand() % size(UAV_list);
	int rnd_uav_no = rand() % size(UAV_list[rnd_uav_list]);
	auto uav_ch = UAV_list[rnd_uav_list][rnd_uav_no];
	int path_node_1;
	int path_node_2;
	vector<UAV> UAV_g1;
	for (int ti = 0; ti < size(UAV_list); ti++) {
		for (int tii = 0; tii < size(UAV_list[ti]); tii++) {
			if (size(UAV_list[ti][tii].Path) > 1) {
				UAV_g1.push_back(UAV_list[ti][tii]);
			}
		}
	}
	if (size(UAV_g1) < 1) {
		return;
	}
	else {
		rnd_uav_list = rand() % size(UAV_g1);
		uav_ch = UAV_g1[rnd_uav_list];
	}
	//while(size(uav_ch.Path) <= 1) {
	//	rnd_uav_list = rand() % size(UAV_list);
	//	rnd_uav_no = rand() % size(UAV_list[rnd_uav_list]);
	//	uav_ch = UAV_list[rnd_uav_list][rnd_uav_no];
	//}
	set<int> no_set;
	while (size(no_set) < 2) {
		int rand_no = rand() % size(uav_ch.Path);
		if (no_set.find(rand_no) == no_set.end()) {
			no_set.insert(rand_no);
		}
	}
	auto iter_i = no_set.begin();
	path_node_1 = uav_ch.Path[*iter_i].id;
	iter_i++;
	path_node_2 = uav_ch.Path[*iter_i].id;

	assign_node_no();
	int loc_1 = id_to_node(path_node_1).no;
	int loc_2 = id_to_node(path_node_2).no;
	int n_all = abs(loc_2 - loc_1) + 1;
	int min_loc = min(loc_1, loc_2);
	int max_loc = max(loc_1, loc_2);
	vector<Working_node> node_list_tmp = node_list;
	for (int i = 0; i < n_all-1; i++) {
		auto node_e = node_list_tmp[max_loc];
		node_list_tmp.erase(node_list_tmp.begin() + max_loc);
		node_list_tmp.insert(node_list_tmp.begin() + min_loc, node_e);
		min_loc++;
	}
	Solution tmp_sol = *this;
	tmp_sol.clear();
	tmp_sol.node_list = node_list_tmp;
	for (auto node : tmp_sol.node_list) {
		for (int idx_u : node.UAV_id_list) {
			auto& uav_ch_i = tmp_sol.id_to_UAV(idx_u);
			uav_ch_i.Path.push_back(node);
		}
	}
	*this = tmp_sol;
}


//void Solution::ls_node_move() {
//	assign_node_no();
//	vector<Working_node> node_list_g2;
//	for (auto& node_i : node_list) {
//		if (size(node_i.UAV_id_list) >= 2) {
//			node_list_g2.push_back(node_i);
//		}
//	}
//	if (size(node_list_g2) < 1) {
//		return;
//	}
//	int rnd_list_no = rand() % size(node_list_g2);
//	auto node_ch = node_list_g2[rnd_list_no];
//	node_list.erase(node_list.begin() + node_ch.no);
//	for (int idx : node_ch.UAV_id_list) {
//		id_to_UAV(idx).erase_node_i_in_path(node_ch.id);
//	}
//	cal_uav_path_t();
//	float* uav_t = new float[size(node_list)+1];
//	for (int i = 0; i < size(node_list) + 1; i++) {
//		
//	}
//
//	delete[] uav_t;
//}