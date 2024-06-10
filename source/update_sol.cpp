#include "solution.h"
#include <vector>
#include "Working_node.h"
#include <algorithm>
#include "update.h"
#include <iostream>
#include "initialization.h"
#include "map_config.h"

const double pi = 3.1415926535897932384626;
extern float alpha_1;
extern float gamma;
//extern float MAX_PRO;
extern float PC;
extern float max_size;
extern float ls_it;
int opt_size=0;
using namespace std;
extern int M;
extern int iteration_time;
typedef float V_vec[MAP_NODE_NUM];
extern V_vec* V_matrix;
typedef float Q_vec[UAV_NUM];
vector<vector<float>> ER_pool;
extern float dis_matric[MAP_NODE_NUM + 1][MAP_NODE_NUM];
extern float eps;
int sample_size = 100;
int buffer_size = 2000;
typedef float Q_vec_1[MAP_NODE_NUM][UAV_NUM];
extern Q_vec_1* Q_table;
extern Q_vec_1* Q_table_1;
extern int IT;
int MAX_C = 300;
float fac = 5;
extern int mode;
float ls1=0.25;
float ls2 = 0.25;
float ls3 = 0.25;
float ls4 = 0.25;
extern float T;
void update_sol(vector<Solution>& sol_list, vector<Working_node>& wn_list, int flag_mode, int unchange_counter) {

	vector<Solution>  sol_list_l;
	for (int ps_i = 0; ps_i < size(sol_list) / 2; ps_i++) {
		vector<Solution> sol_par = parent_gen(sol_list, 2, 0);
		sol_list_l.push_back(sol_par[0]);
		sol_list_l.push_back(sol_par[1]);
	}
	//int kk = -1;
	//Q_vec_1* Q_matrix = new Q_vec_1[MAP_NODE_NUM];
	//for (auto sol_i : sol_list) {
	//	//kk++;
	//	//if (kk == 0) {
	//	//	continue;
	//	//}
	//	sol_i.cal_uav_path_t();
	//	sol_i.clear_ext_path_node(1);
	//	
	//	float min_size = FLT_MAX;
	//	for (auto node_ll : sol_i.node_list) {
	//		if (node_ll.node_size < min_size) {
	//			min_size = node_ll.node_size;
	//		}
	//	}
	//	float t_left=0;
	//	for (auto uav_t : sol_i.UAV_list) {
	//		for (auto uav_tt : uav_t) {
	//			t_left += T - uav_tt.t_used;
	//		}
	//	}
	//	if (t_left*UAV::eff < min_size) {
	//	
	//		continue;
	//	}
	//	//else {
	//	//	cout << "update" << endl;
	//	//}
	//	float r=0;
	//	vector<vector<Working_node>> sub_paths = sol_i.sub_path_generation();
	//	//for (auto sub_path : sub_paths) {
	//	//	for (auto sub_node : sub_path) {
	//	//		cout << sub_node.id << "-->";
	//	//	}
	//	//	cout << endl;
	//	//}
	//	//vector<int> vect_rel_num;
	//	//vector<float> vec_size_rel;
	//	//vector<float> t_eff_vec;
	//	//vector <float> delta_vec;
	//	//for (int node_no =0; node_no <size(sol_i.node_list); node_no++) {
	//	//	float t_us = 0;
	//	//	auto& node_i = sol_i.node_list[node_no];
	//	//	float t_sum = 0;
	//	//	for (int uav_iu_no = 0; uav_iu_no < size(node_i.UAV_id_list);uav_iu_no++) {
	//	//		int uav_iu = node_i.UAV_id_list[uav_iu_no];
	//	//		int path_no = node_i.Path_node_no_list[uav_iu_no];
	//	//		auto uav_iuu = sol_i.id_to_UAV(uav_iu);
	//	//		float t_ee = 0;
	//	//		t_sum += uav_iuu.Path[path_no].t_start;
	//	//		if (path_no == 0) {
	//	//			t_ee = uav_iuu.Path_node_t_list[0];
	//	//		}
	//	//		else {
	//	//			t_ee = uav_iuu.Path_node_t_list[path_no]- uav_iuu.Path_node_t_list[path_no-1];
	//	//		}
	//	//		float t_u = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path[path_no].t_start;
	//	//		t_us += t_u / t_ee;
	//	//	}
	//	//	float t_ave = t_sum / size(node_i.UAV_id_list);
	//	//	t_eff_vec.push_back(t_us/size(node_i.UAV_id_list));
	//	//	//cout << t_us / size(node_i.UAV_id_list) << endl;
	//	//	//int rel_num = 0;
	//	//	//float size_rel = 0;
	//	//	//
	//	//	//set<int> type_i = node_i.type;
	//	//	//if (node_no > 0) {
	//	//	//	for (int node_rel = 0; node_rel < node_no; node_rel++) {
	//	//	//		auto& node_ii = sol_i.node_list[node_rel];
	//	//	//		set<int> type_ii = node_ii.type;
	//	//	//		int re_flag = 0;
	//	//	//		for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
	//	//	//			for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
	//	//	//				if (*iter == *iter_i) {
	//	//	//					re_flag = 1;
	//	//	//				}
	//	//	//			}
	//	//	//		}
	//	//	//		if (re_flag == 1) {
	//	//	//			rel_num++;
	//	//	//			size_rel += node_ii.node_size;
	//	//	//		}
	//	//	//	}
	//	//	//}
	//	//	//vect_rel_num.push_back(rel_num);
	//	//	//vec_size_rel.push_back(size_rel);
	//	//}
	//	vector<vector<float>> t_eff_vec;
	//	vector<vector<float>> t_left_vec;
	//	vector <float> delta_vec;
	//	vector<vector<float>> t_eff_min;
	//	vector<vector<float>> path_l;
	//	for (int path_no = 0; path_no < size(sub_paths); path_no++) {
	//		vector<float> sub_t_eff;
	//		vector<float> sub_t_left;
	//		vector<float> sub_t_min;
	//		vector<float> sub_path_l;
	//		for (int node_no =0; node_no <size(sub_paths[path_no]); node_no++) {
	//			float t_us = 0;
	//			float t_l = 0;
	//			auto& node_i = sub_paths[path_no][node_no];
	//			float f_sum=0;
	//			float t_sum = 0;
	//			float t_min= FLT_MAX; 
	//			float p_l = 0;
	//			for (int uav_iu_no = 0; uav_iu_no < size(node_i.UAV_id_list);uav_iu_no++) {
	//				int uav_iu = node_i.UAV_id_list[uav_iu_no];
	//				int path_no = node_i.Path_node_no_list[uav_iu_no];
	//				auto uav_iuu = sol_i.id_to_UAV(uav_iu);
	//				float t_ee = 0;
	//				t_sum += uav_iuu.Path[path_no].t_start;
	//				if (path_no == 0) {
	//					t_ee = uav_iuu.Path_node_t_list[0];
	//					p_l += dis_matric[MAP_NODE_NUM][node_i.id];
	//				}
	//				else {
	//					t_ee = uav_iuu.Path_node_t_list[path_no]- uav_iuu.Path_node_t_list[path_no-1];
	//					p_l += dis_matric[uav_iuu.Path[path_no-1].id][node_i.id];
	//				}
	//				float t_u = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path[path_no].t_start;
	//				t_us += t_u / t_ee;
	//				t_min = min(t_min, t_u / t_ee);
	//			}
	//			sub_path_l.push_back(p_l);
	//			float t_ave = t_sum / size(node_i.UAV_id_list);
	//			sub_t_eff.push_back(t_us / size(node_i.UAV_id_list));
	//			f_sum += node_i.node_size;
	//			sub_t_min.push_back(t_min);
	//		}
	//		t_eff_min.push_back(sub_t_min);
	//		t_eff_vec.push_back(sub_t_eff);	
	//		path_l.push_back(sub_path_l);
	//	}
	//	for (int path_no = 0; path_no < size(sub_paths); path_no++) {
	//		float rr = 0;
	//		for (int node_no = size(sub_paths[path_no])-1; node_no >-1; node_no--) {
	//			
	//			auto& node_i = sub_paths[path_no][node_no];
	//			//if (node_no == 0) {
	//			//	r = node_i.node_size / path_l[path_no][node_no];
	//			//}
	//			//else {
	//			//	r = node_i.node_size / path_l[path_no][node_no];
	//			//}
	//			//r = (delta_vec[path_no]-rr) * t_eff_vec[path_no][node_no]/(node_no+1);
	//			//cout << t_eff_min[path_no][node_no]<<endl;
	//			//r = node_i.node_size * t_eff_min[path_no][node_no];
	//			//r = node_i.node_size ;//* t_eff_vec[path_no][node_no]
	//			/*r = node_i.node_size * t_eff_vec[path_no][node_no];*/
	//			r = node_i.node_size/ path_l[path_no][node_no];
	//			rr += node_i.node_size;
	//			//r = (node_i.node_size)* t_eff_vec[path_no][node_no]/(sol_i.fitness-rr) ;// / (max_size)
	//			//r = (sol_i.fitness) / (max_size) * (node_i.node_size) / ( max_size)*t_eff_vec[node_no] ;
	//			//r = (vec_size_rel[node_no]+node_i.node_size)*t_eff_vec[node_no]/ (max_size*(vect_rel_num[node_no]+1));//
	//			//rr += node_i.node_size;
	//			Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = r;
	//			if (node_no == size(sub_paths[path_no]) - 1) {
	//				Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1]);
	//			}
	//			else {
	//				int next = sol_i.node_list[node_no + 1].id;
	//				float max_next_q = 0;
	//				for (int next_p = 0; next_p < UAV_NUM; next_p++) {
	//					if (Q_table_1[next][node_no+1][next_p] > max_next_q) {
	//						max_next_q = Q_table_1[next][node_no + 1][next_p];
	//					}
	//				}
	//				Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1]);
	//			}
	//		}
	//		
	//	}
	//	//for (auto kk : vect_rel_num) {
	//	//	cout << kk<<endl;
	//	//}
	//	//for (int node_no = size(sol_i.node_list)-1; node_no >-1 ; node_no--) {
	//	//	int rel_num = 0;
	//	//	auto& node_i = sol_i.node_list[node_no];
	//	//	
	//	//	Q_table[node_i.id][node_no][size(node_i.UAV_id_list) - 1] += 1;
	//	//	r = (node_i.node_size) / (max_size);
	//	//	//r = (sol_i.fitness-node_i.node_size)/ (max_size*(node_no+1));
	//	//	Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = r;
	//	//	if (node_no == size(sol_i.node_list) - 1) {
	//	//		Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1]);
	//	//	}
	//	//	else {
	//	//		int next = sol_i.node_list[node_no + 1].id;
	//	//		float max_next_q = 0;
	//	//		for (int next_p = 0; next_p < UAV_NUM; next_p++) {
	//	//			if (Q_table_1[next][node_no + 1][next_p] > max_next_q) {
	//	//				max_next_q = Q_table_1[next][node_no + 1][next_p];
	//	//			}
	//	//		}
	//	//		Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1]);
	//	//	}
	//	//}
	//	//float rr = 0;
	//	//for (int node_no = size(sol_i.node_list) - 1; node_no > -1; node_no--) {
	//	//	int rel_num = 0;
	//	//	auto& node_i = sol_i.node_list[node_no];
	//	//	Q_table[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] += 1;
	//	//	r = (node_i.node_size)*t_eff_vec[node_no] / (max_size);//
	//	//	//r = (sol_i.fitness) / (max_size) * (node_i.node_size) / ( max_size)*t_eff_vec[node_no] ;
	//	//	//r = (vec_size_rel[node_no]+node_i.node_size)*t_eff_vec[node_no]/ (max_size*(vect_rel_num[node_no]+1));//
	//	//	//rr += node_i.node_size;
	//	//	Q_matrix[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] = r;
	//	//	if (node_no == size(sol_i.node_list) - 1) {
	//	//		Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1]);
	//	//	}
	//	//	else {
	//	//		int next = sol_i.node_list[node_no + 1].id;
	//	//		float max_next_q = 0;
	//	//		for (int next_p = 0; next_p < UAV_NUM; next_p++) {
	//	//			if (Q_table_1[next][vect_rel_num[node_no+1]][next_p] > max_next_q) {
	//	//				max_next_q = Q_table_1[next][vect_rel_num[node_no + 1]][next_p];
	//	//			}
	//	//		}
	//	//		Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1]);
	//	//	}
	//	//}
	//}
	////for (auto sol_i : ors_all) {
	////	for (int node_no = 0; node_no < size(sol_i.node_list); node_no++) {
	////		auto& node_i = sol_i.node_list[node_no];
	////		Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = sol_i.fitness / max_size;
	////		vector<float> ex;
	////		ex.push_back(node_i.id);
	////		ex.push_back(node_no);
	////		ex.push_back(size(node_i.UAV_id_list)-1);
	////		ex.push_back(sol_i.fitness / max_size);
	////		if (node_no == size(sol_i.node_list) - 1) {
	////		}
	////		else {
	////			int next = sol_i.node_list[node_no + 1].id;
	////			ex.push_back(next);
	////			ex.push_back(node_no + 1);
	////		}
	////		ER_pool.push_back(ex);
	////		if (size(ER_pool) > buffer_size) {
	////			ER_pool.erase(ER_pool.begin());
	////		}
	////	}
	////}
	////for (int ep = 0; ep < min(sample_size, (int)size(ER_pool)); ep++) {
	////	int no_ep = rand() % size(ER_pool);
	////	if (size(ER_pool) == 4) {
	////		auto ep_i = ER_pool[no_ep];
	////		Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] = Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] + alpha_1 * (ep_i[3] - Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]]);
	////	}
	////	else {
	////		auto ep_i = ER_pool[no_ep];
	////		int next = (int)ep_i[4];
	////		float max_next_q = 0;
	////		for (int next_p = 0; next_p < UAV_NUM; next_p++) {
	////			if (Q_table_1[next][(int)ep_i[5]][next_p] > max_next_q) {
	////				max_next_q = Q_table_1[next][(int)ep_i[5]][next_p];
	////			}
	////		}
	////		Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] = Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] + alpha_1 * (ep_i[3]+gamma* max_next_q - Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]]);
	////	}
	////}
	//delete[] Q_matrix;
	vector<Solution> ors_all;
	vector<Solution> sol_list_tmp;
	for (auto os : sol_list_l) {
		os.mutate(wn_list, flag_mode, unchange_counter);//变异
		//auto os_copy = os;
		//os.local_search(3);//交换
		os.repair_ig(flag_mode);//修复
		//os_copy.repair(flag_mode);
		os.clear_ext_path_node(1);
		////auto sol_tmp = os;
		//int s1 = size(os.node_list);
		//os.insert_rnd_nodes(wn_list, unchange_counter);//插入
		//int s2 = size(os.node_list);
		//cout << s2 - s1 << endl;
		//os.clear_ext_path_node(1);
		////int s1 = size(os.node_list);
		//complement(os, wn_list);
		//int s2 = size(os.node_list);
		//cout << s2 - s1 << endl;
		os.cal_fitness();
		ors_all.push_back(os);

	}
	int ls_num[4] = { 0, 0, 0, 0 };
	for (int m = 0; m < size(ors_all); m++) {
		if (mode == 4) {
			break;
		}
		auto os = ors_all[m];
		float f_fitness = os.fitness;
		//float min_size = FLT_MAX;
		//for (auto node_ll : wn_list) {
		//	if (!os.check_node_id(node_ll.id)) {
		//		min_size = min(min_size, node_ll.node_size);
		//	}
		//}
		//float t_left = 0;
		//for (auto uav_t : os.UAV_list) {
		//	for (auto uav_tt : uav_t) {
		//		t_left += T - uav_tt.t_used;
		//	}
		//}
		//if (t_left * UAV::eff < min_size) {
		//	vector<Solution> os_ls_list;
		//	int success_flag = 0;
		//	for (int k = 0; k < ls_it; k++) {
		//		auto sol_tmp = os;
		//		sol_tmp.ls_node_swap();
		//		sol_tmp.cal_uav_path_t();
		//		//sol_tmp.clear_ext_path_node(1);
		//		//Solution sol_test = sol_tmp;
		//		sol_tmp.ls_node_exchange(wn_list, unchange_counter);
		//		if (!sol_tmp.need_repair()) {
		//			ls_num[1]++;
		//			//cout << "s2" << endl;
		//			//sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
		//			sol_tmp.cal_fitness();
		//			os_ls_list.push_back(sol_tmp);
		//			success_flag = 1;
		//			break;
		//		}
		//		//if (!sol_tmp.need_repair()) {
		//		//	//cout << "ls_3" << endl;
		//		//	//sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
		//		//	//sol_tmp.cal_fitness();
		//		//	//os_ls_list.push_back(sol_tmp);
		//		//	//success_flag = 1;
		//		//	sol_tmp.ls_node_exchange(wn_list, unchange_counter);
		//		//	if (!sol_tmp.need_repair()) {
		//		//		sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
		//		//		sol_tmp.cal_fitness();
		//		//		os_ls_list.push_back(sol_tmp);
		//		//		success_flag = 1;
		//		//	}
		//		//}
		//	}
		//	sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		//	if (success_flag == 1) {
		//		os = os_ls_list[0];
		//		//ls_num[1]++;
		//	}
		//	continue;
		//	
		//}
		float rnd_pro = (float)rand() / RAND_MAX;
		float t_left_l = 0;
		for (auto uav_t : os.UAV_list) {
			for (auto uav_tt : uav_t) {
				t_left_l += T - uav_tt.t_used;
			}
		}		
//		if (rnd_pro >= 0) {//0
////rnd_pro >= 0.5 && rnd_pro < 0.75
//			vector<Solution> os_ls_list;
//			int success_flag = 0;
//			float min_size = FLT_MAX;
//			for (auto node_ll : wn_list) {
//				if (!os.check_node_id(node_ll.id)) {
//					min_size = min(min_size, node_ll.node_size);
//				}
//			}
//			float t_left = 0;
//			for (auto uav_t : os.UAV_list) {
//				for (auto uav_tt : uav_t) {
//					t_left += T - uav_tt.t_used;
//				}
//			}
//			if (t_left * UAV::eff < min_size) {
//				continue;
//			}
//			else {
//				os.insert_rnd_nodes(wn_list, 0);
//				os.cal_fitness();
//			}
//		}

		//if (rnd_pro <= 0.5) {
		//	vector<Solution> os_ls_list;
		//	int success_flag = 0;
		//	for (int k = 0; k < ls_it; k++) {
		//		ls_num[0]++;
		//		auto sol_tmp = os;
		//		sol_tmp.local_search(1);
		//		//sol_tmp.cal_uav_path_t();
		//		////Solution sol_test = sol_tmp;
		//		////sol_tmp.clear_ext_path_node(1);
		//		//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
		//		if (!sol_tmp.need_repair()) {
		//			//cout << "s1" << endl;
		//			//sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
		//			float t_left_sub = 0;
		//			for (auto uav_t : sol_tmp.UAV_list) {
		//				for (auto uav_tt : uav_t) {
		//					t_left_sub += T - uav_tt.t_used;
		//				}
		//			}
		//			if (t_left_sub > t_left_l) {
		//				t_left_l = t_left_sub;
		//				sol_tmp.cal_fitness();
		//				os_ls_list.push_back(sol_tmp);
		//				success_flag = 1;
		//				break;
		//			}
		//		}
		//		//if (!sol_tmp.need_repair()) {
		//		//	//sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
		//		//	//sol_tmp.cal_fitness();
		//		//	//os_ls_list.push_back(sol_tmp);
		//		//	//success_flag = 1;
		//		//	sol_tmp.ls_node_exchange(wn_list, unchange_counter);
		//		//	if (!sol_tmp.need_repair()) {
		//		//		sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
		//		//		sol_tmp.cal_fitness();
		//		//		os_ls_list.push_back(sol_tmp);
		//		//		success_flag = 1;
		//		//	}
		//		//}
		//	}
		//	//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		//	if (success_flag == 1 ) {
		//		//ls_num[0]++;
		//		os = os_ls_list[size(os_ls_list) - 1];
		//	}
		//	//else {
		//	//	os.insert_rnd_nodes(wn_list, unchange_counter);//插入
		//	//	os.cal_fitness();
		//	//}
		//}

		////if (rnd_pro< (ls1+ls2) &&rnd_pro>= ls1) {
		////	vector<Solution> os_ls_list;
		////	int success_flag = 0;
		////	for (int k = 0; k < ls_it; k++) {
		////		auto sol_tmp = os;
		////		 sol_tmp.ls_node_swap();
		////		 sol_tmp.cal_uav_path_t();
		////		 //sol_tmp.clear_ext_path_node(1);
		////		 //Solution sol_test = sol_tmp;
		////		 sol_tmp.ls_node_exchange(wn_list, unchange_counter);
		////		 if (!sol_tmp.need_repair()) {
		////			 ls_num[1]++;
		////			 //cout << "s2" << endl;
		////			 //sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
		////			 sol_tmp.cal_fitness();
		////			 os_ls_list.push_back(sol_tmp);
		////			 success_flag = 1;
		////			 break;
		////		 }
		////		//if (!sol_tmp.need_repair()) {
		////		//	//cout << "ls_3" << endl;
		////		//	//sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
		////		//	//sol_tmp.cal_fitness();
		////		//	//os_ls_list.push_back(sol_tmp);
		////		//	//success_flag = 1;
		////		//	sol_tmp.ls_node_exchange(wn_list, unchange_counter);
		////		//	if (!sol_tmp.need_repair()) {
		////		//		sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
		////		//		sol_tmp.cal_fitness();
		////		//		os_ls_list.push_back(sol_tmp);
		////		//		success_flag = 1;
		////		//	}
		////		//}
		////	}
		////	sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		////	if (success_flag == 1) {
		////		os = os_ls_list[0];
		////		//ls_num[1]++;
		////	}
		////	//else {
		////	//	os.insert_rnd_nodes(wn_list,  unchange_counter);//插入
		////	//	os.cal_fitness();
		////	//}
		////}

		if (rnd_pro < 0.33) {//rnd_pro < 0.66 && rnd_pro >= 0.33
			vector<Solution> os_ls_list;
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = os;
				sol_tmp.ls_path_intersect();
				//sol_tmp.cal_uav_path_t();
				////Solution sol_test = sol_tmp;
				////sol_tmp.clear_ext_path_node(1);
				//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
				if (!sol_tmp.need_repair()) {
					//cout << "s4" << endl;
					//sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
					float t_left_sub = 0;
					for (auto uav_t : sol_tmp.UAV_list) {
						for (auto uav_tt : uav_t) {
							t_left_sub += T - uav_tt.t_used;
						}
					}
					if (t_left_sub > t_left_l) {
						t_left_l = t_left_sub;
						sol_tmp.cal_fitness();
						os_ls_list.push_back(sol_tmp);
						success_flag = 1;
						//break;
					}
				}
				//if (!sol_tmp.need_repair()) {
				//	//sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
				//	//sol_tmp.cal_fitness();
				//	//os_ls_list.push_back(sol_tmp);
				//	//success_flag = 1;
				//	sol_tmp.ls_node_exchange(wn_list, unchange_counter);
				//	if (!sol_tmp.need_repair()) {
				//		sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
				//		sol_tmp.cal_fitness();
				//		os_ls_list.push_back(sol_tmp);
				//		success_flag = 1;
				//	}
				//}
			}
			//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				os = os_ls_list[size(os_ls_list)-1];
			}
		}
		//Solution sol_test = os;
		//if (sol_test.need_repair()) {
		//	cout << "wrong" << endl;
		//}
		//os.clear_ext_path_node(1);
		if (rnd_pro >= 0.33&& rnd_pro < 0.67) {//0
			//rnd_pro >= 0.5 && rnd_pro < 0.75
			vector<Solution> os_ls_list;
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				ls_num[2]++;
				auto sol_tmp = os;
				/*sol_tmp.ls_mutate(wn_list, flag_mode, col_num_choosed_pro_list, unchange_counter);*/
				sol_tmp.ls_node_exchange_m(wn_list, unchange_counter);
				if (!sol_tmp.need_repair()) {
					//cout << "s3_1" << endl;
					//sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
					sol_tmp.cal_fitness();
					//os = sol_tmp;
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				os = os_ls_list[0];
				//ls_num[2]++;
			}
		}

		if (rnd_pro >= 0.67) {//0
	//rnd_pro >= 0.5 && rnd_pro < 0.75
			vector<Solution> os_ls_list;
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				ls_num[2]++;
				auto sol_tmp = os;
				/*sol_tmp.ls_mutate(wn_list, flag_mode, col_num_choosed_pro_list, unchange_counter);*/
				sol_tmp.ls_node_del();
				if (!sol_tmp.need_repair()) {
					//cout << "s3_1" << endl;
					//sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
					sol_tmp.cal_fitness();
					//os = sol_tmp;
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				os = os_ls_list[0];
				//ls_num[2]++;
			}
		}
		if (rnd_pro >=0 ) {//0
			//rnd_pro >= 0.5 && rnd_pro < 0.75
			vector<Solution> os_ls_list;
			int success_flag = 0;
			float min_size = FLT_MAX;
			for (auto node_ll : wn_list) {
				if (!os.check_node_id(node_ll.id)) {
					min_size = min(min_size, node_ll.node_size);
				}
			}
			float t_left = 0;
			for (auto uav_t : os.UAV_list) {
				for (auto uav_tt : uav_t) {
					t_left += T - uav_tt.t_used;
				}
			}
			if (t_left * UAV::eff < min_size) {
				continue;
			}
			else {
				os.insert_rnd_nodes(wn_list, 0);
				os.cal_fitness();
			}
		}
		if (os.fitness >= f_fitness) {
			ors_all.erase(ors_all.begin() + m);
			ors_all.insert(ors_all.begin() + m, os);
		}
		//else {
		//	float pro_rnd = (float)rand() / RAND_MAX;
		//	float accept_c = (float)(f_fitness - os.fitness);
		//	if (pro_rnd < exp(-accept_c / 5000)) {
		//		ors_all.erase(ors_all.begin() + m);
		//		ors_all.insert(ors_all.begin() + m, os);
		//	}
		//}
	}
	int kk = -1;
	Q_vec_1* Q_matrix = new Q_vec_1[MAP_NODE_NUM];
	for (auto sol_i : ors_all) {
		//kk++;
		//if (kk == size(sol_list)) {
		//	break;
		//}
		sol_i.cal_uav_path_t();
		sol_i.clear_ext_path_node(1);

		float min_size = FLT_MAX;
		for (auto node_ll : wn_list) {
			if (!sol_i.check_node_id(node_ll.id )) {
				min_size = min(min_size,node_ll.node_size);
			}
		}
		for (auto node_ll : sol_i.node_list) {
			min_size = min(min_size, node_ll.node_size);
		}
		float t_left = 0;
		for (auto uav_t : sol_i.UAV_list) {
			for (auto uav_tt : uav_t) {
				t_left += T - uav_tt.t_used;
			}
		}
		if (t_left * UAV::eff < min_size) {
			continue;
		}
		//else {
		//	cout << "update" << endl;
		//}
		float r = 0;
		vector<vector<Working_node>> sub_paths = sol_i.sub_path_generation();
		//for (auto sub_path : sub_paths) {
		//	for (auto sub_node : sub_path) {
		//		cout << sub_node.id << "-->";
		//	}
		//	cout << endl;
		//}
		//vector<int> vect_rel_num;
		//vector<float> vec_size_rel;
		//vector<float> t_eff_vec;
		//vector <float> delta_vec;
		//for (int node_no =0; node_no <size(sol_i.node_list); node_no++) {
		//	float t_us = 0;
		//	auto& node_i = sol_i.node_list[node_no];
		//	float t_sum = 0;
		//	for (int uav_iu_no = 0; uav_iu_no < size(node_i.UAV_id_list);uav_iu_no++) {
		//		int uav_iu = node_i.UAV_id_list[uav_iu_no];
		//		int path_no = node_i.Path_node_no_list[uav_iu_no];
		//		auto uav_iuu = sol_i.id_to_UAV(uav_iu);
		//		float t_ee = 0;
		//		t_sum += uav_iuu.Path[path_no].t_start;
		//		if (path_no == 0) {
		//			t_ee = uav_iuu.Path_node_t_list[0];
		//		}
		//		else {
		//			t_ee = uav_iuu.Path_node_t_list[path_no]- uav_iuu.Path_node_t_list[path_no-1];
		//		}
		//		float t_u = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path[path_no].t_start;
		//		t_us += t_u / t_ee;
		//	}
		//	float t_ave = t_sum / size(node_i.UAV_id_list);

		//	t_eff_vec.push_back(t_us/size(node_i.UAV_id_list));
		//	//cout << t_us / size(node_i.UAV_id_list) << endl;
		//	//int rel_num = 0;
		//	//float size_rel = 0;
		//	//
		//	//set<int> type_i = node_i.type;
		//	//if (node_no > 0) {
		//	//	for (int node_rel = 0; node_rel < node_no; node_rel++) {
		//	//		auto& node_ii = sol_i.node_list[node_rel];
		//	//		set<int> type_ii = node_ii.type;
		//	//		int re_flag = 0;
		//	//		for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
		//	//			for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
		//	//				if (*iter == *iter_i) {
		//	//					re_flag = 1;
		//	//				}
		//	//			}
		//	//		}
		//	//		if (re_flag == 1) {
		//	//			rel_num++;
		//	//			size_rel += node_ii.node_size;
		//	//		}
		//	//	}
		//	//}
		//	//vect_rel_num.push_back(rel_num);
		//	//vec_size_rel.push_back(size_rel);
		//}
		
		vector<vector<int>> vect_rel_num;
		vector<vector<float>> t_eff_vec;
		vector<vector<float>> t_left_vec;
		vector <float> delta_vec;
		vector<vector<float>> t_eff_min;
		vector<vector<float>> path_l;
		for (int path_no = 0; path_no < size(sub_paths); path_no++) {
			vector<float> sub_t_eff;
			vector<float> sub_t_left;
			vector<float> sub_t_min;
			vector<float> sub_path_l;
			vector<int> sub_rel_l;
			for (int node_no = 0; node_no < size(sub_paths[path_no]); node_no++) {
				float t_us = 0;
				float t_l = 0;
				auto& node_i = sub_paths[path_no][node_no];
				float f_sum = 0;
				float t_sum = 0;
				float t_min = FLT_MAX;
				float p_l = 0;
				for (int uav_iu_no = 0; uav_iu_no < size(node_i.UAV_id_list); uav_iu_no++) {
					int uav_iu = node_i.UAV_id_list[uav_iu_no];
					int path_no = node_i.Path_node_no_list[uav_iu_no];
					auto uav_iuu = sol_i.id_to_UAV(uav_iu);
					if (uav_iuu.type != node_i.sub_path_id) {
						continue;
					}
					float t_ee = 0;
					t_sum += uav_iuu.Path[path_no].t_start;
					if (path_no == 0) {
						t_ee = uav_iuu.Path_node_t_list[0];
						p_l += dis_matric[MAP_NODE_NUM][node_i.id];
					}
					else {
						t_ee = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path_node_t_list[path_no - 1];
						p_l += dis_matric[uav_iuu.Path[path_no - 1].id][node_i.id];
					}
					float t_u = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path[path_no].t_start;
					t_us += t_u / t_ee;
					t_min = min(t_min, t_u / t_ee);
				}
				sub_path_l.push_back(p_l);
				float t_ave = t_sum / size(node_i.UAV_id_list);
				sub_t_eff.push_back(t_us / size(node_i.UAV_id_list));
				f_sum += node_i.node_size;
				sub_t_min.push_back(t_min);
				int rel_num = 0;
				float size_rel = 0;
	
				set<int> type_i = node_i.type;
				if (node_no > 0) {
					for (int node_rel = 0; node_rel < node_no; node_rel++) {
						auto& node_ii = sol_i.node_list[node_rel];
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
							size_rel += node_ii.node_size;
						}
					}
				}
				sub_rel_l.push_back(rel_num);
			}
			t_eff_min.push_back(sub_t_min);
			t_eff_vec.push_back(sub_t_eff);
			path_l.push_back(sub_path_l);
			vect_rel_num.push_back(sub_rel_l);
		}

		for (int path_no = 0; path_no < size(sub_paths); path_no++) {
			float rr = 0;
			for (int node_no = size(sub_paths[path_no]) - 1; node_no > -1; node_no--) {

				auto& node_i = sub_paths[path_no][node_no];
				//if (node_no == 0) {
				//	r = node_i.node_size / path_l[path_no][node_no];
				//}
				//else {
				//	r = node_i.node_size / path_l[path_no][node_no];
				//}
				//r = (delta_vec[path_no]-rr) * t_eff_vec[path_no][node_no]/(node_no+1);
				//cout << t_eff_min[path_no][node_no]<<endl;
				//r = node_i.node_size * t_eff_min[path_no][node_no];
				//r = node_i.node_size ;//* t_eff_vec[path_no][node_no]
				/*r = node_i.node_size * t_eff_vec[path_no][node_no];*/
				r = node_i.node_size / path_l[path_no][node_no];
				rr += node_i.node_size;
				//r = (node_i.node_size)* t_eff_vec[path_no][node_no]/(sol_i.fitness-rr) ;// / (max_size)
				//r = (sol_i.fitness) / (max_size) * (node_i.node_size) / ( max_size)*t_eff_vec[node_no] ;
				//r = (vec_size_rel[node_no]+node_i.node_size)*t_eff_vec[node_no]/ (max_size*(vect_rel_num[node_no]+1));//
				//rr += node_i.node_size;
				Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = r;
				if (node_no == size(sub_paths[path_no]) - 1) {
					Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1]);
				}
				else {
					int next = sol_i.node_list[node_no + 1].id;
					float max_next_q = 0;
					for (int next_p = 0; next_p < UAV_NUM; next_p++) {
						if (Q_table_1[next][node_no + 1][next_p] > max_next_q) {
							max_next_q = Q_table_1[next][vect_rel_num[path_no][node_no+ 1]][next_p];
						}
					}
					Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1]);
				}
			}

		}
		//for (auto kk : vect_rel_num) {
		//	cout << kk<<endl;
		//}
		//for (int node_no = size(sol_i.node_list)-1; node_no >-1 ; node_no--) {
		//	int rel_num = 0;
		//	auto& node_i = sol_i.node_list[node_no];
		//	
		//	Q_table[node_i.id][node_no][size(node_i.UAV_id_list) - 1] += 1;
		//	r = (node_i.node_size) / (max_size);
		//	//r = (sol_i.fitness-node_i.node_size)/ (max_size*(node_no+1));
		//	Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = r;
		//	if (node_no == size(sol_i.node_list) - 1) {
		//		Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1]);
		//	}
		//	else {
		//		int next = sol_i.node_list[node_no + 1].id;
		//		float max_next_q = 0;
		//		for (int next_p = 0; next_p < UAV_NUM; next_p++) {
		//			if (Q_table_1[next][node_no + 1][next_p] > max_next_q) {
		//				max_next_q = Q_table_1[next][node_no + 1][next_p];
		//			}
		//		}
		//		Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][node_no][size(node_i.UAV_id_list) - 1]);
		//	}
		//}
		//float rr = 0;
		//for (int node_no = size(sol_i.node_list) - 1; node_no > -1; node_no--) {
		//	int rel_num = 0;
		//	auto& node_i = sol_i.node_list[node_no];

		//	Q_table[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] += 1;
		//	r = (node_i.node_size)*t_eff_vec[node_no] / (max_size);//
		//	//r = (sol_i.fitness) / (max_size) * (node_i.node_size) / ( max_size)*t_eff_vec[node_no] ;
		//	//r = (vec_size_rel[node_no]+node_i.node_size)*t_eff_vec[node_no]/ (max_size*(vect_rel_num[node_no]+1));//
		//	//rr += node_i.node_size;
		//	Q_matrix[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] = r;
		//	if (node_no == size(sol_i.node_list) - 1) {
		//		Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1]);
		//	}
		//	else {
		//		int next = sol_i.node_list[node_no + 1].id;
		//		float max_next_q = 0;
		//		for (int next_p = 0; next_p < UAV_NUM; next_p++) {
		//			if (Q_table_1[next][vect_rel_num[node_no+1]][next_p] > max_next_q) {
		//				max_next_q = Q_table_1[next][vect_rel_num[node_no + 1]][next_p];
		//			}
		//		}
		//		Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[node_no]][size(node_i.UAV_id_list) - 1]);
		//	}
		//}
	}

	//for (auto sol_i : ors_all) {
	//	for (int node_no = 0; node_no < size(sol_i.node_list); node_no++) {
	//		auto& node_i = sol_i.node_list[node_no];
	//		Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = sol_i.fitness / max_size;
	//		vector<float> ex;
	//		ex.push_back(node_i.id);
	//		ex.push_back(node_no);
	//		ex.push_back(size(node_i.UAV_id_list)-1);
	//		ex.push_back(sol_i.fitness / max_size);
	//		if (node_no == size(sol_i.node_list) - 1) {
	//		}
	//		else {
	//			int next = sol_i.node_list[node_no + 1].id;
	//			ex.push_back(next);
	//			ex.push_back(node_no + 1);
	//		}
	//		ER_pool.push_back(ex);
	//		if (size(ER_pool) > buffer_size) {
	//			ER_pool.erase(ER_pool.begin());
	//		}
	//	}
	//}
	//for (int ep = 0; ep < min(sample_size, (int)size(ER_pool)); ep++) {
	//	int no_ep = rand() % size(ER_pool);
	//	if (size(ER_pool) == 4) {
	//		auto ep_i = ER_pool[no_ep];
	//		Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] = Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] + alpha_1 * (ep_i[3] - Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]]);
	//	}
	//	else {
	//		auto ep_i = ER_pool[no_ep];
	//		int next = (int)ep_i[4];
	//		float max_next_q = 0;
	//		for (int next_p = 0; next_p < UAV_NUM; next_p++) {
	//			if (Q_table_1[next][(int)ep_i[5]][next_p] > max_next_q) {
	//				max_next_q = Q_table_1[next][(int)ep_i[5]][next_p];
	//			}
	//		}
	//		Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] = Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] + alpha_1 * (ep_i[3]+gamma* max_next_q - Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]]);
	//	}
	//}

	delete[] Q_matrix;
	//float ls_pro[4] = { 0,0,0,0 };
	//ls_pro[0] = (float)ls_num[0] / (ls1 * size(ors_all)* ls_it);
	//ls_pro[1] = (float)ls_num[1] / (ls2 * size(ors_all));
	//ls_pro[2] = (float)ls_num[2] / (ls3 * size(ors_all));
	//ls_pro[3] = (float)ls_num[3] / (ls4 * size(ors_all));
	//float ls_all = ls_pro[0] + ls_pro[1] + ls_pro[2] + ls_pro[3];
	//if (ls_all == 0) {
	//
	//	ls1 = 0.25;
	//	ls2 = 0.25;
	//	ls3 = 0.25;
	//	ls4 = 0.25;
	//}
	//else {
	//	ls1 = ls_pro[0] / ls_all;
	//	ls2 = ls_pro[1] / ls_all;
	//	ls3 = ls_pro[2] / ls_all;
	//	ls4 = ls_pro[3] / ls_all;

	//}
	//ls1 = max((float)0.1, ls1);
	//ls2 = max((float)0.1, ls2);
	//ls3 = max((float)0.1, ls3);
	//ls4 = max((float)0.1, ls4);
	//ls1 = ls1 / (ls1 + ls2 + ls3 + ls4);
	//ls2 = ls2 / (ls1 + ls2 + ls3 + ls4);
	//ls3 = ls3 / (ls1 + ls2 + ls3 + ls4);
	//ls4 = ls4 / (ls1 + ls2 + ls3 + ls4);
	//cout << ls1 << endl;
	//cout << ls2 << endl;
	//cout << ls3 << endl;
	//cout << ls4 << endl;
	cout << "------------------------------------" << endl;
	//int sum = 0;
	//for (auto& ur : ors_all) {
	//	sum += ur.fitness;
	//}
	//sum = sum / size(ors_all);
	//cout << "mode_0: ave_os_fitness: " << sum << endl;
	if (!flag_mode) {
		ors_all.insert(ors_all.end(), sol_list.begin(), sol_list.begin()+1);//1+round((float)size(sol_list)/10)
		sort(ors_all.begin(), ors_all.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });

		////for (auto sol_i : ors_all) {
		////	for (int node_no = 0; node_no < size(sol_i.node_list); node_no++) {
		////		auto& node_i = sol_i.node_list[node_no];
		////		Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = sol_i.fitness / max_size;
		////		vector<float> ex;
		////		ex.push_back(node_i.id);
		////		ex.push_back(node_no);
		////		ex.push_back(size(node_i.UAV_id_list)-1);
		////		ex.push_back(sol_i.fitness / max_size);
		////		if (node_no == size(sol_i.node_list) - 1) {
		////		}
		////		else {
		////			int next = sol_i.node_list[node_no + 1].id;
		////			ex.push_back(next);
		////			ex.push_back(node_no + 1);
		////		}
		////		ER_pool.push_back(ex);
		////		if (size(ER_pool) > buffer_size) {
		////			ER_pool.erase(ER_pool.begin());
		////		}
		////	}
		////}
		////for (int ep = 0; ep < min(sample_size, (int)size(ER_pool)); ep++) {
		////	int no_ep = rand() % size(ER_pool);
		////	if (size(ER_pool) == 4) {
		////		auto ep_i = ER_pool[no_ep];
		////		Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] = Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] + alpha_1 * (ep_i[3] - Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]]);
		////	}
		////	else {
		////		auto ep_i = ER_pool[no_ep];
		////		int next = (int)ep_i[4];
		////		float max_next_q = 0;
		////		for (int next_p = 0; next_p < UAV_NUM; next_p++) {
		////			if (Q_table_1[next][(int)ep_i[5]][next_p] > max_next_q) {
		////				max_next_q = Q_table_1[next][(int)ep_i[5]][next_p];
		////			}
		////		}
		////		Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] = Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]] + alpha_1 * (ep_i[3]+gamma* max_next_q - Q_table_1[(int)ep_i[0]][(int)ep_i[1]][(int)ep_i[2]]);
		////	}
		////}

		//delete[] Q_matrix;
		//delete[] N_matrix;
		for (int j = 0; j < size(sol_list); j++) {
			//complement(ors_all[j], wn_list);
			//ors_all[j].cal_fitness();
			ors_all[j].sol_id = j;
			//if (sol_list_l[j].fitness < ors_all[j].fitness) {
			//	sol_list_l[j] = ors_all[j];
			//}
			sol_list_tmp.push_back(ors_all[j]);
		}
		//for (int i = 0; i < size(wn_list); i++) {
		//	for (int j = 0; j < wn_list[i].col_UAV_num; j++) {
		//		col_num_choosed_pro_list[i][j] = max((float)0.2, col_num_choosed_pro_list[i][j]);
		//		col_num_choosed_pro_list[i][j] = min((float)2, col_num_choosed_pro_list[i][j]);
		//	}
		//}
	}
	//else {
	//	sort(ors_all.begin(), ors_all.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
	//	//for (int i = 0; i < size(wn_list); i++) {
	//	//	for (int j = 0; j < wn_list[i].col_UAV_num; j++) {
	//	//		col_num_choosed_pro_list[i][j] *= 0.95;
	//	//	}
	//	//}
	//	//for (int i = 0; i < size(ors_all[0].node_list); i++) {
	//	//	col_num_choosed_pro_list[ors_all[0].node_list[i].id][size(ors_all[0].node_list[i].UAV_id_list)-1] += ors_all[0].fitness / (600000*2);
	//	//}
	//	for (int j = 0; j < size(sol_list)-1; j++) {
	//		ors_all[j].sol_id = j;
	//		sol_list_tmp.push_back(ors_all[j]);
	//	}
	//	sol_list[0].sol_id = size(sol_list);
	//	sol_list_tmp.push_back(sol_list[0]);
	//}

	////opt_size = size(sol_list_tmp[0].node_list);
	sol_list = sol_list_tmp;
	//sol_list = sol_list_l;

}

void update_sol_pso(Solution* p_best_sol_list, Solution* a_best_sol, vector<Solution>& sol_list, vector<Working_node>& wn_list, int flag_mode, vector<float*>& col_num_choosed_pro_list, int unchange_counter) {
	vector<Solution> sol_tmp_list;
	int kk = -1;
	Q_vec_1* Q_matrix = new Q_vec_1[MAP_NODE_NUM];
	for (auto sol_i : sol_list) {
		//kk++;
		//if (kk == size(sol_list)) {
		//	break;
		//}
		sol_i.cal_uav_path_t();
		sol_i.clear_ext_path_node(1);

		float min_size = FLT_MAX;
		for (auto node_ll : wn_list) {
			if (!sol_i.check_node_id(node_ll.id)) {
				min_size = min(min_size, node_ll.node_size);
			}
		}
		//for (auto node_ll : sol_i.node_list) {
		//	min_size = min(min_size, node_ll.node_size);
		//}
		float t_left = 0;
		for (auto uav_t : sol_i.UAV_list) {
			for (auto uav_tt : uav_t) {
				t_left += T - uav_tt.t_used;
			}
		}
		if (t_left * UAV::eff < min_size) {
			continue;
		}

		float r = 0;
		vector<vector<Working_node>> sub_paths = sol_i.sub_path_generation();

		vector<vector<int>> vect_rel_num;
		vector<vector<float>> t_eff_vec;
		vector<vector<float>> t_left_vec;
		vector <float> delta_vec;
		vector<vector<float>> t_eff_min;
		vector<vector<float>> path_l;
		for (int path_no = 0; path_no < size(sub_paths); path_no++) {
			vector<float> sub_t_eff;
			vector<float> sub_t_left;
			vector<float> sub_t_min;
			vector<float> sub_path_l;
			vector<int> sub_rel_l;
			for (int node_no = 0; node_no < size(sub_paths[path_no]); node_no++) {
				float t_us = 0;
				float t_l = 0;
				auto& node_i = sub_paths[path_no][node_no];
				float f_sum = 0;
				float t_sum = 0;
				float t_min = FLT_MAX;
				float p_l = 0;
				for (int uav_iu_no = 0; uav_iu_no < size(node_i.UAV_id_list); uav_iu_no++) {
					int uav_iu = node_i.UAV_id_list[uav_iu_no];
					int path_no = node_i.Path_node_no_list[uav_iu_no];
					auto uav_iuu = sol_i.id_to_UAV(uav_iu);
					if (uav_iuu.type != node_i.sub_path_id) {
						continue;
					}
					float t_ee = 0;
					t_sum += uav_iuu.Path[path_no].t_start;
					if (path_no == 0) {
						t_ee = uav_iuu.Path_node_t_list[0];
						p_l += dis_matric[MAP_NODE_NUM][node_i.id];
					}
					else {
						t_ee = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path_node_t_list[path_no - 1];
						p_l += dis_matric[uav_iuu.Path[path_no - 1].id][node_i.id];
					}
					float t_u = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path[path_no].t_start;
					t_us += t_u / t_ee;
					t_min = min(t_min, t_u / t_ee);
				}
				sub_path_l.push_back(p_l);
				float t_ave = t_sum / size(node_i.UAV_id_list);
				sub_t_eff.push_back(t_us / size(node_i.UAV_id_list));
				f_sum += node_i.node_size;
				sub_t_min.push_back(t_min);
				int rel_num = 0;
				float size_rel = 0;

				set<int> type_i = node_i.type;
				if (node_no > 0) {
					for (int node_rel = 0; node_rel < node_no; node_rel++) {
						auto& node_ii = sol_i.node_list[node_rel];
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
							size_rel += node_ii.node_size;
						}
					}
				}
				sub_rel_l.push_back(rel_num);
			}
			t_eff_min.push_back(sub_t_min);
			t_eff_vec.push_back(sub_t_eff);
			path_l.push_back(sub_path_l);
			vect_rel_num.push_back(sub_rel_l);
		}

		for (int path_no = 0; path_no < size(sub_paths); path_no++) {
			float rr = 0;
			for (int node_no = size(sub_paths[path_no]) - 1; node_no > -1; node_no--) {

				auto& node_i = sub_paths[path_no][node_no];
				//if (node_no == 0) {
				//	r = node_i.node_size / path_l[path_no][node_no];
				//}
				//else {
				//	r = node_i.node_size / path_l[path_no][node_no];
				//}
				//r = (delta_vec[path_no]-rr) * t_eff_vec[path_no][node_no]/(node_no+1);
				//cout << t_eff_min[path_no][node_no]<<endl;
				//r = node_i.node_size * t_eff_min[path_no][node_no];
				//r = node_i.node_size ;//* t_eff_vec[path_no][node_no]
				/*r = node_i.node_size * t_eff_vec[path_no][node_no];*/
				r = node_i.node_size / path_l[path_no][node_no];
				rr += node_i.node_size;
				//r = (node_i.node_size)* t_eff_vec[path_no][node_no]/(sol_i.fitness-rr) ;// / (max_size)
				//r = (sol_i.fitness) / (max_size) * (node_i.node_size) / ( max_size)*t_eff_vec[node_no] ;
				//r = (vec_size_rel[node_no]+node_i.node_size)*t_eff_vec[node_no]/ (max_size*(vect_rel_num[node_no]+1));//
				//rr += node_i.node_size;
				Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = r;
				if (node_no == size(sub_paths[path_no]) - 1) {
					Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1]);
				}
				else {
					int next = sol_i.node_list[node_no + 1].id;
					float max_next_q = 0;
					for (int next_p = 0; next_p < UAV_NUM; next_p++) {
						if (Q_table_1[next][node_no + 1][next_p] > max_next_q) {
							max_next_q = Q_table_1[next][vect_rel_num[path_no][node_no + 1]][next_p];
						}
					}
					Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1]);
				}
			}

		}
	}

	delete[] Q_matrix;
 	for (int i = 0; i < size(sol_list); i++) {
		vector<Solution> L_1;
		L_1.push_back(*a_best_sol);
		L_1.push_back(sol_list[i]);
		vector<Solution> L_1_os = cross_over(L_1, flag_mode, unchange_counter);
		vector<Solution> L_2;
		L_2.push_back(p_best_sol_list[i]);
		L_2.push_back(L_1[1]);
		vector<Solution> L_2_ors = cross_over(L_2, flag_mode, unchange_counter);
		Solution final_sol = L_2_ors[1];
		final_sol.mutate(wn_list, flag_mode,  unchange_counter);
		final_sol.repair_ig(flag_mode);
		final_sol.clear_ext_path_node(1);
		final_sol.insert_rnd_nodes(wn_list,  unchange_counter);
		final_sol.cal_fitness();
		float rnd_pro = (float)rand() / RAND_MAX;
		if (rnd_pro < 0.33) {
			vector<Solution> os_ls_list;
			os_ls_list.push_back(final_sol);
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = final_sol;
				sol_tmp.ls_node_del();
				if (!sol_tmp.need_repair()) {
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				final_sol = os_ls_list[0];
				final_sol.insert_rnd_nodes(wn_list, unchange_counter);
				final_sol.cal_fitness();
			}
		}

		if (rnd_pro < 0.67 && rnd_pro >= 0.33) {
			vector<Solution> os_ls_list;
			os_ls_list.push_back(final_sol);
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = final_sol;
				sol_tmp.ls_node_exchange_m(wn_list, unchange_counter);
				if (!sol_tmp.need_repair()) {
					//cout << "ls_3" << endl;
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				final_sol = os_ls_list[0];
				final_sol.insert_rnd_nodes(wn_list, unchange_counter);
				final_sol.cal_fitness();
			}
		}

		//if (rnd_pro >= 0.5 && rnd_pro < 0.7) {
		//	vector<Solution> os_ls_list;
		//	os_ls_list.push_back(final_sol);
		//	int success_flag = 0;
		//	for (int k = 0; k < ls_it; k++) {
		//		auto sol_tmp = final_sol;
		//		sol_tmp.ls_node_swap();
		//		if (!sol_tmp.need_repair()) {
		//			sol_tmp.cal_fitness();
		//			os_ls_list.push_back(sol_tmp);
		//			success_flag = 1;
		//		}
		//	}
		//	sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		//	if (success_flag == 1) {
		//		final_sol = os_ls_list[0];
		//		final_sol.insert_rnd_nodes(wn_list, unchange_counter);
		//		final_sol.cal_fitness();
		//	}
		//}

		if (rnd_pro >= 0.67) {
			vector<Solution> os_ls_list;
			os_ls_list.push_back(final_sol);
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = final_sol;
				sol_tmp.ls_path_intersect();
				if (!sol_tmp.need_repair()) {
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				final_sol = os_ls_list[0];
				final_sol.insert_rnd_nodes(wn_list, unchange_counter);
				final_sol.cal_fitness();
			}
		}
		final_sol.clear_ext_path_node(1);
		sol_tmp_list.push_back(final_sol);
		if (final_sol.fitness > p_best_sol_list[i].fitness) {
			p_best_sol_list[i] = final_sol;
		}
		if (final_sol.fitness > (*a_best_sol).fitness) {
			*a_best_sol = final_sol;
		}
	}
	//for (int i = 0; i < size(wn_list); i++) {
	//	for (int j = 0; j < min(wn_list[i].col_UAV_num,UAV_NUM); j++) {
	//		col_num_choosed_pro_list[i][j] *= 0.92;
	//	}
	//}
	//for (int i = 0; i < size((*a_best_sol).node_list); i++) {
	//	col_num_choosed_pro_list[(*a_best_sol).node_list[i].id][size((*a_best_sol).node_list[i].UAV_id_list) - 1] += (*a_best_sol).node_list[i].node_size / (*a_best_sol).fitness / 5;
	//}
	//Solution sol_p_b = sol_tmp_list[0];
	//for (int i = 0; i < size(sol_list); i++) {
	//	if (sol_p_b.fitness < sol_tmp_list[i].fitness) {
	//		sol_p_b = sol_tmp_list[i];
	//	}
	//}
	//for (int i = 0; i < size(sol_p_b.node_list); i++) {
	//	col_num_choosed_pro_list[sol_p_b.node_list[i].id][size(sol_p_b.node_list[i].UAV_id_list) - 1] += sol_p_b.node_list[i].node_size / sol_p_b.fitness / 5;
	//}
	sol_list = sol_tmp_list;
}
 
void update_sol_ig(Solution& ini_sol_1, vector<Working_node>& wn_list, int unchange_counter) {
	auto os = ini_sol_1;
	float t_left_l = 0;
	for (auto uav_t : os.UAV_list) {
		for (auto uav_tt : uav_t) {
			t_left_l += T - uav_tt.t_used;
		}
	}
	float rnd_pro = (float)rand() / RAND_MAX;
	if (rnd_pro < 1) {//rnd_pro < 0.66 && rnd_pro >= 0.33
		vector<Solution> os_ls_list;
		int success_flag = 0;
		for (int k = 0; k < ls_it; k++) {
			auto sol_tmp = os;
			sol_tmp.ls_path_intersect();
			//sol_tmp.cal_uav_path_t();
			////Solution sol_test = sol_tmp;
			////sol_tmp.clear_ext_path_node(1);
			//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
			if (!sol_tmp.need_repair()) {
				//cout << "s4" << endl;
				//sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
				float t_left_sub = 0;
				for (auto uav_t : sol_tmp.UAV_list) {
					for (auto uav_tt : uav_t) {
						t_left_sub += T - uav_tt.t_used;
					}
				}
				if (t_left_sub > t_left_l) {
					t_left_l = t_left_sub;
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					//break;
				}
			}
			//if (!sol_tmp.need_repair()) {
			//	//sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
			//	//sol_tmp.cal_fitness();
			//	//os_ls_list.push_back(sol_tmp);
			//	//success_flag = 1;
			//	sol_tmp.ls_node_exchange(wn_list, unchange_counter);
			//	if (!sol_tmp.need_repair()) {
			//		sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
			//		sol_tmp.cal_fitness();
			//		os_ls_list.push_back(sol_tmp);
			//		success_flag = 1;
			//	}
			//}
		}
		//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		if (success_flag == 1) {
			os = os_ls_list[size(os_ls_list) - 1];
		}
	}
	if (rnd_pro < 1&&rnd_pro>0.25) {//rnd_pro < 0.66 && rnd_pro >= 0.33
		vector<Solution> os_ls_list;
		int success_flag = 0;
		for (int k = 0; k < ls_it; k++) {
			auto sol_tmp = os;
			sol_tmp.ls_node_del();
			//sol_tmp.cal_uav_path_t();
			////Solution sol_test = sol_tmp;
			////sol_tmp.clear_ext_path_node(1);
			//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
			if (!sol_tmp.need_repair()) {
				sol_tmp.cal_fitness();
				os_ls_list.push_back(sol_tmp);
				success_flag = 1;
				break;
			}
		}
		//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		if (success_flag == 1) {
			os = os_ls_list[size(os_ls_list) - 1];
		}
	}
	if (rnd_pro < 1 && rnd_pro>0.5) {//rnd_pro < 0.66 && rnd_pro >= 0.33
		vector<Solution> os_ls_list;
		int success_flag = 0;
		for (int k = 0; k < ls_it; k++) {
			auto sol_tmp = os;
			sol_tmp.mutate(wn_list,0,0);
			//sol_tmp.cal_uav_path_t();
			////Solution sol_test = sol_tmp;
			////sol_tmp.clear_ext_path_node(1);
			//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
			if (!sol_tmp.need_repair()) {
				sol_tmp.cal_fitness();
				os_ls_list.push_back(sol_tmp);
				success_flag = 1;
				break;
			}
		}
		//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		if (success_flag == 1) {
			os = os_ls_list[size(os_ls_list) - 1];
		}
	}
	if (rnd_pro >= 0) {//rnd_pro < 0.66 && rnd_pro >= 0.33
		vector<Solution> os_ls_list;
		int success_flag = 0;
		for (int k = 0; k < ls_it; k++) {
			auto sol_tmp = os;
			sol_tmp.ls_node_exchange_m(wn_list,0);
			//sol_tmp.cal_uav_path_t();
			////Solution sol_test = sol_tmp;
			////sol_tmp.clear_ext_path_node(1);
			//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
			if (!sol_tmp.need_repair()) {
				sol_tmp.cal_fitness();
				os_ls_list.push_back(sol_tmp);
				success_flag = 1;
				break;
			}
		}
		//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		if (success_flag == 1) {
			os = os_ls_list[size(os_ls_list) - 1];
		}
	}
	os.insert_rnd_nodes(wn_list,0);
	os.need_repair();
	os.cal_fitness();
	if (os.fitness > ini_sol_1.fitness) {
		ini_sol_1 = os;
	}
	if (unchange_counter > MAX_C) {
		unchange_counter = 0;
		ini_sol_1.repair_ig(0);
		ini_sol_1.cal_fitness();
		ini_sol_1.need_repair();
	}
	//auto ini_sol = ini_sol_1;
	////ini_sol.mutate(wn_list, flag_mode, col_num_choosed_pro_list, unchange_counter);
	//ini_sol.repair_ig(flag_mode);
	//ini_sol.clear_ext_path_node(1);
	//ini_sol.insert_rnd_nodes(wn_list, unchange_counter);
	//ini_sol.cal_fitness();
	//float rnd_pro = (float)rand() / RAND_MAX;
	//if (rnd_pro < 0.2) {
	//	vector<Solution> os_ls_list;
	//	os_ls_list.push_back(ini_sol);
	//	int success_flag = 0;
	//	for (int k = 0; k < 30; k++) {
	//		auto sol_tmp = ini_sol;
	//		sol_tmp.local_search(1);
	//		if (!sol_tmp.need_repair()) {
	//			sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
	//			sol_tmp.cal_fitness();
	//			os_ls_list.push_back(sol_tmp);
	//			success_flag = 1;
	//		}
	//	}
	//	sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
	//	if (success_flag == 1) {
	//		ini_sol = os_ls_list[0];
	//	}
	//	else {
	//		ini_sol.insert_rnd_nodes(wn_list,  unchange_counter);//插入
	//		ini_sol.cal_fitness();
	//	}
	//}
	//if (rnd_pro < 0.5 && rnd_pro >= 0.2) {
	//	vector<Solution> os_ls_list;
	//	os_ls_list.push_back(ini_sol);
	//	int success_flag = 0;
	//	for (int k = 0; k < 30; k++) {
	//		auto sol_tmp = ini_sol;
	//		sol_tmp.ls_node_exchange(wn_list, unchange_counter);
	//		if (!sol_tmp.need_repair()) {
	//			//cout << "ls_3" << endl;
	//			sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
	//			sol_tmp.cal_fitness();
	//			os_ls_list.push_back(sol_tmp);
	//			success_flag = 1;
	//		}
	//	}
	//	sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
	//	if (success_flag == 1) {
	//		ini_sol = os_ls_list[0];
	//	}
	//	else {
	//		ini_sol.insert_rnd_nodes(wn_list, unchange_counter);//插入
	//		ini_sol.cal_fitness();
	//	}
	//}
	//if (rnd_pro >= 0.5 && rnd_pro < 0.7) {
	//	vector<Solution> os_ls_list;
	//	os_ls_list.push_back(ini_sol);
	//	int success_flag = 0;
	//	for (int k = 0; k < 30; k++) {
	//		auto sol_tmp = ini_sol;
	//		sol_tmp.ls_mutate(wn_list, flag_mode, col_num_choosed_pro_list, unchange_counter);
	//		if (!sol_tmp.need_repair()) {
	//			sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
	//			sol_tmp.cal_fitness();
	//			os_ls_list.push_back(sol_tmp);
	//			success_flag = 1;
	//		}
	//	}
	//	sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
	//	if (success_flag == 1) {
	//		ini_sol = os_ls_list[0];
	//	}
	//	else {
	//		ini_sol.insert_rnd_nodes(wn_list,  unchange_counter);//插入
	//		ini_sol.cal_fitness();
	//	}
	//}
	//if (rnd_pro >= 0.7) {
	//	vector<Solution> os_ls_list;
	//	os_ls_list.push_back(ini_sol);
	//	int success_flag = 0;
	//	for (int k = 0; k < 30; k++) {
	//		auto sol_tmp = ini_sol;
	//		sol_tmp.ls_path_intersect();
	//		if (!sol_tmp.need_repair()) {
	//			sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
	//			sol_tmp.cal_fitness();
	//			os_ls_list.push_back(sol_tmp);
	//			success_flag = 1;
	//		}
	//	}
	//	sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
	//	if (success_flag == 1) {
	//		ini_sol = os_ls_list[0];
	//	}
	//	else {
	//		ini_sol.insert_rnd_nodes(wn_list, unchange_counter);//插入
	//		ini_sol.cal_fitness();
	//	}
	//}
	//ini_sol.clear_ext_path_node(1);
	//for (int i = 0; i < size(wn_list); i++) {
	//	for (int j = 0; j < wn_list[i].col_UAV_num; j++) {
	//		col_num_choosed_pro_list[i][j] *= 0.92;
	//	}
	//}
	//if (ini_sol.fitness > ini_sol_1.fitness) {
	//	ini_sol_1 = ini_sol;
	//	for (int i = 0; i < size(ini_sol_1.node_list); i++) {
	//		col_num_choosed_pro_list[ini_sol_1.node_list[i].id][size(ini_sol_1.node_list[i].UAV_id_list) - 1] += ini_sol_1.node_list[i].node_size / ini_sol_1.fitness / 10;
	//	}
	//}
	//else {
	//	float pro_rnd = (float)rand() / RAND_MAX; 
	//	float accept_c = (float)(ini_sol_1.fitness - ini_sol.fitness);
	//	if (pro_rnd < exp(-accept_c/10000)) {
	//		ini_sol_1 = ini_sol;
	//		for (int i = 0; i < size(ini_sol_1.node_list); i++) {
	//			col_num_choosed_pro_list[ini_sol_1.node_list[i].id][size(ini_sol_1.node_list[i].UAV_id_list) - 1] += ini_sol_1.node_list[i].node_size / ini_sol_1.fitness / 10;
	//		}
	//	}
	//}
}

void update_sol_abc(std::vector<Solution>& sol_list, vector<Working_node>& wn_list,int unchange_counter) {
	//employed
	int i = -1;
	for (auto os : sol_list) {
		i++;
		float t_left_l = 0;
		for (auto uav_t : os.UAV_list) {
			for (auto uav_tt : uav_t) {
				t_left_l += T - uav_tt.t_used;
			}
		}
		float rnd_pro = (float)rand() / RAND_MAX;
		if (rnd_pro < 0.25) {//rnd_pro < 0.66 && rnd_pro >= 0.33
			vector<Solution> os_ls_list;
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = os;
				sol_tmp.ls_path_intersect();
				//sol_tmp.cal_uav_path_t();
				////Solution sol_test = sol_tmp;
				////sol_tmp.clear_ext_path_node(1);
				//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
				if (!sol_tmp.need_repair()) {
					//cout << "s4" << endl;
					//sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
					float t_left_sub = 0;
					for (auto uav_t : sol_tmp.UAV_list) {
						for (auto uav_tt : uav_t) {
							t_left_sub += T - uav_tt.t_used;
						}
					}
					if (t_left_sub > t_left_l) {
						t_left_l = t_left_sub;
						sol_tmp.cal_fitness();
						os_ls_list.push_back(sol_tmp);
						success_flag = 1;
						break;
					}
				}
			}
			//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				os = os_ls_list[size(os_ls_list) - 1];
			}
		}
		if (rnd_pro < 0.5 && rnd_pro>0.25) {//rnd_pro < 0.66 && rnd_pro >= 0.33
			vector<Solution> os_ls_list;
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = os;
				sol_tmp.ls_node_del();
				//sol_tmp.cal_uav_path_t();
				////Solution sol_test = sol_tmp;
				////sol_tmp.clear_ext_path_node(1);
				//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
				if (!sol_tmp.need_repair()) {
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				os = os_ls_list[size(os_ls_list) - 1];
			}
		}
		if (rnd_pro < 0.75 && rnd_pro>0.5) {//rnd_pro < 0.66 && rnd_pro >= 0.33
			vector<Solution> os_ls_list;
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = os;
				sol_tmp.mutate(wn_list, 0, 0);
				if (!sol_tmp.need_repair()) {
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			if (success_flag == 1) {
				os = os_ls_list[size(os_ls_list) - 1];
			}
		}
		if (rnd_pro >= 0.75) {//rnd_pro < 0.66 && rnd_pro >= 0.33
			vector<Solution> os_ls_list;
			int success_flag = 0;
			for (int k = 0; k < ls_it; k++) {
				auto sol_tmp = os;
				sol_tmp.ls_node_exchange_m(wn_list, 0);
				//sol_tmp.cal_uav_path_t();
				////Solution sol_test = sol_tmp;
				////sol_tmp.clear_ext_path_node(1);
				//sol_tmp.ls_node_exchange(wn_list, unchange_counter);
				if (!sol_tmp.need_repair()) {
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			//sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				os = os_ls_list[size(os_ls_list) - 1];
			}
		}
		os.insert_rnd_nodes(wn_list, 0);
		os.need_repair();
		os.cal_fitness();
		if (os.fitness > sol_list[i].fitness) {
			sol_list[i] = os;
		}
	}

	//onlooker
	vector<float> fit_vec;
	float sum_fit = 0;
	for (auto sol_i : sol_list) {
		fit_vec.push_back(sol_i.fitness);
		sum_fit += sol_i.fitness;
	}
	for (int fi_i = 0; fi_i < size(fit_vec); fi_i++) {
		fit_vec[fi_i] /= sum_fit;
	}
	float pro_f = (float)rand() / RAND_MAX;
	float acc_f = 0;
	int flag_i = 0;
	int ch_no = 0;
	for (int fi_i = 0; fi_i < size(fit_vec); fi_i++) {
		acc_f += fit_vec[fi_i];
		if (acc_f > pro_f) {
			flag_i = 1;
			ch_no = fi_i;
			break;
		}
	}
	if (flag_i == 0) {
		ch_no = size(fit_vec) - 1;
	}
	auto node_ch = sol_list[ch_no];
	float f_o = node_ch.fitness;
	while(true){
		node_ch.ls_node_del();
		node_ch.insert_rnd_nodes(wn_list,0);
		node_ch.cal_fitness();
		if (f_o < node_ch.fitness) {
			node_ch.need_repair();
			sol_list[ch_no] = node_ch;
			f_o = node_ch.fitness;

		}
		else {
			break;
		}
	}
	//scout
	Solution sol_s = parent_gen(sol_list, 2, 0)[0];
	sol_s.mutate(wn_list,0,0);
	sol_s.repair_ig(0);
	sol_s.cal_fitness();
	sol_s.need_repair();
	sol_list.push_back(sol_s);
	sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
	sol_list.pop_back();
};

void update_sol_hs(vector<Solution>& sol_list, vector<Working_node>& wn_list, int unchange_counter) {
	float HMCR = PC;
	float PAR = eps;
	int min_size = INT_MAX;
	auto ini_sol = sol_list[0];
	ini_sol.clear();
	for (int i = 0; i < size(sol_list); i++) {
		min_size = min((int)size(sol_list[i].node_list), min_size);
	}
	for (int k = 0; k < min_size; k++) {
		float rnd_pro = (float)rand() / RAND_MAX;
		if (rnd_pro < HMCR) {
			float rnd_pro_par = (float)rand() / RAND_MAX;
			if (rnd_pro_par < PAR) {
				Working_node node_tmp_1;
				int count_e = 0;
				while (true) {
					int rnd_no = rand() % size(sol_list);
					node_tmp_1= sol_list[rnd_no].node_list[k];
					if (!ini_sol.check_node_id(node_tmp_1.id)) {
						break;
					}
					else {
						count_e++;
					}
					if (count_e > 10) {
						break;
					}
				}
				if (count_e <= 10) {
					ini_sol.node_list.push_back(node_tmp_1);
					for (int idx : node_tmp_1.UAV_id_list) {
						ini_sol.id_to_UAV(idx).Path.push_back(node_tmp_1);
					}
				}
				if (count_e > 10) {
					int rnd_no;
					while (true) {
						rnd_no = rand() % size(wn_list);
						if (!ini_sol.check_node_id(wn_list[rnd_no].id)) {
							break;
						}
					}
					Working_node node_tmp = wn_list[rnd_no];
					set<int> type = node_tmp.type;
					vector<int> uav_id_list;
					auto iter = type.begin();
					for (int i = 0; i < size(type); i++) {
						int type_i = *iter;
						for (auto uav_i : ini_sol.UAV_list[type_i]) {
							uav_id_list.push_back(uav_i.id);
						}
						iter++;
					}
					int final_col_num = min(node_tmp.col_UAV_num, (int)size(uav_id_list));
					vector<int> uav_col_list;
					set<int> choosed_no;
					while (size(uav_col_list) < final_col_num) {
						int rnd_id_no = rand() % size(uav_id_list);
						if (choosed_no.find(rnd_id_no) == choosed_no.end()) {
							choosed_no.insert(rnd_id_no);
							uav_col_list.push_back(uav_id_list[rnd_id_no]);
						}
					}
					node_tmp.UAV_id_list = uav_col_list;
					ini_sol.node_list.push_back(node_tmp);
					for (int idx : node_tmp.UAV_id_list) {
						ini_sol.id_to_UAV(idx).Path.push_back(node_tmp);
					}
				}
			}
			else {
				Working_node node_from_best = sol_list[0].node_list[k];
				if (!ini_sol.check_node_id(node_from_best.id)) {
					ini_sol.node_list.push_back(node_from_best);
					for (int idx : node_from_best.UAV_id_list) {
						ini_sol.id_to_UAV(idx).Path.push_back(node_from_best);
					}
				}
				else {
					int rnd_no;
					while (true) {
						rnd_no = rand() % size(wn_list);
						if (!ini_sol.check_node_id(wn_list[rnd_no].id)) {
							break;
						}
					}
					Working_node node_tmp = wn_list[rnd_no];
					set<int> type = node_tmp.type;
					vector<int> uav_id_list;
					auto iter = type.begin();
					for (int i = 0; i < size(type); i++) {
						int type_i = *iter;
						for (auto uav_i : ini_sol.UAV_list[type_i]) {
							uav_id_list.push_back(uav_i.id);
						}
						iter++;
					}
					int final_col_num = min(node_tmp.col_UAV_num, (int)size(uav_id_list));
					vector<int> uav_col_list;
					set<int> choosed_no;
					while (size(uav_col_list) < final_col_num) {
						int rnd_id_no = rand() % size(uav_id_list);
						if (choosed_no.find(rnd_id_no) == choosed_no.end()) {
							choosed_no.insert(rnd_id_no);
							uav_col_list.push_back(uav_id_list[rnd_id_no]);
						}
					}
					node_tmp.UAV_id_list = uav_col_list;
					ini_sol.node_list.push_back(node_tmp);
					for (int idx : node_tmp.UAV_id_list) {
						ini_sol.id_to_UAV(idx).Path.push_back(node_tmp);
					}
				}
			}
		}
		else {
			int rnd_no;
			while (true) {
				rnd_no = rand() % size(wn_list);
				if (!ini_sol.check_node_id(wn_list[rnd_no].id)) {
					break;
				}
			}
			Working_node node_tmp = wn_list[rnd_no];
			set<int> type = node_tmp.type;
			vector<int> uav_id_list;
			auto iter = type.begin();
			for (int i = 0; i < size(type); i++) {
				int type_i = *iter;
				for (auto uav_i : ini_sol.UAV_list[type_i]) {
					uav_id_list.push_back(uav_i.id);
				}
				iter++;
			}
			int final_col_num = min(node_tmp.col_UAV_num, (int)size(uav_id_list));
			vector<int> uav_col_list;
			set<int> choosed_no;
			while (size(uav_col_list) < final_col_num) {
				int rnd_id_no = rand() % size(uav_id_list);
				if (choosed_no.find(rnd_id_no) == choosed_no.end()) {
					choosed_no.insert(rnd_id_no);
					uav_col_list.push_back(uav_id_list[rnd_id_no]);
				}
			}
			node_tmp.UAV_id_list = uav_col_list;
			ini_sol.node_list.push_back(node_tmp);
			for (int idx : node_tmp.UAV_id_list) {
				ini_sol.id_to_UAV(idx).Path.push_back(node_tmp);
			}
		}
	}
	ini_sol.insert_rnd_nodes(wn_list,0);
	ini_sol.mutate(wn_list,0,0);
	ini_sol.repair_ig(0);
	ini_sol.clear_ext_path_node(1);
	ini_sol.cal_fitness();
	float rnd_pro = (float)rand() / RAND_MAX;
	if (rnd_pro < 0.3) {
		vector<Solution> os_ls_list;
		os_ls_list.push_back(ini_sol);
		int success_flag = 0;
		for (int k = 0; k < 30; k++) {
			auto sol_tmp = ini_sol;
			sol_tmp.ls_path_intersect();
			if (!sol_tmp.need_repair()) {
				sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
				sol_tmp.cal_fitness();
				os_ls_list.push_back(sol_tmp);
				success_flag = 1;
				break;
			}
		}
		sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		if (success_flag == 1) {
			ini_sol = os_ls_list[0];
		}
		else {
			ini_sol.insert_rnd_nodes(wn_list, unchange_counter);//插入
			ini_sol.cal_fitness();
		}
	}

	if (rnd_pro < 0.7 && rnd_pro >= 0.3) {
		vector<Solution> os_ls_list;
		os_ls_list.push_back(ini_sol);
		int success_flag = 0;
		for (int k = 0; k < 30; k++) {
			auto sol_tmp = ini_sol;
			sol_tmp.ls_node_exchange_m(wn_list,unchange_counter);
			if (!sol_tmp.need_repair()) {
				//cout << "ls_3" << endl;
				sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
				sol_tmp.cal_fitness();
				os_ls_list.push_back(sol_tmp);
				success_flag = 1;
				break;
			}
		}
		sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		if (success_flag == 1) {
			ini_sol = os_ls_list[0];
		}
		else {
			ini_sol.insert_rnd_nodes(wn_list,unchange_counter);//插入
			ini_sol.cal_fitness();
		}
	}

	if (rnd_pro >= 0.7) {
		vector<Solution> os_ls_list;
		os_ls_list.push_back(ini_sol);
		int success_flag = 0;
		for (int k = 0; k < 30; k++) {
			auto sol_tmp = ini_sol;
			sol_tmp.ls_node_del();
			if (!sol_tmp.need_repair()) {
				sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
				sol_tmp.cal_fitness();
				os_ls_list.push_back(sol_tmp);
				success_flag = 1;
				break;
			}
		}
		sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
		if (success_flag == 1) {
			ini_sol = os_ls_list[0];
		}
		else {
			ini_sol.insert_rnd_nodes(wn_list,  unchange_counter);//插入
			ini_sol.cal_fitness();
		}
	}
	ini_sol.clear_ext_path_node(1);
	ini_sol.insert_rnd_nodes(wn_list,0);
	ini_sol.need_repair();
	if (ini_sol.fitness > sol_list[size(sol_list)-1].fitness) {
		sol_list[size(sol_list)-1] = ini_sol;
	}
	else {
		float pro_rnd = (float)rand() / RAND_MAX;
		float accept_c = (float)(sol_list[size(sol_list) - 1].fitness - ini_sol.fitness);
		if (pro_rnd < exp(-accept_c / 1000)) {
			sol_list[size(sol_list) - 1] = ini_sol;
		}
	}
}

void update_sol_gwo(vector<Solution>& sol_list, vector<Working_node>& wn_list,int unchange_counter) {
	vector<Solution> sol_tmp_list;
	for (int i = 0; i < size(sol_list); i++) {
		vector<Solution> L_1;
		L_1.push_back(sol_list[0]);
		L_1.push_back(sol_list[i]);
		vector<Solution> L_1_os = cross_over(L_1, 0,unchange_counter);
		vector<Solution> L_2;
		L_2.push_back(sol_list[1]);
		L_2.push_back(L_1[1]);
		vector<Solution> L_2_ors = cross_over(L_2, 0, unchange_counter);
		vector<Solution> L_3;
		L_3.push_back(sol_list[2]);
		L_3.push_back(L_2[1]);
		vector<Solution> L_3_ors = cross_over(L_3, 0, unchange_counter);
		Solution final_sol = L_3_ors[1];
		final_sol.mutate(wn_list, unchange_counter,0);
		final_sol.repair_ig(0);
		final_sol.clear_ext_path_node(1);
		final_sol.insert_rnd_nodes(wn_list,  unchange_counter);
		final_sol.cal_fitness();
		float rnd_pro = (float)rand() / RAND_MAX;
		if (rnd_pro < 0.2) {
			vector<Solution> os_ls_list;
			os_ls_list.push_back(final_sol);
			int success_flag = 0;
			for (int k = 0; k < 30; k++) {
				auto sol_tmp = final_sol;
				sol_tmp.local_search(1);
				if (!sol_tmp.need_repair()) {
					sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				final_sol = os_ls_list[0];
			}
			else {
				final_sol.insert_rnd_nodes(wn_list,  unchange_counter);//插入
				final_sol.cal_fitness();
			}
		}

		if (rnd_pro < 0.5 && rnd_pro >= 0.2) {
			vector<Solution> os_ls_list;
			os_ls_list.push_back(final_sol);
			int success_flag = 0;
			for (int k = 0; k < 30; k++) {
				auto sol_tmp = final_sol;
				sol_tmp.ls_node_del();
				if (!sol_tmp.need_repair()) {
					//cout << "ls_3" << endl;
					sol_tmp.insert_rnd_nodes(wn_list, unchange_counter);
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				final_sol = os_ls_list[0];
			}
			else {
				final_sol.insert_rnd_nodes(wn_list,  unchange_counter);//插入
				final_sol.cal_fitness();
			}
		}

		if (rnd_pro >= 0.5 && rnd_pro < 0.7) {
			vector<Solution> os_ls_list;
			os_ls_list.push_back(final_sol);
			int success_flag = 0;
			for (int k = 0; k < 30; k++) {
				auto sol_tmp = final_sol;
				sol_tmp.ls_node_exchange_m(wn_list,0);
				if (!sol_tmp.need_repair()) {
					sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				final_sol = os_ls_list[0];
			}
			else {
				final_sol.insert_rnd_nodes(wn_list, unchange_counter);//插入
				final_sol.cal_fitness();
			}
		}

		if (rnd_pro >= 0.7) {
			vector<Solution> os_ls_list;
			os_ls_list.push_back(final_sol);
			int success_flag = 0;
			for (int k = 0; k < 30; k++) {
				auto sol_tmp = final_sol;
				sol_tmp.mutate(wn_list, unchange_counter, 0);
				if (!sol_tmp.need_repair()) {
					sol_tmp.insert_rnd_nodes(wn_list,  unchange_counter);
					sol_tmp.cal_fitness();
					os_ls_list.push_back(sol_tmp);
					success_flag = 1;
					break;
				}
			}
			sort(os_ls_list.begin(), os_ls_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			if (success_flag == 1) {
				final_sol = os_ls_list[0];
			}
			else {
				final_sol.insert_rnd_nodes(wn_list,  unchange_counter);//插入
				final_sol.cal_fitness();
			}
		}
		final_sol.insert_rnd_nodes(wn_list,0);
		final_sol.clear_ext_path_node(1);
		final_sol.need_repair();
		sol_tmp_list.push_back(final_sol);
	}
	for (int i = 0; i < size(sol_tmp_list); i++) {
		if (sol_list[i].fitness < sol_tmp_list[i].fitness) {
			sol_list[i] = sol_tmp_list[i];
		}
	}
}

//随机顺序的交叉
//vector<Solution> cross_over_r(vector<Solution> ps, int unchange_counter) {
//	vector<vector<Working_node>> c12;
//	//生成父代各自的任务点子序列
//	for (auto& sol : ps) {
//		c12.push_back(rnd_gen_sub_wn_list(sol.node_list,1, unchange_counter));
//	}
//	vector<Solution> urs;
//	//开始交叉进程
//	for (int i = 0; i < 2; i++) {
//		auto c_choosed = c12[i];
//		int ps_no;
//		//确定生成子代的父代序号
//		((i + 1) < 2) ? ps_no = 1 : ps_no = 0;
//		int pre_p = -1;
//		for (auto& node : c_choosed) {
//			if (ps[ps_no].check_node_id(node.id)) {
//				auto node_copy = ps[ps_no].id_to_node(node.id);
//				int i = 0;
//				for (; i < size(ps[ps_no].node_list); i++) {
//					if (ps[ps_no].node_list[i].id == node.id) {
//						break;
//					}
//				}
//				//uavs erase the node in path
//				for (int idx : ps[ps_no].node_list[i].UAV_id_list) {
//					auto& uav_e = ps[ps_no].id_to_UAV(idx);
//					uav_e.erase_node_i_in_path(node.id);
//				}
//				//erase the node in the solution
//				ps[ps_no].node_list.erase(ps[ps_no].node_list.begin() + i);
//				//insert
//				//ps[ps_no].node_list.insert(ps[ps_no].node_list.begin() + rnd_insert_p,node);
//			}
//		}
//		for (auto& node : c_choosed) {
//			int rnd_insert_p = rand() % (size(ps[ps_no].node_list)+1);
//			if (ps[ps_no].check_node_id(node.id)) {
//				auto node_copy = ps[ps_no].id_to_node(node.id);
//				auto& inserted_node = ps[ps_no].node_list[rnd_insert_p];
//				set<int> uav_can_id_set;
//				for (auto idx : node_copy.UAV_id_list) {
//					uav_can_id_set.insert(idx);
//				}
//				for (auto idx : inserted_node.UAV_id_list) {
//					uav_can_id_set.insert(idx);
//				}
//				int max_col_num = max(node_copy.col_UAV_num, int(size(uav_can_id_set)));
//				int rnd_col_num = rand() % (max_col_num + 1);
//				vector<int> choosed_uav_id_set;
//				set<int> choosed_no_set;
//				while (size(choosed_no_set) < rnd_col_num) {
//					int rnd_no = rand() % (size(uav_can_id_set));
//					if (choosed_no_set.find(rnd_no) == choosed_no_set.end()) {
//						auto iter = uav_can_id_set.begin();
//						for (; iter != uav_can_id_set.end(); iter++) {
//							break;
//						}
//						choosed_uav_id_set.push_back(*iter);
//						choosed_no_set.insert(rnd_no);
//					}
//				}
//				inserted_node.UAV_id_list = choosed_uav_id_set;
//				for (auto idx : inserted_node.UAV_id_list) {
//					UAV& uav_1 = ps[ps_no].id_to_UAV(idx);
//					if (size(uav_1.Path) == 0) {
//						uav_1.Path.push_back(inserted_node);
//						continue;
//					}
//					int p_i = 0;
//					int fl = 0;
//					for (; p_i < (size(uav_1.Path) - 1); p_i++) {
//						int node_id_in_Path = uav_1.Path[p_i].id;
//						int node_id_next_in_Path = uav_1.Path[p_i + 1].id;
//						Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
//						Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
//						if (ps[ps_no].compare_node(node_tmp, inserted_node) && (ps[ps_no].compare_node(inserted_node, node_tmp_next))) {
//							fl = 1;
//							break;
//						}
//					}
//					if (fl == 1) {
//						uav_1.Path.insert(uav_1.Path.begin() + p_i + 1, inserted_node);
//					}
//					else {
//						int idx_first = uav_1.Path[0].id;
//						int idx_last = uav_1.Path[size(uav_1.Path) - 1].id;
//						Working_node& node_first = ps[ps_no].id_to_node(idx_first);
//						Working_node& node_last = ps[ps_no].id_to_node(idx_last);
//						if (ps[ps_no].compare_node(inserted_node, node_first)) {
//							uav_1.Path.insert(uav_1.Path.begin(), (inserted_node));
//							continue;
//						}
//						if (ps[ps_no].compare_node(node_last, inserted_node)) {
//							uav_1.Path.insert(uav_1.Path.end(), (inserted_node));
//						}
//					}
//				}
//			}
//			else {
//				int wn_list_size = size(ps[ps_no].node_list);
//				vector<Working_node>& tmp_wn_list = ps[ps_no].node_list;
//				int rnd_insert_idx = rand() % (wn_list_size + 1);
//				tmp_wn_list.insert(tmp_wn_list.begin() + rnd_insert_idx, node);
//				ps[ps_no].assign_node_no();
//				Working_node& node_to_insert = ps[ps_no].node_list[rnd_insert_idx];
//				if (size(node.UAV_id_list) == 1) {
//					UAV& uav_in = ps[ps_no].id_to_UAV(node.UAV_id_list[0]);
//					if (size(uav_in.Path) == 0) {
//						uav_in.Path.push_back(node_to_insert);
//						continue;
//					}
//					int p_i = 0;
//					int fl = 0;
//					for (; p_i < (size(uav_in.Path) - 1); p_i++) {
//						int node_id_in_Path = uav_in.Path[p_i].id;
//						int node_id_next_in_Path = uav_in.Path[p_i + 1].id;
//						Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
//						Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
//						if (ps[ps_no].compare_node(node_tmp, node_to_insert) && (ps[ps_no].compare_node(node_to_insert, node_tmp_next))) {
//							fl = 1;
//							break;
//						}
//					}
//					if (fl == 1) {
//						uav_in.Path.insert(uav_in.Path.begin() + p_i + 1, node_to_insert);
//					}
//					else {
//						int idx_first = uav_in.Path[0].id;
//						int idx_last = uav_in.Path[size(uav_in.Path) - 1].id;
//						Working_node& node_first = ps[ps_no].id_to_node(idx_first);
//						Working_node& node_last = ps[ps_no].id_to_node(idx_last);
//						if (ps[ps_no].compare_node(node_to_insert, node_first)) {
//							uav_in.Path.insert(uav_in.Path.begin(), (node_to_insert));
//							continue;
//						}
//						if (ps[ps_no].compare_node(node_last, node_to_insert)) {
//							uav_in.Path.insert(uav_in.Path.end(), (node_to_insert));
//							continue;
//						}
//					}
//				}
//				else {
//					int rnd_col_num_1 = 1 + rand() % size(node_to_insert.UAV_id_list);
//					vector<int> choosed_UAV_id_list_1;
//					set<int> choosed_UAV_id_set_1;
//					while (size(choosed_UAV_id_set_1) < rnd_col_num_1) {
//						int rnd_uav_no_1 = rand() % size(node_to_insert.UAV_id_list);
//						if (choosed_UAV_id_set_1.find(rnd_uav_no_1) == choosed_UAV_id_set_1.end()) {
//							choosed_UAV_id_set_1.insert(rnd_uav_no_1);
//							choosed_UAV_id_list_1.push_back(node_to_insert.UAV_id_list[rnd_uav_no_1]);
//						}
//					}
//					node_to_insert.UAV_id_list = choosed_UAV_id_list_1;
//					for (auto idx : node_to_insert.UAV_id_list) {
//						UAV& uav_1 = ps[ps_no].id_to_UAV(idx);
//						if (size(uav_1.Path) == 0) {
//							uav_1.Path.push_back(node_to_insert);
//						}
//						int p_i = 0;
//						int fl = 0;
//						for (; p_i < (size(uav_1.Path) - 1); p_i++) {
//							int node_id_in_Path = uav_1.Path[p_i].id;
//							int node_id_next_in_Path = uav_1.Path[p_i + 1].id;
//							Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
//							Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
//							if (ps[ps_no].compare_node(node_tmp, node_to_insert) && (ps[ps_no].compare_node(node_to_insert, node_tmp_next))) {
//								fl = 1;
//								break;
//							}
//						}
//						if (fl == 1) {
//							uav_1.Path.insert(uav_1.Path.begin() + p_i + 1, node_to_insert);
//						}
//						else {
//							int idx_first = uav_1.Path[0].id;
//							int idx_last = uav_1.Path[size(uav_1.Path) - 1].id;
//							Working_node& node_first = ps[ps_no].id_to_node(idx_first);
//							Working_node& node_last = ps[ps_no].id_to_node(idx_last);
//							if (ps[ps_no].compare_node(node_to_insert, node_first)) {
//								uav_1.Path.insert(uav_1.Path.begin(), (node_to_insert));
//								continue;
//							}
//							if (ps[ps_no].compare_node(node_last, node_to_insert)) {
//								uav_1.Path.insert(uav_1.Path.end(), (node_to_insert));
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//	urs = ps;
//	return urs;
//}

//保留父代子序列顺序的交叉方式
vector<Solution> cross_over(vector<Solution> ps_ori, int flag_mode, int unchange_counter) {
	vector<vector<Working_node>> c12;
	//生成父代各自的任务点子序列
	auto ps = ps_ori;
	for (auto& sol : ps) {
		c12.push_back(rnd_gen_sub_wn_list(sol.node_list,flag_mode, unchange_counter));
	}
	vector<Solution> urs;
	//开始交叉进程
	for (int i = 0; i < 2; i++) {
		auto c_choosed = c12[i];
		int ps_no;
		//确定生成子代的父代序号
		((i + 1) < 2) ? ps_no = 1 : ps_no = 0;
		int pre_p = -1;
		for (auto& node : c_choosed) {
			if (ps[ps_no].check_node_id(node.id)) {
				auto node_copy = ps[ps_no].id_to_node(node.id);
				int i = 0;
				for (; i < size(ps[ps_no].node_list); i++) {
					if (ps[ps_no].node_list[i].id == node.id) {
						break;
					}
				}
				//uavs erase the node in path
				for (int idx : ps[ps_no].node_list[i].UAV_id_list) {
					auto& uav_e = ps[ps_no].id_to_UAV(idx);
					uav_e.erase_node_i_in_path(node.id);
				}
				//erase the node in the solution
				ps[ps_no].node_list.erase(ps[ps_no].node_list.begin() + i);
				//insert
				//ps[ps_no].node_list.insert(ps[ps_no].node_list.begin() + rnd_insert_p,node);
			}
		}
		for(auto& node :c_choosed){
			int rnd_insert_p = pre_p+1+ rand() % (size(ps[ps_no].node_list)-pre_p) ;//
            pre_p = rnd_insert_p;
			//cout << pre_p << " ";
			if (ps_ori[ps_no].check_node_id(node.id)) {
				auto node_copy = ps_ori[ps_no].id_to_node(node.id);
				ps[ps_no].node_list.insert(ps[ps_no].node_list.begin() + rnd_insert_p, node);
				ps[ps_no].assign_node_no();
				auto& inserted_node = ps[ps_no].node_list[rnd_insert_p];
				set<int> uav_can_id_set;
				for (auto idx : node_copy.UAV_id_list) {
					uav_can_id_set.insert(idx);
				}
				for (auto idx : inserted_node.UAV_id_list) {
					uav_can_id_set.insert(idx);
				}
				vector<UAV> UAV_can_list;
				for (int ix : uav_can_id_set) {
					UAV_can_list.push_back(ps[ps_no].id_to_UAV(ix));
				}
				float sum_sum = 0;
				int choosed_col_num = 0;
				for (int i = 0; i < inserted_node.col_UAV_num; i++){
					float col_i_arr=0;
					for (int j = 0; j < MAP_NODE_NUM; j++){
						//col_num_pro[i] = Q_table_1[node_to_exchange.id][rnd_no][i];
						col_i_arr += Q_table_1[inserted_node.id][j][i];
						//sum_sum += col_num_pro[i];
					}
					if (sum_sum < col_i_arr) {
						choosed_col_num = i + 1;
						sum_sum = col_i_arr;
					}
				}
				//choosed_col_num = rand() % inserted_node.col_UAV_num+1;
				int rnd_col_num = 1;
				float rnd_pro = (float)rand() / RAND_MAX;
				if (rnd_pro < eps || sum_sum == 0) {
					int max_col_num = min(node_copy.col_UAV_num, int(size(uav_can_id_set)));
					rnd_col_num = rand() % (max_col_num)+1;
				}
				else {
					choosed_col_num = ret_GN(choosed_col_num);
					if (choosed_col_num < 1) {
						choosed_col_num = 1;
					}
					rnd_col_num = min((int)size(uav_can_id_set), choosed_col_num);
				}
				vector<int> choosed_uav_id_set;
				if (rnd_col_num >= size(uav_can_id_set)) {
					for (auto iter_id = uav_can_id_set.begin(); iter_id != uav_can_id_set.end(); iter_id++) {
						choosed_uav_id_set.push_back(*iter_id);
					}
				}
				else {
					sort(UAV_can_list.begin(), UAV_can_list.end(), [](UAV& a, UAV& b) {return (T - a.t_used) > (T - b.t_used);});
					//set<int> choosed_no_set;
					//while (size(choosed_no_set) < rnd_col_num) {
					//	int rnd_no = rand() % (size(uav_can_id_set));
					//	if (choosed_no_set.find(rnd_no) == choosed_no_set.end()) {
					//		auto iter = uav_can_id_set.begin();
					//		int fl_ff = 0;
					//		for (; iter != uav_can_id_set.end(); iter++) {
					//			if (rnd_no == fl_ff) {
					//				break;
					//			}
					//			fl_ff++;
					//		}
					//		choosed_uav_id_set.push_back(*iter);
					//		choosed_no_set.insert(rnd_no);
					//	}
					//}
					for (int ch_i = 0; ch_i < rnd_col_num; ch_i++) {
						choosed_uav_id_set.push_back(UAV_can_list[ch_i].id);
					}
				}
				inserted_node.UAV_id_list = choosed_uav_id_set;
				for (auto idx : inserted_node.UAV_id_list) {
					UAV& uav_1 = ps[ps_no].id_to_UAV(idx);
					if (size(uav_1.Path) == 0) {
						uav_1.Path.push_back(inserted_node);
						continue;
					}
					int p_i = 0;
					int fl = 0;
					for (; p_i < (size(uav_1.Path) - 1); p_i++) {
						int node_id_in_Path = uav_1.Path[p_i].id;
						int node_id_next_in_Path = uav_1.Path[p_i + 1].id;
						Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
						Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
						if (ps[ps_no].compare_node(node_tmp, inserted_node) && (ps[ps_no].compare_node(inserted_node,node_tmp_next))) {
							fl = 1;
							break;
						}
					}
					if (fl == 1) {
						uav_1.Path.insert(uav_1.Path.begin() + p_i + 1, inserted_node);
					}
					else {
						int idx_first = uav_1.Path[0].id;
						int idx_last = uav_1.Path[size(uav_1.Path) - 1].id;
						Working_node& node_first = ps[ps_no].id_to_node(idx_first);
						Working_node& node_last = ps[ps_no].id_to_node(idx_last);
						if (ps[ps_no].compare_node(inserted_node, node_first)) {
							uav_1.Path.insert(uav_1.Path.begin(), (inserted_node));
							continue;
						}
						if (ps[ps_no].compare_node(node_last, inserted_node)) {
							uav_1.Path.insert(uav_1.Path.end(), (inserted_node));
						}
					}
				}
				/*ps[ps_no].node_list.insert(ps[ps_no].node_list.begin() + rnd_insert_p, node);
				ps[ps_no].assign_node_no();
				Working_node& inserted_node = ps[ps_no].node_list[rnd_insert_p];
				for (auto idx : inserted_node.UAV_id_list) {
					UAV& uav_1 = ps[ps_no].id_to_UAV(idx);
					if (size(uav_1.Path) == 0) {
						uav_1.Path.push_back(inserted_node);
						continue;
					}
					int p_i = 0;
					int fl = 0;
					for (; p_i < (size(uav_1.Path) - 1); p_i++) {
						int node_id_in_Path = uav_1.Path[p_i].id;
						int node_id_next_in_Path = uav_1.Path[p_i + 1].id;
						Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
						Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
						if (ps[ps_no].compare_node(node_tmp, inserted_node) && (ps[ps_no].compare_node(inserted_node, node_tmp_next))) {
							fl = 1;
							break;
						}
					}
					if (fl == 1) {
						uav_1.Path.insert(uav_1.Path.begin() + p_i + 1, inserted_node);
					}
					else {
						int idx_first = uav_1.Path[0].id;
						int idx_last = uav_1.Path[size(uav_1.Path) - 1].id;
						Working_node& node_first = ps[ps_no].id_to_node(idx_first);
						Working_node& node_last = ps[ps_no].id_to_node(idx_last);
						if (ps[ps_no].compare_node(inserted_node, node_first)) {
							uav_1.Path.insert(uav_1.Path.begin(), (inserted_node));
							continue;
						}
						if (ps[ps_no].compare_node(node_last, inserted_node)) {
							uav_1.Path.insert(uav_1.Path.end(), (inserted_node));
						}
					}
				}*/


				//


			}
			else {
				//int wn_list_size = size(ps[ps_no].node_list);
				//vector<Working_node>& tmp_wn_list = ps[ps_no].node_list;
				//int rnd_insert_idx = rand() % (wn_list_size + 1);
				//tmp_wn_list.insert(tmp_wn_list.begin() + rnd_insert_idx, node);
				ps[ps_no].node_list.insert(ps[ps_no].node_list.begin() + rnd_insert_p, node);
				ps[ps_no].assign_node_no();
				Working_node& inserted_node = ps[ps_no].node_list[rnd_insert_p];
				for (auto idx : inserted_node.UAV_id_list) {
					UAV& uav_1 = ps[ps_no].id_to_UAV(idx);
					if (size(uav_1.Path) == 0) {
						uav_1.Path.push_back(inserted_node);
						continue;
					}
					int p_i = 0;
					int fl = 0;
					for (; p_i < (size(uav_1.Path) - 1); p_i++) {
						int node_id_in_Path = uav_1.Path[p_i].id;
						int node_id_next_in_Path = uav_1.Path[p_i + 1].id;
						Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
						Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
						if (ps[ps_no].compare_node(node_tmp, inserted_node) && (ps[ps_no].compare_node(inserted_node, node_tmp_next))) {
							fl = 1;
							break;
						}
					}
					if (fl == 1) {
						uav_1.Path.insert(uav_1.Path.begin() + p_i + 1, inserted_node);
					}
					else {
						int idx_first = uav_1.Path[0].id;
						int idx_last = uav_1.Path[size(uav_1.Path) - 1].id;
						Working_node& node_first = ps[ps_no].id_to_node(idx_first);
						Working_node& node_last = ps[ps_no].id_to_node(idx_last);
						if (ps[ps_no].compare_node(inserted_node, node_first)) {
							uav_1.Path.insert(uav_1.Path.begin(), (inserted_node));
							continue;
						}
						if (ps[ps_no].compare_node(node_last, inserted_node)) {
							uav_1.Path.insert(uav_1.Path.end(), (inserted_node));
						}
					}
				}


				/*if (size(node.UAV_id_list) == 1) {
					UAV& uav_in = ps[ps_no].id_to_UAV(node.UAV_id_list[0]);
					if (size(uav_in.Path) == 0) {
						uav_in.Path.push_back(node_to_insert);
						continue;
					}
					int p_i = 0;
					int fl = 0;
					for (; p_i < (size(uav_in.Path) - 1); p_i++) {
						int node_id_in_Path = uav_in.Path[p_i].id;
						int node_id_next_in_Path = uav_in.Path[p_i + 1].id;
						Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
						Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
						if (ps[ps_no].compare_node(node_tmp, node_to_insert) && (ps[ps_no].compare_node(node_to_insert, node_tmp_next))) {
							fl = 1;
							break;
						}
					}
					if (fl == 1) {
						uav_in.Path.insert(uav_in.Path.begin() + p_i + 1, node_to_insert);
					}
					else {
						int idx_first = uav_in.Path[0].id;
						int idx_last = uav_in.Path[size(uav_in.Path) - 1].id;
						Working_node& node_first = ps[ps_no].id_to_node(idx_first);
						Working_node& node_last = ps[ps_no].id_to_node(idx_last);
						if (ps[ps_no].compare_node(node_to_insert, node_first)) {
							uav_in.Path.insert(uav_in.Path.begin(), (node_to_insert));
							continue;
						}
						if (ps[ps_no].compare_node(node_last, node_to_insert)) {
							uav_in.Path.insert(uav_in.Path.end(), (node_to_insert));
							continue;
						}
					}
				}
				else {
					int rnd_col_num_1 = 1 + rand() % size(node_to_insert.UAV_id_list);
					vector<int> choosed_UAV_id_list_1;
					set<int> choosed_UAV_id_set_1;
					while (size(choosed_UAV_id_set_1) < rnd_col_num_1) {
						int rnd_uav_no_1 = rand() % size(node_to_insert.UAV_id_list);
						if (choosed_UAV_id_set_1.find(rnd_uav_no_1) == choosed_UAV_id_set_1.end()) {
							choosed_UAV_id_set_1.insert(rnd_uav_no_1);
							choosed_UAV_id_list_1.push_back(node_to_insert.UAV_id_list[rnd_uav_no_1]);
						}
					}
					node_to_insert.UAV_id_list = choosed_UAV_id_list_1;
					for (auto idx : node_to_insert.UAV_id_list) {
						UAV& uav_1 = ps[ps_no].id_to_UAV(idx);
						if (size(uav_1.Path) == 0) {
							uav_1.Path.push_back(node_to_insert);
						}
						int p_i = 0;
						int fl = 0;
						for (; p_i < (size(uav_1.Path) - 1); p_i++) {
							int node_id_in_Path = uav_1.Path[p_i].id;
							int node_id_next_in_Path = uav_1.Path[p_i + 1].id;
							Working_node& node_tmp = ps[ps_no].id_to_node(node_id_in_Path);
							Working_node& node_tmp_next = ps[ps_no].id_to_node(node_id_next_in_Path);
							if (ps[ps_no].compare_node(node_tmp, node_to_insert) && (ps[ps_no].compare_node(node_to_insert, node_tmp_next))) {
								fl = 1;
								break;
							}
						}
						if (fl == 1) {
							uav_1.Path.insert(uav_1.Path.begin() + p_i + 1, node_to_insert);
						}
						else {
							int idx_first = uav_1.Path[0].id;
							int idx_last = uav_1.Path[size(uav_1.Path) - 1].id;
							Working_node& node_first = ps[ps_no].id_to_node(idx_first);
							Working_node& node_last = ps[ps_no].id_to_node(idx_last);
							if (ps[ps_no].compare_node(node_to_insert, node_first)) {
								uav_1.Path.insert(uav_1.Path.begin(), (node_to_insert));
								continue;
							}
							if (ps[ps_no].compare_node(node_last, node_to_insert)) {
								uav_1.Path.insert(uav_1.Path.end(), (node_to_insert));
							}
						}
					}
				}*/
			}
		}
		//cout << endl;
	}
	urs = ps;
	return urs;
}

vector<Solution> parent_gen(vector<Solution>& sol_list, int MAX_SUB, int flag_mode) {
	vector<Solution> ps;
	int list_size = size(sol_list);
	int se = 0;
	while(size(ps)<2) {
		vector<Solution> sol_list_sub;
		set<int> no_choosed;
		if (flag_mode == 0) {
			while (size(sol_list_sub)< MAX_SUB) {
				int rnd_no = rand() % list_size;
				if (no_choosed.find(rnd_no) == no_choosed.end()) {
					no_choosed.insert(rnd_no);
					//if (sol_list[rnd_no].fitness == 0) {
					//	continue;
					//}
					sol_list_sub.push_back(sol_list[rnd_no]);
				}
			}
			sort(sol_list_sub.begin(),sol_list_sub.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
			//if (size(ps) >= 1) {
			//	if (ps[0].sol_id != sol_list_sub[0].sol_id) {
			//		ps.push_back(sol_list_sub[0]);
			//		break;
			//	}
			//}
			//else {
			//	ps.push_back(sol_list_sub[0]);
			//}
			ps.push_back(sol_list_sub[0]);
		}
		if (flag_mode == 1) {
			if (se == 0) {
				while (size(no_choosed) < MAX_SUB) {
					int rnd_no = rand() % (list_size/3+1);
					if (no_choosed.find(rnd_no) == no_choosed.end()) {
						no_choosed.insert(rnd_no);
						sol_list_sub.push_back(sol_list[rnd_no]);
					}
				}
				sort(sol_list_sub.begin(), sol_list_sub.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
				ps.push_back(sol_list_sub[0]);
				se = 1;
			}
			if (se == 1) {
				while (size(no_choosed) < MAX_SUB) {
					int rnd_no =list_size/3 + 1 + rand() % (list_size*2/3);
					if (no_choosed.find(rnd_no) == no_choosed.end()) {
						no_choosed.insert(rnd_no);
						sol_list_sub.push_back(sol_list[rnd_no]);
					}
				}
				sort(sol_list_sub.begin(), sol_list_sub.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
				ps.push_back(sol_list_sub[0]);
			}
		}
	}		
	return ps;
}

vector<Working_node> rnd_gen_sub_wn_list(vector<Working_node>& wn_list,int flag_mode,int unchange_counter) {
	int list_size = size(wn_list);
	vector<Working_node> sub_wn_list;
	//float MAX_PRO=1/(2+((float)unchange_counter)/10);
	//float MAX_PRO = 0.5 - pow((float)iteration_time / IT,0.5) *0.5;
	float MAX_PRO = 0;
	if ((float)rand() / RAND_MAX < PC) {
		MAX_PRO = (float)(rand() %8+3)/10;
		//MAX_PRO = (float)rand() / RAND_MAX;
		//MAX_PRO = 0;
	}
	for (int i = 0; i < list_size; i++) {
		float rnd_num = (float)rand() / (float)RAND_MAX;
		if (rnd_num<=MAX_PRO)
		{
			sub_wn_list.push_back(wn_list[i]);
		}
		else {
			continue;
		}
	}
	return sub_wn_list;
}


int ret_GN(int N) {
	if (mode != 0) {
		return N;
	}
	double U1 = (float)rand() / (float)RAND_MAX;
	double U2 = (float)rand() / (float)RAND_MAX;
	double Z = sqrt(-2 * log(U1)) * cos(2 * pi * U2);
	double std = (float)1 / 2;
	return round(N + Z * std);
}