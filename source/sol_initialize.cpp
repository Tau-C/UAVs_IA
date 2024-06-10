#include <vector>
#include "UAV.h"
#include "initialization.h"
#include "Working_node.h"
#include "solution.h"
#include <set>
#include <algorithm>
#include<iostream>
extern float eps;
extern float T;
using namespace std;
#include "map_config.h"
extern int* min_node_col;
extern float dis_matric[MAP_NODE_NUM + 1][MAP_NODE_NUM];
extern int mode;
void ini_sol(std::vector<Solution>& sol_list, std::vector<Working_node>& node_list, std::vector<std::vector<UAV>>& UAV_list, int P_size) {
	if (mode == 1) {
		for (int i = 0; i < (float)(P_size); i++) {
			sol_list.push_back(random_generate_sol(node_list, UAV_list));//随机初始化
			(*(sol_list.end() - 1)).sol_id = i;
		}
	}
	else {
		for (int i = 0; i < (int)(P_size)/2; i++) {//
			sol_list.push_back(random_generate_sol(node_list, UAV_list));//随机初始化
			(*(sol_list.end() - 1)).sol_id = i;
		}
		//for (int i = 0; i < (int)(P_size)/3; i++) {//3
		//	sol_list.push_back(heuristic_generate_sol(node_list, UAV_list, 1));
		//	//sol_list.push_back(NEH_generate_sol(node_list, UAV_list, 1));
		//	(*(sol_list.end() - 1)).sol_id = (*(sol_list.end() - 2)).sol_id+1;
		//}
		for (int i = 0; i < (int)(P_size)-(int)(P_size)/2; i++){//(int i = 0; i < P_size-2*(int)P_size/3; i++)(int)(P_size)-(int)(P_size)-
			//P_size - 2 * (int)((float)(P_size) / 3)
			sol_list.push_back(NEH_generate_sol(node_list, UAV_list, 1));
			//sol_list[size(sol_list) - 1].need_repair();
			//sol_list[size(sol_list) - 1].clear_ext_path_node(1);
			//sol_list[size(sol_list) - 1].insert_rnd_nodes(node_list,0);
			//sol_list.push_back(NEH_generate_sol(node_list, UAV_list, 1));
			(*(sol_list.end() - 1)).sol_id = (*(sol_list.end() - 2)).sol_id + 1;
		}
	}
	//sol_list.push_back(heuristic_generate_sol(node_list, UAV_list,1));//贪婪，每次选最近路径点. 1:随机分配无人机个数; 0:按按最大协同无人机数分配
	//(*(sol_list.end() - 1)).sol_id = P_size - 1;
}

Solution random_generate_sol(vector<Working_node>& node_list, vector<vector<UAV>>& UAV_list) {
	int node_size = size(node_list);
	Solution sol_gen;
	sol_gen.UAV_list = UAV_list;
	vector<Working_node> sol_node_list;
	vector<vector<UAV>> sol_UAV_list=UAV_list;
	set<int> choosed_id_set;
	set<int> node_id;
	while ((size(choosed_id_set) < node_size)) {
		int rnd_node_id = rand() % node_size;
		if (choosed_id_set.find(rnd_node_id) == choosed_id_set.end()) {
			node_id.insert(rnd_node_id);
			Working_node node_choosed = node_list[rnd_node_id];
			vector<float> t_lefts;
			vector<vector<UAV>>& uav_list = sol_gen.UAV_list;
			t_lefts.resize(size(uav_list));
			float t_max = 0;
			for (int i = 0; i < size(uav_list); i++) {
				for (int j = 0; j < size(uav_list[i]); j++) {
					t_lefts[i] += T - uav_list[i][j].t_used;
				}
				t_max += t_lefts[i];
			}
			if (node_choosed.node_size / UAV::eff > t_max) {
				continue;
			}
			int rnd_col_n = (rand() % (node_choosed.col_UAV_num)) + 1;
			set<int>& type_allowed = node_choosed.type;
			vector<UAV> UAV_allowed_list;
			for (auto it = type_allowed.begin(); it != type_allowed.end(); it++) {
				int type_ch = *it;
				if (type_ch > int(size(UAV_list) - 1)) {
					break;
				}
				for (int j = 0; j < size(sol_UAV_list[*it]); j++) {
					UAV_allowed_list.push_back(sol_UAV_list[*it][j]);
				}
			}
			vector<UAV> UAV_list_tmp = UAV_allowed_list;
			int idx = -1;
			for (auto uav : UAV_list_tmp) {
				idx++;
				if (uav.cal_t_go_to(node_choosed) > T) {
					auto itr_move = UAV_allowed_list.begin() + idx;
					UAV_allowed_list.erase(itr_move);
					idx--;
					continue;
				}
			}

			if (size(UAV_allowed_list) == 0) {
				continue;
			}
			set<int> UAV_choosed_no_set;
			vector<UAV> UAV_col_tmp;
			int UAV_choosed_num = min(rnd_col_n, int(size(UAV_allowed_list)));
			vector<int> Path_node_no_list;
			//vector<int> UAV_id_tmp_list;
			vector<int> UAV_id_tmp_list=sol_gen.make_uavs_assigned_m(node_choosed, size(sol_gen.node_list), UAV_choosed_num);
#ifdef u_r
			UAV_id_tmp_list = sol_gen.make_uavs_assigned_r(node_choosed, size(sol_gen.node_list), UAV_choosed_num);
#endif // rand_u
			if (size(UAV_id_tmp_list) == 0) {
				continue;
			}
			//while (size(UAV_choosed_no_set) < UAV_choosed_num) {
			//	int rnd_UAV_no = rand() % size(UAV_allowed_list);
			//	if (UAV_choosed_no_set.find(rnd_UAV_no) == UAV_choosed_no_set.end()) {
			//		UAV_choosed_no_set.insert(rnd_UAV_no);
			//		//UAV_col_tmp.push_back(UAV_allowed_list[rnd_UAV_no]);
			//		//(*(UAV_col_tmp.end() - 1)).Path.push_back(node_choosed);
			//		//Path_node_no_list.push_back(size((*(UAV_col_tmp.end() - 1)).Path) - 1);
			//		UAV_id_tmp_list.push_back(UAV_allowed_list[rnd_UAV_no].id);
			//	}
			//}
			//float t_now = 0;
			//for (int i_c : UAV_id_tmp_list) {
			//	t_now += T - sol_gen.id_to_UAV(i_c).t_used;
			//}
			//if (t_now < node_choosed.node_size / UAV::eff) {
			//	continue;
			//}
			node_choosed.UAV_id_list = UAV_id_tmp_list;
			for (int uav_id_i : node_choosed.UAV_id_list) {
				UAV_col_tmp.push_back(sol_gen.id_to_UAV(uav_id_i));
				(*(UAV_col_tmp.end() - 1)).Path.push_back(node_choosed);
				Path_node_no_list.push_back(size((*(UAV_col_tmp.end() - 1)).Path) - 1);
			}
			node_choosed.Path_node_no_list = Path_node_no_list;
			
			float t_end_cal = node_choosed.cal_time_end(UAV_col_tmp);
			if (t_end_cal > T) {
				continue;
			}
			else {
				node_choosed.t_end = t_end_cal;
				sol_node_list.push_back(node_choosed);
				choosed_id_set.insert(rnd_node_id);
				for (auto it = type_allowed.begin(); it != type_allowed.end(); it++) {
					int type_ch = *it;
					if (type_ch > int(size(UAV_list) - 1)) {
						break;
					}
					for (int j = 0; j < size(sol_UAV_list[*it]); j++) {
						for (auto& uav : UAV_col_tmp) {
							if (uav.id == sol_UAV_list[*it][j].id) {
								sol_UAV_list[*it][j].Path = uav.Path;
								sol_UAV_list[*it][j].t_used = uav.t_used;
							}
						}
					}
				}
				sol_gen.node_list = sol_node_list;
				sol_gen.UAV_list = sol_UAV_list;
			}
		}
		if (size(node_id) == size(node_list)) {
			sol_gen.insert_rnd_nodes(node_list, 0);
			//complement(sol_gen, node_list);
			break;
		}
	}
	//sol_gen.node_list = sol_node_list;
	//sol_gen.UAV_list = sol_UAV_list;
	//sol_gen.insert_rnd_nodes(node_list, 0);
	//complement(sol_gen, node_list);
	return sol_gen;
}



Solution heuristic_generate_sol(vector<Working_node>& node_list, vector<vector<UAV>>& UAV_list, int rnd_flag) {
	//Solution sol_gen;
	//sol_gen.UAV_list = UAV_list;
	//vector<Working_node> node_list_for_choose = node_list;
	//vector<vector<UAV>> sol_UAV_list=UAV_list;
	//vector<Working_node> sol_node_list;
	//set<int> node_choosed_id_set;
	//int node_now_id = size(dis_matric) - 1;
	//set<int> min_node_failed_set = {};
	//while (size(sol_node_list) <= size(node_list)) {
	//	if (size(min_node_failed_set) == (size(node_list) - size(node_choosed_id_set))) {
	//		sol_gen.insert_rnd_nodes(node_list, 0);
	//		//complement(sol_gen, node_list);
	//		break;
	//	}
	//	vector<float> t_lefts;
	//	vector<vector<UAV>>& uav_list = sol_gen.UAV_list;
	//	t_lefts.resize(size(uav_list));
	//	float t_max = 0;
	//	for (int i = 0; i < size(uav_list); i++) {
	//		for (int j = 0; j < size(uav_list[i]); j++) {
	//			t_lefts[i] += T - uav_list[i][j].t_used;
	//		}
	//		t_max += t_lefts[i];
	//	}
	//	Working_node min_dis_node;
	//	float dist=INFINITY;
	//	for (int i = 0; i < size(node_list); i++) {
	//		int node_id_choosed = node_list[i].id;
	//		if((node_choosed_id_set.find(node_id_choosed)) == node_choosed_id_set.end() && (min_node_failed_set.find(node_id_choosed) == min_node_failed_set.end())){
	//			float pre_dis = dis_matric[node_now_id][node_id_choosed];//node_list[i].node_size
	//			if (dist >= pre_dis) {
	//				dist = pre_dis;
	//				min_dis_node = node_list[i];
	//			}
	//		}
	//	}		
	//	if (min_dis_node.node_size / UAV::eff > t_max) {
	//		min_node_failed_set.insert(min_dis_node.id);
	//		continue;
	//	}
	//	set<int> type_allowed = min_dis_node.type;
	//	vector<UAV> UAV_allowed_list;
	//	for (auto it = type_allowed.begin(); it != type_allowed.end(); it++) {
	//		int type_ch = *it;
	//		if (type_ch > int(size(UAV_list) - 1)) {
	//			break;
	//		}
	//		for (int j = 0; j < size(sol_UAV_list[*it]); j++) {
	//			UAV_allowed_list.push_back(sol_UAV_list[*it][j]);
	//		}
	//	}
	//	float t_now=0;
	//	for (auto uav_ch_i : UAV_allowed_list) {
	//		t_now += T - uav_ch_i.t_used;
	//	}
	//	if (t_now < min_dis_node.node_size / UAV::eff) {
	//		min_node_failed_set.insert(min_dis_node.id);
	//		continue;
	//	}
	//	vector<UAV> UAV_list_tmp = UAV_allowed_list;
	//	int idx = -1;
	//	for (auto uav : UAV_list_tmp) {
	//		idx++;
	//		if (uav.cal_t_go_to(min_dis_node) > T) {
	//			auto itr_move = UAV_allowed_list.begin() + idx;
	//			UAV_allowed_list.erase(itr_move);
	//			idx--;
	//			continue;
	//		}
	//	}
	//	if (size(UAV_allowed_list) == 0) {
	//		min_node_failed_set.insert(min_dis_node.id);
	//		continue;
	//	}
	//	int rnd_col_num;
	//	if (rnd_flag == 1) {
	//		rnd_col_num = (rand() % (min_dis_node.col_UAV_num)) + 1;
	//	}
	//	else {
	//		rnd_col_num = min_dis_node.col_UAV_num;
	//	}
	//	int UAV_col_num = min(int(size(UAV_allowed_list)), rnd_col_num);
	//	set<int> UAV_col_tmp_id_set;
	//	vector<UAV> tmp_col_UAV_list;
	//	vector<int> Path_node_no_list;
	//	min_dis_node.UAV_id_list.clear();
	//	//while (size(UAV_col_tmp_id_set) < UAV_col_num) {
	//	//	int rnd_UAV_no = rand() % (size(UAV_allowed_list));
	//	//	if(UAV_col_tmp_id_set.find(rnd_UAV_no) == UAV_col_tmp_id_set.end()){
	//	//		UAV_col_tmp_id_set.insert(rnd_UAV_no);
	//	//		tmp_col_UAV_list.push_back(UAV_allowed_list[rnd_UAV_no]);
	//	//		(*(tmp_col_UAV_list.end()-1)).Path.push_back(min_dis_node);
	//	//		Path_node_no_list.push_back(size((*(tmp_col_UAV_list.end() - 1)).Path) - 1);
	//	//		min_dis_node.UAV_id_list.push_back(UAV_allowed_list[rnd_UAV_no].id);
	//	//	}
	//	//}
	//	min_dis_node.UAV_id_list = sol_gen.make_uavs_assigned_m(min_dis_node, size(sol_gen.node_list), UAV_col_num);
	//	if (size(min_dis_node.UAV_id_list) == 0) {
	//		min_node_failed_set.insert(min_dis_node.id);
	//		continue;
	//	}
	//	else {
	//		for (int uav_id_i : min_dis_node.UAV_id_list) {
	//			tmp_col_UAV_list.push_back(sol_gen.id_to_UAV(uav_id_i));
	//			(*(tmp_col_UAV_list.end() - 1)).Path.push_back(min_dis_node);
	//			Path_node_no_list.push_back(size((*(tmp_col_UAV_list.end() - 1)).Path) - 1);
	//		}
	//	}
	//	min_dis_node.Path_node_no_list = Path_node_no_list;
	//	float time_end_real = min_dis_node.cal_time_end(tmp_col_UAV_list);
	//	if (time_end_real <= T) {
	//		min_node_failed_set.clear();
	//		min_dis_node.t_end = time_end_real;
	//		min_node_failed_set.clear();
	//		for (auto it = type_allowed.begin(); it != type_allowed.end(); it++) {
	//			int type_ch = *it;
	//			if (type_ch > int(size(UAV_list) - 1)) {
	//				break;
	//			}
	//			for (int j = 0; j < size(sol_UAV_list[*it]); j++) {
	//				for (auto& uav : tmp_col_UAV_list) {
	//					if (uav.id == sol_UAV_list[*it][j].id) {
	//						sol_UAV_list[*it][j].Path = uav.Path;
	//						sol_UAV_list[*it][j].t_used = uav.t_used;
	//					}
	//				}
	//			}
	//		}
	//		node_choosed_id_set.insert(min_dis_node.id);
	//		node_now_id = min_dis_node.id;
	//		sol_node_list.push_back(min_dis_node);
	//		sol_gen.node_list = sol_node_list;
	//		sol_gen.UAV_list = sol_UAV_list;
	//	}
	//	else {
	//		min_node_failed_set.insert(min_dis_node.id);
	//	}
	//}
	////complement(sol_gen, node_list);
	////sol_gen.insert_rnd_nodes(node_list, 0);
	////complement(sol_gen, node_list);
	//return sol_gen;
	Solution sol_gen;
	sol_gen.UAV_list = UAV_list;
	vector<int> node_hard_in;
	while (true) {
		vector<Working_node> node_left;		
		float t_left = 0;
		for (auto uav_l : sol_gen.UAV_list) {
			for (auto uav_ll : uav_l) {
				t_left += T - uav_ll.t_used;
			}
		}

		for (auto node_l : node_list) {
			if ((!sol_gen.check_node_id(node_l.id))&&(node_l.node_size < t_left * UAV::eff)) {
				if (size(node_hard_in) > 0) {
					for (int j : node_hard_in) {
						if (j == node_l.id) {
							continue;
						}
					}				
				}

				node_left.push_back(node_l);
			}
		}
		if (size(node_left)==0) {
			break;
		}
		//sort(node_left.begin(), node_left.end(), [](Working_node& a, Working_node& b) {return a.node_size < b.node_size; });
		vector<Working_node> node_left_final;
		for (auto& node_i: node_left) {
			auto sol_tt = sol_gen;
			auto type_l = node_i.type;
			int uav_allowed = 0;
			for (auto iter_l = type_l.begin(); iter_l != type_l.end(); iter_l++) {
				uav_allowed += size(UAV_list[*iter_l]);
			}
			int col_num = min(node_i.col_UAV_num, uav_allowed);
			
			float rnd_pro = (float)rand() / RAND_MAX;
			int uav_num = 0;
			uav_num = min(col_num, min_node_col[node_i.id]);
			//if (rnd_pro < eps) {
			//	uav_num = rand() % col_num + 1;
			//}
			//else {
			//	uav_num = min(col_num, min_node_col[node_i.id]);
			//}
			node_i.UAV_id_list = sol_gen.make_uavs_assigned_m(node_i, size(sol_gen.node_list), uav_num);
			sol_tt.node_list.push_back(node_i);
			node_i.no = size(sol_gen.node_list);
			for (int uav_tt_idx : node_i.UAV_id_list) {
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
			while(sol_tt.need_repair()) {
				sol_tt = sol_gen;
				uav_num++;
				node_i.UAV_id_list = sol_gen.make_uavs_assigned_m(node_i, size(sol_gen.node_list), uav_num);
				sol_tt.node_list.push_back(node_i);
				node_i.no = size(sol_gen.node_list);
				for (int uav_tt_idx : node_i.UAV_id_list) {
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
				if (uav_num == col_num) {
					break;
				}
			}
			if (sol_tt.need_repair()) {
				node_hard_in.push_back(node_i.id);
				continue;
			}
			node_left_final.push_back(node_i);
			float dis_l=0;
			for (int uav_tt_idx : node_i.UAV_id_list) {
				UAV& uav_tt = sol_tt.id_to_UAV(uav_tt_idx);
				if (size(uav_tt.Path) == 1) {
					dis_l += dis_matric[node_i.id][MAP_NODE_NUM];
				}
				else {
					dis_l += dis_matric[node_i.id][uav_tt.Path[size(uav_tt.Path) - 1].id];
				}
			}
			node_i.ef = node_i.node_size / dis_l;
		}
		if (size(node_left_final) == 0) {
			break;
		}
		sort(node_left_final.begin(), node_left_final.end(), [](Working_node& a, Working_node& b) {return a.ef > b.ef; });
		auto node_ch = node_left_final[0];
		sol_gen.node_list.push_back(node_ch);
		for (int uav_tt_idx : node_ch.UAV_id_list) {
			UAV& uav_tt = sol_gen.id_to_UAV(uav_tt_idx);
			uav_tt.Path.push_back(node_ch);
		}
	}
	//sol_gen.insert_rnd_nodes(node_list,0);
	return sol_gen;
}

Solution NEH_generate_sol(vector<Working_node>& node_list, vector<vector<UAV>>& UAV_list, int rnd_flag) {
	Solution sol_gen;
	sol_gen.UAV_list = UAV_list;
	vector<Working_node> node_list_size_de = node_list;
	for (auto& node : node_list_size_de) {
		node.ls = dis_matric[node.id][MAP_NODE_NUM];
	}
	sort(node_list_size_de.begin(), node_list_size_de.end(), [](Working_node& a, Working_node& b) {return a.node_size / (a.ls ) > b.node_size / (b.ls); });//a.node_size/a.ls > b.node_size/b.ls
	//for (auto& node : node_list_size_de) {
	//	node.ls = dis_matric[node.id][MAP_NODE_NUM];
	//	auto type_n = node.type;
	//	for (auto node_l : node_list) {
	//		auto type_nl = node_l.type;
	//		int flag_l = 0;
	//		for (auto iter_n = type_n.begin(); iter_n != type_n.end(); iter_n++) {
	//			for (auto iter_nl = type_nl.begin(); iter_nl != type_nl.end(); iter_nl++) {
	//				if (*iter_n == *iter_nl) {
	//					flag_l = 1;
	//					break;
	//				}
	//			}
	//		}
	//		if (flag_l == 1) {
	//			node.ls += dis_matric[node.id][node_l.id];
	//		}
	//	}
	//}
	//sort(node_list_size_de.begin(), node_list_size_de.end(), [](Working_node& a, Working_node& b) {return a.node_size / a.ls > b.node_size / b.ls; });
	set<int> node_id;
	for (int ni = 0; ni < size(node_list);ni++) {
		auto node_i = node_list_size_de[0];
		node_list_size_de.erase(node_list_size_de.begin());
		node_id.insert(node_i.id);
		vector<Working_node> nodes_left;
		vector<float> t_lefts;
		vector<vector<UAV>>& uav_list = sol_gen.UAV_list;
		t_lefts.resize(size(uav_list));
		float t_max = 0;
		for (int i = 0; i < size(uav_list); i++) {
			for (int j = 0; j < size(uav_list[i]); j++) {
				t_lefts[i] += T - uav_list[i][j].t_used;
			}
			t_max += t_lefts[i];
		}
		if (node_i.node_size / UAV::eff > t_max) {
			continue;
		}
		int insert_p = INT_MAX;
		vector<int> IP_v;
		float min_t_max = LONG_MAX;
		set<int> type_allowed = node_i.type;
		vector<UAV> uav_allowed_list;
		vector<int> UAV_allowed_id_list;
		float t_now = 0;
		for (auto it = type_allowed.begin(); it != type_allowed.end(); it++) {
			int type_ch = *it;
			if (type_ch > int(size(UAV_list) - 1)) {
				break;
			}
			for (int j = 0; j < size(UAV_list[*it]); j++) {
				UAV_allowed_id_list.push_back(UAV_list[*it][j].id);
				uav_allowed_list.push_back(sol_gen.UAV_list[*it][j]);
				t_now += T - sol_gen.UAV_list[*it][j].t_used;
			}
		}
		if (t_now < node_i.node_size / UAV::eff) {
			continue;
		}
		int col_num = min((int)size(UAV_allowed_id_list), node_i.col_UAV_num);
		int col_rnd_num=0;
		if (col_num < min_node_col[node_i.id]) {
			continue;
		}
		if (col_num == min_node_col[node_i.id]) {
			col_rnd_num = min_node_col[node_i.id];
		}
		else {
			col_rnd_num = rand() % (col_num-min_node_col[node_i.id])+min_node_col[node_i.id];
		}
		
		//int col_rnd_num = min_node_col[node_i.id];
	/*	int col_rnd_num = col_num;*/
		//set<int> ch_index;
		vector<int> uav_col_id_list;
		//while (size(ch_index) < col_rnd_num) {
		//	//int rnd_in = rand() % size(UAV_allowed_id_list);
		//	//if (ch_index.find(rnd_in) == ch_index.end()) {
		//	//	ch_index.insert(rnd_in);
		//	//	uav_col_id_list.push_back(UAV_allowed_id_list[rnd_in]);
		//	//}
		//	
		//}
		//sort(uav_allowed_list.begin(), uav_allowed_list.end(), [](UAV& a, UAV& b) {return (T - a.t_used) > (T - b.t_used); });
		//for (int i = 0; i < col_rnd_num; i++) {
		//	uav_col_id_list.push_back(uav_allowed_list[i].id);
		//}
		if (size(sol_gen.node_list) == 0) {
			uav_col_id_list = sol_gen.make_uavs_assigned_m(node_i, 0, col_rnd_num);
#ifdef u_r
			uav_col_id_list = sol_gen.make_uavs_assigned_r(node_i, 0, col_rnd_num);
#endif // rand_u
			if (size(uav_col_id_list) == 0) {
				continue;
			}
			node_i.UAV_id_list = uav_col_id_list;
			Solution sol_tt = sol_gen;
			sol_tt.node_list.insert(sol_tt.node_list.begin(), node_i);
			for (int uav_idx : node_i.UAV_id_list) {
				sol_tt.id_to_UAV(uav_idx).Path.push_back(node_i);
			}
			if (!sol_tt.need_repair()) {
				sol_gen.node_list.push_back(node_i);
				for (int uav_idx : node_i.UAV_id_list) {
					sol_gen.id_to_UAV(uav_idx).Path.push_back(node_i);
				}			
			}
		}
		else {
			for (int i = 0; i <= size(sol_gen.node_list); i++) {
				uav_col_id_list = sol_gen.make_uavs_assigned_m(node_i,i, col_rnd_num);
#ifdef u_r
				uav_col_id_list = sol_gen.make_uavs_assigned_r(node_i, i, col_rnd_num);
#endif // rand_u
				if (size(uav_col_id_list) == 0) {
					continue;
				}
				node_i.UAV_id_list = uav_col_id_list;
				Solution sol_tt = sol_gen;
				sol_tt.node_list.insert(sol_tt.node_list.begin() + i, node_i);
				sol_tt.assign_node_no();
				node_i.no = sol_tt.node_list[i].no;
				for (int uav_tt_idx : node_i.UAV_id_list) {
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
					IP_v.push_back(i);
					float max_node_t = 0;
					//for (auto node_ii : sol_tt.node_list) {
					//	//max_node_t = max(max_node_t, node_ii.t_end);
					//	max_node_t += node_ii.t_end;
					//}
					for (auto uav_l : sol_tt.UAV_list) {
						for (auto uav_ll : uav_l) {
							max_node_t += uav_ll.t_used;
						}
					}
					
					if (min_t_max > max_node_t) {
						min_t_max = max_node_t;
						insert_p = i;
					}
				}
			}
			//if (size(IP_v) == 0) {
			//	continue;
			//}
			//else {
			//	insert_p = IP_v[rand() % size(IP_v)];
			//}
			if (insert_p != INT_MAX) {
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
				sol_gen.need_repair();
				for (auto& node : node_list_size_de) {
					set<int> node_iid;
					node.ls = 0;
					node.t_ll = 0;
					auto type_n = node.type;
					int u_a=0;
					for (auto iter_n = type_n.begin(); iter_n != type_n.end(); iter_n++) {
						u_a += size(UAV_list[*iter_n]);
						for (auto uav_n : sol_gen.UAV_list[*iter_n]) {
							node.t_ll += T - uav_n.t_used;
							if (size(uav_n.Path) == 0) {
								node_iid.insert(MAP_NODE_NUM);
							}
							else {
								node_iid.insert(uav_n.Path[size(uav_n.Path) - 1].id);
							}
						}
					}
					for (auto iter_id = node_iid.begin(); iter_id != node_iid.end(); iter_id++) {
						node.ls += dis_matric[node.id][*iter_id];
					}
					node.ls /= size(node_iid);
					node.t_ll /= u_a;
					//node.ls = dis_matric[node.id][MAP_NODE_NUM];
					//
					//for (auto node_l : sol_gen.node_list) {
					//	auto type_nl = node_l.type;
					//	int flag_l = 0;
					//	for (auto iter_n = type_n.begin(); iter_n != type_n.end(); iter_n++) {
					//		for (auto iter_nl = type_nl.begin(); iter_nl != type_nl.end(); iter_nl++) {
					//			if (*iter_n == *iter_nl) {
					//				flag_l = 1;
					//				break;
					//			}
					//		}
					//	}
					//	if (flag_l == 1) {
					//		node.ls += dis_matric[node.id][node_l.id];
					//	}
					//}
				}
				sort(node_list_size_de.begin(), node_list_size_de.end(), [](Working_node& a, Working_node& b) {return a.t_ll* a.node_size / (a.ls ) > b.t_ll * b.node_size / (b.ls ); });//return a.node_size / a.ls > b.node_size / b.ls;
			}
			//else {
			//	counter++;
			//}
		}
		if (size(node_id) == size(node_list)) {
			//sol_gen.insert_rnd_nodes(node_list, 0);
			//complement(sol_gen, node_list);
			break;
		}
	}
	return sol_gen;
}


void complement(Solution& sol_tmp, vector<Working_node> node_list) {
	set<int> node_id;
	for (auto& node_i : sol_tmp.node_list) {
		node_id.insert(node_i.id);
	}
	while (size(node_id) < size(node_list)) {
		vector<Working_node> nodes_left;
		vector<float> t_lefts;
		vector<vector<UAV>>& uav_list = sol_tmp.UAV_list;
		Solution sol_gen = sol_tmp;
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

		for (auto& node_i : node_list) {
			if (node_id.find(node_i.id) == node_id.end() && node_i.node_size / UAV::eff < t_max) {
				nodes_left.push_back(node_i);
			}
			else {
				node_id.insert(node_i.id);
			}
		}
		//sort(nodes_left.begin(), nodes_left.end(), [](Working_node& a, Working_node& b) {return a.node_size > b.node_size; });
		random_shuffle(nodes_left.begin(), nodes_left.end());
		for (auto& node_i : nodes_left) {
			node_id.insert(node_i.id);
			int insert_p = INT_MAX;
			float min_t_max = LONG_MAX;
			set<int> type_node = node_i.type;
			//float t_al = 0;
			//for (auto iter = type_node.begin(); iter != type_node.end(); iter++) {
			//	t_al += t_lefts[*iter];
			//}
			//if (t_al < node_i.node_size / UAV::eff) {
			//	continue;
			//}
			if (size(type_node) == 1) {
				node_i.UAV_id_list = uav_l_l[*type_node.begin()];
			}
			else {
				set<int> uav_ch;
				node_i.UAV_id_list = uav_l_l[*type_node.begin()];
				for (int id : node_i.UAV_id_list) {
					uav_ch.insert(id);
				}
				auto iter = type_node.begin();
				iter++;
				for (; iter != type_node.end(); iter++) {
					auto uav_id_i = uav_l_l[*iter];
					for (int id_i : uav_id_i) {
						if (uav_ch.find(id_i) == uav_ch.end()) {
							uav_ch.insert(id_i);
							node_i.UAV_id_list.push_back(id_i);
						}
					}
				}
			}
			//Solution sol_tt = sol_gen;
			for (int i = 0; i <= size(sol_gen.node_list); i++) {
				Solution sol_tt = sol_gen;
				sol_tt.node_list.insert(sol_tt.node_list.begin() + i, node_i);
				sol_tt.assign_node_no();
				node_i.no = sol_tt.node_list[i].no;
				for (int uav_tt_idx : node_i.UAV_id_list) {
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
					float max_node_t = 0;
					//for (auto node_ii : sol_tt.node_list) {
					//	/*max_node_t = max(max_node_t, node_ii.t_end);*/
					//	max_node_t += node_ii.t_end;
					//}
					for (auto uav_l : sol_tt.UAV_list) {
						for (auto uav_ll : uav_l) {
							max_node_t += uav_ll.t_used;
						}
					}
					if (min_t_max > max_node_t) {
						min_t_max = max_node_t;
						insert_p = i;
					}
				}
			}
			if (insert_p != INT_MAX) {
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
				sol_tmp = sol_gen;
				sol_tmp.need_repair();
				sol_tmp.clear_ext_path_node(1);
				break;
			}
				//sol_tt.node_list.insert(sol_tt.node_list.end(), node_i);
				//for (int uav_idx : node_i.UAV_id_list) {
				//	sol_tt.id_to_UAV(uav_idx).Path.push_back(node_i);
				//}
				//if (!sol_tt.need_repair()) {
				//	sol_gen.node_list.push_back(node_i);
				//	for (int uav_idx : node_i.UAV_id_list) {
				//		sol_gen.id_to_UAV(uav_idx).Path.push_back(node_i);
				//	}
				//	sol_tmp = sol_gen;
				//	sol_tmp.need_repair();
				//	sol_tmp.clear_ext_path_node(1);
				//	break;
				//}
		}
	}
}