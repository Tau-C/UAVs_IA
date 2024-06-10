#include "Working_node.h"
#include <math.h>
#include "UAV.h"
#include<ctime>
#include "solution.h"
#include "initialization.h"
#include "update.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include "map_config.h"
#include "xlsxwriter.h"
#include<chrono>
//算法选择宏
#define MA
//

//#define PSO

//#define IGSA

//#define ABC

//#define HS

//#define GWO
float T = 20000;
int expect_node_num = 0;

//Param

//#define TXT "test_1_"
//int PS_a[4] = { 10,20,30,40 };
//float MP_a[4] = { 0.1,0.2,0.3,0.4 };//0,0.15
//float eps_a[4] = { 0.05,0.2,0.35,0.5 };
//float alp_a[4] = { 0.02,0.04,0.06,0.08 };
////float alp_a[4] = { 0.1,0.2,0.3,0.4};
//float gam_a[4] = { 0.3,0.5,0.7,0.9 };
//float LS_a[4] = { 0.2,0.3,0.4,0.5 };//0,0.3,
//
//
//float IT_a[4] = { 0.2,0.3,0.4,0.5 };
// 

//unsigned int P_size = 40;
////int M = (float)P_size / 2;
////float MAX_PRO = 0.5;
//float PC = 0.95;
//float MAX_MUTATE = 0.4;
//float eps = 0.35;
//float alpha_1 = 0.08;
//float gamma = 0.5;
//float ls_it = 0.3;


//M
//unsigned int P_size = 30;
//int M = (float)P_size / 2;
//float MAX_PRO = 0.5;
//float PC = 0.95;
//float MAX_MUTATE = 0.3;
//float eps = 0.05;
//float alpha_1=0.08;
//float gamma=0.3;
//float ls_it = 0.5;
//int TRY_T_MAX = 0.5;


//L
unsigned int P_size = 7;
//int M = (float)P_size / 2;
//float MAX_PRO = 0.5;
float PC = 1;
float MAX_MUTATE = 0.4;
float eps = 0;
float alpha_1=0.1;
float gamma=0.5;
float ls_it = 0;


using namespace std;
int iteration_time = 0;

int IT =100000;
constexpr int UAV_type_num = 2;
int UAV_type[UAV_type_num] = {7,8};
//int UAV_type[UAV_type_num] = {5,5};//geq 2
float max_size=0;
int* min_node_col = NULL;
int mode_ch_counter = 0;
float dis_matric[MAP_NODE_NUM + 1][MAP_NODE_NUM];

typedef float V_vec[MAP_NODE_NUM];
typedef float Q_vec[UAV_NUM];//change with uav num

typedef float Q_vec_1[MAP_NODE_NUM][UAV_NUM];

V_vec* V_matrix;
Q_vec_1* Q_table_1;
Q_vec_1* Q_table;

//part define
int mode = 4;


void make_working_node(vector<Working_node>* node_vec_ptr, float* coord, float* size, int arr_size, int* co_num, Working_node::node_type* node_type) {
	for (int i = 0; i < arr_size; i+=1) {
		Working_node node_tmp(*coord, *(coord + 1), *size,  *node_type,*co_num);
		node_tmp.id = i;
		(*node_vec_ptr).push_back(node_tmp);
		coord+=2;
		size++;
		co_num++;
		node_type++;
	}
}

void cal_dis(float(*dis_ma_ptr)[MAP_NODE_NUM], vector<Working_node> vec_node) {
	int size_vec = size(vec_node);
	for (int i = 0; i < size_vec; i++) {
		for (int j = 0; j < size_vec; j++) {
			vector<float> coord_i = vec_node[i].coord;
			vector<float> coord_j = vec_node[j].coord;
			float dis_x = coord_i[0] - coord_j[0];
			float dis_y = coord_i[1] - coord_j[1];
			dis_ma_ptr[i][j] = (float)sqrt(dis_x * dis_x + dis_y * dis_y);
		}
		vector<float> coord_k = vec_node[i].coord;
		float dis_x = coord_k[0] - depot_center;
		float dis_y = coord_k[1] - depot_center;
		dis_ma_ptr[size_vec][i] = (float)sqrt(dis_x * dis_x + dis_y * dis_y);
	}
}

void make_UAV(vector<vector<UAV>>& UAV_list, int* UAV_type_list) {
	int count = 0;
	for (int i = 0; i < UAV_type_num; i++) {
		vector<UAV> UAV_sub_list;
		for (int j = 0; j < UAV_type_list[i]; j++) {
			UAV_sub_list.emplace_back(i, count);
			count++;
		}
		UAV_list.push_back(UAV_sub_list);
	}
}

bool is_faster(UAV& a, UAV& b) {
	return a.t_used < b.t_used;
}

void ve_col_clear(vector<float*> col_num_choosed_pro_list, vector<Working_node>& node_list) {
	for (int i = 0; i < size(node_list); i++) {
		for (int j = 0; j < node_list[i].col_UAV_num; j++) {
			col_num_choosed_pro_list[i][j] = 1;
		}
	}
}

void Stringsplit(const string& str, const string& splits, vector<string>& res)
{
	if (str == "")		return;
	//在字符串末尾也加入分隔符，方便截取最后一段
	string strs = str + splits;
	size_t pos = strs.find(splits);
	int step = splits.size();

	// 若找不到内容则字符串搜索函数返回 npos
	while (pos != strs.npos)
	{
		string temp = strs.substr(0, pos);
		res.push_back(temp);
		//去掉已分割的字符串,在剩下的字符串中进行分割
		strs = strs.substr(pos + step, strs.size());
		pos = strs.find(splits);
	}
}

void Stringsplit(const string& str, const char split, vector<string>& res)
{
	if (str == "")		return;
	//在字符串末尾也加入分隔符，方便截取最后一段
	string strs = str + split;
	size_t pos = strs.find(split);

	// 若找不到内容则字符串搜索函数返回 npos
	while (pos != strs.npos)
	{
		string temp = strs.substr(0, pos);
		res.push_back(temp);
		//去掉已分割的字符串,在剩下的字符串中进行分割
		strs = strs.substr(pos + 1, strs.size());
		pos = strs.find(split);
	}
}

void cal_max_size(vector<Working_node> wn_list) {
	max_size = 0;
	for (auto node_i : wn_list) {
		max_size += node_i.node_size;
	}
}


int main() {
	auto now_t = chrono::system_clock::now();
	srand(42);
	//srand(time(nullptr));
	for (int mode_i =4; mode_i <5; mode_i++) {
		mode = mode_i;
		if (mode == 2) {
			eps = 1;
		}
		else {
			//eps = 0.35;
		}
		lxw_workbook* workbook = workbook_new((to_string(mode_i)+"eff1.xlsx").c_str());
        lxw_worksheet* worksheet = workbook_add_worksheet(workbook, NULL);
		for (int map_i = 1; map_i < 21; map_i++) {
			min_node_col = new int[MAP_NODE_NUM];
			string in_txt = "_" + to_string(map_i) + ".txt";
			ifstream in;
			in.open("./map_ini/map_60" + in_txt);
			string s;
			string x_i;
			string y_i;
			string size_i;
			string col_i;
			string type_i;
			vector<Working_node> node_list;
			//构造各任务点
			vector<string> str_list;
			int id_n = 0;
			while (getline(in, s)) {
				Stringsplit(s, " ", str_list);
				x_i = str_list[0];
				y_i = str_list[1];
				size_i = str_list[2];
				col_i = str_list[3];
				type_i = str_list[4];
				set<int> set_tmp;
				if (stoi(type_i) == 0) {
					set_tmp = { 0 };
				}
				if (stoi(type_i) == 1) {
					set_tmp = { 1 };
				}
				if (stoi(type_i) == 2) {
					set_tmp = { 0,1 };
				}
				Working_node node_tmp(stol(x_i), stol(y_i), stol(size_i), set_tmp, stoi(col_i));
				node_tmp.id = id_n;
				id_n++;
				node_list.push_back(node_tmp);
				str_list.clear();
			}
			//make_working_node(&node_list, coord_arr, size_arr, arr_size, co_number, type_list);
			cal_dis(dis_matric, node_list);
			//for (auto& node : node_list) {
			//	for (auto node_l : node_list) {
			//		node.ls += dis_matric[node.id][node_l.id];
			//	}
			//}
			for (int i = 0; i < MAP_NODE_NUM; i++) {
				float size_i = node_list[i].node_size;
				for (int j = 1; j < MAP_NODE_NUM; j++) {
					if (size_i / j <= T) {
						min_node_col[i] = j;
						break;
					}
				}
			}
			cal_max_size(node_list);
			expect_node_num = 3 * T * UAV_NUM / max_size * MAP_NODE_NUM;
			//MAX_MUTATE = 0.3;
			//ls_it =0.4;
			//TRY_T_MAX = 0.5;
			MAX_MUTATE = round(MAX_MUTATE * expect_node_num);
			ls_it = round(ls_it * expect_node_num);
			//TRY_T_MAX =round( TRY_T_MAX * expect_node_num);
			for (int trial = 0; trial < 10; trial++) {
				ofstream out;
				string out_file = "mode" + to_string(mode_i) +"map" + to_string(map_i) +  "trial" +to_string(trial)+".txt";
				out.open("./eff/out_" + out_file);
				ofstream t_out;
				t_out.open("./eff/t_" + out_file);
				V_matrix = new V_vec[MAP_NODE_NUM];
				Q_table_1 = new Q_vec_1[MAP_NODE_NUM];
				Q_table = new Q_vec_1[MAP_NODE_NUM];
				vector<float*> col_num_choosed_pro_list;
				Solution* p_best_sol_list = new Solution[P_size];
				Solution* a_best_sol = new Solution;
				for (int i = 0; i < MAP_NODE_NUM; i++) {
					float* new_f = new float[UAV_NUM]();
					col_num_choosed_pro_list.push_back(new_f);
				}
				for (int i = 0; i < MAP_NODE_NUM; i++) {
					for (int j = 0; j < MAP_NODE_NUM; j++) {
						V_matrix[i][j] = 0;
					}
				}
				for (int i = 0; i < MAP_NODE_NUM; i++) {
					for (int k = 0; k < MAP_NODE_NUM; k++) {
						for (int j = 0; j < UAV_NUM; j++) {
							Q_table_1[i][k][j] = 0;
							Q_table[i][k][j] = 0;
						}
					}
				}
				vector<Solution> sol_list;
				vector<vector<UAV>> UAV_list;
				//构造各无人机
				make_UAV(UAV_list, UAV_type);
				//解初始化
				ini_sol(sol_list, node_list, UAV_list, P_size);
				int i_id = 0;
				for (auto& sol : sol_list) {
					sol.cal_fitness();
					if (sol.fitness > a_best_sol->fitness) {
						*a_best_sol = sol;
					}
					p_best_sol_list[i_id] = sol;
					i_id++;
					//if (sol.need_repair()) {
					//	cout << 1 << endl;
					//}
				}
#ifdef MA
				sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });

#endif // MA
				float opt_fitness = a_best_sol->fitness;
				auto end_t = chrono::system_clock::now();
				cout << "Iteration: " << 0 << " --- " << "fitness: " << opt_fitness << endl;
				auto dur_t = chrono::duration_cast<chrono::microseconds>(  end_t-now_t);
				cout << (double)(dur_t.count()) << endl;
				out << opt_fitness << " ";
				//迭代开始
				time_t start_t = time(NULL);
				t_out << (double)0 << " ";
				time_t iteration_t;
				int unchange_counter=0;
				auto now_tt = chrono::system_clock::now();
				for (int i = 0; i < IT; i++) {
					
					//iteration_t = time(NULL);
					//eps =1-((0.95)* pow(difftime(iteration_t, start_t)/ (10 * expect_node_num),0.5));
					//cout << (eps) << endl;
					iteration_time++;
#ifdef MA
					update_sol(sol_list, node_list, 0, 0);
					sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
					if (opt_fitness < sol_list[0].fitness) {
						opt_fitness = sol_list[0].fitness;
					}
					if (i == 50) {
						for (int cz = 0; cz < UAV_NUM; cz++) {
							for (int tz = 0; tz < MAP_NODE_NUM; tz++) {
								for (int az = 0; az < MAP_NODE_NUM; az++) {
									cout << Q_table_1[tz][az][cz]<<" ";
								}
								cout << endl;
							}							
						}
						cout << "------------" << endl;
					}
#endif // MA
#ifdef PSO
					update_sol_pso(p_best_sol_list,a_best_sol, sol_list,node_list, 0, col_num_choosed_pro_list, 0);
					if (opt_fitness < a_best_sol->fitness) {
						opt_fitness = a_best_sol->fitness;
					}
#endif // PSO
#ifdef IGSA
					update_sol_ig(a_best_sol[0], node_list, unchange_counter);
					if (opt_fitness < a_best_sol[0].fitness) { 
						opt_fitness = a_best_sol[0].fitness;
						unchange_counter = 0;
					}
					else {
						unchange_counter++;
					}
#endif//IGSA
#ifdef ABC
					update_sol_abc(sol_list, node_list, unchange_counter);
					sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
					if (sol_list[0].fitness > opt_fitness) {
						opt_fitness = sol_list[0].fitness;
					}

#endif//ABC
#ifdef HS
					update_sol_hs(sol_list, node_list, unchange_counter);
					sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
					if (sol_list[0].fitness > opt_fitness) {
						opt_fitness = sol_list[0].fitness;
					}

#endif//HS
#ifdef GWO
					update_sol_gwo(sol_list, node_list, unchange_counter);
					sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
					if (sol_list[0].fitness > opt_fitness) {
						opt_fitness = sol_list[0].fitness;
					}
#endif//GWO
					cout <<"mode: "<<mode_i<<" / map: "<<map_i<<" / trial: "<<trial+1 << " / Iteration: " << i + 1 << " -- - " << "fitness : " << opt_fitness << endl;
					out << opt_fitness << " ";
					iteration_t = time(NULL);
					t_out << difftime(iteration_t, start_t) << " ";
					auto s_t = chrono::system_clock::now();
					cout << "Iteration: " << 0 << " --- " << "fitness: " << opt_fitness << endl;
					auto dur_tt = chrono::duration_cast<chrono::microseconds>(s_t - now_tt);
					cout << (double)(dur_tt.count()) << endl;
					if (difftime(iteration_t, start_t) > 10*expect_node_num) {
						break;
					}
				}
				worksheet_write_number(worksheet, map_i-1, trial, opt_fitness, NULL);
				delete[] V_matrix;
				delete[] Q_table_1;
				delete[] Q_table;
				delete[] p_best_sol_list;
				delete a_best_sol;
				for (int i = 0; i < MAP_NODE_NUM; i++) {
					delete[] col_num_choosed_pro_list[i];
				}
				out.close();
				t_out.close();
			}
		in.close();				
		delete[] min_node_col;
		}
		workbook_close(workbook);
	}
	return 0;
}
