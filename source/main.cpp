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
//#define MA
//

#define PSO

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

//int main() {
//#ifdef ABC
//	IT = 400;
//	P_size = 30;
//#endif // ABC
//
//#ifdef HS
//	IT = 1500;
//	P_size = 6;
//#endif // HS
//
//
//#ifdef GWO
//	IT = 160;
//#endif // GWO
//	V_matrix = new V_vec[MAP_NODE_NUM];
//	Q_table_1 = new Q_vec_1[MAP_NODE_NUM];
//	for (int i = 0; i < MAP_NODE_NUM; i++) {
//		for (int j = 0; j < MAP_NODE_NUM; j++) {
//			V_matrix[i][j] = 0;
//		}
//	}
//
//	for (int i = 0; i < MAP_NODE_NUM; i++) {
//		for (int k = 0; k < MAP_NODE_NUM; k++) {
//			for (int j = 0; j < UAV_NUM; j++) {
//				Q_table_1[i][k][j] = 0;
//			}
//		}
//	}
//	Solution* p_best_sol_list = new Solution[P_size];
//	Solution* a_best_sol = new Solution;
//	min_node_col = new int[MAP_NODE_NUM];
//	ifstream in;
//	in.open("C:\\Users\\cty\\source\\repos\\map_ini\\map_ini\\map_60_4.txt");
//	ofstream out;
//	string out_file = TXT;
//	out.open("./ca/out_" + out_file);
//	ofstream t_out;
//	t_out.open("./ca/t_" + out_file);
//	string s;
//	string x_i;
//	string y_i;
//	string size_i;
//	string col_i;
//	string type_i;
//	srand(1);
//	vector<Working_node> node_list;
//	//构造各任务点
//	vector<string> str_list;
//	int id_n = 0;
//	while (getline(in, s)) {
//		Stringsplit(s, " ", str_list);
//		x_i = str_list[0];
//		y_i = str_list[1];
//		size_i = str_list[2];
//		col_i = str_list[3];
//		type_i = str_list[4];
//		set<int> set_tmp;
//		if (stoi(type_i) == 0) {
//			set_tmp = { 0 };
//		}
//		if (stoi(type_i) == 1) {
//			set_tmp = { 1 };
//		}
//		if (stoi(type_i) == 2) {
//			set_tmp = { 0,1 };
//		}
//		Working_node node_tmp(stol(x_i), stol(y_i), stol(size_i), set_tmp, stoi(col_i));
//		node_tmp.id = id_n;
//		id_n++;
//		node_list.push_back(node_tmp);
//		str_list.clear();
//	}
//	//make_working_node(&node_list, coord_arr, size_arr, arr_size, co_number, type_list);
//	cal_dis(dis_matric, node_list);
//
//
//	for (int i = 0; i < MAP_NODE_NUM; i++) {
//		float size_i = node_list[i].node_size;
//		for (int j = 1; j < node_list[i].col_UAV_num; j++) {
//			if (size_i / j <= T) {
//				min_node_col[i] = j;
//				break;
//			}
//		}
//	}
//	vector<vector<UAV>> UAV_list;
//	//构造各无人机
//	make_UAV(UAV_list, UAV_type);
//	cal_max_size(node_list);
//	expect_node_num = 3 * T * UAV_NUM / max_size * MAP_NODE_NUM;
//	MAX_MUTATE = MAX_MUTATE * expect_node_num;
//	ls_it = ls_it * expect_node_num;
//	TRY_T_MAX = TRY_T_MAX * expect_node_num;
//	cout << expect_node_num << endl;
//	vector<Solution> sol_list;
//	//解初始化
//	ini_sol(sol_list, node_list, UAV_list, P_size);
//	for (auto& sol : sol_list) {
//		sol.cal_fitness();
//	}
//	sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//	float opt_fitness = sol_list[0].fitness;
//	cout << "Iteration: " << 0 << " --- " << "fitness: " << opt_fitness << endl;
//	out << opt_fitness << " ";
//	//迭代开始
//	int mode_ch_counter = 0;
//	time_t start_t = time(NULL);
//	t_out << (double)0 << " ";
//
//	time_t iteration_t;
//#ifdef MA
//	for (int i = 0; i < IT; i++) {
//		iteration_time++;
//		update_sol(sol_list, node_list, 0, 0);
//		sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//		if (opt_fitness < sol_list[0].fitness) {
//			opt_fitness = sol_list[0].fitness;
//			mode_ch_counter = 0;
//		}
//
//
//		cout << "Iteration: " << i + 1 << " --- " << "fitness: " << opt_fitness << endl;
//		out << opt_fitness << " ";
//		iteration_t = time(NULL);
//		t_out << difftime(iteration_t, start_t) << " ";
//		//for (int i = 0; i < MAP_NODE_NUM; i++) {
//		//	for (int j = 0; j < 10; j++) {
//		//		cout<<V_matrix[j][i]<<" ";
//		//	}
//		//	cout << endl;
//		//}
//		if (difftime(iteration_t, start_t) > 180) {
//			break;
//		}
//	}
//
//#endif // MA
//
//
//
//#ifdef PSO
//	* a_best_sol = p_best_sol_list[0];
//	for (int i = 0; i < P_size; i++) {
//		p_best_sol_list[i] = sol_list[i];
//		if (p_best_sol_list[i].fitness > (*a_best_sol).fitness) {
//			a_best_sol[0] = p_best_sol_list[i];
//		}
//	}
//
//	for (int i = 0; i < IT; i++) {
//		iteration_time++;
//		float fit_b = a_best_sol->fitness;
//		update_sol_pso(p_best_sol_list, a_best_sol, sol_list, node_list, 0, col_num_choosed_pro_list, mode_ch_counter);
//		if (fit_b == a_best_sol->fitness) {
//			mode_ch_counter++;
//		}
//		else {
//			mode_ch_counter = 0;
//		}
//		cout <<"Iteration: "<<iteration_time<<" --- "<< a_best_sol->fitness << endl;
//		out << a_best_sol->fitness << " ";
//		iteration_t = time(NULL);
//		t_out << difftime(iteration_t, start_t) << " ";
//	}
//	
//#endif // PSO
//
//
//
//#ifdef IG
//	IT = 10000;
//	auto ini_sol = sol_list[0];
//	(*a_best_sol) = ini_sol;
//	for (int i = 0; i < IT; i++) {
//		iteration_time++;
//		update_sol_ig(ini_sol, node_list, 0, col_num_choosed_pro_list, mode_ch_counter);
//		if ((*a_best_sol).fitness < ini_sol.fitness) {
//			mode_ch_counter = 0;
//		}
//		else {
//			mode_ch_counter++;
//		}
//		((*a_best_sol).fitness > (ini_sol.fitness)) ? (*a_best_sol)= (*a_best_sol) : (*a_best_sol) = ini_sol;
//		cout << "Iteration: " << iteration_time<< " --- " << (*a_best_sol).fitness << endl;
//		if (mode_ch_counter % 500 == 0&& mode_ch_counter!=0) {
//			ini_sol = (*a_best_sol);
//			ini_sol.repair_ig(0);
//			ini_sol.clear_ext_path_node(1);
//			ini_sol.insert_rnd_nodes(node_list, col_num_choosed_pro_list, mode_ch_counter);
//			ini_sol.cal_fitness();
//			auto ini_sol_tmp = ini_sol;
//			if (ini_sol.fitness < (*a_best_sol).fitness) {
//				ini_sol = (*a_best_sol);
//			}
//		}
//		if (i != 0 && i % 20 == 0) {
//			out << (*a_best_sol).fitness << " ";
//			iteration_t = time(NULL);
//			t_out << difftime(iteration_t, start_t) << " ";
//		}
//	}
//
//
//#endif // IG
//
//
//
//#ifdef ABC
//	int* MAX_UC = new int[size(sol_list)];
//	for (int i = 0; i < size(sol_list); i++) {
//		MAX_UC[i] = 0;
//	}
//	for (int i = 0; i < IT; i++) {
//		for (int i = 0; i < size(sol_list); i++) {
//			cout<<MAX_UC[i] <<" ";
//		}
//		iteration_time++;
//		update_sol_abc(MAX_UC, sol_list, node_list, 0, col_num_choosed_pro_list, 0, UAV_list);
//		sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//		if (sol_list[0].fitness > opt_fitness) {
//			opt_fitness = sol_list[0].fitness;
//			mode_ch_counter = 0;
//		}
//		else {
//			mode_ch_counter++;
//		}
//
//
//		for (int i = 0; i < size(node_list); i++) {
//			for (int j = 0; j < node_list[i].col_UAV_num; j++) {
//				col_num_choosed_pro_list[i][j] *= 0.92;
//			}
//		}
//		for (int i = 0; i < size((sol_list[0]).node_list); i++) {
//			col_num_choosed_pro_list[(sol_list[0]).node_list[i].id][size((sol_list[0]).node_list[i].UAV_id_list) - 1] += (sol_list[0]).node_list[i].node_size / (sol_list[0]).fitness / 5;
//		}
//
//		cout <<"Iteration "<<iteration_time << ": fitness: " << opt_fitness << endl;
//		out << opt_fitness << " ";
//		iteration_t = time(NULL);
//		t_out << difftime(iteration_t, start_t) << " ";
//	}
//	delete[] MAX_UC;
//#endif //ABC
//
//#ifdef HS
//	(*a_best_sol) = sol_list[0];
//	for (int i = 0; i < IT; i++) {
//		iteration_time++;
//		update_sol_hs( sol_list, node_list, 0, col_num_choosed_pro_list, mode_ch_counter);
//		sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//		if (a_best_sol->fitness>sol_list[0].fitness) {
//			mode_ch_counter++;
//		}
//		else {
//			(*a_best_sol) = sol_list[0];
//			mode_ch_counter = 0;
//		}
//		cout << "Iteration: " << iteration_time << " --- " << a_best_sol->fitness << endl;
//		cout << sol_list[size(sol_list) - 1].fitness << endl;
//		if (i != 0 && i % 20 == 0) {
//			out << (*a_best_sol).fitness << " ";
//			iteration_t = time(NULL);
//			t_out << difftime(iteration_t, start_t) << " ";
//		}
//	}
//#endif//HS
//
//
//#ifdef GWO
//	* a_best_sol = p_best_sol_list[0];
//	for (int i = 0; i < P_size; i++) {
//		p_best_sol_list[i] = sol_list[i];
//		if (p_best_sol_list[i].fitness > (*a_best_sol).fitness) {
//			a_best_sol[0] = p_best_sol_list[i];
//		}
//	}
//
//	for (int i = 0; i < IT; i++) {
//		iteration_time++;
//		float fit_b = a_best_sol->fitness;
//		update_sol_gwo(sol_list, node_list, 0, col_num_choosed_pro_list, mode_ch_counter);
//		sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//		if (sol_list[0].fitness < a_best_sol->fitness) {
//			mode_ch_counter++;
//		}
//		else {
//			*a_best_sol = sol_list[0];
//			mode_ch_counter = 0;
//		}
//		cout << "Iteration: " << iteration_time << " --- " << a_best_sol->fitness << endl;
//		out << a_best_sol->fitness << " ";
//		iteration_t = time(NULL);
//		t_out << difftime(iteration_t, start_t) << " ";
//	}
//#endif //GWO
//
//
//
//	delete a_best_sol;
//	delete[] p_best_sol_list;
//	delete[] V_matrix;
//	delete[] Q_table_1;
//	delete[] min_node_col;
//	out.close();
//	in.close();
//	t_out.close();
//	return 0;
//}


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

				//int kk = -1;
				//Q_vec_1* Q_matrix = new Q_vec_1[MAP_NODE_NUM];
				//for (auto sol_i : sol_list) {
				//	//kk++;
				//	//if (kk == size(sol_list)) {
				//	//	break;
				//	//}
				//	sol_i.cal_uav_path_t();
				//	sol_i.clear_ext_path_node(1);

				//	float min_size = FLT_MAX;
				//	for (auto node_ll :node_list) {
				//		if (!sol_i.check_node_id(node_ll.id )) {
				//			min_size = min(min_size,node_ll.node_size);
				//		}
				//	}
				//	//for (auto node_ll : sol_i.node_list) {
				//	//	min_size = min(min_size, node_ll.node_size);
				//	//}
				//	float t_left = 0;
				//	for (auto uav_t : sol_i.UAV_list) {
				//		for (auto uav_tt : uav_t) {
				//			t_left += T - uav_tt.t_used;
				//		}
				//	}
				//	if (t_left * UAV::eff < min_size) {
				//		continue;
				//	}

				//	float r = 0;
				//	vector<vector<Working_node>> sub_paths = sol_i.sub_path_generation();

				//	vector<vector<int>> vect_rel_num;
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
				//		vector<int> sub_rel_l;
				//		for (int node_no = 0; node_no < size(sub_paths[path_no]); node_no++) {
				//			float t_us = 0;
				//			float t_l = 0;
				//			auto& node_i = sub_paths[path_no][node_no];
				//			float f_sum = 0;
				//			float t_sum = 0;
				//			float t_min = FLT_MAX;
				//			float p_l = 0;
				//			for (int uav_iu_no = 0; uav_iu_no < size(node_i.UAV_id_list); uav_iu_no++) {
				//				int uav_iu = node_i.UAV_id_list[uav_iu_no];
				//				int path_no = node_i.Path_node_no_list[uav_iu_no];
				//				auto uav_iuu = sol_i.id_to_UAV(uav_iu);
				//				if (uav_iuu.type != node_i.sub_path_id) {
				//					continue;
				//				}
				//				float t_ee = 0;
				//				t_sum += uav_iuu.Path[path_no].t_start;
				//				if (path_no == 0) {
				//					t_ee = uav_iuu.Path_node_t_list[0];
				//					p_l += dis_matric[MAP_NODE_NUM][node_i.id];
				//				}
				//				else {
				//					t_ee = uav_iuu.Path_node_t_list[path_no] - uav_iuu.Path_node_t_list[path_no - 1];
				//					p_l += dis_matric[uav_iuu.Path[path_no - 1].id][node_i.id];
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
				//			int rel_num = 0;
				//			float size_rel = 0;

				//			set<int> type_i = node_i.type;
				//			if (node_no > 0) {
				//				for (int node_rel = 0; node_rel < node_no; node_rel++) {
				//					auto& node_ii = sol_i.node_list[node_rel];
				//					set<int> type_ii = node_ii.type;
				//					int re_flag = 0;
				//					for (auto iter = type_ii.begin(); iter != type_ii.end(); iter++) {
				//						for (auto iter_i = type_i.begin(); iter_i != type_i.end(); iter_i++) {
				//							if (*iter == *iter_i) {
				//								re_flag = 1;
				//							}
				//						}
				//					}
				//					if (re_flag == 1) {
				//						rel_num++;
				//						size_rel += node_ii.node_size;
				//					}
				//				}
				//			}
				//			sub_rel_l.push_back(rel_num);
				//		}
				//		t_eff_min.push_back(sub_t_min);
				//		t_eff_vec.push_back(sub_t_eff);
				//		path_l.push_back(sub_path_l);
				//		vect_rel_num.push_back(sub_rel_l);
				//	}

				//	for (int path_no = 0; path_no < size(sub_paths); path_no++) {
				//		float rr = 0;
				//		for (int node_no = size(sub_paths[path_no]) - 1; node_no > -1; node_no--) {

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
				//			r = node_i.node_size / path_l[path_no][node_no];
				//			rr += node_i.node_size;
				//			//r = (node_i.node_size)* t_eff_vec[path_no][node_no]/(sol_i.fitness-rr) ;// / (max_size)
				//			//r = (sol_i.fitness) / (max_size) * (node_i.node_size) / ( max_size)*t_eff_vec[node_no] ;
				//			//r = (vec_size_rel[node_no]+node_i.node_size)*t_eff_vec[node_no]/ (max_size*(vect_rel_num[node_no]+1));//
				//			//rr += node_i.node_size;
				//			Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] = r;
				//			if (node_no == size(sub_paths[path_no]) - 1) {
				//				Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1]);
				//			}
				//			else {
				//				int next = sol_i.node_list[node_no + 1].id;
				//				float max_next_q = 0;
				//				for (int next_p = 0; next_p < UAV_NUM; next_p++) {
				//					if (Q_table_1[next][node_no + 1][next_p] > max_next_q) {
				//						max_next_q = Q_table_1[next][vect_rel_num[path_no][node_no + 1]][next_p];
				//					}
				//				}
				//				Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] = Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1] + alpha_1 * (gamma * max_next_q + Q_matrix[node_i.id][node_no][size(node_i.UAV_id_list) - 1] - Q_table_1[node_i.id][vect_rel_num[path_no][node_no]][size(node_i.UAV_id_list) - 1]);
				//			}
				//		}

				//	}
				//}
				//delete[] Q_matrix;

#endif // !
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


//cal

//int main() {
//#ifdef ABC
//	IT = 400;
//	P_size = 30;
//#endif // ABC
//
//#ifdef HS
//	IT = 1500;
//	P_size = 6;
//#endif // HS
//
//
//#ifdef GWO
//	IT = 160;
//#endif // GWO
//	ifstream in;
//	in.open("./map_ini/map_10_1.txt");
//	lxw_workbook* workbook = workbook_new("cal_data_m_1.xlsx");
//	lxw_worksheet* worksheet = workbook_add_worksheet(workbook, NULL);
//	string s;
//	string x_i;
//	string y_i;
//	string size_i;
//	string col_i;
//	string type_i;
//	//srand(time(nullptr));
//	srand(42);
//	vector<Working_node> node_list;
//	//构造各任务点
//	vector<string> str_list;
//	int id_n = 0;
//	while (getline(in, s)) {
//		Stringsplit(s, " ", str_list);
//		x_i = str_list[0];
//		y_i = str_list[1];
//		size_i = str_list[2];
//		col_i = str_list[3];
//		type_i = str_list[4];
//		set<int> set_tmp;
//		if (stoi(type_i) == 0) {
//			set_tmp = { 0 };
//		}
//		if (stoi(type_i) == 1) {
//			set_tmp = { 1 };
//		}
//		if (stoi(type_i) == 2) {
//			set_tmp = { 0,1 };
//		}
//		Working_node node_tmp(stol(x_i), stol(y_i), stol(size_i), set_tmp, stoi(col_i));
//		node_tmp.id = id_n;
//		id_n++;
//		node_list.push_back(node_tmp);
//		str_list.clear();
//	}
//	//make_working_node(&node_list, coord_arr, size_arr, arr_size, co_number, type_list);
//	cal_dis(dis_matric, node_list);
//	min_node_col = new int[MAP_NODE_NUM];
//	for (int i = 0; i < MAP_NODE_NUM; i++) {
//		float size_i = node_list[i].node_size;
//		for (int j = 1; j < node_list[i].col_UAV_num; j++) {
//			if (size_i / j <= T) {
//				min_node_col[i] = j;
//				break;
//			}
//		}
//	}
//	vector<vector<UAV>> UAV_list;
//	//构造各无人机
//	make_UAV(UAV_list, UAV_type);
//	cal_max_size(node_list);
//	//define para_array
//	float CP_a[4] = { 0.65,0.75,0.85,0.95 };
//
//
//	/*int PS_a[4] = { 6,12,18,24 };*/
//	int PS_a[4] = { 10,20,30,40 };
//	float MP_a[4] = { 0.1,0.2,0.3,0.4};//0,0.15
//	float eps_a[4] = {0.05,0.2,0.35,0.5 };
//	float alp_a[4] = { 0.02,0.04,0.06,0.08 };
//	//float alp_a[4] = { 0.1,0.2,0.3,0.4};
//	float gam_a[4] = { 0.3,0.5,0.7,0.9 };
//	float LS_a[4] = {0.2,0.3,0.4,0.5};//0,0.3,
//
//
//	float IT_a[4] = { 0.2,0.3,0.4,0.5};
//
//	//define p_setting
//	int Psetting[32][8] = { {1,1,1,1,1,1,1,1},{1,1,2,2,4,4,3,3},{1,2,3,4,1,2,3,4},{1,2,4,3,4,3,1,2},{1,3,1,3,2,4,2,4},{1,3,2,4,3,1,4,2},
//		{1,4,3,2,2,3,4,1},{1,4,4,1,3,2,2,3},{2,1,3,4,3,4,2,1},{2,1,4,3,2,1,4,3},{2,2,1,1,3,3,4,4},{2,2,2,2,2,2,2,2},{2,3,3,2,4,1,1,4},{2,3,4,1,1,4,3,2},
//		{2,4,1,3,4,2,3,1},{2,4,2,4,1,3,1,3},{3,1,3,1,4,2,4,2},{3,1,4,2,1,3,2,4},{3,2,1,4,4,1,2,3},{3,2,2,3,1,4,4,1},{3,3,3,3,3,3,3,3},{3,3,4,4,2,2,1,1},
//		{3,4,1,2,3,4,1,2},{3,4,2,1,2,1,3,4},{4,1,1,4,2,3,3,2},{4,1,2,3,3,2,1,4},{4,2,3,1,2,4,1,3},{4,2,4,2,3,1,3,1},{4,3,1,2,1,2,4,3},{4,3,2,1,4,3,2,1},{4,4,3,3,1,1,2,2},{4,4,4,4,4,4,4,4}};
//	//int Psetting[32][6] = { {1,1,1,1,1,1},{1,1,2,2,4,4},{1,2,3,4,1,2},{1,2,4,3,4,3},{1,3,1,3,2,4},{1,3,2,4,3,1},
//	//	{1,4,3,2,2,3},{1,4,4,1,3,2},{2,1,3,4,3,4},{2,1,4,3,2,1},{2,2,1,1,3,3},{2,2,2,2,2,2},{2,3,3,2,4,1},{2,3,4,1,1,4},
//	//	{2,4,1,3,4,2},{2,4,2,4,1,3},{3,1,3,1,4,2},{3,1,4,2,1,3},{3,2,1,4,4,1},{3,2,2,3,1,4},{3,3,3,3,3,3},{3,3,4,4,2,2},
//	//	{3,4,1,2,3,4},{3,4,2,1,2,1},{4,1,1,4,2,3},{4,1,2,3,3,2},{4,2,3,1,2,4},{4,2,4,2,3,1},{4,3,1,2,1,2},{4,3,2,1,4,3},{4,4,3,3,1,1},{4,4,4,4,4,4}};
//	for (int P_c =0; P_c <32; P_c++) {
//		P_size = PS_a[Psetting[P_c][0] - 1];
//		PC = CP_a[Psetting[P_c][1] - 1];
//		MAX_MUTATE = MP_a[Psetting[P_c][2] - 1];
//		eps = eps_a[Psetting[P_c][3] - 1];
//		alpha_1 = alp_a[Psetting[P_c][4] - 1];
//		gamma = gam_a[Psetting[P_c][5] - 1];
//		ls_it = LS_a[Psetting[P_c][6] - 1];
//		expect_node_num = 3 * T * UAV_NUM / max_size * MAP_NODE_NUM;
//		TRY_T_MAX = round(IT_a[Psetting[P_c][7] - 1] * expect_node_num);
//		//cout << TRY_T_MAX << endl;
//		MAX_MUTATE = round(MAX_MUTATE * expect_node_num);
//		ls_it = round(ls_it * expect_node_num);
//		//cout << expect_node_num <<" "<<MAX_MUTATE << endl;
//		string TXT = to_string(P_c + 1)+"ca_";
//		for (int in_t = 0; in_t < 20; in_t++) {
//			V_matrix = new V_vec[MAP_NODE_NUM];
//			Q_table_1 = new Q_vec_1[MAP_NODE_NUM];
//			Q_table = new Q_vec_1[MAP_NODE_NUM];
//			for (int i = 0; i < MAP_NODE_NUM; i++) {
//				for (int j = 0; j < MAP_NODE_NUM; j++) {
//					V_matrix[i][j] = 0;
//				}
//			}
//
//			for (int i = 0; i < MAP_NODE_NUM; i++) {
//				for (int k = 0; k < MAP_NODE_NUM; k++) {
//					for (int j = 0; j < UAV_NUM; j++) {
//						Q_table_1[i][k][j] = 0;
//						Q_table[i][k][j] = 0;
//					}
//				}
//			}
//			Solution* p_best_sol_list = new Solution[P_size];
//			Solution* a_best_sol = new Solution;
//			ofstream out;
//			string time_ch = to_string(in_t + 1) + ".txt";
//			string out_file = TXT + time_ch;
//			out.open("./eff1/out_" + out_file);
//			ofstream t_out;
//			t_out.open("./eff1/t_" + out_file);
//			vector<float*> col_num_choosed_pro_list;
//			for (int i = 0; i < size(node_list); i++) {
//				float* col_num_choosed_pro = new float[node_list[i].col_UAV_num]();
//				for (int j = 0; j < node_list[i].col_UAV_num; j++) {
//					col_num_choosed_pro[j] = 1;
//				}
//				col_num_choosed_pro_list.push_back(col_num_choosed_pro);
//			}
//			vector<Solution> sol_list;
//			//解初始化
//			ini_sol(sol_list, node_list, UAV_list, P_size);
//			for (auto& sol : sol_list) {
//				sol.cal_fitness();
//			}
//			sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//			float opt_fitness = sol_list[0].fitness;
//			cout << "Iteration: " << 0 << " --- " << "fitness: " << opt_fitness << endl;
//			out << opt_fitness << " ";
//			//迭代开始
//			int mode_ch_counter = 0;
//			time_t start_t = time(NULL);
//			t_out << (double)0 << " ";
//
//			time_t iteration_t;
//#ifdef MA
//			int MAX_COUNTER = 60;
//			int opr = 30;
//			int INI_P_COUNTER = INT_MAX;
//			for (int i = 0; i < IT; i++) {
//				iteration_time++;
//				if ((mode_ch_counter - MAX_COUNTER) % 60 == 0 && mode_ch_counter != 0) {
//					//for (int i = 0; i < MAP_NODE_NUM; i++) {
//					//	for (int j = 0; j < UAV_NUM; j++) {
//					//		cout << Q_table[i][j] << " ";
//					//	}
//					//	cout << endl;
//					//}
//
//					//for (int i = 0; i < MAP_NODE_NUM; i++) {
//					//	for (int k = 0; k < MAP_NODE_NUM; k++) {
//					//		for (int j = 0; j < UAV_NUM; j++) {
//					//			cout << Q_table_1[i][k][j] << " ";
//					//		}
//					//		cout << endl;
//					//	}
//					//	cout << "---------------" << endl;
//					//}
//					INI_P_COUNTER = 0;
//					opr = 60;
//					cout << "| mode_1 |" << endl;
//					//for (int i = 0; i < size(node_list); i++) {
//					//	float* float_ptr = col_num_choosed_pro_list[i];
//					//	for (int k = 0; k < (node_list[i].col_UAV_num); k++) {
//					//		float_ptr[k] = 1;
//					//	}
//					//}
//					update_sol(sol_list, node_list, 0, 0);
//					sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//					if (opt_fitness < sol_list[0].fitness) {
//						mode_ch_counter = 0;
//						opt_fitness = sol_list[0].fitness;
//
//					}
//					else {
//						mode_ch_counter++;
//					}
//
//					//show pheromone 
//					//cout << "[----------------Pheromone--------------]" << endl;
//					//for (int k = 0; k < size(node_list); k++) {
//					//	for (int j = 0; j < node_list[k].col_UAV_num; j++) {
//					//		cout << col_num_choosed_pro_list[k][j] << " ";
//					//	}
//					//	cout << endl;
//					//}
//					//cout << "[----------------------------------------]" << endl;
//					//ve_col_clear(col_num_choosed_pro_list, node_list);
//				}
//				else {
//					if (INI_P_COUNTER != INT_MAX) {
//						INI_P_COUNTER++;
//					}
//					if (mode_ch_counter % opr == 0 && mode_ch_counter != 0) {
//
//						//show
//						//cout << "[----------------Pheromone--------------}" << endl;
//						//for (int i = 0; i < size(node_list); i++) {
//						//	for (int j = 0; j < node_list[i].col_UAV_num; j++) {
//						//		cout << col_num_choosed_pro_list[i][j]<<" ";
//						//	}
//						//	cout << endl;
//						//}
//						//cout << "[----------------------------------------]" << endl;
//						//ve_col_clear(col_num_choosed_pro_list, node_list);
//					}
//					update_sol(sol_list, node_list, 0, min(INI_P_COUNTER, mode_ch_counter));
// 					sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//					if (opt_fitness < sol_list[0].fitness) {
//						opt_fitness = sol_list[0].fitness;
//						mode_ch_counter = 0;
//						opr = 30;
//					}
//					else {
//						mode_ch_counter++;
//					}
//				}
//
//				cout <<"Psetting:"<<P_c+1<< " /IT: " << in_t + 1 << " /Iteration: " << i + 1 << " --- " << "fitness : " << opt_fitness << endl;
//				out << opt_fitness << " ";
//				iteration_t = time(NULL);
//				t_out << difftime(iteration_t, start_t) << " ";
//				//for (int i = 0; i < MAP_NODE_NUM; i++) {
//				//	for (int j = 0; j < 10; j++) {
//				//		cout<<V_matrix[j][i]<<" ";
//				//	}
//				//	cout << endl;
//				//}
//				if (difftime(iteration_t, start_t) > 10*expect_node_num) {
//					break;
//				}
//			}
//			worksheet_write_number(worksheet, P_c, in_t, opt_fitness, NULL);
//#endif // MA
//
//
//
//#ifdef PSO
//			* a_best_sol = p_best_sol_list[0];
//			for (int i = 0; i < P_size; i++) {
//				p_best_sol_list[i] = sol_list[i];
//				if (p_best_sol_list[i].fitness > (*a_best_sol).fitness) {
//					a_best_sol[0] = p_best_sol_list[i];
//				}
//			}
//			iteration_time = 0;
//			for (int i = 0; i < IT; i++) {
//				iteration_time++;
//				float fit_b = a_best_sol->fitness;
//				update_sol_pso(p_best_sol_list, a_best_sol, sol_list, node_list, 0, col_num_choosed_pro_list, mode_ch_counter);
//				if (fit_b == a_best_sol->fitness) {
//					mode_ch_counter++;
//				}
//				else {
//					mode_ch_counter = 0;
//				}
//				cout << "Iteration: " << iteration_time << " --- " << a_best_sol->fitness << endl;
//				out << a_best_sol->fitness << " ";
//				iteration_t = time(NULL);
//				t_out << difftime(iteration_t, start_t) << " ";
//				if (difftime(iteration_t, start_t) > 10 * expect_node_num) {
//					break;
//				}
//				opt_fitness = a_best_sol->fitness;
//			}
//			worksheet_write_number(worksheet, P_c, in_t, opt_fitness, NULL);
//
//#endif // PSO
//
//
//
//#ifdef IG
//			IT = 10000;
//			auto ini_sol = sol_list[0];
//			(*a_best_sol) = ini_sol;
//			int unchange_counter = 0;
//			for (int i = 0; i < IT; i++) {
//				iteration_time++;
//
//				update_sol_ig(a_best_sol[0], node_list, unchange_counter);
//				if (opt_fitness < a_best_sol[0].fitness) {
//					opt_fitness = a_best_sol[0].fitness;
//					unchange_counter = 0;
//				}
//				else {
//					unchange_counter++;
//				}
//
//				//cout << "Iteration: " << iteration_time << " --- " << (*a_best_sol).fitness << endl;
//				cout << "Psetting:" << P_c + 1 << " /IT: " << in_t + 1 << " /Iteration: " << i + 1 << " --- " << "fitness : " << opt_fitness << endl;
//				iteration_t = time(NULL);
//				if (difftime(iteration_t, start_t) > 10 * expect_node_num) {
//					break;
//				}
//			}
//			worksheet_write_number(worksheet, P_c, in_t, opt_fitness, NULL);
//
//
//#endif // IG
//
//
//
//#ifdef ABC
//			iteration_time = 0;
//			int unchange_counter = 0;
//			for (int i = 0; i < IT; i++) {
//				iteration_time++;
//				update_sol_abc(sol_list, node_list, unchange_counter);
//				std::sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//				if (sol_list[0].fitness > opt_fitness) {
//					opt_fitness = sol_list[0].fitness;
//				}
//				cout << "Psetting:" << P_c + 1 << " /IT: " << in_t + 1 << " /Iteration: " << i + 1 << " --- " << "fitness : " << opt_fitness << endl;
//				iteration_t = time(NULL);
//				if (difftime(iteration_t, start_t) > 10 * expect_node_num) {
//					break;
//				}
//			}
//			worksheet_write_number(worksheet, P_c, in_t, opt_fitness, NULL);
//#endif //ABC
//
//#ifdef HS
//			int unchange_counter = 0;
//			for (int i = 0; i < IT; i++) {
//				iteration_time++;
//				update_sol_hs(sol_list, node_list, unchange_counter);
//				std::sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//				if (sol_list[0].fitness > opt_fitness) {
//					opt_fitness = sol_list[0].fitness;
//					unchange_counter = 0;
//				}
//				else {
//					unchange_counter++;
//				}
//				cout << "Psetting:" << P_c + 1 << " /IT: " << in_t + 1 << " /Iteration: " << i + 1 << " --- " << "fitness : " << opt_fitness << endl;
//				iteration_t = time(NULL);
//				if (difftime(iteration_t, start_t) > 10 * expect_node_num) {
//					break;
//				}
//			}
//			worksheet_write_number(worksheet, P_c, in_t, opt_fitness, NULL);
//#endif//HS
//
//
//#ifdef GWO
//			* a_best_sol = p_best_sol_list[0];
//			for (int i = 0; i < P_size; i++) {
//				p_best_sol_list[i] = sol_list[i];
//				if (p_best_sol_list[i].fitness > (*a_best_sol).fitness) {
//					a_best_sol[0] = p_best_sol_list[i];
//				}
//			}
//
//			for (int i = 0; i < IT; i++) {
//				iteration_time++;
//				float fit_b = a_best_sol->fitness;
//				update_sol_gwo(sol_list, node_list, 0, col_num_choosed_pro_list, mode_ch_counter);
//				sort(sol_list.begin(), sol_list.end(), [](Solution& a, Solution& b) {return a.fitness > b.fitness; });
//				if (sol_list[0].fitness < a_best_sol->fitness) {
//					mode_ch_counter++;
//				}
//				else {
//					*a_best_sol = sol_list[0];
//					mode_ch_counter = 0;
//				}
//				cout << "Iteration: " << iteration_time << " --- " << a_best_sol->fitness << endl;
//				out << a_best_sol->fitness << " ";
//				iteration_t = time(NULL);
//				t_out << difftime(iteration_t, start_t) << " ";
//				}
//#endif //GWO
//			//if (difftime(iteration_t, start_t) > 10 * expect_node_num) {
//			//	break;
//			//}
//			//worksheet_write_number(worksheet, P_c, in_t, opt_fitness, NULL);
//			//for (int i = 0; i < size(node_list); i++) {
//			//	delete[] col_num_choosed_pro_list[i];
//			//}
//			//delete a_best_sol;
//			//delete[] p_best_sol_list;
//			//delete[] V_matrix;
//			//delete[] Q_table_1;
//			//delete[] Q_table;
//			out.close();
//			t_out.close();
//		}
//	}
//	workbook_close(workbook);
//	delete[] min_node_col;
//	in.close();
//	return 0;
//}
