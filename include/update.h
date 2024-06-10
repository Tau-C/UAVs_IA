#pragma once
#include<vector>
#include"solution.h"

void update_sol(std::vector<Solution>& sol_list, std::vector<Working_node>&, int, int unchange_counter);
std::vector<Solution> cross_over(std::vector<Solution> ps, int, int);
std::vector<Solution> parent_gen(std::vector<Solution>& sol_list, int, int);
std::vector<Working_node> rnd_gen_sub_wn_list(std::vector<Working_node>& wn_list, int, int);
std::vector<Solution> cross_over_r(std::vector<Solution> ps, int);
void update_sol_pso(Solution*, Solution*, std::vector<Solution>& sol_list, std::vector<Working_node>&, int, std::vector<float*>&, int unchange_counter);

void update_sol_ig(Solution& ini_sol, std::vector<Working_node>&,int unchange_counter);

void update_sol_abc(std::vector<Solution>& sol_list, std::vector<Working_node>&, int unchange_counter);

void update_sol_hs(std::vector<Solution>& sol_list, std::vector<Working_node>&, int unchange_counter);

void update_sol_gwo(std::vector<Solution>& sol_list, std::vector<Working_node>&, int unchange_counter);
int ret_GN(int N);

