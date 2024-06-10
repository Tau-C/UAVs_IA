#pragma once
#include <vector>
#include "solution.h"
void ini_sol(std::vector<Solution>& sol_list, std::vector<Working_node>& node_list, std::vector<std::vector<UAV>> &UAV_list, int );

Solution random_generate_sol(std::vector<Working_node>&, std::vector<std::vector<UAV>>&);

Solution heuristic_generate_sol(std::vector<Working_node>& node_list, std::vector<std::vector<UAV>>& UAV_list, int rnd_flag);

Solution NEH_generate_sol(std::vector<Working_node>& node_list, std::vector<std::vector<UAV>>& UAV_list, int rnd_flag);


void complement(Solution& sol_tmp, std::vector<Working_node> node_list);