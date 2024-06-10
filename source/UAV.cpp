#include "UAV.h"
#include "Working_node.h"
#include <iostream>
#include "map_config.h"
extern float dis_matric[MAP_NODE_NUM+1][MAP_NODE_NUM];

using namespace std;
float UAV::cal_t_go_to(const Working_node& node) const{
	if (size(Path) == 0) {
		//
		return dis_matric[MAP_NODE_NUM][node.id]/speed;
	}
	else {
		Path_node node_last = *(Path.end() - 1);
		float dis = dis_matric[node_last.id][node.id];
		return t_used + dis / speed;
	}
}

float UAV::cal_t_gone(const Working_node& node) const {
	if (size(Path) == 1) {
		//
		return dis_matric[MAP_NODE_NUM][node.id] / speed;
	}
	else {
		Path_node node_last = *(Path.end() - 2);
		float dis = dis_matric[node_last.id][node.id];
		return t_used + dis / speed;
	}
}

void UAV::Path_node_no_assign() {
	for (int i = 0; i < size(Path); i++) {
		Path[i].node_no = i;
	}
}

//you should make sure the t_used is cleared first, before use this function
float UAV::cal_t_to_node_no_j(int j) {
	if (j == 0) {
		return dis_matric[MAP_NODE_NUM][Path[j].id] / speed;
	}
	else {
		Path_node node_last = Path[j-1];
		float dis = dis_matric[node_last.id][Path[j].id];
		return t_used + dis / speed;
	}
}

//not finish yet
float UAV::cal_t_to_node_to_insert(Working_node node_insert, int insert_place) {
	if (insert_place == 0) {
		return dis_matric[MAP_NODE_NUM][node_insert.id]/speed;
	}
	else {
		float t_used_f = Path_node_t_list[insert_place-1];
		int node_f_id = Path[insert_place - 1].id;
		return dis_matric[node_f_id][node_insert.id] / speed + t_used_f;
	}
}

float UAV::ret_PathLength_from_node_j() {
	if (size(Path)==tmp_node_no) {
		return 0;
	}
	if (tmp_node_no == 0) {
		return dis_matric[MAP_NODE_NUM][Path[tmp_node_no].id];
	}
	return dis_matric[Path[tmp_node_no - 1].id][Path[tmp_node_no].id];
}
//
float UAV::ret_PathLength_to_node_insert(Working_node node_insert) {
	if (tmp_node_no == 0) {
		return dis_matric[MAP_NODE_NUM][node_insert.id];
	}
	else {
		//cout << "tmp: " << tmp_node_no << endl;
		return dis_matric[Path[tmp_node_no - 1].id][node_insert.id];
	}
}

float UAV::ret_PathLength_from_node_insert(Working_node node_insert) {
	if (size(Path) == tmp_node_no) {
		return 0;
	}
	return dis_matric[node_insert.id][Path[tmp_node_no].id];
}

void UAV::clear_t_used() {
	t_used = 0;
}

void UAV::erase_node_i_in_path(int idx) {
	int i = 0;
	int flag = 0;
	for (; i < size(Path); i++) {
		if (Path[i].id == idx) {
			flag = 1;
			break;
		}
	}
	if (flag == 0) {
		cout << "cant find node to erase" << endl;
	}
	Path.erase(Path.begin() + i);
}
