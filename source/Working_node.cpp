#include "Working_node.h"
#include <vector>
#include"UAV.h"
#include <algorithm>
using namespace std;

bool is_faster(UAV&, UAV&);
//确定末尾工作点完成时间
Working_node::time_count Working_node::cal_time_end(vector<UAV>& UAVs_allowed) {
	for (int i = 0; i < size(UAVs_allowed); i++) {
		UAVs_allowed[i].t_used = UAVs_allowed[i].cal_t_gone(*this);
		UAVs_allowed[i].Path[Path_node_no_list[i]].t_start = UAVs_allowed[i].t_used;
		UAVs_allowed[i].tmp_node_no = Path_node_no_list[i];
	}
	sort(UAVs_allowed.begin(), UAVs_allowed.end(), is_faster);
	float t_end_tmp;
	float t_mark = node_size / UAV::eff;
	int col_num_re = 1;
	t_end_tmp = t_mark + UAVs_allowed[0].t_used;
	int flag_f = 0;
	t_mark = node_size / UAV::eff+ UAVs_allowed[0].t_used;
	for (auto& UAV : UAVs_allowed) {
		if (flag_f == 0) {
			flag_f++;
			continue;
		}
		if (t_end_tmp <= UAV.t_used) {
			break;
		}
		t_mark += UAV.t_used;
		col_num_re += 1;
		t_end_tmp = t_mark / col_num_re;
	}
	for (int i = 0; i < col_num_re; i++) {
		UAVs_allowed[i].t_used = t_end_tmp;
		UAVs_allowed[i].Path[UAVs_allowed[i].tmp_node_no].t_end = UAVs_allowed[i].t_used;
	}
	for (int j = col_num_re; j < size(UAVs_allowed); j++) {
		UAVs_allowed[j].Path[UAVs_allowed[j].tmp_node_no].t_end = UAVs_allowed[j].t_used;
	}
	return t_end_tmp;
}

Working_node::time_count Working_node::cal_time_end_with_complete_Path(vector<UAV>& col_uav_list) {
	for (int i = 0; i < size(col_uav_list); i++) {
		col_uav_list[i].tmp_node_no = Path_node_no_list[i];
		col_uav_list[i].t_used = col_uav_list[i].cal_t_to_node_no_j(Path_node_no_list[i]);
		col_uav_list[i].Path[Path_node_no_list[i]].t_start = col_uav_list[i].t_used;
	}
	sort(col_uav_list.begin(), col_uav_list.end(),is_faster);
	float t_end_tmp;
	float t_mark = node_size / UAV::eff;
	int col_num_re = 1;
	t_end_tmp = t_mark+ col_uav_list[0].t_used; //
	t_mark = t_mark + col_uav_list[0].t_used;
	int flag_f = 0;
	for (auto& UAV : col_uav_list) {
		if (flag_f == 0) {
			flag_f++;
			continue;
		}
		if (t_end_tmp <= UAV.t_used) {
			break;
		}
		t_mark += UAV.t_used;
		col_num_re += 1;
		t_end_tmp = t_mark / col_num_re;
	}
	for (int i = 0; i < col_num_re; i++) {
		col_uav_list[i].t_used = t_end_tmp;
		col_uav_list[i].Path[col_uav_list[i].tmp_node_no].t_end = col_uav_list[i].t_used;
	}
	for (int j = col_num_re; j < size(col_uav_list); j++) {
		col_uav_list[j].Path[col_uav_list[j].tmp_node_no].t_end = col_uav_list[j].t_used;
	}
	return t_end_tmp;
}