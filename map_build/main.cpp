#include <string>
#include <fstream>
#include <iostream>
#include "Node.h"
#include <vector>
#include <ctime>
//#define MAP_SIZE 1800
//#define MAX_LEN 300
//#define MIN_LEN 50
//#define TYPE_NUM 2

#define MAP_SIZE 200
#define MAX_LEN 10
#define MIN_LEN 5
#define TYPE_NUM 2
using namespace std;

Node make_node(float px_1, float px_2, float py_1, float py_2) {
	float min_px = min(px_1, px_2);
	float max_px = max(px_1, px_2);
	float min_py = min(py_1, py_2);
	float max_py = max(py_1, py_2);
	Point p1(min_px, min_py);
	Point p2(max_px, min_py);
	Point p3(max_px, max_py);
	Point p4(min_px, max_py);
	return Node(p1, p2, p3, p4);
}

int main(int argc, char* argv[]) {
	if (argc != 3) {
		cout << "input number is not right" << endl;
	}
	int num = stoi(argv[1]);
	vector<Node> node_list;
	srand(time(0));
	ofstream out;
	out.open(argv[2]);
	float x_1 = MAP_SIZE *(float)rand() /(float) RAND_MAX;
	float y_1 = MAP_SIZE * (float)rand() / (float)RAND_MAX;
	int l_1 = MIN_LEN + rand() % (MAX_LEN - MIN_LEN);
	int l_2 = MIN_LEN + rand() % (MAX_LEN - MIN_LEN);
	float px_1 = x_1;
	float px_2 = x_1 + l_1;
	float py_1 = y_1;
	float py_2 = y_1 + l_2;
	node_list.push_back(make_node(px_1, px_2, py_1,py_2));
	node_list[size(node_list) - 1].l_1 = l_1;
	node_list[size(node_list) - 1].l_2 = l_2;
	node_list[size(node_list)-1].type = rand() % (TYPE_NUM + 1);
	for (int i = 1; i < num; i++) {
		while(true){
			x_1 = MAP_SIZE * (float)rand() / (float)RAND_MAX;
			y_1 = MAP_SIZE * (float)rand() / (float)RAND_MAX;
			l_1 = MIN_LEN + rand() % (MAX_LEN - MIN_LEN);
			l_2 = MIN_LEN + rand() % (MAX_LEN - MIN_LEN);
			px_1 = x_1;
			px_2 = x_1 + l_1;
			py_1 = y_1;
			py_2 = y_1 + l_2;
			int flag = 0;
			for (int j = 0; j < i; j++) {
				flag = 0;
				if (px_2 <node_list[j].p1.x) {
					flag = 1;
					continue;
				}
				if (px_1 > node_list[j].p2.x) {
					flag = 1;
					continue;
				}
				if (py_2 < node_list[j].p1.y) {
					flag = 1;
					continue;
				}
				if (py_1 > node_list[j].p3.y) {
					flag = 1;
					continue;
				}
				if (flag == 0) {
					break;
				}
			}
			if (flag == 0) {
				continue;
			}
			if (flag == 1) {
				break;
			}
		}
		node_list.push_back(make_node(px_1, px_2, py_1, py_2));
		node_list[size(node_list) - 1].l_1 = l_1;
		node_list[size(node_list) - 1].l_2 = l_2;
		node_list[size(node_list) - 1].type = rand() % (TYPE_NUM + 1);
	}
	for (int i = 0; i < size(node_list); i++) {
		out <<node_list[i].center.x<<" "<<node_list[i].center.y<<" "<<node_list[i].size<<" "<<node_list[i].col_num<<" " <<node_list[i].type<<" "\
			<< node_list[i].p1.x << " " << node_list[i].p1.y << " " << node_list[i].l_1 << " " << node_list[i].l_2 << endl;
	}
	out.close();
	return 0;
}