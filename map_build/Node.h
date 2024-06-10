#pragma once
#include <utility>
#define SINGLE_A 1500
class Point {
public:
	float x;
	float y;
	Point() = default;
	Point(float x_i, float y_i):x(x_i),y(y_i){}
};
class Node {
public:
	Point p1;
	Point p2;
	Point p3;
	Point p4;
	Point center;
	int l_1=0;
	int l_2=0;
	int size;
	int col_num;
	int type;
	Node() = default;
	Node(const Point& p1_i, const Point& p2_i, const Point& p3_i, const Point& p4_i):p1(p1_i),\
		p2(p2_i), p3(p3_i), p4(p4_i), size((p2.x - p1.x) * (p4.y - p1.y)), col_num(std::max((int)size/SINGLE_A,1)), center((p1.x+p2.x)/2, (p1.y + p3.y) / 2) {}
};