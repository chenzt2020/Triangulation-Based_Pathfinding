#pragma once
#include <cmath>
#define INF 1e20
#define EPS 1e-7

int sgn(double x) {
	if (fabs(x) < EPS)return 0;
	if (x < 0)return -1;
	else return 1;
}

class Point
{
public:
	double x;
	double y;
	Point() {}
	Point(double _x, double _y) :x(_x), y(_y) {}
	Point operator - (const Point& b) const {
		return Point(x - b.x, y - b.y);
	}
	bool operator == (const Point& b)const {
		// 由于精度原因，导致return x == b.x && y == b.y;
		// 并非每次运算结果都相同，因此添加了精度容错
		return sgn(x - b.x) == 0 && sgn(y - b.y) == 0;
	}
	bool operator != (const Point& b)const {
		return sgn(x - b.x) || sgn(y - b.y);
	}
	// 叉积
	double operator ^(const Point& b)const {
		return x * b.y - y * b.x;
	}
	// 点积
	double operator *(const Point& b)const {
		return x * b.x + y * b.y;
	}
	// 两点间距离
	double distance(const Point& b)const {
		return hypot(x - b.x, y - b.y);
	}
};

class Triangle
{
public:
	Point A;
	Point B;
	Point C;
	Triangle() {}
	Triangle(Point _A, Point _B, Point _C) :A(_A), B(_B), C(_C) {}
	// 判断三角形内是否有点p
	bool havePoint(const Point p);
	// 判断三角形顶点是否呈逆时针排列
	bool isCCW() {
		return ((B - A) ^ (C - A)) > 0;
	};
};
bool Triangle::havePoint(const Point p) {
	Point v0 = C - A;
	Point v1 = B - A;
	Point v2 = p - A;
	double dot00 = v0 * v0;
	double dot01 = v0 * v1;
	double dot02 = v0 * v2;
	double dot11 = v1 * v1;
	double dot12 = v1 * v2;
	double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
	return (u >= 0) && (v >= 0) && (u + v <= 1);
}

struct tri_int {
	int a, b, c;
};

struct qnode {
	int id;
	double c;
	qnode() {}
	qnode(int _id, double _c) :id(_id), c(_c) {}
	bool operator <(const qnode& b)const {
		return c > b.c;
	}
};