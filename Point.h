#pragma once
#include <iostream>
#include <cmath>
#define INF 1e20
#define EPS 1e-7


template<typename Scalar>
inline int sgn(const Scalar& x) {
	if (fabs(x) < EPS)return 0;
	if (x < 0)return -1;
	else return 1;
}

template<typename Scalar>
class Point {
public:
	Scalar x;
	Scalar y;
	Point() {}
	Point(const Scalar& _x, const Scalar& _y) :x(_x), y(_y) {}
	Point operator - (const Point& b) const {
		return Point(x - b.x, y - b.y);
	}
	bool operator ==(const Point& b) const {
		// 由于精度原因，导致return x == b.x && y == b.y;
		// 并非每次运算结果都相同，因此添加了精度容错
		return sgn(x - b.x) == 0 && sgn(y - b.y) == 0;
	}
	bool operator !=(const Point& b) const {
		return !(*this == b);
	}
	// 叉积
	Scalar operator ^(const Point& b)const {
		return x * b.y - y * b.x;
	}
	// 点积
	Scalar operator *(const Point& b)const {
		return x * b.x + y * b.y;
	}
	// 两点间距离
	Scalar distance(const Point& b)const {
		return hypot(x - b.x, y - b.y);
	}
};
typedef Point<double>Pointd;

template<typename Scalar>
class Triangle
{
public:
	Point<Scalar> A;
	Point<Scalar> B;
	Point<Scalar> C;
	Triangle() {}
	Triangle(const Point<Scalar>& _A, const Point<Scalar>& _B, const Point<Scalar>& _C) :A(_A), B(_B), C(_C) {}
	friend std::ostream& operator <<(std::ostream& os, const Triangle& t) {
		os << "A:(" << t.A.x << "," << t.A.y << ") "
			<< "B:(" << t.B.x << "," << t.B.y << ") "
			<< "C:(" << t.C.x << "," << t.C.y << ")\n";
		return os;
	}
	// 判断三角形内是否有点p
	bool havePoint(const Point<Scalar>& p);
	bool havePoint2(const Point<Scalar>& p);
	// 判断三角形顶点是否呈逆时针排列
	bool isCCW() {
		return ((B - A) ^ (C - A)) > 0;
	};
};
typedef Triangle<double>Triangled;

struct tri_int {
	int a, b, c;
};

template<typename Scalar>
struct qnode {
	int id;
	Scalar c;
	qnode() {}
	qnode(int _id, const Scalar& _c) :id(_id), c(_c) {}
	bool operator <(const qnode& b)const {
		return c > b.c;
	}
};

template<typename Scalar>
bool Triangle<Scalar>::havePoint(const Point<Scalar>& p) {
	Point<Scalar> PA = A - p;
	Point<Scalar> PB = B - p;
	Point<Scalar> PC = C - p;
	Scalar v0 = PA ^ PB;
	Scalar v1 = PB ^ PC;
	Scalar v2 = PC ^ PA;
	return (v0 >= 0 && v1 >= 0 && v2 >= 0) || (v0 <= 0 && v1 <= 0 && v2 <= 0);
}

template<typename Scalar>
bool Triangle<Scalar>::havePoint2(const Point<Scalar>& p) {
	Point<Scalar> v0 = C - A;
	Point<Scalar> v1 = B - A;
	Point<Scalar> v2 = p - A;
	Scalar dot00 = v0 * v0;
	Scalar dot01 = v0 * v1;
	Scalar dot02 = v0 * v2;
	Scalar dot11 = v1 * v1;
	Scalar dot12 = v1 * v2;
	double tmp = 1 / (dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * tmp;
	if (u < 0 || u > 1)return 0;
	double v = (dot00 * dot12 - dot01 * dot02) * tmp;
	return (v >= 0) && (u + v <= 1);
}
