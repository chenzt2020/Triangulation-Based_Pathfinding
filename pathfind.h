#pragma once
#include "Point.h"
#include <vector>
#include <queue>

/// <summary>
/// ��.map�ļ�ת��Ϊ.poly�ļ�
/// </summary>
void map2poly(const char* mapfile, const char* polyfile) {
	FILE* fm;
	FILE* fp;
	if (fopen_s(&fm, mapfile, "r") | fopen_s(&fp, polyfile, "w"))return;

	int n_hole;
	Point bl, tr;
	fscanf_s(fm, "%d", &n_hole);
	fscanf_s(fm, "%lf%lf%lf%lf", &bl.x, &bl.y, &tr.x, &tr.y);
	std::vector<std::vector<Point>>v;
	v.insert(v.begin(), n_hole, std::vector<Point>());

	int n_total = 4;
	for (auto& it : v) {
		int n_point;
		double x, y;
		fscanf_s(fm, "%d", &n_point);
		n_total += n_point;
		for (int j = 0; j < n_point; j++) {
			fscanf_s(fm, "%lf%lf", &x, &y);
			it.emplace_back(x, y);
		}
	}
	fprintf(fp, "%d 2 0 1\n", n_total);
	fprintf(fp, "1 %lf %lf 0\n2 %lf %lf 0\n", bl.x, bl.y, tr.x, bl.y);
	fprintf(fp, "3 %lf %lf 0\n4 %lf %lf 0\n", tr.x, tr.y, bl.x, tr.y);
	int l = 5;
	for (auto it : v) {
		for (auto jt : it) {
			fprintf(fp, "%d %lf %lf 0\n", l, jt.x, jt.y);
			l++;
		}
	}
	fprintf(fp, "%d\n1 1 2\n2 2 3\n3 3 4\n4 4 1\n", n_total);
	l = 5;
	for (auto it : v) {
		int n_point = it.size();
		for (int i = l; i < l + n_point - 1; i++)
			fprintf(fp, "%d %d %d\n", i, i, i + 1);
		fprintf(fp, "%d %d %d\n", l + n_point - 1, l + n_point - 1, l);
		l += n_point;
	}

	fprintf(fp, "%d\n", n_hole);
	for (int i = 1; i <= n_hole; i++) {
		double x, y;
		fscanf_s(fm, "%lf%lf", &x, &y);
		fprintf(fp, "%d %lf %lf\n", i, x, y);
	}
	fclose(fm);
	fclose(fp);
}

/// <summary>
/// ��©���㷨�����·��
/// </summary>
/// <param name="vt">����㵽�յ㾭������������</param>
/// <param name="start">���</param>
/// <param name="end">�յ�</param>
/// <param name="type">�����Ƿ��ӡ</param>
/// <returns></returns>
double funnel(std::vector<Triangle>& vt, Point start, Point end, const int type = 0) {
	int n = vt.size();
	if (n <= 1)return 0;
	int leng = 0;
	Point* left = new Point[n];
	Point* right = new Point[n];
	for (auto& it : vt) {
		if (!it.isCCW()) {
			Point t = it.B;
			it.B = it.C;
			it.C = t;
		}
	}
	for (auto it = vt.begin(); it < vt.end() - 1; it++) {
		Triangle t1 = *it, t2 = *(it + 1);
		int type = 0;
		if (t1.A == t2.A)type = 1;
		if (t1.A == t2.B)type = 2;
		if (t1.A == t2.C)type = 3;
		if (!type)type = 4;
		switch (type) {
		case 1:
			if (t1.B == t2.C) {
				left[leng] = t1.A;
				right[leng] = t1.B;
				leng++;
			}
			else {
				left[leng] = t1.C;
				right[leng] = t1.A;
				leng++;
			}
			break;
		case 2:
			if (t1.B == t2.A) {
				left[leng] = t1.A;
				right[leng] = t1.B;
				leng++;
			}
			else {
				left[leng] = t1.C;
				right[leng] = t1.A;
				leng++;
			}
			break;
		case 3:
			if (t1.B == t2.B) {
				left[leng] = t1.A;
				right[leng] = t1.B;
				leng++;
			}
			else {
				left[leng] = t1.C;
				right[leng] = t1.A;
				leng++;
			}
			break;
		case 4:
			left[leng] = t1.B;
			right[leng] = t1.C;
			leng++;
		}
	}

	Point s = start;
	Point minl, minr, minu, minv;
	int i = 0, minli = -1, minri;
	double min;
	std::vector<Point>path;
	path.emplace_back(s);
	left[n - 1] = end;
	right[n - 1] = end;
	auto angle = [](Point _u, Point _v) {return acos(_u * _v / hypot(_u.x, _u.y) / hypot(_v.x, _v.y)); };
	while (1) {
		if (i > n)break;
		Point l = left[i];
		Point r = right[i];
		Point u = l - s;
		Point v = r - s;
		if (minli == -1) {
			minl = l; minr = r;
			minli = minri = i;
			minu = u; minv = v;
			min = angle(minu, minv);
			i++;
			continue;
		}
		double angle_l = angle(u, minv);
		if (angle_l < min && (u ^ minv) >= 0) {
			minl = l; minli = i;
			minu = u;
			min = angle_l;
		}
		double angle_r = angle(minu, v);
		if (angle_r < min && (minu ^ v) >= 0) {
			minr = r; minri = i;
			minv = v;
			min = angle_r;
		}
		bool is_turn = 0;
		if ((u ^ minv) < 0) {
			s = minr; i = minri;
			is_turn = 1;
		}
		if ((minu ^ v) < 0) {
			s = minl; i = minli;
			is_turn = 1;
		}
		if (is_turn) {
			while (i <= n && (left[i] == s || right[i] == s))i++;
			if (i > n)break;
			minli = -1;
			path.emplace_back(s);
			continue;
		}
		i++;
	}
	if (s != end)path.emplace_back(end);
	double dist = 0;
	for (auto it = path.begin(); it < path.end() - 1; it++) {
		dist += it->distance(*(it + 1));
	}
	if (type == 1) {
		for (auto it : path) {
			printf("(%.3lf,%.3lf) ", it.x, it.y);
		}
		printf("\n\ndistance:%lf\n\n", dist);
	}
	delete[]left;
	delete[]right;
	return dist;
}

/// <summary>
/// ��©���㷨���㵱ǰ�����ε����ľ��룬�����㵽�յ�Ĺ���
/// </summary>
/// <param name="vt">����㵽��ǰ�����ξ�������������</param>
/// <param name="start">���</param>
/// <param name="end">�յ�</param>
/// <param name="dist">���� ����+���� ��С���Ǹ��㣬�������㵽��ǰ�����εľ���</param>
/// <param name="cost">���� ����+���� ��С���Ǹ��㣬����ĵ�ǰ�����ε��յ�Ĺ���</param>
void funnelcost(std::vector<Triangle> vt, Point start, Point end, double& dist, double& cost) {
	int n = vt.size();
	if (n <= 1)return;
	Triangle mid = *(vt.end() - 1);
	if (mid.havePoint(end)) {
		dist = funnel(vt, start, end);
		cost = 0;
		return;
	}
	int leng = 0;
	Point* left = new Point[n];
	Point* right = new Point[n];
	for (auto it = vt.begin(); it < vt.end() - 1; it++) {
		Triangle t1 = *it, t2 = *(it + 1);
		int type = 0;
		if (t1.A == t2.A)type = 1;
		if (t1.A == t2.B)type = 2;
		if (t1.A == t2.C)type = 3;
		if (!type)type = 4;
		switch (type) {
		case 1:
			if (t1.B == t2.C) {
				left[leng] = t1.A;
				right[leng] = t1.B;
				leng++;
			}
			else {
				left[leng] = t1.C;
				right[leng] = t1.A;
				leng++;
			}
			break;
		case 2:
			if (t1.B == t2.A) {
				left[leng] = t1.A;
				right[leng] = t1.B;
				leng++;
			}
			else {
				left[leng] = t1.C;
				right[leng] = t1.A;
				leng++;
			}
			break;
		case 3:
			if (t1.B == t2.B) {
				left[leng] = t1.A;
				right[leng] = t1.B;
				leng++;
			}
			else {
				left[leng] = t1.C;
				right[leng] = t1.A;
				leng++;
			}
			break;
		case 4:
			left[leng] = t1.B;
			right[leng] = t1.C;
			leng++;
		}
	}

	Point s, e;
	Point minl, minr, minu, minv;
	int i, minli, minri;
	double min;
	std::vector<Point>path;
	auto angle = [](Point _u, Point _v) {return acos(_u * _v / hypot(_u.x, _u.y) / hypot(_v.x, _v.y)); };
	double mindist = INF;
	for (int turn = 1; turn <= 3; turn++) {
		switch (turn) {
		case 1:
			e = mid.A;
			break;
		case 2:
			e = mid.B;
			break;
		case 3:
			e = mid.C;
		}
		s = start;
		left[n - 1] = e;
		right[n - 1] = e;
		path.clear();
		path.emplace_back(s);
		i = 0;
		minli = -1;
		while (1) {
			if (i > n)break;
			Point l = left[i];
			Point r = right[i];
			Point u = l - s;
			Point v = r - s;
			if (minli == -1) {
				minl = l; minr = r;
				minli = minri = i;
				minu = u; minv = v;
				min = angle(minu, minv);
				i++;
				continue;
			}
			double angle_l = angle(u, minv);
			if (angle_l < min && (u ^ minv) >= 0) {
				minl = l; minli = i;
				minu = u;
				min = angle_l;
			}
			double angle_r = angle(minu, v);
			if (angle_r < min && (minu ^ v) >= 0) {
				minr = r; minri = i;
				minv = v;
				min = angle_r;
			}
			bool is_turn = 0;
			if ((u ^ minv) < 0) {
				s = minr; i = minri;
				is_turn = 1;
			}
			if ((minu ^ v) < 0) {
				s = minl; i = minli;
				is_turn = 1;
			}
			if (is_turn) {
				while (i <= n && (left[i] == s || right[i] == s))i++;
				if (i > n)break;
				minli = -1;
				path.emplace_back(s);
				continue;
			}
			i++;
		}
		if (s != e)path.emplace_back(e);
		double distance = 0;
		for (auto it = path.begin(); it < path.end() - 1; it++) {
			distance += it->distance(*(it + 1));
		}
		if (mindist > distance + e.distance(end)) {
			mindist = distance + e.distance(end);
			dist = distance;
			cost = e.distance(end);
		}
#ifdef DEBUG
		for (auto it : path) {
			printf("%lf %lf\n", it.x, it.y);
		}
		printf("%lf %lf\n\n", distance, e.distance(end));
#endif // DEBUG
	}

	delete[]left;
	delete[]right;
}

/// <summary>
/// ��ȡ.poly����������������ڽӱ�����A*�㷨��©���㷨�������·��
/// </summary>
void readpoly() {

	/* 1 ��.node��.ele�ļ��ж�ȡ�����β������ڽӱ� */

	FILE* fnode;
	FILE* fele;
	if (fopen_s(&fnode, "a.1.node", "r") || fopen_s(&fele, "a.1.ele", "r"))return;

	// ��.node�ļ��ж�ȡ����
	int n_point;
	fscanf_s(fnode, "%d 2 0 1\n", &n_point);
	Point* vp = new Point[n_point];
	for (int i = 0; i < n_point; i++) {
		int a;
		double x, y;
		fscanf_s(fnode, "%d%lf%lf%d", &a, &x, &y, &a);
		vp[i] = Point(x, y);
	}

	// ��.ele�ļ��ж���������
	int n_tri;
	fscanf_s(fele, "%d 3 0\n", &n_tri);
	tri_int* vt_int = new tri_int[n_tri];
	Triangle* vt = new Triangle[n_tri];
	for (int i = 0; i < n_tri; i++) {
		int a, b, c;
		fscanf_s(fele, "%d%d%d%d", &a, &a, &b, &c);
		vt[i] = Triangle(vp[a - 1], vp[b - 1], vp[c - 1]);
		// ����Ϊ�˷���©���㷨�ļ��㣬��vt�������ζ��㰴��ʱ������
		if (!vt[i].isCCW()) {
			Point t = vt[i].B;
			vt[i].B = vt[i].C;
			vt[i].C = t;
		}
		// Ϊ�˷�������������ڽӹ�ϵ���˴���vt_int������������
		if (a > b)std::swap(a, b);
		if (b > c)std::swap(b, c);
		if (a > b)std::swap(a, b);
		vt_int[i].a = a;
		vt_int[i].b = b;
		vt_int[i].c = c;
	}
	fclose(fnode);
	fclose(fele);

	// ���������ε��ڽӹ�ϵ�������ڽӱ�map
	std::vector<std::vector<int>>map;
	map.insert(map.begin(), n_tri, std::vector<int>());
	for (int i = 0; i < n_tri; i++) {
		for (int j = i + 1; j < n_tri; j++) {
			// ��ȡ������ʱ�Ѿ��������������մ�С����������������������ڽ�ʱ������9�����
			if ((vt_int[i].a == vt_int[j].a && (vt_int[i].b == vt_int[j].b || vt_int[i].b == vt_int[j].c || vt_int[i].c == vt_int[j].b || vt_int[i].c == vt_int[j].c))
				|| (vt_int[i].a == vt_int[j].b && (vt_int[i].b == vt_int[j].c || vt_int[i].c == vt_int[j].c))
				|| (vt_int[i].b == vt_int[j].a && (vt_int[i].c == vt_int[j].b || vt_int[i].c == vt_int[j].c))
				|| (vt_int[i].b == vt_int[j].b && vt_int[i].c == vt_int[j].c))
			{
				map[i].push_back(j);
				map[j].push_back(i);
			}
		}
	}

	// ��ӡ�ڽӱ�DEBUG��
	printf("map:\n");
	int i = 0;
	for (auto it : map) {
		printf("%d:", i + 1);
		i++;
		for (auto jt : it) {
			printf(" %d", jt + 1);
		}
		printf("\n");
	}


	/* 2 ���ڽӱ����A*Ѱ· */

	Point s_point, e_point;
	s_point = Point(0, 0);
	e_point = Point(7, 10);

	// ���������յ����ĸ�������
	int s_tri, e_tri;
	for (int i = 0; i < n_tri; i++) {
		if (vt[i].havePoint(s_point)) {
			s_tri = i;
			break;
		}
	}
	for (int i = 0; i < n_tri; i++) {
		if (vt[i].havePoint(e_point)) {
			e_tri = i;
			break;
		}
	}
	printf("start triangle:%d\nend triangle:%d\n\n", s_tri + 1, e_tri + 1);

	// ��ʼ��������顢��������͸��������
	bool* vis = new bool[n_tri]();
	double* dist = new double[n_tri];
	for (int i = 0; i < n_tri; i++)
		dist[i] = INF;
	int* pre = new int[n_tri];

	// ���ȶ���A*Ѱ·
	std::priority_queue<qnode>q;
	dist[s_tri] = 0;
	while (!q.empty())q.pop();
	q.push(qnode(s_tri, 0));
	qnode tmp;
	while (!q.empty()) {
		tmp = q.top();
		q.pop();
		int u = tmp.id;
		if (u == e_tri)break;
		if (vis[u])continue;
		vis[u] = true;

		// �������㵽��ǰ���������·�������������ʹ��ۣ����㷨����Ҫ��ʱ��
		std::vector<Triangle>path_tri;
		int f = u;
		while (1) {
			if (f == s_tri)break;
			path_tri.emplace_back(vt[f]);
			f = pre[f];
		}
		path_tri.emplace_back(vt[s_tri]);
		reverse(path_tri.begin(), path_tri.end());
		for (auto v : map[u]) {
			if (vis[v])continue;
			double distv, cost;
			path_tri.emplace_back(vt[v]);
			funnelcost(path_tri, s_point, e_point, distv, cost);
			path_tri.pop_back();
			if (dist[v] > distv) {
				dist[v] = distv;
				q.push(qnode(v, dist[v] + cost));
				pre[v] = u;
			}
		}
	}

	// ��ӡ·��
	if (dist[e_tri] == INF)return;
	int* path_tri_id = new int[n_tri];
	std::vector<Triangle>path_tri;
	path_tri_id[0] = pre[e_tri];
	i = 0;
	while (1) {
		if (path_tri_id[i] == s_tri)break;
		path_tri.emplace_back(vt[path_tri_id[i]]);
		path_tri_id[i + 1] = pre[path_tri_id[i]];
		i++;
	}
	path_tri.emplace_back(vt[s_tri]);
	reverse(path_tri.begin(), path_tri.end());
	printf("path triangle:\n");
	for (; i >= 0; i--)
		printf("%d->", path_tri_id[i] + 1);
	printf("%d\n\n", e_tri + 1);
	printf("path point:\n");
	funnel(path_tri, s_point, e_point, 1);

	delete[]vp;
	delete[]vt_int;
	delete[]vt;
	delete[]vis;
	delete[]dist;
	delete[]pre;
	delete[]path_tri_id;
}