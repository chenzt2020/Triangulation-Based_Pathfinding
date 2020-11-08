#pragma once
#include "Point.h"
#include <vector>
#include <queue>
#include <memory>

/// <summary>
/// 将.map文件转换为.poly文件
/// </summary>
void map2poly(const char* mapfile, const char* polyfile) {
	FILE* fm;
	FILE* fp;
	if (fopen_s(&fm, mapfile, "r") | fopen_s(&fp, polyfile, "w"))return;

	int n_hole;
	Pointd bl, tr;
	fscanf_s(fm, "%d", &n_hole);
	fscanf_s(fm, "%lf%lf%lf%lf", &bl.x, &bl.y, &tr.x, &tr.y);
	std::vector<std::vector<Pointd>>v;
	v.insert(v.begin(), n_hole, std::vector<Pointd>());

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
		for (auto const& jt : it) {
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
/// 用漏斗算法求最短路径
/// </summary>
/// <param name="vt">从起点到终点经过的三角网格</param>
/// <param name="start">起点</param>
/// <param name="end">终点</param>
/// <param name="type">控制是否打印</param>
/// <returns></returns>
template<typename Scalar>
Scalar funnel(std::vector<Triangle<Scalar>>& vt, const Point<Scalar>& start, const Point<Scalar>& end, const int type = 0) {
	int n = vt.size();
	if (n <= 1)return 0;
	for (auto& it : vt) {
		if (!it.isCCW()) {
			Point<Scalar> t = it.B;
			it.B = it.C;
			it.C = t;
		}
	}

	int leng = 0;
	std::unique_ptr<Point<Scalar>[]>left(new Point<Scalar>[n]);
	std::unique_ptr<Point<Scalar>[]>right(new Point<Scalar>[n]);
	for (auto it = vt.begin(); it < vt.end() - 1; it++) {
		Triangle<Scalar> t1 = *it, t2 = *(it + 1);
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

	Point<Scalar> s = start;
	Point<Scalar> minl, minr, minu, minv;
	int i = 0, minli = -1, minri;
	Scalar min;
	std::vector<Point<Scalar>>path;
	path.emplace_back(s);
	left[n - 1] = end;
	right[n - 1] = end;
	auto angle = [](Point<Scalar> _u, Point<Scalar> _v) {return acos(_u * _v / hypot(_u.x, _u.y) / hypot(_v.x, _v.y)); };
	while (1) {
		if (i > n)break;
		Point<Scalar> l = left[i];
		Point<Scalar> r = right[i];
		Point<Scalar> u = l - s;
		Point<Scalar> v = r - s;
		if (minli == -1) {
			minl = l; minr = r;
			minli = minri = i;
			minu = u; minv = v;
			min = angle(minu, minv);
			i++;
			continue;
		}
		Scalar angle_l = angle(u, minv);
		if (angle_l < min && (u ^ minv) >= 0) {
			minl = l; minli = i;
			minu = u;
			min = angle_l;
		}
		Scalar angle_r = angle(minu, v);
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
	Scalar dist = 0;
	for (auto it = path.begin(); it < path.end() - 1; it++) {
		dist += it->distance(*(it + 1));
	}
	if (type == 1) {
		for (auto const& it : path) {
			printf("(%.3lf,%.3lf) ", it.x, it.y);
		}
		printf("\n\ndistance:%lf\n\n", dist);
	}
	return dist;
}

/// <summary>
/// 用漏斗算法计算当前三角形到起点的距离，并计算到终点的估价
/// </summary>
/// <param name="vt">从起点到当前三角形经过的三角网格</param>
/// <param name="start">起点</param>
/// <param name="end">终点</param>
/// <param name="dist">按照 距离+估价 最小的那个点，计算的起点到当前三角形的距离</param>
/// <param name="cost">按照 距离+估价 最小的那个点，计算的当前三角形到终点的估价</param>
template<typename Scalar>
void funnelcost(std::vector<Triangle<Scalar>>& vt, const Point<Scalar>& start, const Point<Scalar>& end, Scalar& dist, Scalar& cost) {
	int n = vt.size();
	if (n <= 1)return;

	//若终点在当前三角形，则直接计算最短路径
	Triangle<Scalar> mid = *(vt.end() - 1);
	if (mid.havePoint(end)) {
		dist = funnel(vt, start, end);
		cost = 0;
		return;
	}

	int leng = 0;
	std::unique_ptr<Point<Scalar>[]>left(new Point<Scalar>[n]);
	std::unique_ptr<Point<Scalar>[]>right(new Point<Scalar>[n]);
	for (auto it = vt.begin(); it < vt.end() - 1; it++) {
		Triangle<Scalar> t1 = *it, t2 = *(it + 1);
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

	Point<Scalar> s, e;
	Point<Scalar> minl, minr, minu, minv;
	int i, minli, minri;
	Scalar min;
	std::vector<Point<Scalar>>path;
	auto angle = [](Point<Scalar> _u, Point<Scalar> _v) {return acos(_u * _v / hypot(_u.x, _u.y) / hypot(_v.x, _v.y)); };
	Scalar mindist = INF;
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
			Point<Scalar> l = left[i];
			Point<Scalar> r = right[i];
			Point<Scalar> u = l - s;
			Point<Scalar> v = r - s;
			if (minli == -1) {
				minl = l; minr = r;
				minli = minri = i;
				minu = u; minv = v;
				min = angle(minu, minv);
				i++;
				continue;
			}
			Scalar angle_l = angle(u, minv);
			if (angle_l < min && (u ^ minv) >= 0) {
				minl = l; minli = i;
				minu = u;
				min = angle_l;
			}
			Scalar angle_r = angle(minu, v);
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
		Scalar distance = 0;
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
	}

/// <summary>
/// 读取.poly，构造三角网格的邻接表，并用A*算法和漏斗算法计算最短路径
/// </summary>
void readpoly() {

	/* 1 从.node和.ele文件中读取三角形并构建邻接表 */

	FILE* fnode;
	FILE* fele;
	if (fopen_s(&fnode, "a.1.node", "r") || fopen_s(&fele, "a.1.ele", "r"))return;

	// 从.node文件中读取顶点
	int n_point;
	fscanf_s(fnode, "%d 2 0 1\n", &n_point);
	std::unique_ptr<Pointd[]>vp(new Pointd[n_point]);
	for (int i = 0; i < n_point; i++) {
		int a;
		double x, y;
		fscanf_s(fnode, "%d%lf%lf%d", &a, &x, &y, &a);
		vp[i] = Pointd(x, y);
	}

	// 从.ele文件中读入三角形
	int n_tri;
	fscanf_s(fele, "%d 3 0\n", &n_tri);
	std::unique_ptr<tri_int[]>vt_int(new tri_int[n_tri]);
	std::unique_ptr<Triangled[]>vt(new Triangled[n_tri]);
	for (int i = 0; i < n_tri; i++) {
		int a, b, c;
		fscanf_s(fele, "%d%d%d%d", &a, &a, &b, &c);
		vt[i] = Triangled(vp[a - 1], vp[b - 1], vp[c - 1]);
		// 方便为了方便漏斗算法的计算，将vt中三角形顶点按逆时针排序
		if (!vt[i].isCCW()) {
			Pointd t = vt[i].B;
			vt[i].B = vt[i].C;
			vt[i].C = t;
		}
		// 为了方便计算三角形邻接关系，此处对vt_int顶点索引排序
		if (a > b)std::swap(a, b);
		if (b > c)std::swap(b, c);
		if (a > b)std::swap(a, b);
		vt_int[i].a = a;
		vt_int[i].b = b;
		vt_int[i].c = c;
	}
	fclose(fnode);
	fclose(fele);

	// 计算三角形的邻接关系并存入邻接表map
	std::vector<std::vector<int>>map;
	map.insert(map.begin(), n_tri, std::vector<int>());
	for (int i = 0; i < n_tri; i++) {
		for (int j = i + 1; j < n_tri; j++) {
			// 读取三角形时已经将顶点索引按照从小到大排序，因此两个三角形邻接时，共有9种情况
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

	// 打印邻接表（DEBUG）
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

	/* 2 对邻接表进行A*寻路 */

	Pointd s_point, e_point;
	printf("start point (2 double):");
	scanf_s("%lf %lf", &s_point.x, &s_point.y);
	printf("end point (2 double):");
	scanf_s("%lf %lf", &e_point.x, &e_point.y);

	// 计算起点和终点在哪个三角形
	int s_tri = -1, e_tri = -1;
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
	if (s_tri == -1 || e_tri == -1) {
		printf("The start or end point is not on the map.\n");
		return;
	}
	else printf("start triangle:%d\nend triangle:%d\n\n", s_tri + 1, e_tri + 1);

	// 初始化标记数组、距离数组和父结点数组
	std::unique_ptr<bool[]>vis(new bool[n_tri]());
	std::unique_ptr<double[]>dist(new double[n_tri]);
	for (int i = 0; i < n_tri; i++)
		dist[i] = INF;
	std::unique_ptr<int[]>pre(new int[n_tri]);

	// 优先队列A*寻路
	std::priority_queue<qnode<double>>q;
	dist[s_tri] = 0;
	while (!q.empty())q.pop();
	q.push(qnode<double>(s_tri, 0));
	qnode<double> tmp;
	while (!q.empty()) {
		tmp = q.top();
		q.pop();
		int u = tmp.id;
		if (u == e_tri)break; // 注释后可以用性能换精度
		if (vis[u])continue;
		vis[u] = true;

		// 构造从起点到当前点的三角形路径，并计算距离和代价，是算法的主要耗时点
		std::vector<Triangled>path_tri;
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
				q.push(qnode<double>(v, dist[v] + cost));
				pre[v] = u;
			}
		}
	}

	// 打印路径
	if (dist[e_tri] == INF) {
		return;
	}
	std::unique_ptr<int[]>path_tri_id(new int[n_tri]); // 可用vector代替
	std::vector<Triangled>path_tri;
	path_tri_id[0] = e_tri;
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
	for (; i >= 1; i--)
		printf("%d->", path_tri_id[i] + 1);
	printf("%d\n\n", e_tri + 1);
	printf("path point:\n");
	funnel(path_tri, s_point, e_point, 1);
}