# Triangulation-Based_Pathfinding

三角剖分A*寻路

Pathfinding based on triangulation and A* algorithm

## [算法原理](https://chenzt2020.github.io/_posts/2020-07-23-%E5%B9%B3%E9%9D%A2%E5%87%A0%E4%BD%95%E6%9C%80%E7%9F%AD%E8%B7%AF/)

- 通过三角剖分将连续平面转换为三角网格，用到了开源库 [Triangle](http://www.cs.cmu.edu/~quake/triangle.html)
- 在三角网格上通过 A* 算法，找到从起点到终点经过的三角网格路径
- 在三角网格路径上应用漏斗算法，求得从起点到终点的最短路径

## 使用说明

### 1. 函数介绍

- `map2poly` 将输入文件转换为 .poly 文件
- `funnel` 用漏斗算法，求出三角网格路径上起点到终点的最短路径
- `readpoly` 读取 .poly ，构造三角网格的邻接表，并用 A* 算法和拐角点法计算最短路径

### 2. 示例

test.txt

    3           # 障碍的个数
    -1 -1       # 地图边界为(-1,1)~(8,11)
    11 8
    3           # 第1个障碍边界点的个数
    2 1         # 障碍边界点(1,2)
    4 1
    2 3
    4
    1 6
    2 6
    2 7
    1 7
    3
    8 4
    9 6
    7 5

    3 1.5       # 每个障碍内部取1点
    1.5 6.5
    8 5

![map_0](https://s1.ax1x.com/2020/07/23/UXSUdx.png)

键盘输入

    0 0         # 起点和终点坐标 
    10 7

运行结果

    ...
    map:
    1: 2 3
    2: 1 10
    3: 1 5 6
    4: 5 7
    5: 3 4
    6: 3 8
    7: 4 17
    8: 6 14 18
    9: 10 11 12
    10: 2 9
    11: 9 16
    12: 9 18
    13: 14 15
    14: 8 13
    15: 13 16 17
    16: 11 15
    17: 7 15
    18: 8 12
    start point (2 double):0 0
    end point (2 double):10 7
    start triangle:1
    end triangle:15

    path triangle:
    1->3->6->8->14->13->15

    path point:
    (0.000,0.000) (2.000,3.000) (10.000,7.000)

    distance:12.549823

![map](https://s1.ax1x.com/2020/07/23/UXSao6.png)
