#include <iostream>
#include "pathfind.h"

int main()
{
	map2poly("test.txt", "a.poly");
	system("triangle -P a.poly");
	readpoly();
}