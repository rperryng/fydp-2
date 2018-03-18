#pragma once

#include <vector>

class DisjointSet {
public:
	DisjointSet(int numNodes);
	int find(int element);
	void doUnion(int a, int b);

private:
	std::vector<int> m_parent, m_rank;
};