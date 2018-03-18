#include "DisjointSet.h"

DisjointSet::DisjointSet(int num_nodes) :
	m_parent(num_nodes), m_rank(num_nodes, 0)
{
	for (int i = 0; i < num_nodes; ++i) m_parent[i] = i;
}

int DisjointSet::find(int element)
{
	if (m_parent[element] == element) return element;
	//1)path compression
	return m_parent[element] = find(m_parent[element]);
}

void DisjointSet::doUnion(int a, int b)
{
	if (m_parent[a] == m_parent[b]) return;
	int fa = find(a), fb = find(b);
	//2)union by rank
	if (m_rank[fa] < m_rank[fb]) {
		m_parent[fa] = fb;
	}
	else {
		m_parent[fb] = fa;
		if (m_rank[fa] == m_rank[fb]) m_rank[fa]++;
	}
}
