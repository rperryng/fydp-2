#pragma once

#include "DisjointSet.h"

#include <utility>

class ComponentPolarizer {
public:
	ComponentPolarizer(UINT16 *grid, int gridHeight, int gridWidth);
	void Polarize(int x, int y, int cutoffY);

private:
	// Member Variables
	UINT16 *m_grid;
	int m_gridHeight;
	int m_gridWidth;
	DisjointSet m_disjointSet;

	// Functions
	UINT16 valueAt(int x, int y);
};
