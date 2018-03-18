#pragma once

#include "basetsd.h"
#include "DisjointSet.h"

#include <utility>

#define DEFAULT_POLARIZATION_THRESHOLD 25

class ComponentPolarizer {
private:

	// Member Variables
	UINT16 *m_grid;
	int m_gridHeight;
	int m_gridWidth;
	DisjointSet m_disjointSet;

	// Functions
	UINT16 valueAt(int x, int y);

public:
	ComponentPolarizer(UINT16 *grid, int gridHeight, int gridWidth);
	void Polarize(int x, int y, int threshold = DEFAULT_POLARIZATION_THRESHOLD);
};
