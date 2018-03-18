#include "ComponentPolarizer.h"

ComponentPolarizer::ComponentPolarizer(UINT16 *grid, int gridHeight, int gridWidth) {
	m_grid = grid;
	m_gridHeight = gridHeight;
	m_gridWidth = gridWidth;
	m_disjointSet = DisjointSet(gridHeight * gridWidth);
}

UINT16 ComponentPolarizer::valueAt(int x, int y) {
	return m_grid[(y * m_gridWidth) + x];
}

void ComponentPolarizer::Polarize(int componentX, int componentY, int threshold) {
	for (int i = 1; i < m_gridWidth; i++) {
		if (abs(valueAt(i, 0) - valueAt(i-1, 0)) < threshold) {
			m_disjointSet.doUnion(i, i-1);
		}
	}

	for (int i = 1; i < m_gridHeight; i++) {
		if (abs(valueAt(0, i) - valueAt(0, i - 1)) < threshold) {
			m_disjointSet.doUnion(i*m_gridWidth, (i - 1)*m_gridWidth);
		}
	}

	for (int i = 1; i < m_gridHeight; i++) {
		for (int j = 1; j < m_gridWidth; j++) {
			if (abs(valueAt(j, i) - valueAt(j, i - 1)) < threshold) {
				m_disjointSet.doUnion(i*m_gridWidth + j, (i - 1)*m_gridWidth + j);
			}
			if (abs(valueAt(j - 1, i) - valueAt(j, i)) < threshold) {
				m_disjointSet.doUnion(i*m_gridWidth + j, i*m_gridWidth + j - 1);
			}
		}
	}

	int personComponent = m_disjointSet.find((m_gridWidth * componentY) + componentX);

	for (int i = 0; i < m_gridHeight; i++) {
		for (int j = 0; j < m_gridWidth; j++) {
			int num = m_disjointSet.find(m_gridWidth * i + j);
			if (num == personComponent) {
				m_grid[i * m_gridWidth + j] = USHRT_MAX;
			}
			else {
				char zero = 0;
				m_grid[i * m_gridWidth + j] = 0;
			}
		}
	}
}