// Package gridutil provides utility functions for grid-based pathfinding.
// All functions work with any grid type that satisfies the minimal interfaces
// defined here (GridLOS, GridWorld). No grid/finder package imports needed.
package gridutil

// GridLOS is the interface for tile-space operations (LOS, smoothing).
// All grid types (*OrthogonalGrid, *HexGrid, *StaggeredGrid) satisfy it.
type GridLOS interface {
	Width() int
	Height() int
	IsWalkableAt(x, y int) bool
}

// GridWorld extends GridLOS with world coordinate conversion.
// Needed by SmoothenPathDense and other world-space utilities.
type GridWorld interface {
	GridLOS
	TileToWorld(tx, ty int) (float32, float32)
	WorldToTile(wx, wy float32) (int, int)
	TileWidth() int
	TileHeight() int
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}
