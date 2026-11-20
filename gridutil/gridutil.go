// Package gridutil provides utility functions for grid-based pathfinding.
// All functions work with any grid type that satisfies the minimal interfaces
// defined here (GridLOS, GridWorld). No grid/finder package imports needed.
package gridutil

// GridLOS is the interface for tile-space operations (LOS, smoothing).
// All grid types (*OrthogonalGrid, *HexGrid, *StaggeredGrid) satisfy it.
type GridLOS interface {
	Width() int32
	Height() int32
	IsWalkableAt(x, y int32) bool
}

// GridWorld extends GridLOS with world coordinate conversion.
// Needed by SmoothenPathDense and other world-space utilities.
type GridWorld interface {
	GridLOS
	TileToWorld(tx, ty int32) (float32, float32)
	WorldToTile(wx, wy float32) (int32, int32)
	TileWidth() int32
	TileHeight() int32
}

func abs(x int32) int32 {
	if x < 0 {
		return -x
	}
	return x
}

func maxInt(a, b int32) int32 {
	if a > b {
		return a
	}
	return b
}

func minInt(a, b int32) int32 {
	if a < b {
		return a
	}
	return b
}
