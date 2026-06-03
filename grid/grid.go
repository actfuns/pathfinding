package grid

import "github.com/actfuns/pathfinding/finder"

// GridType identifies the tile grid layout.
type GridType int

const (
	// Orthogonal is a standard square-tile grid (4-directional or 8-directional).
	Orthogonal GridType = iota
	// Staggered is a 45-degree isometric/staggered grid (diamond-shaped tiles).
	Staggered
	// Hexagonal is a hexagonal grid (both pointy-top and flat-top layouts).
	Hexagonal
)

// NewMatrix creates an empty (all zero) matrix with the given dimensions.
// All cells are 0 (walkable), ready to pass to NewOrthogonalGrid, NewHexGrid,
// or NewStaggeredGrid.
func NewMatrix(w, h int) [][]int {
	m := make([][]int, h)
	for i := range m {
		m[i] = make([]int, w)
	}
	return m
}

// Grid defines the interface that all grid types implement.
// It focuses on grid-consumer operations: coordinate conversion, walkability
// queries, pathfinding, and rendering. This is separate from finder.Grid,
// which only exposes what pathfinding algorithms need.
type Grid interface {
	// Tile coordinate queries
	Width() int
	Height() int
	IsInside(x, y int) bool
	IsWalkableAt(x, y int) bool
	SetWalkableAt(x, y int, walkable bool)
	GetNodeAt(x, y int) *finder.Node
	ObstacleCount() int

	// World coordinate conversion
	WorldToTile(wx, wy float32) (int, int)
	TileToWorld(tx, ty int) (float32, float32)
	IsWalkableAtWorld(wx, wy float32) bool
	SetWalkableAtWorld(wx, wy float32, walkable bool)

	// Nearest walkable search
	FindNearestWalkable(wx, wy float32, maxRadius int, edgeInset float32) (float32, float32, bool)
	FindNearestWalkableTile(wx, wy float32, maxRadius int) (int, int, bool)

	// Pathfinding
	Finder() finder.Finder
	FindPath(wx1, wy1, wx2, wy2 float32) [][2]float32
	FindSmoothPath(wx1, wy1, wx2, wy2 float32) [][2]float32
	SmoothenTilePath(path [][2]int) [][2]int

	// Rendering
	RenderSVG(path [][2]int, startX, startY, endX, endY int) string
}
