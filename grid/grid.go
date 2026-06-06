package grid

import (
	"github.com/actfuns/pathfinding/finder"
)

// Smoother is a path smoothing function. It takes a tile path and returns a
// simplified path with fewer waypoints, typically by removing intermediate
// tiles that have line-of-sight to each other.
type Smoother func(path [][2]int) [][2]int

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
	finder.Grid

	// TileWidth returns the width of a tile in pixels.
	TileWidth() int
	// TileHeight returns the height of a tile in pixels.
	TileHeight() int
	// ObstacleCount returns the number of non-walkable tiles.
	ObstacleCount() int
	// TileIndex returns the flat array index for tile (x, y).
	// Equivalent to y*Width + x. Panics if outside the grid.
	TileIndex(x, y int) int
	// TileXY returns the tile coordinates for a flat array index.
	// Equivalent to (index % Width, index / Width).
	TileXY(index int) (int, int)

	// SetWeightAt sets the per-tile movement cost multiplier for tile (x, y).
	// Weight 1.0 is the default; higher values make movement more expensive.
	SetWeightAt(x, y int, weight float64)
	// GetWeightAt returns the movement cost multiplier for tile (x, y).
	// Returns 1.0 for tiles outside the grid.
	GetWeightAt(x, y int) float64

	// WorldToTile converts a world-space coordinate to tile-space.
	// Results outside the grid should be checked with IsInside.
	WorldToTile(wx, wy float32) (int, int)
	// TileToWorld converts a tile coordinate to world-space, returning
	// the centre point of the tile.
	TileToWorld(tx, ty int) (float32, float32)
	// IsWalkableAtWorld reports whether the tile at world position (wx, wy) is walkable.
	IsWalkableAtWorld(wx, wy float32) bool
	// SetWalkableAtWorld sets the walkability of the tile at world position (wx, wy).
	SetWalkableAtWorld(wx, wy float32, walkable bool)

	// HasLineOfSight reports whether two tiles can see each other.
	// Uses the appropriate algorithm for the grid type (Bresenham for orthogonal,
	// world-space dense sampling for hex and staggered).
	HasLineOfSight(x1, y1, x2, y2 int) bool
	// HasLineOfSightWorld reports whether two world positions can see
	// each other. Converts to tile coordinates before checking LOS.
	HasLineOfSightWorld(x1, y1, x2, y2 float32) bool
	// FindNearestWalkable is like FindNearestWalkableWorld but returns tile
	// coordinates instead of edge-clamped world coordinates.
	FindNearestWalkable(wx, wy float32, maxRadius int) (int, int, bool)
	// FindNearestWalkableWorld finds the nearest walkable tile within maxRadius
	// (Chebyshev distance in tiles) from world position (wx, wy).
	// Returns the closest point on the edge of the nearest walkable tile, or
	// (0, 0, false) if no walkable tile exists within the search radius.
	FindNearestWalkableWorld(wx, wy float32, maxRadius int, edgeInset float32) (float32, float32, bool)

	// Finder returns the pathfinder used by this grid.
	Finder() finder.Finder
	// FindPath finds a path between two tile positions.
	// Result is backed by an internal buffer — valid only until the next FindPath call.
	FindPath(x1, y1, x2, y2 int) [][2]int
	// FindPathWorld finds a path between two world positions through the grid.
	// Uses the grid's Finder. Returns the path in world coordinates.
	// Result is backed by an internal buffer — valid only until the next FindPathWorld call.
	FindPathWorld(wx1, wy1, wx2, wy2 float32) [][2]float32

	// RenderSVG renders the grid and paths as an SVG string.
	// Tiles are coloured by their Weight value with terrain legend.
	// Multiple paths are drawn in different colours (1st=blue, 2nd=red dashed).
	// Start/end markers are derived from the first path.
	// Pass nil to use the default SVG options.
	RenderSVG(cfg *SVGOpts, paths ...[][2]int) string
}
