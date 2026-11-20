package grid

import (
	"errors"
	"math"

	"github.com/actfuns/pathfinding/finder"
)

// ErrFinderRequired is returned by CloneWith when the source finder is
// stateful and the edit callback does not install a replacement finder.
var ErrFinderRequired = errors.New("grid: CloneWith requires an explicit finder for a stateful finder")

// Smoother is a path smoothing function. It takes a tile path and returns a
// simplified path with fewer waypoints, typically by removing intermediate
// tiles that have line-of-sight to each other.
type Smoother func(path [][2]int32) [][2]int32

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
func NewMatrix(w, h int32) [][]int32 {
	m := make([][]int32, int(h))
	for i := range m {
		m[i] = make([]int32, int(w))
	}
	return m
}

// validWeight reports whether w is a finite positive movement multiplier.
func validWeight(w float32) bool {
	return w > 0 && !math.IsNaN(float64(w)) && !math.IsInf(float64(w), 0)
}

// setNodeWeight maintains a lower bound capped at the default weight of 1.
func setNodeWeight(nodes []finder.Node, index int, weight float32, lowerBound *float32) {
	previous := nodes[index].Weight
	nodes[index].Weight = weight
	if weight < *lowerBound {
		*lowerBound = weight
	} else if previous == *lowerBound && previous < 1 && weight > previous {
		bound := float32(1)
		for _, node := range nodes {
			bound = min(bound, node.Weight)
		}
		*lowerBound = bound
	}
}

// PathBuff holds reusable tile and world-coordinate buffers for
// FindPathWorld. The zero value means normal allocation.
type PathBuff struct {
	Tile  [][2]int32
	World [][2]float32
}

// NewPathBuff creates a reusable PathBuff with the given starting capacities.
func NewPathBuff(tileCap, worldCap int) *PathBuff {
	if tileCap < 0 {
		tileCap = 0
	}
	if worldCap < 0 {
		worldCap = 0
	}
	return &PathBuff{
		Tile:  make([][2]int32, 0, tileCap),
		World: make([][2]float32, 0, worldCap),
	}
}

// Reset clears both lengths while keeping the capacities.
func (b *PathBuff) Reset() {
	if b == nil {
		return
	}
	b.Tile = b.Tile[:0]
	b.World = b.World[:0]
}

// Grid defines the interface that all grid types implement.
// It focuses on grid-consumer operations: coordinate conversion, walkability
// queries, pathfinding, and rendering. This is separate from finder.Grid,
// which only exposes what pathfinding algorithms need.
type Grid interface {
	finder.Grid

	// TileWidth returns the width of a tile in pixels.
	TileWidth() int32
	// TileHeight returns the height of a tile in pixels.
	TileHeight() int32
	// ObstacleCount returns the number of non-walkable tiles.
	ObstacleCount() int32
	// TileIndex returns the flat array index for tile (x, y).
	// Equivalent to y*Width + x. Panics if outside the grid.
	TileIndex(x, y int32) int
	// TileXY returns the tile coordinates for a flat array index.
	// Equivalent to (index % Width, index / Width).
	TileXY(index int) (int32, int32)

	// GetWeightAt returns the movement cost multiplier for tile (x, y).
	// Returns 1.0 for tiles outside the grid.
	GetWeightAt(x, y int32) float32
	// GetWeightAtWorld returns the movement cost multiplier at a world position.
	GetWeightAtWorld(wx, wy float32) float32

	// WorldToTile converts a world-space coordinate to tile-space.
	// Results outside the grid should be checked with IsInside.
	WorldToTile(wx, wy float32) (int32, int32)
	// TileToWorld converts a tile coordinate to world-space, returning
	// the centre point of the tile.
	TileToWorld(tx, ty int32) (float32, float32)
	// IsWalkableAtWorld reports whether the tile at world position (wx, wy) is walkable.
	IsWalkableAtWorld(wx, wy float32) bool

	// HasLineOfSight reports whether two tiles can see each other.
	// Uses the appropriate algorithm for the grid type (Bresenham for orthogonal,
	// world-space dense sampling for hex and staggered).
	HasLineOfSight(x1, y1, x2, y2 int32) bool
	// HasLineOfSightWorld reports whether two world positions can see
	// each other. Converts to tile coordinates before checking LOS.
	HasLineOfSightWorld(x1, y1, x2, y2 float32) bool
	// FindNearestWalkable is like FindNearestWalkableWorld but returns tile
	// coordinates instead of edge-clamped world coordinates.
	FindNearestWalkable(wx, wy float32, maxRadius int32) (int32, int32, bool)
	// FindNearestWalkableWorld finds the nearest walkable tile within maxRadius
	// (Chebyshev distance in tiles) from world position (wx, wy).
	// Returns the closest point on the edge of the nearest walkable tile, or
	// (0, 0, false) if no walkable tile exists within the search radius.
	FindNearestWalkableWorld(wx, wy float32, maxRadius int32, edgeInset float32) (float32, float32, bool)

	// Clone returns a read-only copy with its own node storage. A stateless
	// Finder is shared; a stateful Finder returns ErrFinderRequired. Use
	// CloneWith to install a Finder prepared for the new snapshot.
	Clone() (Grid, error)
	// CloneWith copies the grid once, applies edits to the private copy, and
	// returns it as a new read-only snapshot. The callback must not retain the
	// MutableGrid it receives. A stateless Finder is shared automatically; for a
	// stateful Finder (JPS+, HPA, Waypoint), the callback must call SetFinder
	// with a Finder prepared for the new grid, otherwise ErrFinderRequired is
	// returned.
	CloneWith(edit func(MutableGrid)) (Grid, error)
	// Finder returns the pathfinder used by this grid.
	Finder() finder.Finder
	// FindPath finds a path between two tile positions.
	// A nil path with a nil error means no route exists; a non-nil error means
	// the search could not complete (e.g. finder.ErrTimeLimit), so the answer is
	// unknown. The returned path is independent and owned by the caller.
	// dst is treated as dst[:0]; pass nil for normal allocation.
	// Concurrent read-only searches are safe when the configured Finder is
	// concurrency-safe; mutating the grid or finder during a search is not.
	FindPath(x1, y1, x2, y2 int32, dst [][2]int32) ([][2]int32, error)

	// FindPathWorld finds a path between two world positions through the grid.
	// buff.World is the world-space output; buff.Tile is the Finder's tile-space
	// scratch. nil buff lets both steps allocate. The buffer's slice headers are
	// updated if either backing array grows.
	FindPathWorld(wx1, wy1, wx2, wy2 float32, buff *PathBuff) ([][2]float32, error)

	// RenderSVG renders the grid and paths as an SVG string.
	// Tiles are coloured by their Weight value with terrain legend.
	// Multiple paths are drawn in different colours (1st=blue, 2nd=red dashed).
	// Start/end markers are derived from the first path.
	// Pass nil to use the default SVG options.
	RenderSVG(cfg *SVGOpts, paths ...[][2]int32) string
}

// MutableGrid is the optional write capability of a grid. For a zero-copy
// mutable view of a Grid, assert the Grid value to MutableGrid. The caller is
// responsible for ensuring there are no concurrent readers or searches, and
// for synchronizing publication of the changes.
type MutableGrid interface {
	Grid
	SetWalkableAt(x, y int32, walkable bool)
	SetWeightAt(x, y int32, weight float32)
	SetWalkableAtWorld(wx, wy float32, walkable bool)
	SetWeightAtWorld(wx, wy float32, weight float32)
	SetFinder(f finder.Finder)
}
