package grid

import (
	"github.com/actfuns/pathfinding/finder"
)

// OrthogonalGrid is a standard rectangular grid. Coordinates are in tile space.
// World-space helpers convert using tileW/tileH.
type OrthogonalGrid struct {
	tileW  int // world units per tile horizontally
	tileH  int // world units per tile vertically
	width  int // tiles across
	height int // tiles down
	nodes  []*finder.Node
	finder finder.Finder
}

// Finder returns the pathfinder associated with this grid.
func (g *OrthogonalGrid) Finder() finder.Finder { return g.finder }

// OrthogonalOption configures an OrthogonalGrid.
type OrthogonalOption func(*OrthogonalGrid)

// WithOrthogonalTileSize sets the tile dimensions for world coordinate conversion.
func WithOrthogonalTileSize(w, h int) OrthogonalOption {
	return func(g *OrthogonalGrid) { g.tileW = w; g.tileH = h }
}

// WithOrthogonalFinder sets the pathfinder associated with this grid.
func WithOrthogonalFinder(f finder.Finder) OrthogonalOption {
	return func(g *OrthogonalGrid) { g.finder = f }
}

// NewOrthogonalGrid creates an OrthogonalGrid from a matrix (0=walkable, non-zero=obstacle).
// Default tile size is 1×1 world unit. Default finder is AStarFinder.
func NewOrthogonalGrid(matrix [][]int, opts ...OrthogonalOption) *OrthogonalGrid {
	h := len(matrix)
	w := len(matrix[0])
	nodes := make([]*finder.Node, 0, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			n := finder.NewNode(x, y)
			n.Walkable = matrix[y][x] == 0
			nodes = append(nodes, n)
		}
	}
	g := &OrthogonalGrid{width: w, height: h, tileW: 1, tileH: 1, nodes: nodes, finder: finder.NewAStarFinder()}
	for _, opt := range opts {
		opt(g)
	}
	return g
}

// --- tile coordinate implementations of finder.Grid ---

func (g *OrthogonalGrid) index(x, y int) int { return y*g.width + x }
func (g *OrthogonalGrid) Width() int         { return g.width }
func (g *OrthogonalGrid) Height() int        { return g.height }
func (g *OrthogonalGrid) TileWidth() int     { return g.tileW }
func (g *OrthogonalGrid) TileHeight() int    { return g.tileH }
func (g *OrthogonalGrid) IsInside(x, y int) bool {
	return x >= 0 && x < g.width && y >= 0 && y < g.height
}
func (g *OrthogonalGrid) GetNodeAt(x, y int) *finder.Node { return g.nodes[g.index(x, y)] }

func (g *OrthogonalGrid) IsWalkableAt(x, y int) bool {
	if !g.IsInside(x, y) {
		return false
	}
	return g.nodes[g.index(x, y)].Walkable
}

func (g *OrthogonalGrid) SetWalkableAt(x, y int, walkable bool) {
	g.nodes[g.index(x, y)].Walkable = walkable
}

func (g *OrthogonalGrid) Clone() finder.Grid {
	ng := &OrthogonalGrid{width: g.width, height: g.height, tileW: g.tileW, tileH: g.tileH, finder: g.finder}
	ng.nodes = make([]*finder.Node, len(g.nodes))
	for i, n := range g.nodes {
		cp := *n
		cp.Parent = nil
		ng.nodes[i] = &cp
	}
	return ng
}

// SupportsJPSCanonicalPruning returns true — orthogonal grids have axis-aligned topology
// and support the standard JPS pruning rules.
func (g *OrthogonalGrid) SupportsJPSCanonicalPruning() bool { return true }

// --- world coordinate helpers ---

func (g *OrthogonalGrid) WorldToTile(wx, wy float32) (int, int) {
	tx := int(wx) / g.tileW
	ty := int(wy) / g.tileH
	return tx, ty
}

func (g *OrthogonalGrid) TileToWorld(tx, ty int) (float32, float32) {
	return float32(tx*g.tileW + g.tileW/2), float32(ty*g.tileH + g.tileH/2)
}

func (g *OrthogonalGrid) IsWalkableAtWorld(wx, wy float32) bool {
	tx, ty := g.WorldToTile(wx, wy)
	return g.IsWalkableAt(tx, ty)
}

func (g *OrthogonalGrid) SetWalkableAtWorld(wx, wy float32, walkable bool) {
	tx, ty := g.WorldToTile(wx, wy)
	g.SetWalkableAt(tx, ty, walkable)
}

// FindPath finds a path between two world positions through the grid.
func (g *OrthogonalGrid) FindPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
	if g.finder == nil {
		return nil
	}
	sx, sy := g.WorldToTile(wx1, wy1)
	ex, ey := g.WorldToTile(wx2, wy2)
	if !g.IsInside(sx, sy) || !g.IsInside(ex, ey) {
		return nil
	}
	path := g.finder.FindPath(sx, sy, ex, ey, g)
	if path == nil {
		return nil
	}
	wp := make([][2]float32, len(path))
	for i, p := range path {
		wp[i][0], wp[i][1] = g.TileToWorld(p[0], p[1])
	}
	// Use original world coordinates for first and last points
	wp[0][0], wp[0][1] = wx1, wy1
	wp[len(wp)-1][0], wp[len(wp)-1][1] = wx2, wy2
	return wp
}

// FindSmoothPath finds a path between two world positions and smooths it.
func (g *OrthogonalGrid) FindSmoothPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
	if g.finder == nil {
		return nil
	}
	sx, sy := g.WorldToTile(wx1, wy1)
	ex, ey := g.WorldToTile(wx2, wy2)
	if !g.IsInside(sx, sy) || !g.IsInside(ex, ey) {
		return nil
	}
	path := g.finder.FindPath(sx, sy, ex, ey, g)
	if path == nil {
		return nil
	}
	path = g.SmoothenTilePath(path)
	wp := make([][2]float32, len(path))
	for i, p := range path {
		wp[i][0], wp[i][1] = g.TileToWorld(p[0], p[1])
	}
	wp[0][0], wp[0][1] = wx1, wy1
	wp[len(wp)-1][0], wp[len(wp)-1][1] = wx2, wy2
	return wp
}

// SmoothenTilePath smooths a tile-coordinate path by removing unnecessary waypoints.
func (g *OrthogonalGrid) SmoothenTilePath(path [][2]int) [][2]int {
	if len(path) < 2 {
		return path
	}
	smooth := make([]int, 0, len(path))
	smooth = append(smooth, 0)
	last := len(path) - 1
	for i := 2; i < len(path); i++ {
		line := finder.Interpolate(path[smooth[len(smooth)-1]][0], path[smooth[len(smooth)-1]][1], path[i][0], path[i][1])
		blocked := false
		for j := 1; j < len(line); j++ {
			if !g.IsWalkableAt(line[j][0], line[j][1]) {
				blocked = true
				break
			}
		}
		if blocked {
			smooth = append(smooth, i-1)
		}
	}
	if smooth[len(smooth)-1] != last {
		smooth = append(smooth, last)
	}
	result := make([][2]int, len(smooth))
	for i, idx := range smooth {
		result[i] = path[idx]
	}
	return result
}

// --- neighbors ---

func (g *OrthogonalGrid) GetNeighbors(node *finder.Node, diagonal finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	x, y := node.X, node.Y
	w := g.width
	nodes := g.nodes
	neighbors := buffer[:0]

	s0, d0 := false, false
	s1, d1 := false, false
	s2, d2 := false, false
	s3, d3 := false, false

	if g.IsWalkableAt(x, y-1) {
		neighbors = append(neighbors, nodes[(y-1)*w+x])
		s0 = true
	}
	if g.IsWalkableAt(x+1, y) {
		neighbors = append(neighbors, nodes[y*w+x+1])
		s1 = true
	}
	if g.IsWalkableAt(x, y+1) {
		neighbors = append(neighbors, nodes[(y+1)*w+x])
		s2 = true
	}
	if g.IsWalkableAt(x-1, y) {
		neighbors = append(neighbors, nodes[y*w+x-1])
		s3 = true
	}

	if diagonal == finder.DiagonalNever {
		return neighbors
	}

	switch diagonal {
	case finder.DiagonalOnlyWhenNoObstacles:
		d0 = s3 && s0
		d1 = s0 && s1
		d2 = s1 && s2
		d3 = s2 && s3
	case finder.DiagonalIfAtMostOneObstacle:
		d0 = s3 || s0
		d1 = s0 || s1
		d2 = s1 || s2
		d3 = s2 || s3
	case finder.DiagonalAlways:
		d0, d1, d2, d3 = true, true, true, true
	}

	if d0 && g.IsWalkableAt(x-1, y-1) {
		neighbors = append(neighbors, nodes[(y-1)*w+x-1])
	}
	if d1 && g.IsWalkableAt(x+1, y-1) {
		neighbors = append(neighbors, nodes[(y-1)*w+x+1])
	}
	if d2 && g.IsWalkableAt(x+1, y+1) {
		neighbors = append(neighbors, nodes[(y+1)*w+x+1])
	}
	if d3 && g.IsWalkableAt(x-1, y+1) {
		neighbors = append(neighbors, nodes[(y+1)*w+x-1])
	}

	return neighbors
}
