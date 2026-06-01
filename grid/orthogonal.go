package grid

import (
	"github.com/actfuns/navpath/finder"
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

// NewOrthogonalGrid creates an OrthogonalGrid from a matrix (0=walkable, non-zero=obstacle).
// Tile size defaults to 1×1 world unit.
func NewOrthogonalGrid(matrix [][]int) *OrthogonalGrid {
	return NewOrthogonalGridWH(matrix, 1, 1)
}

// NewOrthogonalGridWH creates an OrthogonalGrid with explicit tile dimensions.
func NewOrthogonalGridWH(matrix [][]int, tileW, tileH int) *OrthogonalGrid {
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
	return &OrthogonalGrid{width: w, height: h, tileW: tileW, tileH: tileH, nodes: nodes}
}

// WithFinder sets the pathfinder and returns the grid.
func (g *OrthogonalGrid) WithFinder(f finder.Finder) *OrthogonalGrid {
	g.finder = f
	return g
}

// --- tile coordinate implementations of finder.Grid ---

func (g *OrthogonalGrid) index(x, y int) int       { return y*g.width + x }
func (g *OrthogonalGrid) Width() int                { return g.width }
func (g *OrthogonalGrid) Height() int               { return g.height }
func (g *OrthogonalGrid) TileWidth() int            { return g.tileW }
func (g *OrthogonalGrid) TileHeight() int           { return g.tileH }
func (g *OrthogonalGrid) IsInside(x, y int) bool    { return x >= 0 && x < g.width && y >= 0 && y < g.height }
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
	ng := &OrthogonalGrid{width: g.width, height: g.height, tileW: g.tileW, tileH: g.tileH}
	ng.nodes = make([]*finder.Node, len(g.nodes))
	for i, n := range g.nodes {
		cp := *n
		cp.Parent = nil
		ng.nodes[i] = &cp
	}
	return ng
}

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
	return wp
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
