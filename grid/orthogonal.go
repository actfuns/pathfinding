package grid

import (
	"fmt"
	"math"
	"strings"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/gridutil"
	"github.com/actfuns/pathfinding/internal/walkability"
)

// OrthogonalGrid is a standard rectangular grid. Coordinates are in tile space.
// World-space helpers convert using tileW/tileH.
type OrthogonalGrid struct {
	tileW                  int32 // world units per tile horizontally
	tileH                  int32 // world units per tile vertically
	width                  int32 // tiles across
	height                 int32 // tiles down
	obstacleCount          int32 // number of non-walkable tiles
	walkabilityFingerprint uint64
	nodes                  []finder.Node
	finder                 finder.Finder
	smoothMode             SmoothMode
	smoother               Smoother // smoothing function; nil = default
}

// Finder returns the pathfinder associated with this grid.
func (g *OrthogonalGrid) Finder() finder.Finder { return g.finder }

func NewOrthogonalGrid(matrix [][]int32, opts ...GridOption) Grid {
	h := len(matrix)
	w := len(matrix[0])
	nodes := make([]finder.Node, w*h)
	obs := 0
	var fingerprint uint64
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			i := y*w + x
			walkable := matrix[y][x] == 0
			if !walkable {
				obs++
				fingerprint = walkability.Toggle(fingerprint, uint64(i))
			}
			nodes[i] = finder.Node{X: int32(x), Y: int32(y), Walkable: walkable, Weight: 1.0}
		}
	}
	var cfg gridOptions
	cfg.tileW = 1
	cfg.tileH = 1
	for _, opt := range opts {
		opt(&cfg)
	}
	g := &OrthogonalGrid{width: int32(w), height: int32(h), tileW: cfg.tileW, tileH: cfg.tileH, obstacleCount: int32(obs), smoothMode: cfg.smoothMode, nodes: nodes, finder: cfg.finder}
	g.walkabilityFingerprint = fingerprint
	if cfg.finder == nil {
		g.finder = finder.NewAStarFinder()
	} else {
		g.finder = cfg.finder
	}
	g.initSmoother()
	return g
}

// SetFinder replaces the pathfinder used by this mutable grid view.
func (g *OrthogonalGrid) SetFinder(f finder.Finder) { g.finder = f }

// --- tile coordinate implementations of finder.Grid ---

func (g *OrthogonalGrid) index(x, y int32) int { return int(y)*int(g.width) + int(x) }

// TileIndex returns the flat array index for tile (x, y).
func (g *OrthogonalGrid) TileIndex(x, y int32) int { return g.index(x, y) }

// TileXY returns the tile coordinates for a flat array index.
func (g *OrthogonalGrid) TileXY(index int) (int32, int32) {
	w := int(g.width)
	return int32(index % w), int32(index / w)
}

// Width returns the number of tiles horizontally in the grid.
func (g *OrthogonalGrid) Width() int32 { return g.width }

// Height returns the number of tiles vertically in the grid.
func (g *OrthogonalGrid) Height() int32 { return g.height }

// TileWidth returns the world-space width of a single tile.
func (g *OrthogonalGrid) TileWidth() int32 { return g.tileW }

// TileHeight returns the world-space height of a single tile.
func (g *OrthogonalGrid) TileHeight() int32 { return g.tileH }

// ObstacleCount returns the number of non-walkable tiles in the grid.
func (g *OrthogonalGrid) ObstacleCount() int32 { return g.obstacleCount }

// WalkabilityFingerprint returns a fingerprint of the current obstacle layout.
func (g *OrthogonalGrid) WalkabilityFingerprint() uint64 { return g.walkabilityFingerprint }

// IsInside reports whether the tile coordinate (x, y) is within the grid bounds.
func (g *OrthogonalGrid) IsInside(x, y int32) bool {
	return x >= 0 && x < g.width && y >= 0 && y < g.height
}

// GetNodeAt returns the node at tile coordinate (x, y). The caller must ensure
// the coordinate is inside the grid (see IsInside); otherwise the method panics.
func (g *OrthogonalGrid) GetNodeAt(x, y int32) *finder.Node { return &g.nodes[g.index(x, y)] }

// NodesView exposes the contiguous node slice for read-only fast paths.
// Callers must not mutate the returned nodes.
func (g *OrthogonalGrid) NodesView() []finder.Node { return g.nodes }

// IsWalkableAt reports whether the tile at (x, y) is walkable. Coordinates
// outside the grid are reported as not walkable.
func (g *OrthogonalGrid) walkableAt(x, y int32) bool {
	if uint32(x) >= uint32(g.width) || uint32(y) >= uint32(g.height) {
		return false
	}
	return g.nodes[int(y)*int(g.width)+int(x)].Walkable
}

func (g *OrthogonalGrid) IsWalkableAt(x, y int32) bool {
	return g.walkableAt(x, y)
}

// SetWalkableAt sets the walkability of the tile at (x, y). The caller must
// ensure the coordinate is inside the grid; otherwise the method panics.
func (g *OrthogonalGrid) SetWalkableAt(x, y int32, walkable bool) {
	index := g.index(x, y)
	node := &g.nodes[index]
	if node.Walkable == walkable {
		return
	}
	if walkable {
		g.obstacleCount--
	} else {
		g.obstacleCount++
	}
	node.Walkable = walkable
	g.walkabilityFingerprint = walkability.Toggle(g.walkabilityFingerprint, uint64(index))
}

// SetWeightAt sets the movement cost multiplier for the tile at (x, y).
// A weight of 1.0 is the default; higher values make the tile more costly to
// traverse. Panics if (x, y) is outside the grid or weight is not a finite
// positive number.
func (g *OrthogonalGrid) SetWeightAt(x, y int32, weight float32) {
	if !validWeight(weight) {
		panic("grid: weight must be a finite positive number")
	}
	g.nodes[g.index(x, y)].Weight = weight
}

// GetWeightAt returns the movement cost multiplier for the tile at (x, y).
// Returns the default weight (1.0) for tiles outside the grid.
func (g *OrthogonalGrid) GetWeightAt(x, y int32) float32 {
	if !g.IsInside(x, y) {
		return 1.0
	}
	return g.nodes[g.index(x, y)].Weight
}

// GetWeightAtWorld returns the movement cost multiplier at a world position.
func (g *OrthogonalGrid) GetWeightAtWorld(wx, wy float32) float32 {
	tx, ty := g.WorldToTile(wx, wy)
	return g.GetWeightAt(tx, ty)
}

// HasLineOfSight reports whether two tiles can see each other using Bresenham.
func (g *OrthogonalGrid) HasLineOfSight(x1, y1, x2, y2 int32) bool {
	return gridutil.HasLineOfSightBresenham(g, x1, y1, x2, y2)
}

// HasLineOfSightWorld reports whether two world positions can see each other.
func (g *OrthogonalGrid) HasLineOfSightWorld(x1, y1, x2, y2 float32) bool {
	tx1, ty1 := g.WorldToTile(x1, y1)
	tx2, ty2 := g.WorldToTile(x2, y2)
	return g.HasLineOfSight(tx1, ty1, tx2, ty2)
}

// FindNearestWalkable finds the nearest walkable tile within maxRadius (tile rings)
// from the given world position (wx, wy). Returns the world-space center of the
// nearest walkable tile and true if found; returns (0, 0, false) if no walkable
// tile exists within the search radius. The search uses concentric square expansion
// (Chebyshev distance). Within each ring, the walkable tile with the smallest
// Euclidean distance to (wx, wy) is selected.

// tileEdgePoint returns the closest point on the edge of tile (tx, ty) to (wx, wy),
// inset by one world-space unit inward from the tile boundary.
func (g *OrthogonalGrid) tileEdgePoint(tx, ty int32, wx, wy, inset float32) (float32, float32) {
	left := float32(tx*g.tileW) + inset
	right := float32((tx+1)*g.tileW) - inset
	top := float32(ty*g.tileH) + inset
	bottom := float32((ty+1)*g.tileH) - inset
	px := wx
	if px < left {
		px = left
	} else if px > right {
		px = right
	}
	py := wy
	if py < top {
		py = top
	} else if py > bottom {
		py = bottom
	}
	return px, py
}

// FindNearestWalkable finds the nearest walkable tile within maxRadius (tile rings)
// from the given world position (wx, wy). See the method doc on Grid for details.
func (g *OrthogonalGrid) FindNearestWalkableWorld(wx, wy float32, maxRadius int32, edgeInset float32) (float32, float32, bool) {
	if maxRadius < 0 {
		return 0, 0, false
	}
	tx, ty := g.WorldToTile(wx, wy)
	if g.IsWalkableAt(tx, ty) {
		ww, wh := g.tileEdgePoint(tx, ty, wx, wy, edgeInset)
		return ww, wh, true
	}
	// Sub-tile offset: determines which edge tile is closest to (wx, wy)
	cx, cy := float32(tx*g.tileW+g.tileW/2), float32(ty*g.tileH+g.tileH/2)
	ox, oy := wx-cx, wy-cy
	var aox float32 = ox
	if aox < 0 {
		aox = -aox
	}
	var aoy float32 = oy
	if aoy < 0 {
		aoy = -aoy
	}

	for r := int32(1); r <= maxRadius; r++ {
		top, bottom := ty-r, ty+r
		left, right := tx-r, tx+r

		// Edge cells (same row/col as tile center) are always closer than corners.
		// Order by sub-tile offset direction.
		var e1x, e1y, e2x, e2y, e3x, e3y, e4x, e4y int32
		if aox >= aoy {
			if ox < 0 {
				e1x, e1y = left, ty
				e2x, e2y = right, ty
			} else {
				e1x, e1y = right, ty
				e2x, e2y = left, ty
			}
			if oy < 0 {
				e3x, e3y = tx, top
				e4x, e4y = tx, bottom
			} else {
				e3x, e3y = tx, bottom
				e4x, e4y = tx, top
			}
		} else {
			if oy < 0 {
				e1x, e1y = tx, top
				e2x, e2y = tx, bottom
			} else {
				e1x, e1y = tx, bottom
				e2x, e2y = tx, top
			}
			if ox < 0 {
				e3x, e3y = left, ty
				e4x, e4y = right, ty
			} else {
				e3x, e3y = right, ty
				e4x, e4y = left, ty
			}
		}
		if g.IsWalkableAt(e1x, e1y) {
			rx, ry := g.tileEdgePoint(e1x, e1y, wx, wy, edgeInset)
			return rx, ry, true
		}
		if g.IsWalkableAt(e2x, e2y) {
			rx, ry := g.tileEdgePoint(e2x, e2y, wx, wy, edgeInset)
			return rx, ry, true
		}
		if g.IsWalkableAt(e3x, e3y) {
			rx, ry := g.tileEdgePoint(e3x, e3y, wx, wy, edgeInset)
			return rx, ry, true
		}
		if g.IsWalkableAt(e4x, e4y) {
			rx, ry := g.tileEdgePoint(e4x, e4y, wx, wy, edgeInset)
			return rx, ry, true
		}

		// Full perimeter scan (excluding edge cells already checked)
		for cx := left; cx <= right; cx++ {
			if g.IsWalkableAt(cx, top) {
				rx, ry := g.tileEdgePoint(cx, top, wx, wy, edgeInset)
				return rx, ry, true
			}
		}
		for cy := top + 1; cy <= bottom; cy++ {
			if g.IsWalkableAt(right, cy) {
				rx, ry := g.tileEdgePoint(right, cy, wx, wy, edgeInset)
				return rx, ry, true
			}
		}
		for cx := right - 1; cx >= left; cx-- {
			if g.IsWalkableAt(cx, bottom) {
				rx, ry := g.tileEdgePoint(cx, bottom, wx, wy, edgeInset)
				return rx, ry, true
			}
		}
		for cy := bottom - 1; cy >= top+1; cy-- {
			if g.IsWalkableAt(left, cy) {
				rx, ry := g.tileEdgePoint(left, cy, wx, wy, edgeInset)
				return rx, ry, true
			}
		}
	}
	return 0, 0, false
}

// FindNearestWalkable finds the nearest walkable tile within maxRadius (tile rings)
// from the given world position (wx, wy). Returns the tile coordinates and true if found;
// returns (0, 0, false) if no walkable tile exists within the search radius.
func (g *OrthogonalGrid) FindNearestWalkable(wx, wy float32, maxRadius int32) (int32, int32, bool) {
	tx, ty := g.WorldToTile(wx, wy)
	if g.IsWalkableAt(tx, ty) {
		return tx, ty, true
	}
	var aox float32 = wx - float32(tx*g.tileW+g.tileW/2)
	if aox < 0 {
		aox = -aox
	}
	var aoy float32 = wy - float32(ty*g.tileH+g.tileH/2)
	if aoy < 0 {
		aoy = -aoy
	}
	for r := int32(1); r <= maxRadius; r++ {
		top, bottom := ty-r, ty+r
		left, right := tx-r, tx+r

		var e1x, e1y, e2x, e2y, e3x, e3y, e4x, e4y int32
		if aox >= aoy {
			if wx-float32(tx*g.tileW+g.tileW/2) < 0 {
				e1x, e1y = left, ty
				e2x, e2y = right, ty
			} else {
				e1x, e1y = right, ty
				e2x, e2y = left, ty
			}
			if wy-float32(ty*g.tileH+g.tileH/2) < 0 {
				e3x, e3y = tx, top
				e4x, e4y = tx, bottom
			} else {
				e3x, e3y = tx, bottom
				e4x, e4y = tx, top
			}
		} else {
			if wy-float32(ty*g.tileH+g.tileH/2) < 0 {
				e1x, e1y = tx, top
				e2x, e2y = tx, bottom
			} else {
				e1x, e1y = tx, bottom
				e2x, e2y = tx, top
			}
			if wx-float32(tx*g.tileW+g.tileW/2) < 0 {
				e3x, e3y = left, ty
				e4x, e4y = right, ty
			} else {
				e3x, e3y = right, ty
				e4x, e4y = left, ty
			}
		}
		if g.IsWalkableAt(e1x, e1y) {
			return e1x, e1y, true
		}
		if g.IsWalkableAt(e2x, e2y) {
			return e2x, e2y, true
		}
		if g.IsWalkableAt(e3x, e3y) {
			return e3x, e3y, true
		}
		if g.IsWalkableAt(e4x, e4y) {
			return e4x, e4y, true
		}

		for cx := left; cx <= right; cx++ {
			if g.IsWalkableAt(cx, top) {
				return cx, top, true
			}
		}
		for cy := top + 1; cy <= bottom; cy++ {
			if g.IsWalkableAt(right, cy) {
				return right, cy, true
			}
		}
		for cx := right - 1; cx >= left; cx-- {
			if g.IsWalkableAt(cx, bottom) {
				return cx, bottom, true
			}
		}
		for cy := bottom - 1; cy >= top+1; cy-- {
			if g.IsWalkableAt(left, cy) {
				return left, cy, true
			}
		}
	}
	return 0, 0, false
}

func (g *OrthogonalGrid) initSmoother() {
	switch g.smoothMode {
	case SmoothBresenham:
		g.smoother = func(path [][2]int32) [][2]int32 {
			return gridutil.SmoothenBresenham(g, path)
		}
	case SmoothBresenhamStrict:
		g.smoother = func(path [][2]int32) [][2]int32 {
			return gridutil.SmoothenBresenhamStrict(g, path)
		}
	case SmoothDense:
		g.smoother = func(path [][2]int32) [][2]int32 {
			return gridutil.SmoothenDense(g, path)
		}
	case SmoothSupercover:
		g.smoother = func(path [][2]int32) [][2]int32 {
			return gridutil.SmoothenSupercover(g, path)
		}
	}
}

// Clone creates a deep copy of the grid with its own node slice. The finder
// reference and tile dimensions are shared from the original.
func (g *OrthogonalGrid) clone() *OrthogonalGrid {
	ng := &OrthogonalGrid{width: g.width, height: g.height, tileW: g.tileW, tileH: g.tileH, obstacleCount: g.obstacleCount, smoothMode: g.smoothMode, finder: g.finder}
	ng.walkabilityFingerprint = g.walkabilityFingerprint
	ng.nodes = make([]finder.Node, len(g.nodes))
	copy(ng.nodes, g.nodes)
	ng.initSmoother()
	return ng
}

func (g *OrthogonalGrid) Clone() (Grid, error) {
	if !finder.IsStateless(g.finder) {
		return nil, ErrFinderRequired
	}
	return g.clone(), nil
}

func (g *OrthogonalGrid) CloneWith(edit func(MutableGrid)) (Grid, error) {
	ng := g.clone()
	if !finder.IsStateless(g.finder) {
		ng.finder = nil
	}
	if edit != nil {
		edit(ng)
	}
	if ng.finder == nil {
		return nil, ErrFinderRequired
	}
	return ng, nil
}

// SupportsJPSCanonicalPruning returns true — orthogonal grids have axis-aligned topology
// and support the standard JPS pruning rules.
func (g *OrthogonalGrid) SupportsJPSCanonicalPruning() bool { return true }

// --- world coordinate helpers ---

// WorldToTile converts a world-space coordinate to a tile-space coordinate.
// It floors toward negative infinity, so negative world coordinates map to
// negative tile coordinates. Results outside the grid should be checked with
// IsInside before use.
func (g *OrthogonalGrid) WorldToTile(wx, wy float32) (int32, int32) {
	tx := int(math.Floor(float64(wx) / float64(g.tileW)))
	ty := int(math.Floor(float64(wy) / float64(g.tileH)))
	return int32(tx), int32(ty)
}

// TileToWorld converts a tile-space coordinate to world-space, returning the
// center point of the tile (offset by half the tile dimensions).
func (g *OrthogonalGrid) TileToWorld(tx, ty int32) (float32, float32) {
	return float32(tx*g.tileW + g.tileW/2), float32(ty*g.tileH + g.tileH/2)
}

// IsWalkableAtWorld reports whether the tile at the given world-space
// coordinate is walkable. The coordinate is converted to tile-space
// via WorldToTile before checking.
func (g *OrthogonalGrid) IsWalkableAtWorld(wx, wy float32) bool {
	tx, ty := g.WorldToTile(wx, wy)
	return g.IsWalkableAt(tx, ty)
}

// SetWalkableAtWorld sets the walkability of the tile at the given world-space
// coordinate. The coordinate is converted to tile-space via WorldToTile before
// applying the change.
func (g *OrthogonalGrid) SetWalkableAtWorld(wx, wy float32, walkable bool) {
	tx, ty := g.WorldToTile(wx, wy)
	g.SetWalkableAt(tx, ty, walkable)
}

// SetWeightAtWorld sets the weight of the tile at the given world-space
// coordinate. The coordinate is converted to tile-space via WorldToTile before
// applying the change.
func (g *OrthogonalGrid) SetWeightAtWorld(wx, wy float32, weight float32) {
	tx, ty := g.WorldToTile(wx, wy)
	g.SetWeightAt(tx, ty, weight)
}

// FindPath finds a path between two tile positions.
// Applies smoothing if configured via WithSmoothBresenham, WithSmoothBresenhamStrict, or WithSmoothDense.
func (g *OrthogonalGrid) FindPath(x1, y1, x2, y2 int32, dst [][2]int32) ([][2]int32, error) {
	if g.finder == nil {
		return nil, nil
	}
	if !g.IsInside(x1, y1) || !g.IsInside(x2, y2) {
		return nil, finder.ErrOutOfBounds
	}
	path, err := g.finder.FindPath(x1, y1, x2, y2, g, dst)
	if err != nil || path == nil {
		return path, err
	}
	if g.smoother != nil {
		path = g.smoother(path)
	}
	if dst != nil {
		path = append(dst[:0], path...)
	}
	return path, nil
}

func (g *OrthogonalGrid) FindPathWorld(wx1, wy1, wx2, wy2 float32, buff *PathBuff) ([][2]float32, error) {
	var dst [][2]float32
	var tileDst [][2]int32
	if buff != nil {
		dst = buff.World[:0]
		tileDst = buff.Tile[:0]
	}
	if g.finder == nil {
		return nil, nil
	}
	sx, sy := g.WorldToTile(wx1, wy1)
	ex, ey := g.WorldToTile(wx2, wy2)
	if !g.IsInside(sx, sy) || !g.IsInside(ex, ey) {
		return nil, finder.ErrOutOfBounds
	}
	if g.obstacleCount == 0 {
		var out [][2]float32
		if dst != nil && cap(dst) >= 2 {
			out = dst[:2]
		} else {
			out = make([][2]float32, 2)
		}
		out[0][0], out[0][1] = wx1, wy1
		out[1][0], out[1][1] = wx2, wy2
		if buff != nil {
			buff.World = out
		}
		return out, nil
	}
	path, err := g.finder.FindPath(sx, sy, ex, ey, g, tileDst)
	if err != nil {
		return nil, err
	}
	if path == nil {
		return nil, nil
	}
	if g.smoother != nil {
		path = g.smoother(path)
	}
	if path == nil {
		return nil, nil
	}
	if buff != nil {
		buff.Tile = path
	}
	var out [][2]float32
	if dst != nil && cap(dst) >= len(path) {
		out = dst[:len(path)]
	} else {
		out = make([][2]float32, len(path))
	}
	for i, p := range path {
		out[i][0], out[i][1] = g.TileToWorld(p[0], p[1])
	}
	out[0][0], out[0][1] = wx1, wy1
	out[len(out)-1][0], out[len(out)-1][1] = wx2, wy2
	if buff != nil {
		buff.World = out
	}
	return out, nil
}

// --- neighbors ---

// GetNeighbors returns the walkable neighbors of the given node. The diagonal
// parameter controls which diagonal moves are permitted. The result is written
// into the provided buffer slice (which is resliced to zero and reused), so the
// caller must not hold references past the next call.
func (g *OrthogonalGrid) GetNeighbors(node *finder.Node, diagonal finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	x, y := int(node.X), int(node.Y)
	w, h := int(g.width), int(g.height)
	nodes := g.nodes
	if x < 0 || y < 0 || x >= w || y >= h {
		return buffer[:0]
	}
	index := y*w + x

	neighbors := buffer[:0]

	// Cardinal neighbors.
	s0 := y > 0 && nodes[index-w].Walkable
	s1 := x+1 < w && nodes[index+1].Walkable
	s2 := y+1 < h && nodes[index+w].Walkable
	s3 := x > 0 && nodes[index-1].Walkable

	if s0 {
		neighbors = append(neighbors, &nodes[index-w])
	}
	if s1 {
		neighbors = append(neighbors, &nodes[index+1])
	}
	if s2 {
		neighbors = append(neighbors, &nodes[index+w])
	}
	if s3 {
		neighbors = append(neighbors, &nodes[index-1])
	}

	if diagonal == finder.DiagonalNever {
		return neighbors
	}

	// Determine which diagonal directions are allowed.
	var d0, d1, d2, d3 bool

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

	// Diagonal neighbors.
	if d0 && x > 0 && y > 0 {
		i := index - w - 1
		if nodes[i].Walkable {
			neighbors = append(neighbors, &nodes[i])
		}
	}

	if d1 && x+1 < w && y > 0 {
		i := index - w + 1
		if nodes[i].Walkable {
			neighbors = append(neighbors, &nodes[i])
		}
	}

	if d2 && x+1 < w && y+1 < h {
		i := index + w + 1
		if nodes[i].Walkable {
			neighbors = append(neighbors, &nodes[i])
		}
	}

	if d3 && x > 0 && y+1 < h {
		i := index + w - 1
		if nodes[i].Walkable {
			neighbors = append(neighbors, &nodes[i])
		}
	}

	return neighbors
}

// RenderSVG renders the grid with weight-colored tiles and one or more paths.
// Each path in paths is drawn in a different color. The first path uses the
// default blue; additional paths cycle through red, green, and purple.
// Walkable tiles are colored by their Weight field:
//
//	weight=1.0 (default): light green
//	weight<1.0 (road):    light blue
//	weight>1.0 (swamp):   yellow → orange gradient
//
// Blocked tiles are shown in dark gray.
func (g *OrthogonalGrid) RenderSVG(cfg *SVGOpts, paths ...[][2]int32) string {
	// Use defaults when nil
	if cfg == nil {
		cfg = DefaultSVGOpts
	}
	// Derive start/end marker positions from the first path
	startX, startY := int32(0), int32(0)
	endX, endY := int32(0), int32(0)
	if len(paths) > 0 && len(paths[0]) > 0 {
		startX, startY = paths[0][0][0], paths[0][0][1]
		endX, endY = paths[0][len(paths[0])-1][0], paths[0][len(paths[0])-1][1]
	}
	cellW := 40
	cellH := 40
	padding := 20
	legendW := 120
	leftPad := padding + legendW + padding
	w, h := int(g.width), int(g.height)
	width := w*cellW + leftPad + padding
	height := h*cellH + padding*2

	var b strings.Builder
	b.WriteString(xmlHeader(width, height))

	// Weight-to-color helper (from cfg or defaults)
	weightColor := func(w float32) string {
		return weightToColor(w, cfg)
	}

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			rx := leftPad + x*cellW
			ry := padding + (h-1-y)*cellH
			var fill, stroke string
			if !g.IsWalkableAt(int32(x), int32(y)) {
				fill = "#555555"
				stroke = "#444444"
			} else {
				fill = weightColor(g.nodes[g.index(int32(x), int32(y))].Weight)
				stroke = "#cccccc"
			}
			fmt.Fprintf(&b, `<rect x="%d" y="%d" width="%d" height="%d" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				rx, ry, cellW, cellH, fill, stroke)
		}
	}

	// Draw each path with a distinct color and start/end markers
	drawPathAndMarkers(&b, startX, startY, endX, endY,
		func(tx, ty int32) (float64, float64) {
			return float64(leftPad + int(tx)*cellW + cellW/2),
				float64(padding + (int(g.height)-1-int(ty))*cellH + cellH/2)
		}, paths...)
	renderLegend(&b, float64(padding), float64(padding), cfg)
	b.WriteString("</svg>\n")
	return b.String()
}
