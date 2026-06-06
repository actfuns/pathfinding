package grid

import (
	"fmt"
	"strings"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/gridutil"
)

// OrthogonalGrid is a standard rectangular grid. Coordinates are in tile space.
// World-space helpers convert using tileW/tileH.
type OrthogonalGrid struct {
	tileW         int // world units per tile horizontally
	tileH         int // world units per tile vertically
	width         int // tiles across
	height        int // tiles down
	obstacleCount int // number of non-walkable tiles
	nodes         []*finder.Node
	finder        finder.Finder
	worldBuf      [][2]float32
	smoother      func(path [][2]int) [][2]int
}

// Finder returns the pathfinder associated with this grid.
func (g *OrthogonalGrid) Finder() finder.Finder { return g.finder }

func NewOrthogonalGrid(matrix [][]int, opts ...GridOption) *OrthogonalGrid {
	h := len(matrix)
	w := len(matrix[0])
	nodes := make([]*finder.Node, 0, w*h)
	obs := 0
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			n := finder.NewNode(x, y)
			n.Walkable = matrix[y][x] == 0
			if !n.Walkable {
				obs++
			}
			nodes = append(nodes, n)
		}
	}
	var cfg gridOptions
	cfg.tileW = 1
	cfg.tileH = 1
	for _, opt := range opts {
		opt(&cfg)
	}
	g := &OrthogonalGrid{width: w, height: h, tileW: cfg.tileW, tileH: cfg.tileH, obstacleCount: obs, nodes: nodes, finder: cfg.finder}
	if cfg.finder == nil {
		g.finder = finder.NewAStarFinder()
	} else {
		g.finder = cfg.finder
	}
	switch cfg.smoothMode {
	case SmoothBresenham:
		g.smoother = func(path [][2]int) [][2]int {
			return gridutil.SmoothenBresenham(g, path)
		}
	case SmoothBresenhamStrict:
		g.smoother = func(path [][2]int) [][2]int {
			return gridutil.SmoothenBresenhamStrict(g, path)
		}
	case SmoothDense:
		g.smoother = func(path [][2]int) [][2]int {
			return gridutil.SmoothenDense(g, path)
		}
	}
	return g
}

// --- tile coordinate implementations of finder.Grid ---

func (g *OrthogonalGrid) index(x, y int) int { return y*g.width + x }

// TileIndex returns the flat array index for tile (x, y).
func (g *OrthogonalGrid) TileIndex(x, y int) int { return g.index(x, y) }

// TileXY returns the tile coordinates for a flat array index.
func (g *OrthogonalGrid) TileXY(index int) (int, int) {
	return index % g.width, index / g.width
}

// Width returns the number of tiles horizontally in the grid.
func (g *OrthogonalGrid) Width() int { return g.width }

// Height returns the number of tiles vertically in the grid.
func (g *OrthogonalGrid) Height() int { return g.height }

// TileWidth returns the world-space width of a single tile.
func (g *OrthogonalGrid) TileWidth() int { return g.tileW }

// TileHeight returns the world-space height of a single tile.
func (g *OrthogonalGrid) TileHeight() int { return g.tileH }

// ObstacleCount returns the number of non-walkable tiles in the grid.
func (g *OrthogonalGrid) ObstacleCount() int { return g.obstacleCount }

// IsInside reports whether the tile coordinate (x, y) is within the grid bounds.
func (g *OrthogonalGrid) IsInside(x, y int) bool {
	return x >= 0 && x < g.width && y >= 0 && y < g.height
}

// GetNodeAt returns the node at tile coordinate (x, y). The caller must ensure
// the coordinate is inside the grid (see IsInside); otherwise the method panics.
func (g *OrthogonalGrid) GetNodeAt(x, y int) *finder.Node { return g.nodes[g.index(x, y)] }

// IsWalkableAt reports whether the tile at (x, y) is walkable. Coordinates
// outside the grid are reported as not walkable.
func (g *OrthogonalGrid) IsWalkableAt(x, y int) bool {
	if !g.IsInside(x, y) {
		return false
	}
	return g.nodes[g.index(x, y)].Walkable
}

// SetWalkableAt sets the walkability of the tile at (x, y). The caller must
// ensure the coordinate is inside the grid; otherwise the method panics.
func (g *OrthogonalGrid) SetWalkableAt(x, y int, walkable bool) {
	node := g.nodes[g.index(x, y)]
	if node.Walkable == walkable {
		return
	}
	if walkable {
		g.obstacleCount--
	} else {
		g.obstacleCount++
	}
	node.Walkable = walkable
}

// SetWeightAt sets the movement cost multiplier for the tile at (x, y).
// A weight of 1.0 is the default; higher values make the tile more costly
// to traverse. Panics if (x, y) is outside the grid.
func (g *OrthogonalGrid) SetWeightAt(x, y int, weight float64) {
	g.nodes[g.index(x, y)].Weight = weight
}

// GetWeightAt returns the movement cost multiplier for the tile at (x, y).
// Returns the default weight (1.0) for tiles outside the grid.
func (g *OrthogonalGrid) GetWeightAt(x, y int) float64 {
	if !g.IsInside(x, y) {
		return 1.0
	}
	return g.nodes[g.index(x, y)].Weight
}

// HasLineOfSight reports whether two tiles can see each other using Bresenham.
func (g *OrthogonalGrid) HasLineOfSight(x1, y1, x2, y2 int) bool {
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
func (g *OrthogonalGrid) tileEdgePoint(tx, ty int, wx, wy, inset float32) (float32, float32) {
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
func (g *OrthogonalGrid) FindNearestWalkableWorld(wx, wy float32, maxRadius int, edgeInset float32) (float32, float32, bool) {
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

	for r := 1; r <= maxRadius; r++ {
		top, bottom := ty-r, ty+r
		left, right := tx-r, tx+r

		// Edge cells (same row/col as tile center) are always closer than corners.
		// Order by sub-tile offset direction.
		var e1x, e1y, e2x, e2y, e3x, e3y, e4x, e4y int
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

// FindNearestWalkableTile finds the nearest walkable tile within maxRadius (tile rings)
// from the given world position (wx, wy). Returns the tile coordinates and true if found;
// returns (0, 0, false) if no walkable tile exists within the search radius.
func (g *OrthogonalGrid) FindNearestWalkable(wx, wy float32, maxRadius int) (int, int, bool) {
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
	for r := 1; r <= maxRadius; r++ {
		top, bottom := ty-r, ty+r
		left, right := tx-r, tx+r

		var e1x, e1y, e2x, e2y, e3x, e3y, e4x, e4y int
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

// Clone creates a deep copy of the grid. The returned grid has its own node
// slice; each node is copied, but the parent pointer is cleared. The finder
// reference and tile dimensions are shared from the original.
func (g *OrthogonalGrid) Clone() finder.Grid {
	ng := &OrthogonalGrid{width: g.width, height: g.height, tileW: g.tileW, tileH: g.tileH, obstacleCount: g.obstacleCount, finder: g.finder, smoother: g.smoother}
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

// WorldToTile converts a world-space coordinate to a tile-space coordinate.
// The conversion uses integer truncation; results outside the grid should be
// checked with IsInside before use.
func (g *OrthogonalGrid) WorldToTile(wx, wy float32) (int, int) {
	tx := int(wx) / g.tileW
	ty := int(wy) / g.tileH
	return tx, ty
}

// TileToWorld converts a tile-space coordinate to world-space, returning the
// center point of the tile (offset by half the tile dimensions).
func (g *OrthogonalGrid) TileToWorld(tx, ty int) (float32, float32) {
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

// FindPath finds a path between two tile positions.
// Applies smoothing if configured via WithSmoothBresenham, WithSmoothBresenhamStrict, or WithSmoothDense.
func (g *OrthogonalGrid) FindPath(x1, y1, x2, y2 int) [][2]int {
	if g.finder == nil {
		return nil
	}
	path := g.finder.FindPath(x1, y1, x2, y2, g)
	if path == nil {
		return nil
	}
	if g.smoother != nil {
		path = g.smoother(path)
	}
	return path
}

func (g *OrthogonalGrid) FindPathWorld(wx1, wy1, wx2, wy2 float32) [][2]float32 {
	if g.finder == nil {
		return nil
	}
	sx, sy := g.WorldToTile(wx1, wy1)
	ex, ey := g.WorldToTile(wx2, wy2)
	if !g.IsInside(sx, sy) || !g.IsInside(ex, ey) {
		return nil
	}
	if g.obstacleCount == 0 {
		if cap(g.worldBuf) < 2 {
			g.worldBuf = make([][2]float32, 2)
		}
		g.worldBuf = g.worldBuf[:2]
		g.worldBuf[0][0], g.worldBuf[0][1] = wx1, wy1
		g.worldBuf[1][0], g.worldBuf[1][1] = wx2, wy2
		return g.worldBuf
	}
	path := g.finder.FindPath(sx, sy, ex, ey, g)
	if path == nil {
		return nil
	}
	if g.smoother != nil {
		path = g.smoother(path)
	}
	if cap(g.worldBuf) >= len(path) {
		g.worldBuf = g.worldBuf[:len(path)]
	} else {
		g.worldBuf = make([][2]float32, len(path))
	}
	for i, p := range path {
		g.worldBuf[i][0], g.worldBuf[i][1] = g.TileToWorld(p[0], p[1])
	}
	g.worldBuf[0][0], g.worldBuf[0][1] = wx1, wy1
	g.worldBuf[len(g.worldBuf)-1][0], g.worldBuf[len(g.worldBuf)-1][1] = wx2, wy2
	return g.worldBuf
}

// --- neighbors ---

// GetNeighbors returns the walkable neighbors of the given node. The diagonal
// parameter controls which diagonal moves are permitted. The result is written
// into the provided buffer slice (which is resliced to zero and reused), so the
// caller must not hold references past the next call.
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
func (g *OrthogonalGrid) RenderSVG(cfg *SVGOpts, paths ...[][2]int) string {
	// Use defaults when nil
	if cfg == nil {
		cfg = DefaultSVGOpts
	}
	// Derive start/end marker positions from the first path
	startX, startY := 0, 0
	endX, endY := 0, 0
	if len(paths) > 0 && len(paths[0]) > 0 {
		startX, startY = paths[0][0][0], paths[0][0][1]
		endX, endY = paths[0][len(paths[0])-1][0], paths[0][len(paths[0])-1][1]
	}
	cellW := 40
	cellH := 40
	padding := 20
	legendW := 120
	leftPad := padding + legendW + padding
	width := g.width*cellW + leftPad + padding
	height := g.height*cellH + padding*2

	var b strings.Builder
	b.WriteString(xmlHeader(width, height))

	// Weight-to-color helper (from cfg or defaults)
	weightColor := func(w float64) string {
		return weightToColor(w, cfg)
	}

	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			rx := leftPad + x*cellW
			ry := padding + (g.height-1-y)*cellH
			var fill, stroke string
			if !g.IsWalkableAt(x, y) {
				fill = "#555555"
				stroke = "#444444"
			} else {
				fill = weightColor(g.nodes[g.index(x, y)].Weight)
				stroke = "#cccccc"
			}
			fmt.Fprintf(&b, `<rect x="%d" y="%d" width="%d" height="%d" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				rx, ry, cellW, cellH, fill, stroke)
		}
	}

	// Draw each path with a distinct color and start/end markers
	drawPathAndMarkers(&b, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			return float64(leftPad + tx*cellW + cellW/2),
				float64(padding + (g.height-1-ty)*cellH + cellH/2)
		}, paths...)
	renderLegend(&b, float64(padding), float64(padding), cfg)
	b.WriteString("</svg>\n")
	return b.String()
}
