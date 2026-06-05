package grid

import (
	"fmt"
	"math"
	"strings"

	"github.com/actfuns/pathfinding/finder"
)

// HexGrid is a hexagonal grid using axial coordinates (X, Y).
// Supports both flat-top (stagger on X) and pointy-top (stagger on Y) layouts,
// matching Tiled's "hexagonal" orientation.
//
// Derived layout parameters (sideOffX/Y, colW, rowH) are precomputed in
// NewHexGrid to avoid per-call overhead.
type HexGrid struct {
	width       int  // tiles across
	height      int  // tiles down
	staggerX    bool // true = flat-top (odd cols offset), false = pointy-top (odd rows offset)
	staggerEven bool // true = even rows/cols shifted
	tileW       int  // pixel width of a tile
	tileH       int  // pixel height of a tile
	hexSide     int  // length of the hex side (non-stagger direction)

	// Derived layout params (precomputed, mirrors Tiled's RenderParams)
	sideLenX int // hexSideLength (non-stagger axis)
	sideLenY int // hexSideLength (non-stagger axis)
	sideOffX int // (tileW - sideLenX) / 2
	sideOffY int // (tileH - sideLenY) / 2
	colW     int // sideOffX + sideLenX
	rowH     int // sideOffY + sideLenY
	renW     int // colW + sideOffX  (= effective tileW for render)
	renH     int // rowH + sideOffY  (= effective tileH for render)

	nodes         []*finder.Node
	finder        finder.Finder
	worldBuf      [][2]float32
	obstacleCount int

	// Precomputed neighbor offsets (indexed by parity)
	neighborsEven [6][2]int
	neighborsOdd  [6][2]int
}

// Finder returns the pathfinder associated with this grid.
func (g *HexGrid) Finder() finder.Finder { return g.finder }

// HexOption configures a HexGrid.
type HexOption func(*HexGrid)

// WithHexTileSize sets the pixel dimensions of the hex tile.
func WithHexTileSize(w, h int) HexOption {
	return func(g *HexGrid) { g.tileW = w; g.tileH = h }
}

// WithHexSide sets the hex side length in pixels.
func WithHexSide(side int) HexOption {
	return func(g *HexGrid) { g.hexSide = side }
}

// WithHexFlatTop configures flat-top hexagons (stagger on X axis). Default is pointy-top.
func WithHexFlatTop() HexOption {
	return func(g *HexGrid) { g.staggerX = true }
}

// WithHexEvenStagger configures even-index stagger offset. Default is odd-index.
func WithHexEvenStagger() HexOption {
	return func(g *HexGrid) { g.staggerEven = true }
}

// WithHexFinder sets the pathfinder and returns the grid.
func WithHexFinder(f finder.Finder) HexOption {
	return func(g *HexGrid) { g.finder = f }
}

// NewHexGrid creates a HexGrid from a walkability matrix (0=walkable, non-zero=obstacle).
// Default tile size is 64×64 pixels. Default finder is AStarFinder.
func NewHexGrid(matrix [][]int, opts ...HexOption) *HexGrid {
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

	g := &HexGrid{
		width:         w,
		height:        h,
		tileW:         64,
		tileH:         64,
		obstacleCount: obs,
		nodes:         nodes,
		finder:        finder.NewAStarFinder(),
	}
	for _, opt := range opts {
		opt(g)
	}
	g.initParams()
	return g
}

func (g *HexGrid) initParams() {
	if g.staggerX {
		g.sideLenX = g.hexSide
	} else {
		g.sideLenY = g.hexSide
	}
	g.sideOffX = (g.tileW - g.sideLenX) / 2
	g.sideOffY = (g.tileH - g.sideLenY) / 2
	g.colW = g.sideOffX + g.sideLenX
	g.rowH = g.sideOffY + g.sideLenY
	g.renW = g.colW + g.sideOffX
	g.renH = g.rowH + g.sideOffY
	g.initNeighbors()
}

func (g *HexGrid) initNeighbors() {
	if g.staggerX {
		// Flat-top hex: stagger on X axis
		g.neighborsEven = [6][2]int{
			{-1, -1}, // NW
			{0, -1},  // NE
			{1, 0},   // E
			{0, 1},   // SE
			{-1, 1},  // SW
			{-1, 0},  // W
		}
		g.neighborsOdd = [6][2]int{
			{0, -1}, // NW
			{1, -1}, // NE
			{1, 0},  // E
			{1, 1},  // SE
			{0, 1},  // SW
			{-1, 0}, // W
		}
	} else {
		// Pointy-top hex: stagger on Y axis
		g.neighborsEven = [6][2]int{
			{1, -1}, // NE
			{0, -1}, // NW
			{-1, 0}, // W
			{-1, 1}, // SW
			{0, 1},  // SE
			{1, 0},  // E
		}
		g.neighborsOdd = [6][2]int{
			{1, 0},   // NE
			{0, -1},  // NW
			{-1, -1}, // W
			{-1, 0},  // SW
			{0, 1},   // SE
			{1, 1},   // E
		}
	}
}

func (g *HexGrid) doStaggerX(x int) bool {
	return g.staggerX && ((x&1) != 0) != g.staggerEven
}

func (g *HexGrid) doStaggerY(y int) bool {
	return !g.staggerX && ((y&1) != 0) != g.staggerEven
}

// --- tile coordinate implementations of finder.Grid ---

func (g *HexGrid) index(x, y int) int { return y*g.width + x }

// TileIndex returns the flat array index for tile (x, y).
func (g *HexGrid) TileIndex(x, y int) int { return g.index(x, y) }

// TileXY returns the tile coordinates for a flat array index.
func (g *HexGrid) TileXY(index int) (int, int) {
	return index % g.width, index / g.width
}

// Width returns the number of tiles along the X axis.
func (g *HexGrid) Width() int { return g.width }

// Height returns the number of tiles along the Y axis.
func (g *HexGrid) Height() int { return g.height }

// IsInside reports whether the given tile coordinates are within the grid bounds.
func (g *HexGrid) IsInside(x, y int) bool { return x >= 0 && x < g.width && y >= 0 && y < g.height }

// GetNodeAt returns the node at the given tile coordinates. The coordinates must be inside the grid; the caller should check IsInside first.
func (g *HexGrid) GetNodeAt(x, y int) *finder.Node { return g.nodes[g.index(x, y)] }

// IsWalkableAt reports whether the tile at the given coordinates is walkable.
// Coordinates outside the grid bounds are considered not walkable.
func (g *HexGrid) IsWalkableAt(x, y int) bool {
	if !g.IsInside(x, y) {
		return false
	}
	return g.nodes[g.index(x, y)].Walkable
}

// SetWalkableAt sets the walkability of the tile at the given coordinates.
func (g *HexGrid) SetWalkableAt(x, y int, walkable bool) {
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
func (g *HexGrid) SetWeightAt(x, y int, weight float64) {
	g.nodes[g.index(x, y)].Weight = weight
}

// GetWeightAt returns the movement cost multiplier for the tile at (x, y).
func (g *HexGrid) GetWeightAt(x, y int) float64 {
	if !g.IsInside(x, y) {
		return 1.0
	}
	return g.nodes[g.index(x, y)].Weight
}

// ObstacleCount returns the number of non-walkable tiles in the grid.
func (g *HexGrid) ObstacleCount() int { return g.obstacleCount }

// Clone returns a deep copy of the hex grid, including a copy of all nodes with
// parent references cleared. The clone shares the finder and precomputed layout
// parameters but owns its own node slice.
func (g *HexGrid) Clone() finder.Grid {
	ng := &HexGrid{
		width:         g.width,
		height:        g.height,
		staggerX:      g.staggerX,
		staggerEven:   g.staggerEven,
		tileW:         g.tileW,
		tileH:         g.tileH,
		hexSide:       g.hexSide,
		sideLenX:      g.sideLenX,
		sideLenY:      g.sideLenY,
		sideOffX:      g.sideOffX,
		sideOffY:      g.sideOffY,
		colW:          g.colW,
		rowH:          g.rowH,
		renW:          g.renW,
		renH:          g.renH,
		neighborsEven: g.neighborsEven,
		neighborsOdd:  g.neighborsOdd,
		obstacleCount: g.obstacleCount,
		finder:        g.finder,
	}
	ng.nodes = make([]*finder.Node, len(g.nodes))
	for i, n := range g.nodes {
		cp := *n
		cp.Parent = nil
		ng.nodes[i] = &cp
	}
	return ng
}

// --- world coordinate conversion (Tiled-style) ---

// tileToScreenCoords converts tile coords to pixel/screen coords.
// Mirrors HexagonalRenderer::tileToScreenCoords.
func (g *HexGrid) tileToScreenCoords(tx, ty int) (float64, float64) {
	if g.staggerX {
		px := float64(tx * g.colW)
		py := float64(ty*(g.renH+g.sideLenY)) + float64(g.rowH)/2
		if g.doStaggerX(tx) {
			py += float64(g.rowH)
		}
		return px, py
	}
	px := float64(tx*(g.renW+g.sideLenX)) + float64(g.colW)/2
	if g.doStaggerY(ty) {
		px += float64(g.colW)
	}
	py := float64(ty * g.rowH)
	return px, py
}

// screenToTileCoords converts pixel/screen coords to tile coords.
// Mirrors HexagonalRenderer::screenToTileCoords — nearest-of-4-centers approach.
func (g *HexGrid) screenToTileCoords(sx, sy float64) (int, int) {
	x, y := sx, sy

	if g.staggerX {
		if g.staggerEven {
			x -= float64(g.renW)
		} else {
			x -= float64(g.sideOffX)
		}
	} else {
		if g.staggerEven {
			y -= float64(g.renH)
		} else {
			y -= float64(g.sideOffY)
		}
	}

	refX := int(math.Floor(x / float64(g.colW*2)))
	refY := int(math.Floor(y / float64(g.rowH*2)))

	relX := x - float64(refX*g.colW*2)
	relY := y - float64(refY*g.rowH*2)

	staggerIdx := refX
	if !g.staggerX {
		staggerIdx = refY
	}
	staggerIdx *= 2
	if g.staggerEven {
		staggerIdx++
	}
	if g.staggerX {
		refX = staggerIdx
	} else {
		refY = staggerIdx
	}

	type vec2 struct{ x, y float64 }
	var centers [4]vec2

	if g.staggerX {
		left := float64(g.sideLenX) / 2
		cx := left + float64(g.colW)
		cy := float64(g.renH) / 2
		centers = [4]vec2{
			{left, cy},
			{cx, cy - float64(g.rowH)},
			{cx, cy + float64(g.rowH)},
			{cx + float64(g.colW), cy},
		}
	} else {
		top := float64(g.sideLenY) / 2
		cx := float64(g.renW) / 2
		cy := top + float64(g.rowH)
		centers = [4]vec2{
			{cx, top},
			{cx - float64(g.colW), cy},
			{cx + float64(g.colW), cy},
			{cx, cy + float64(g.rowH)},
		}
	}

	nearest := 0
	minDist := math.MaxFloat64
	for i, c := range centers {
		dx := c.x - relX
		dy := c.y - relY
		d := dx*dx + dy*dy
		if d < minDist {
			minDist = d
			nearest = i
		}
	}

	var offsets [4][2]int
	if g.staggerX {
		offsets = [4][2]int{
			{0, 0},
			{1, -1},
			{1, 0},
			{2, 0},
		}
	} else {
		offsets = [4][2]int{
			{0, 0},
			{-1, 1},
			{0, 1},
			{0, 2},
		}
	}

	return refX + offsets[nearest][0], refY + offsets[nearest][1]
}

// WorldToTile converts world (pixel) coordinates to hex tile coordinates.
func (g *HexGrid) WorldToTile(wx, wy float32) (int, int) {
	return g.screenToTileCoords(float64(wx), float64(wy))
}

// TileToWorld converts hex tile coordinates to world (pixel) position (tile center).
func (g *HexGrid) TileToWorld(tx, ty int) (float32, float32) {
	px, py := g.tileToScreenCoords(tx, ty)
	return float32(px), float32(py)
}

// IsWalkableAtWorld checks walkability at a world position.
func (g *HexGrid) IsWalkableAtWorld(wx, wy float32) bool {
	tx, ty := g.WorldToTile(wx, wy)
	return g.IsWalkableAt(tx, ty)
}

// SetWalkableAtWorld sets walkability at a world position.
func (g *HexGrid) SetWalkableAtWorld(wx, wy float32, walkable bool) {
	tx, ty := g.WorldToTile(wx, wy)
	g.SetWalkableAt(tx, ty, walkable)
}

// tileEdgePoint returns the closest point on the edge of tile (tx, ty) to (wx, wy),
// inset by one world-space unit inward from the tile boundary.
func (g *HexGrid) tileEdgePoint(tx, ty int, wx, wy, inset float32) (float32, float32) {
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
// from the given world position (wx, wy). See Grid.FindNearestWalkable for details.
func (g *HexGrid) FindNearestWalkableWorld(wx, wy float32, maxRadius int, edgeInset float32) (float32, float32, bool) {
	if maxRadius < 0 {
		return 0, 0, false
	}
	tx, ty := g.WorldToTile(wx, wy)
	if g.IsWalkableAt(tx, ty) {
		ww, wh := g.tileEdgePoint(tx, ty, wx, wy, edgeInset)
		return ww, wh, true
	}
	// Sub-tile offset — determines which edge tile is closest to (wx, wy)
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

		// Full perimeter scan (excluding cells already checked above)
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
func (g *HexGrid) FindNearestWalkable(wx, wy float32, maxRadius int) (int, int, bool) {
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

// FindPath finds a path between two world positions through the hex grid.
// The returned [][2]float32 is backed by an internal buffer and is only
// valid until the next FindPath/FindSmoothPath call on the same grid.
// FindPath finds a path between two tile positions.
func (g *HexGrid) FindPath(x1, y1, x2, y2 int) [][2]int {
	if g.finder == nil {
		return nil
	}
	return g.finder.FindPath(x1, y1, x2, y2, g)
}

func (g *HexGrid) FindPathWorld(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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

// FindSmoothPath finds a path between two world positions and smooths it.
// Same buffer contract as FindPath.
func (g *HexGrid) FindSmoothPath(x1, y1, x2, y2 int) [][2]int {
	if g.finder == nil {
		return nil
	}
	path := g.finder.FindPath(x1, y1, x2, y2, g)
	if path == nil {
		return nil
	}
	return g.SmoothenPath(path)
}

// FindSmoothPathWorld finds a path between two world positions and smooths it.
func (g *HexGrid) FindSmoothPathWorld(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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
	path = g.SmoothenPath(path)
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

// SmoothenTilePath smooths a tile-coordinate hex path by removing unnecessary waypoints.
// Writes the smoothed result in-place over the input (finder's cached pathBuf).
func (g *HexGrid) SmoothenPath(path [][2]int) [][2]int {
	if len(path) < 2 {
		return path
	}
	writeIdx := 1
	for i := 2; i < len(path); i++ {
		if !g.hexLineOfSight(path[writeIdx-1], path[i]) {
			path[writeIdx] = path[i-1]
			writeIdx++
		}
	}
	if path[writeIdx-1] != path[len(path)-1] {
		path[writeIdx] = path[len(path)-1]
		writeIdx++
	}
	return path[:writeIdx]
}

// hexLineOfSight checks if there is a straight pixel line between two hex tiles with no obstacles.
func (g *HexGrid) hexLineOfSight(a, b [2]int) bool {
	ax, ay := g.TileToWorld(a[0], a[1])
	bx, by := g.TileToWorld(b[0], b[1])
	steps := math.Sqrt(float64((bx-ax)*(bx-ax)+(by-ay)*(by-ay))) / float64(min(g.tileW, g.tileH)) * 2
	if steps < 1 {
		steps = 1
	}
	for t := 0; t < int(steps); t++ {
		f := float64(t) / steps
		wx := float64(ax) + float64(bx-ax)*f
		wy := float64(ay) + float64(by-ay)*f
		tx, ty := g.screenToTileCoords(wx, wy)
		if !g.IsWalkableAt(tx, ty) {
			return false
		}
	}
	return true
}

// GetNeighbors returns the 6 hex neighbors. Diagonal parameter is ignored
// because hex grids have no separate diagonal concept — all 6 are cardinal moves.
func (g *HexGrid) GetNeighbors(node *finder.Node, _ finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	x, y := node.X, node.Y
	neighbors := buffer[:0]
	w := g.width
	nodes := g.nodes

	var dirs [6][2]int
	if (g.staggerX && x&1 == 1) || (!g.staggerX && y&1 == 1) {
		dirs = g.neighborsOdd
	} else {
		dirs = g.neighborsEven
	}

	for _, d := range dirs {
		nx := x + d[0]
		ny := y + d[1]
		if nx >= 0 && nx < w && ny >= 0 && ny < g.height && nodes[ny*w+nx].Walkable {
			neighbors = append(neighbors, nodes[ny*w+nx])
		}
	}

	return neighbors
}

// RenderSVG renders the hex grid with weight-colored tiles and paths.
func (g *HexGrid) RenderSVG(cfg *SVGOpts, paths ...[][2]int) string {
	// Use defaults when nil
	if cfg == nil {
		cfg = DefaultSVGOpts
	}
	// Derive start/end from first path
	startX, startY, endX, endY := 0, 0, 0, 0
	if len(paths) > 0 && len(paths[0]) > 0 {
		startX, startY = paths[0][0][0], paths[0][0][1]
		endX, endY = paths[0][len(paths[0])-1][0], paths[0][len(paths[0])-1][1]
	}

	padding := 20.0

	// Find bounds of all tile center points
	minX, minY := math.MaxFloat64, math.MaxFloat64
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, g.height-1-y)
			if cx < minX {
				minX = cx
			}
			if cy < minY {
				minY = cy
			}
		}
	}

	// Build hex polygon vertices relative to tile center
	var hexOffsets [][2]float64
	if g.staggerX {
		// Flat-top: pointy top and bottom, flat top and bottom
		h := float64(g.renH)
		w := float64(g.tileW)
		sideOff := float64(g.sideOffX)
		hexOffsets = [][2]float64{
			{sideOff, 0},
			{w - sideOff, 0},
			{w, h / 2},
			{w - sideOff, h},
			{sideOff, h},
			{0, h / 2},
		}
	} else {
		// Pointy-top: flat top and bottom, pointy left and right
		w := float64(g.renW)
		h := float64(g.tileH)
		sideOff := float64(g.sideOffY)
		hexOffsets = [][2]float64{
			{w / 2, 0},
			{w, sideOff},
			{w, h - sideOff},
			{w / 2, h},
			{0, h - sideOff},
			{0, sideOff},
		}
	}

	// Find bounds of all hex vertices to compute SVG size
	vxMin, vyMin := math.MaxFloat64, math.MaxFloat64
	vxMax, vyMax := -math.MaxFloat64, -math.MaxFloat64
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, g.height-1-y)
			for _, off := range hexOffsets {
				vx := cx + off[0]
				vy := cy + off[1]
				if vx < vxMin {
					vxMin = vx
				}
				if vy < vyMin {
					vyMin = vy
				}
				if vx > vxMax {
					vxMax = vx
				}
				if vy > vyMax {
					vyMax = vy
				}
			}
		}
	}

	svgW := (vxMax - vxMin) + padding*2 + 120
	svgH := (vyMax - vyMin) + padding*2
	dx := padding + 120 + padding - vxMin
	dy := padding - vyMin

	var b strings.Builder
	b.WriteString(xmlHeader(int(math.Ceil(svgW)), int(math.Ceil(svgH))))

	// Draw tiles
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, g.height-1-y)
			fill, stroke := "#c8e6c9", "#cccccc"
			if !g.IsWalkableAt(x, y) {
				fill, stroke = "#555555", "#444444"
			} else {
				fill = weightToColor(g.nodes[g.index(x, y)].Weight, cfg)
			}
			pts := make([]string, len(hexOffsets))
			for i, off := range hexOffsets {
				pts[i] = fmt.Sprintf("%.1f,%.1f", cx+off[0]+dx, cy+off[1]+dy)
			}
			fmt.Fprintf(&b, `<polygon points="%s" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				strings.Join(pts, " "), fill, stroke)
		}
	}

	drawPathAndMarkers(&b, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			cx, cy := g.tileToScreenCoords(tx, g.height-1-ty)
			var cxOff, cyOff float64
			if g.staggerX {
				cxOff = float64(g.tileW) / 2
				cyOff = float64(g.renH) / 2
			} else {
				cxOff = float64(g.renW) / 2
				cyOff = float64(g.tileH) / 2
			}
			return cx + cxOff + dx, cy + cyOff + dy
		}, paths...)

	renderLegend(&b, padding, padding, cfg)
	b.WriteString("</svg>\n")
	return b.String()
}
