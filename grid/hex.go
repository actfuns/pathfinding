package grid

import (
	"math"

	"github.com/actfuns/navpath/finder"
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

	nodes  []*finder.Node
	finder finder.Finder
}

// NewHexGrid creates a HexGrid from a walkability matrix (0=walkable, non-zero=obstacle).
//   tileW, tileH: pixel dimensions of the tile (see Tiled map properties).
//   hexSide: length of the hex side in pixels (0 means sideLengthX/Y = 0, giving a rhombus).
//   staggerX: true = flat-top hexagons (stagger on X axis; odd columns shift down),
//             false = pointy-top hexagons (stagger on Y axis; odd rows shift right).
//   staggerEven: true = even rows/columns are the shifted ones; false = odd ones.
func NewHexGrid(matrix [][]int, tileW, tileH, hexSide int, staggerX, staggerEven bool) *HexGrid {
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

	g := &HexGrid{
		width:       w,
		height:      h,
		staggerX:    staggerX,
		staggerEven: staggerEven,
		tileW:       tileW,
		tileH:       tileH,
		hexSide:     hexSide,
		nodes:       nodes,
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
}

func (g *HexGrid) doStaggerX(x int) bool {
	return g.staggerX && ((x&1) != 0) != g.staggerEven
}

func (g *HexGrid) doStaggerY(y int) bool {
	return !g.staggerX && ((y&1) != 0) != g.staggerEven
}

// WithFinder sets the pathfinder and returns the grid.
func (g *HexGrid) WithFinder(f finder.Finder) *HexGrid {
	g.finder = f
	return g
}

// --- tile coordinate implementations of finder.Grid ---

func (g *HexGrid) index(x, y int) int              { return y*g.width + x }
func (g *HexGrid) Width() int                      { return g.width }
func (g *HexGrid) Height() int                     { return g.height }
func (g *HexGrid) IsInside(x, y int) bool          { return x >= 0 && x < g.width && y >= 0 && y < g.height }
func (g *HexGrid) GetNodeAt(x, y int) *finder.Node { return g.nodes[g.index(x, y)] }

func (g *HexGrid) IsWalkableAt(x, y int) bool {
	if !g.IsInside(x, y) {
		return false
	}
	return g.nodes[g.index(x, y)].Walkable
}

func (g *HexGrid) SetWalkableAt(x, y int, walkable bool) {
	g.nodes[g.index(x, y)].Walkable = walkable
}

func (g *HexGrid) Clone() finder.Grid {
	ng := &HexGrid{
		width:       g.width,
		height:      g.height,
		staggerX:    g.staggerX,
		staggerEven: g.staggerEven,
		tileW:       g.tileW,
		tileH:       g.tileH,
		hexSide:     g.hexSide,
		sideLenX:    g.sideLenX,
		sideLenY:    g.sideLenY,
		sideOffX:    g.sideOffX,
		sideOffY:    g.sideOffY,
		colW:        g.colW,
		rowH:        g.rowH,
		renW:        g.renW,
		renH:        g.renH,
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

// FindPath finds a path between two world positions through the hex grid.
func (g *HexGrid) FindPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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

// GetNeighbors returns the 6 hex neighbors. Diagonal parameter is ignored
// because hex grids have no separate diagonal concept — all 6 are cardinal moves.
func (g *HexGrid) GetNeighbors(node *finder.Node, _ finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	x, y := node.X, node.Y
	neighbors := buffer[:0]
	w := g.width
	nodes := g.nodes

	var dirs [6][2]int
	if g.staggerX {
		// Flat-top: odd column shifts down
		if x%2 == 1 {
			dirs = [6][2]int{
				{0, -1}, // NW
				{1, -1}, // NE
				{1, 0},  // E
				{1, 1},  // SE
				{0, 1},  // SW
				{-1, 0}, // W
			}
		} else {
			dirs = [6][2]int{
				{-1, -1}, // NW
				{0, -1},  // NE
				{1, 0},   // E
				{0, 1},   // SE
				{-1, 1},  // SW
				{-1, 0},  // W
			}
		}
	} else {
		// Pointy-top: odd row shifts right
		if y%2 == 1 {
			dirs = [6][2]int{
				{1, 0},   // NE
				{0, -1},  // NW
				{-1, -1}, // W
				{-1, 0},  // SW
				{0, 1},   // SE
				{1, 1},   // E
			}
		} else {
			dirs = [6][2]int{
				{1, -1}, // NE
				{0, -1}, // NW
				{-1, 0}, // W
				{-1, 1}, // SW
				{0, 1},  // SE
				{1, 0},  // E
			}
		}
	}

	for _, d := range dirs {
		nx := x + d[0]
		ny := y + d[1]
		if g.IsWalkableAt(nx, ny) {
			neighbors = append(neighbors, nodes[ny*w+nx])
		}
	}

	return neighbors
}
