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

	nodes    []*finder.Node
	finder   finder.Finder
	worldBuf [][2]float32

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
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			n := finder.NewNode(x, y)
			n.Walkable = matrix[y][x] == 0
			nodes = append(nodes, n)
		}
	}

	g := &HexGrid{
		width:  w,
		height: h,
		tileW:  64,
		tileH:  64,
		nodes:  nodes,
		finder: finder.NewAStarFinder(),
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
	g.nodes[g.index(x, y)].Walkable = walkable
}

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

// FindPath finds a path between two world positions through the hex grid.
// The returned [][2]float32 is backed by an internal buffer and is only
// valid until the next FindPath/FindSmoothPath call on the same grid.
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
func (g *HexGrid) FindSmoothPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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
func (g *HexGrid) SmoothenTilePath(path [][2]int) [][2]int {
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

// RenderSVG renders the hex grid and an optional path as an SVG string.
func (g *HexGrid) RenderSVG(path [][2]int, startX, startY, endX, endY int) string {
	padding := 20.0

	// Find bounds of all tile center points
	minX, minY := math.MaxFloat64, math.MaxFloat64
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
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
			cx, cy := g.tileToScreenCoords(x, y)
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

	svgW := (vxMax - vxMin) + padding*2
	svgH := (vyMax - vyMin) + padding*2
	dx := padding - vxMin
	dy := padding - vyMin

	var b strings.Builder
	b.WriteString(xmlHeader(int(math.Ceil(svgW)), int(math.Ceil(svgH))))

	// Draw tiles
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			fill := "#ffffff"
			stroke := "#cccccc"
			if !g.IsWalkableAt(x, y) {
				fill = "#333333"
				stroke = "#333333"
			}
			pts := make([]string, len(hexOffsets))
			for i, off := range hexOffsets {
				pts[i] = fmt.Sprintf("%.1f,%.1f", cx+off[0]+dx, cy+off[1]+dy)
			}
			fmt.Fprintf(&b, `<polygon points="%s" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				strings.Join(pts, " "), fill, stroke)
		}
	}

	drawPathAndMarkers(&b, path, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			cx, cy := g.tileToScreenCoords(tx, ty)
			var cxOff, cyOff float64
			if g.staggerX {
				cxOff = float64(g.tileW) / 2
				cyOff = float64(g.renH) / 2
			} else {
				cxOff = float64(g.renW) / 2
				cyOff = float64(g.tileH) / 2
			}
			return cx + cxOff + dx, cy + cyOff + dy
		})

	b.WriteString("</svg>\n")
	return b.String()
}
