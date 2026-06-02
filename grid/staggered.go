package grid

import (
	"math"

	"github.com/actfuns/pathfinding/finder"
)

// StaggeredGrid is a 45-degree isometric/staggered grid (diamond-shaped tiles).
// It mirrors Tiled's "staggered" orientation, using the HexagonalRenderer math
// but overrides screenToTileCoords with a 4-corner detection + 45° rotation
// (exactly as Tiled's StaggeredRenderer does).
type StaggeredGrid struct {
	staggerAxis  string // "x" or "y" — which axis is staggered
	staggerIndex string // "even" or "odd" — which indexes are shifted
	tileW        int    // pixel width of a tile
	tileH        int    // pixel height of a tile
	width        int    // tiles across
	height       int    // tiles down
	nodes        []*finder.Node
	finder       finder.Finder
}

// Finder returns the pathfinder associated with this grid.
func (g *StaggeredGrid) Finder() finder.Finder { return g.finder }

// StaggerOption configures a StaggeredGrid.
type StaggerOption func(*StaggeredGrid)

// WithStaggerTileSize sets the pixel dimensions of the staggered tile.
func WithStaggerTileSize(w, h int) StaggerOption {
	return func(g *StaggeredGrid) { g.tileW = w; g.tileH = h }
}

// WithStaggerAxis sets the stagger axis ("x" or "y"). Default is "y".
func WithStaggerAxis(axis string) StaggerOption {
	return func(g *StaggeredGrid) { g.staggerAxis = axis }
}

// WithStaggerEvenIndex configures even-index stagger offset. Default is odd-index.
func WithStaggerEvenIndex() StaggerOption {
	return func(g *StaggeredGrid) { g.staggerIndex = "even" }
}

// WithStaggerFinder sets the pathfinder and returns the grid.
func WithStaggerFinder(f finder.Finder) StaggerOption {
	return func(g *StaggeredGrid) { g.finder = f }
}

// NewStaggeredGrid creates a StaggeredGrid from a matrix (0=walkable, non-zero=obstacle).
// Default tile size is 64x64. Default stagger axis is "y" (odd rows shift right).
// Default finder is AStarFinder.
func NewStaggeredGrid(matrix [][]int, opts ...StaggerOption) *StaggeredGrid {
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
	g := &StaggeredGrid{
		staggerAxis:  "y",
		staggerIndex: "odd",
		tileW:        64,
		tileH:        64,
		width:        w,
		height:       h,
		nodes:        nodes,
		finder:       finder.NewAStarFinder(),
	}
	for _, opt := range opts {
		opt(g)
	}
	return g
}

func (g *StaggeredGrid) isStaggerX() bool    { return g.staggerAxis == "x" }
func (g *StaggeredGrid) isStaggerEven() bool { return g.staggerIndex == "even" }
func (g *StaggeredGrid) isShifted(index int) bool {
	// Use bitwise AND for correct behavior with negative indices (matching Tiled's C++ (y & 1))
	if g.isStaggerEven() {
		return index&1 == 0
	}
	return index&1 == 1
}

// --- tile coordinate implementations of finder.Grid ---

func (g *StaggeredGrid) index(x, y int) int { return y*g.width + x }
func (g *StaggeredGrid) Width() int         { return g.width }
func (g *StaggeredGrid) Height() int        { return g.height }
func (g *StaggeredGrid) IsInside(x, y int) bool {
	return x >= 0 && x < g.width && y >= 0 && y < g.height
}
func (g *StaggeredGrid) GetNodeAt(x, y int) *finder.Node { return g.nodes[g.index(x, y)] }

func (g *StaggeredGrid) IsWalkableAt(x, y int) bool {
	if !g.IsInside(x, y) {
		return false
	}
	return g.nodes[g.index(x, y)].Walkable
}

func (g *StaggeredGrid) SetWalkableAt(x, y int, walkable bool) {
	g.nodes[g.index(x, y)].Walkable = walkable
}

func (g *StaggeredGrid) Clone() finder.Grid {
	ng := &StaggeredGrid{
		staggerAxis:  g.staggerAxis,
		staggerIndex: g.staggerIndex,
		tileW:        g.tileW,
		tileH:        g.tileH,
		width:        g.width,
		height:       g.height,
		finder:       g.finder,
	}
	ng.nodes = make([]*finder.Node, len(g.nodes))
	for i, n := range g.nodes {
		cp := *n
		cp.Parent = nil
		ng.nodes[i] = &cp
	}
	return ng
}

// --- world coordinate conversion (Tiled StaggeredRenderer) ---

// tileToScreenCoords converts tile coords to pixel coords for staggered isometric.
// Uses Tiled's HexagonalRenderer::tileToScreenCoords with hexSide=0 (diamond shape).
// With hexSide=0: sideOffsetX = tileW/2, sideOffsetY = tileH/2, columnWidth = tileW/2, rowHeight = tileH/2.
func (g *StaggeredGrid) tileToScreenCoords(tx, ty int) (float64, float64) {
	tileW := float64(g.tileW)
	tileH := float64(g.tileH)

	if g.isStaggerX() {
		// stagger on X: odd columns shift down
		px := float64(tx) * (tileW / 2)
		py := float64(ty) * (tileH)
		if g.isShifted(tx) {
			py += tileH / 2
		}
		return px, py
	}
	// stagger on Y: odd rows shift right
	px := float64(tx) * tileW
	if g.isShifted(ty) {
		px += tileW / 2
	}
	py := float64(ty) * (tileH / 2)
	return px, py
}

// screenToTileCoords converts pixel/screen coords to tile coords for staggered isometric.
// Mirrors Tiled's StaggeredRenderer::screenToTileCoords with 4-corner detection + 45° rotation.
func (g *StaggeredGrid) screenToTileCoords(sx, sy float64) (int, int) {
	tileW := float64(g.tileW)
	tileH := float64(g.tileH)
	halfW := tileW / 2
	halfH := tileH / 2

	alignedX, alignedY := sx, sy
	if g.isStaggerX() {
		if g.isStaggerEven() {
			alignedX -= halfW
		}
	} else {
		if g.isStaggerEven() {
			alignedY -= halfH
		}
	}

	// Start with grid-aligned tile coordinates
	refX := int(math.Floor(alignedX / tileW))
	refY := int(math.Floor(alignedY / tileH))

	// Relative position within the base square
	relX := alignedX - float64(refX)*tileW
	relY := alignedY - float64(refY)*tileH

	// Adjust reference: stagger axis index *=2, +1 if staggerEven
	if g.isStaggerX() {
		refX *= 2
		if g.isStaggerEven() {
			refX++
		}
	} else {
		refY *= 2
		if g.isStaggerEven() {
			refY++
		}
	}

	// y_pos = relX * (tileH / tileW)
	yPos := relX * (tileH / tileW)

	// 4-corner detection using sideOffset = tileW/2 for staggered (diamond shape)
	sideOffY := halfH // equivalent to (tileH - 0)/2 for staggered

	if sideOffY-yPos > relY {
		// topLeft
		refX, refY = staggeredTopLeft(refX, refY, g.isStaggerX(), g.isStaggerEven())
	} else if sideOffY*(-1)+yPos > relY {
		// topRight
		refX, refY = staggeredTopRight(refX, refY, g.isStaggerX(), g.isStaggerEven())
	} else if sideOffY+yPos < relY {
		// bottomLeft
		refX, refY = staggeredBottomLeft(refX, refY, g.isStaggerX(), g.isStaggerEven())
	} else if sideOffY*3-yPos < relY {
		// bottomRight
		refX, refY = staggeredBottomRight(refX, refY, g.isStaggerX(), g.isStaggerEven())
	}

	// Sub-tile offset: convert to isometric local coords
	// Get the pixel position of the found tile
	tilePX, tilePY := g.tileToScreenCoords(refX, refY)
	localX := sx - tilePX
	localY := sy - tilePY

	// Translate to center of diamond, then rotate -45°
	// Mirrors Tiled's StaggeredRenderer::screenToTileCoords
	tileLocalX := localX - tileW/2
	tileLocalY := localY

	// Scale Y by tileW/tileH before rotation (isometric correction)
	tileLocalY *= tileW / tileH

	// Rotate -45 degrees
	cosA := 0.7071067811865476
	sinA := -0.7071067811865476
	rx := tileLocalX*cosA - tileLocalY*sinA
	ry := tileLocalX*sinA + tileLocalY*cosA

	// Scale: divide by (tileW/sqrt(2))
	scale := tileW * 0.7071067811865476
	rx /= scale
	ry /= scale

	// Add raw fractional offset and floor (Tiled returns QPointF, the caller floors it)
	return int(math.Floor(float64(refX) + rx)), int(math.Floor(float64(refY) + ry))
}

// staggeredTopLeft returns the top-left neighbor in staggered coordinates.
// Mirrors HexagonalRenderer::topLeft.
func staggeredTopLeft(x, y int, staggerX, staggerEven bool) (int, int) {
	if staggerX {
		if (x & 1) != 0 != staggerEven {
			return x - 1, y
		}
		return x - 1, y - 1
	}
	// staggerY
	if (y & 1) != 0 != staggerEven {
		return x, y - 1
	}
	return x - 1, y - 1
}

// staggeredTopRight returns the top-right neighbor in staggered coordinates.
func staggeredTopRight(x, y int, staggerX, staggerEven bool) (int, int) {
	if staggerX {
		if (x & 1) != 0 != staggerEven {
			return x + 1, y
		}
		return x + 1, y - 1
	}
	if (y & 1) != 0 != staggerEven {
		return x + 1, y - 1
	}
	return x, y - 1
}

// staggeredBottomLeft returns the bottom-left neighbor in staggered coordinates.
func staggeredBottomLeft(x, y int, staggerX, staggerEven bool) (int, int) {
	if staggerX {
		if (x & 1) != 0 != staggerEven {
			return x - 1, y + 1
		}
		return x - 1, y
	}
	if (y & 1) != 0 != staggerEven {
		return x, y + 1
	}
	return x - 1, y + 1
}

// staggeredBottomRight returns the bottom-right neighbor in staggered coordinates.
func staggeredBottomRight(x, y int, staggerX, staggerEven bool) (int, int) {
	if staggerX {
		if (x & 1) != 0 != staggerEven {
			return x + 1, y + 1
		}
		return x + 1, y
	}
	if (y & 1) != 0 != staggerEven {
		return x + 1, y + 1
	}
	return x, y + 1
}

// WorldToTile converts world coordinates to staggered tile coordinates.
// Mirrors Tiled's StaggeredRenderer::screenToTileCoords.
func (g *StaggeredGrid) WorldToTile(wx, wy float32) (int, int) {
	return g.screenToTileCoords(float64(wx), float64(wy))
}

// TileToWorld converts tile coordinates to world position.
func (g *StaggeredGrid) TileToWorld(tx, ty int) (float32, float32) {
	px, py := g.tileToScreenCoords(tx, ty)
	return float32(px), float32(py)
}

// IsWalkableAtWorld checks walkability at a world position.
func (g *StaggeredGrid) IsWalkableAtWorld(wx, wy float32) bool {
	tx, ty := g.WorldToTile(wx, wy)
	return g.IsWalkableAt(tx, ty)
}

// SetWalkableAtWorld sets walkability at a world position.
func (g *StaggeredGrid) SetWalkableAtWorld(wx, wy float32, walkable bool) {
	tx, ty := g.WorldToTile(wx, wy)
	g.SetWalkableAt(tx, ty, walkable)
}

// FindPath finds a path between two world positions through the staggered grid.
func (g *StaggeredGrid) FindPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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
	wp[0][0], wp[0][1] = wx1, wy1
	wp[len(wp)-1][0], wp[len(wp)-1][1] = wx2, wy2
	return wp
}

// FindSmoothPath finds a path between two world positions and smooths it.
func (g *StaggeredGrid) FindSmoothPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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

// SmoothenTilePath smooths a tile-coordinate staggered path by removing unnecessary waypoints.
// Uses a pixel-based ray cast along the line between tiles.
func (g *StaggeredGrid) SmoothenTilePath(path [][2]int) [][2]int {
	if len(path) < 2 {
		return path
	}
	smooth := make([]int, 0, len(path))
	smooth = append(smooth, 0)
	last := len(path) - 1
	for i := 2; i < len(path); i++ {
		if !g.staggeredLineOfSight(path[smooth[len(smooth)-1]], path[i]) {
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

// staggeredLineOfSight checks if there is a straight pixel line between two staggered tiles with no obstacles.
func (g *StaggeredGrid) staggeredLineOfSight(a, b [2]int) bool {
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

// GetNeighbors returns neighbors for staggered isometric grids.
func (g *StaggeredGrid) GetNeighbors(node *finder.Node, diagonal finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	x, y := node.X, node.Y
	neighbors := buffer[:0]
	w := g.width
	nodes := g.nodes

	shifted := g.isShifted(y) // default stagger axis is "y"

	// Cardinal neighbors
	var cardinals [][2]int
	if g.isStaggerX() {
		shifted = g.isShifted(x)
		// x-axis stagger
		if shifted {
			cardinals = [][2]int{
				{x - 1, y}, // W
				{x + 1, y}, // E
				{x, y - 1}, // N (same x)
				{x, y + 1}, // S (same x)
			}
		} else {
			cardinals = [][2]int{
				{x - 1, y}, // W
				{x + 1, y}, // E
				{x, y - 1}, // N (same x)
				{x, y + 1}, // S (same x)
			}
		}
	} else {
		// y-axis stagger
		cardinals = [][2]int{
			{x, y - 1}, // N
			{x, y + 1}, // S
			{x - 1, y}, // W
			{x + 1, y}, // E
		}
	}

	for _, d := range cardinals {
		if g.IsWalkableAt(d[0], d[1]) {
			neighbors = append(neighbors, nodes[d[1]*w+d[0]])
		}
	}

	if diagonal == finder.DiagonalNever {
		return neighbors
	}

	// Diagonal neighbors — depends on stagger axis and shift
	var diagonals [][2]int
	if g.isStaggerX() {
		if shifted {
			diagonals = [][2]int{
				{x, y - 1},     // NE (same x)
				{x - 1, y - 1}, // NW
				{x, y + 1},     // SE (same x)
				{x - 1, y + 1}, // SW
			}
		} else {
			diagonals = [][2]int{
				{x + 1, y - 1}, // NE
				{x, y - 1},     // NW (same x)
				{x + 1, y + 1}, // SE
				{x, y + 1},     // SW (same x)
			}
		}
	} else {
		if shifted {
			diagonals = [][2]int{
				{x, y - 1},     // NE (same x)
				{x - 1, y - 1}, // NW
				{x, y + 1},     // SE (same x)
				{x - 1, y + 1}, // SW
			}
		} else {
			diagonals = [][2]int{
				{x + 1, y - 1}, // NE
				{x, y - 1},     // NW (same x)
				{x + 1, y + 1}, // SE
				{x, y + 1},     // SW (same x)
			}
		}
	}

	// Apply diagonal obstacle rules
	var dFlags [4]bool
	switch diagonal {
	case finder.DiagonalAlways:
		dFlags = [4]bool{true, true, true, true}
	case finder.DiagonalOnlyWhenNoObstacles:
		for i := 0; i < 4; i++ {
			d := diagonals[i]
			dx, dy := d[0]-x, d[1]-y
			ok := true
			if dx != 0 && g.IsInside(x+dx, y) {
				if !g.IsWalkableAt(x+dx, y) {
					ok = false
				}
			}
			if dy != 0 && g.IsInside(x, y+dy) {
				if !g.IsWalkableAt(x, y+dy) {
					ok = false
				}
			}
			dFlags[i] = ok
		}
	case finder.DiagonalIfAtMostOneObstacle:
		for i := 0; i < 4; i++ {
			d := diagonals[i]
			dx, dy := d[0]-x, d[1]-y
			blocked := 0
			if dx != 0 && g.IsInside(x+dx, y) && !g.IsWalkableAt(x+dx, y) {
				blocked++
			}
			if dy != 0 && g.IsInside(x, y+dy) && !g.IsWalkableAt(x, y+dy) {
				blocked++
			}
			dFlags[i] = blocked <= 1
		}
	default:
		return neighbors
	}

	for i, d := range diagonals {
		if dFlags[i] && g.IsWalkableAt(d[0], d[1]) {
			neighbors = append(neighbors, nodes[d[1]*w+d[0]])
		}
	}

	return neighbors
}
