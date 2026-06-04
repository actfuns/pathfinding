package grid

import (
	"fmt"
	"math"
	"strings"

	"github.com/actfuns/pathfinding/finder"
)

// fastRand returns a pseudo-random uint32 (xorshift).


// StaggeredGrid is a 45-degree isometric/staggered grid (diamond-shaped tiles).
// It mirrors Tiled's "staggered" orientation, using the HexagonalRenderer math
// but overrides screenToTileCoords with a 4-corner detection + 45° rotation
// (exactly as Tiled's StaggeredRenderer does).
type StaggeredGrid struct {
	staggerAxis   string // "x" or "y" — which axis is staggered
	staggerIndex  string // "even" or "odd" — which indexes are shifted
	tileW         int    // pixel width of a tile
	tileH         int    // pixel height of a tile
	width         int    // tiles across
	height        int    // tiles down
	nodes         []*finder.Node
	finder        finder.Finder
	worldBuf      [][2]float32
	obstacleCount int

	// Precomputed cardinal and diagonal neighbor offsets
	cardinalOffsets  [4][2]int
	diagNormOffsets  [4][2]int // diagonal offsets for non-shifted rows/cols
	diagShiftOffsets [4][2]int // diagonal offsets for shifted rows/cols
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
	g := &StaggeredGrid{
		staggerAxis:   "y",
		staggerIndex:  "odd",
		tileW:         64,
		tileH:         64,
		width:         w,
		height:        h,
		obstacleCount: obs,
		nodes:         nodes,
		finder:        finder.NewAStarFinder(),
	}
	for _, opt := range opts {
		opt(g)
	}
	g.initNeighbors()
	return g
}

func (g *StaggeredGrid) initNeighbors() {
	g.cardinalOffsets = [4][2]int{
		{0, -1}, // N
		{0, 1},  // S
		{-1, 0}, // W
		{1, 0},  // E
	}
	if g.isStaggerX() {
		g.diagNormOffsets = [4][2]int{
			{0, -1},  // NE
			{-1, -1}, // NW
			{0, 1},   // SE
			{-1, 1},  // SW
		}
		g.diagShiftOffsets = [4][2]int{
			{1, -1}, // NE
			{0, -1}, // NW
			{1, 1},  // SE
			{0, 1},  // SW
		}
	} else {
		g.diagNormOffsets = [4][2]int{
			{1, -1},  // NE
			{-1, -1}, // NW
			{1, 1},   // SE
			{-1, 1},  // SW
		}
		g.diagShiftOffsets = [4][2]int{
			{1, 0},  // NE
			{-1, 0}, // NW
			{1, 0},  // SE
			{-1, 0}, // SW
		}
	}
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

// Width returns the number of tiles horizontally in the staggered grid.
func (g *StaggeredGrid) Width() int { return g.width }

// Height returns the number of tiles vertically in the staggered grid.
func (g *StaggeredGrid) Height() int { return g.height }

// IsInside checks whether the tile coordinates (x, y) are within the grid bounds.
func (g *StaggeredGrid) IsInside(x, y int) bool {
	return x >= 0 && x < g.width && y >= 0 && y < g.height
}

// GetNodeAt returns the node at the given tile coordinates.
func (g *StaggeredGrid) GetNodeAt(x, y int) *finder.Node { return g.nodes[g.index(x, y)] }

// IsWalkableAt returns whether the tile at (x, y) is walkable (no obstacle).
func (g *StaggeredGrid) IsWalkableAt(x, y int) bool {
	if !g.IsInside(x, y) {
		return false
	}
	return g.nodes[g.index(x, y)].Walkable
}

// SetWalkableAt sets the walkability of the tile at (x, y).
func (g *StaggeredGrid) SetWalkableAt(x, y int, walkable bool) {
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
func (g *StaggeredGrid) SetWeightAt(x, y int, weight float64) {
	g.nodes[g.index(x, y)].Weight = weight
}

// GetWeightAt returns the movement cost multiplier for the tile at (x, y).
func (g *StaggeredGrid) GetWeightAt(x, y int) float64 {
	if !g.IsInside(x, y) {
		return 1.0
	}
	return g.nodes[g.index(x, y)].Weight
}

// RandomWalkableTile returns a random walkable tile coordinate.
// Returns (-1, -1) if no walkable tile exists.
func (g *StaggeredGrid) RandomWalkableTile() (int, int) {
	if g.obstacleCount >= g.width*g.height {
		return -1, -1
	}
	for i := 0; i < 100; i++ {
		x := int(uint32(g.width) * fastRand())
		y := int(uint32(g.height) * fastRand())
		if g.IsWalkableAt(x, y) {
			return x, y
		}
	}
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			if g.IsWalkableAt(x, y) {
				return x, y
			}
		}
	}
	return -1, -1
}

// RandomWalkableTileWorld returns the world center of a random walkable tile.
func (g *StaggeredGrid) RandomWalkableTileWorld() (float32, float32, bool) {
	tx, ty := g.RandomWalkableTile()
	if tx < 0 {
		return 0, 0, false
	}
	wx, wy := g.TileToWorld(tx, ty)
	return wx, wy, true
}

// RandomWalkableTileInRadius returns a random walkable tile within radius tiles of (cx, cy).
func (g *StaggeredGrid) RandomWalkableTileInRadius(cx, cy, radius int) (int, int) {
	for i := 0; i < 50; i++ {
		dx := int(uint32(2*radius+1)*fastRand()) - radius
		dy := int(uint32(2*radius+1)*fastRand()) - radius
		x, y := cx+dx, cy+dy
		if g.IsWalkableAt(x, y) {
			return x, y
		}
	}
	return -1, -1
}

// RandomWalkableTileInRadiusWorld returns the world center of a random walkable tile
// within radius tiles of (wx, wy).
func (g *StaggeredGrid) RandomWalkableTileInRadiusWorld(wx, wy float32, radius int) (float32, float32, bool) {
	tx, ty := g.WorldToTile(wx, wy)
	rtx, rty := g.RandomWalkableTileInRadius(tx, ty, radius)
	if rtx < 0 {
		return 0, 0, false
	}
	wx2, wy2 := g.TileToWorld(rtx, rty)
	return wx2, wy2, true
}

// HasLineOfSight reports whether two tiles see each other via Bresenham.
func (g *StaggeredGrid) HasLineOfSight(x1, y1, x2, y2 int) bool {
	dx := x2 - x1
	dy := y2 - y1
	var sx, sy int
	if dx < 0 { dx = -dx; sx = -1 } else { sx = 1 }
	if dy < 0 { dy = -dy; sy = -1 } else { sy = 1 }
	err := dx - dy
	x, y := x1, y1
	for {
		if !g.IsWalkableAt(x, y) {
			return false
		}
		if x == x2 && y == y2 {
			break
		}
		e2 := 2 * err
		if e2 > -dy { err -= dy; x += sx }
		if e2 < dx { err += dx; y += sy }
	}
	return true
}

// HasLineOfSightWorld reports whether two world positions can see each other.
func (g *StaggeredGrid) HasLineOfSightWorld(x1, y1, x2, y2 float32) bool {
	tx1, ty1 := g.WorldToTile(x1, y1)
	tx2, ty2 := g.WorldToTile(x2, y2)
	return g.HasLineOfSight(tx1, ty1, tx2, ty2)
}


// ObstacleCount returns the number of non-walkable tiles in the grid.
func (g *StaggeredGrid) ObstacleCount() int { return g.obstacleCount }

// Clone returns a deep copy of the staggered grid with independent node data.
func (g *StaggeredGrid) Clone() finder.Grid {
	ng := &StaggeredGrid{
		cardinalOffsets:  g.cardinalOffsets,
		diagNormOffsets:  g.diagNormOffsets,
		diagShiftOffsets: g.diagShiftOffsets,
		staggerAxis:      g.staggerAxis,
		staggerIndex:     g.staggerIndex,
		tileW:            g.tileW,
		tileH:            g.tileH,
		width:            g.width,
		height:           g.height,
		obstacleCount:    g.obstacleCount,
		finder:           g.finder,
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

// tileEdgePoint returns the closest point on the edge of tile (tx, ty) to (wx, wy),
// inset by one world-space unit inward from the tile boundary.
func (g *StaggeredGrid) tileEdgePoint(tx, ty int, wx, wy, inset float32) (float32, float32) {
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
func (g *StaggeredGrid) FindNearestWalkable(wx, wy float32, maxRadius int, edgeInset float32) (float32, float32, bool) {
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
func (g *StaggeredGrid) FindNearestWalkableTile(wx, wy float32, maxRadius int) (int, int, bool) {
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

// FindPath finds a path between two world positions through the staggered grid.
// The returned [][2]float32 is backed by an internal buffer and is only
// valid until the next FindPath/FindSmoothPath call on the same grid.
func (g *StaggeredGrid) FindPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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
func (g *StaggeredGrid) FindSmoothPath(wx1, wy1, wx2, wy2 float32) [][2]float32 {
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

// SmoothenTilePath smooths a tile-coordinate staggered path by removing unnecessary waypoints.
// Writes the smoothed result in-place over the input (finder's cached pathBuf).
func (g *StaggeredGrid) SmoothenTilePath(path [][2]int) [][2]int {
	if len(path) < 2 {
		return path
	}
	writeIdx := 1
	for i := 2; i < len(path); i++ {
		if !g.staggeredLineOfSight(path[writeIdx-1], path[i]) {
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

// GetNeighbors returns neighbors for a node on a staggered isometric grid, respecting the
// diagonal movement rule. The result is written into the provided buffer.
func (g *StaggeredGrid) GetNeighbors(node *finder.Node, diagonal finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	x, y := node.X, node.Y
	neighbors := buffer[:0]
	w := g.width
	h := g.height
	nodes := g.nodes

	shifted := g.isShifted(y)
	if g.isStaggerX() {
		shifted = g.isShifted(x)
	}

	// Cardinal neighbors (same 4 directions regardless of stagger)
	for _, d := range g.cardinalOffsets {
		nx := x + d[0]
		ny := y + d[1]
		if nx >= 0 && nx < w && ny >= 0 && ny < h && nodes[ny*w+nx].Walkable {
			neighbors = append(neighbors, nodes[ny*w+nx])
		}
	}

	if diagonal == finder.DiagonalNever {
		return neighbors
	}

	// Diagonal neighbors using precomputed offsets
	var diagOffsets [4][2]int
	if shifted {
		diagOffsets = g.diagShiftOffsets
	} else {
		diagOffsets = g.diagNormOffsets
	}

	// Build absolute diagonal positions
	type diagPos struct{ nx, ny int }
	var diags [4]diagPos
	for i, d := range diagOffsets {
		diags[i] = diagPos{x + d[0], y + d[1]}
	}

	// Apply diagonal obstacle rules
	var dFlags [4]bool
	switch diagonal {
	case finder.DiagonalAlways:
		dFlags = [4]bool{true, true, true, true}
	case finder.DiagonalOnlyWhenNoObstacles:
		for i, d := range diagOffsets {
			dx, dy := d[0], d[1]
			ok := true
			if dx != 0 && x+dx >= 0 && x+dx < w && !nodes[y*w+x+dx].Walkable {
				ok = false
			}
			if dy != 0 && y+dy >= 0 && y+dy < h && !nodes[(y+dy)*w+x].Walkable {
				ok = false
			}
			dFlags[i] = ok
		}
	case finder.DiagonalIfAtMostOneObstacle:
		for i, d := range diagOffsets {
			dx, dy := d[0], d[1]
			blocked := 0
			if dx != 0 && x+dx >= 0 && x+dx < w && !nodes[y*w+x+dx].Walkable {
				blocked++
			}
			if dy != 0 && y+dy >= 0 && y+dy < h && !nodes[(y+dy)*w+x].Walkable {
				blocked++
			}
			dFlags[i] = blocked <= 1
		}
	}

	for i, d := range diags {
		if dFlags[i] && d.nx >= 0 && d.nx < w && d.ny >= 0 && d.ny < h && nodes[d.ny*w+d.nx].Walkable {
			neighbors = append(neighbors, nodes[d.ny*w+d.nx])
		}
	}

	return neighbors
}

// RenderSVG renders the staggered grid with weight-colored tiles and paths.
func (g *StaggeredGrid) RenderSVG(cfg *SVGOpts, paths ...[][2]int) string {
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
	tileW := float64(g.tileW)
	tileH := float64(g.tileH)

	// Diamond vertices relative to tile center
	diamond := [][2]float64{
		{tileW / 2, 0},
		{tileW, tileH / 2},
		{tileW / 2, tileH},
		{0, tileH / 2},
	}

	// Find bounds of all diamond vertices
	vxMin, vyMin := math.MaxFloat64, math.MaxFloat64
	vxMax, vyMax := -math.MaxFloat64, -math.MaxFloat64
	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			for _, off := range diamond {
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
	dx := padding - vxMin
	dy := padding - vyMin

	var b strings.Builder
	b.WriteString(xmlHeader(int(math.Ceil(svgW)), int(math.Ceil(svgH))))

	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			cx, cy := g.tileToScreenCoords(x, y)
			fill, stroke := "#c8e6c9", "#cccccc"
			if !g.IsWalkableAt(x, y) {
				fill, stroke = "#555555", "#444444"
			} else {
				fill = weightToColor(g.nodes[g.index(x, y)].Weight, cfg)
			}
			pts := make([]string, len(diamond))
			for i, off := range diamond {
				pts[i] = fmt.Sprintf("%.1f,%.1f", cx+off[0]+dx, cy+off[1]+dy)
			}
			fmt.Fprintf(&b, `<polygon points="%s" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				strings.Join(pts, " "), fill, stroke)
		}
	}

	drawPathAndMarkers(&b, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			cx, cy := g.tileToScreenCoords(tx, ty)
			return cx + tileW/2 + dx, cy + tileH/2 + dy
		}, paths...)

	renderLegend(&b, svgW-130, padding, cfg)
	b.WriteString("</svg>\n")
	return b.String()
}

// drawPathAndMarkers draws the path polyline and start/end markers.
// centerOf returns the SVG pixel coordinates for a tile.
