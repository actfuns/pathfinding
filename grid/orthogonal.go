package grid

import (
	"fmt"
	"strings"

	"github.com/actfuns/pathfinding/finder"
)

// OrthogonalGrid is a standard rectangular grid. Coordinates are in tile space.
// World-space helpers convert using tileW/tileH.
type OrthogonalGrid struct {
	tileW    int // world units per tile horizontally
	tileH    int // world units per tile vertically
	width    int // tiles across
	height   int // tiles down
	nodes    []*finder.Node
	finder   finder.Finder
	worldBuf [][2]float32
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

// Width returns the number of tiles horizontally in the grid.
func (g *OrthogonalGrid) Width() int { return g.width }

// Height returns the number of tiles vertically in the grid.
func (g *OrthogonalGrid) Height() int { return g.height }

// TileWidth returns the world-space width of a single tile.
func (g *OrthogonalGrid) TileWidth() int { return g.tileW }

// TileHeight returns the world-space height of a single tile.
func (g *OrthogonalGrid) TileHeight() int { return g.tileH }

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
	g.nodes[g.index(x, y)].Walkable = walkable
}

// Clone creates a deep copy of the grid. The returned grid has its own node
// slice; each node is copied, but the parent pointer is cleared. The finder
// reference and tile dimensions are shared from the original.
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

// FindPath finds a path between two world positions through the grid.
// The returned [][2]float32 is backed by an internal buffer and is only
// valid until the next FindPath/FindSmoothPath call on the same grid.
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
	if cap(g.worldBuf) >= len(path) {
		g.worldBuf = g.worldBuf[:len(path)]
	} else {
		g.worldBuf = make([][2]float32, len(path))
	}
	for i, p := range path {
		g.worldBuf[i][0], g.worldBuf[i][1] = g.TileToWorld(p[0], p[1])
	}
	// Use original world coordinates for first and last points
	g.worldBuf[0][0], g.worldBuf[0][1] = wx1, wy1
	g.worldBuf[len(g.worldBuf)-1][0], g.worldBuf[len(g.worldBuf)-1][1] = wx2, wy2
	return g.worldBuf
}

// FindSmoothPath finds a path between two world positions and smooths it.
// Same buffer contract as FindPath.
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

// SmoothenTilePath smooths a tile-coordinate path by removing unnecessary waypoints.
// Writes the result in-place over the input buffer (which is the finder's cached pathBuf),
// so the smoothed path reuses the same allocation.
func (g *OrthogonalGrid) SmoothenTilePath(path [][2]int) [][2]int {
	if len(path) < 2 {
		return path
	}
	writeIdx := 1
	for i := 2; i < len(path); i++ {
		if !g.tileLineOfSight(path[writeIdx-1], path[i]) {
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

// tileLineOfSight checks walkability along a Bresenham line between two tiles without allocating.
func (g *OrthogonalGrid) tileLineOfSight(a, b [2]int) bool {
	x0, y0 := a[0], a[1]
	x1, y1 := b[0], b[1]
	dx := x1 - x0
	dy := y1 - y0
	var sx, sy int
	if dx < 0 {
		dx = -dx
		sx = -1
	} else {
		sx = 1
	}
	if dy < 0 {
		dy = -dy
		sy = -1
	} else {
		sy = 1
	}
	err := dx - dy

	for {
		if !g.IsWalkableAt(x0, y0) {
			return false
		}
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
	return true
}

// --- neighbors ---

// GetNeighbors returns the walkable neighbors of the given node. The diagonal
// parameter controls which diagonal moves are permitted. The result is written
// into the provided buffer slice (which is resliced to zero and reused), so the
// caller must not hold references past the next call.
func (g *OrthogonalGrid) GetNeighbors(node *finder.Node, diagonal finder.DiagonalMovement, buffer []*finder.Node) []*finder.Node {
	x, y := node.X, node.Y
	nodes := g.nodes
	neighbors := buffer[:0]

	s0, d0 := false, false
	s1, d1 := false, false
	s2, d2 := false, false
	s3, d3 := false, false

	// Interior nodes (not on any edge) have all 4 orthogonal neighbors in bounds
	// — skip the IsInside check for those.
	if x > 0 && x < g.width-1 && y > 0 && y < g.height-1 {
		w := g.width
		if nodes[(y-1)*w+x].Walkable {
			neighbors = append(neighbors, nodes[(y-1)*w+x])
			s0 = true
		}
		if nodes[y*w+x+1].Walkable {
			neighbors = append(neighbors, nodes[y*w+x+1])
			s1 = true
		}
		if nodes[(y+1)*w+x].Walkable {
			neighbors = append(neighbors, nodes[(y+1)*w+x])
			s2 = true
		}
		if nodes[y*w+x-1].Walkable {
			neighbors = append(neighbors, nodes[y*w+x-1])
			s3 = true
		}
	} else {
		w := g.width
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
		neighbors = append(neighbors, nodes[(y-1)*g.width+x-1])
	}
	if d1 && g.IsWalkableAt(x+1, y-1) {
		neighbors = append(neighbors, nodes[(y-1)*g.width+x+1])
	}
	if d2 && g.IsWalkableAt(x+1, y+1) {
		neighbors = append(neighbors, nodes[(y+1)*g.width+x+1])
	}
	if d3 && g.IsWalkableAt(x-1, y+1) {
		neighbors = append(neighbors, nodes[(y+1)*g.width+x-1])
	}

	return neighbors
}

// RenderSVG renders the grid and an optional path as an SVG string.
func (g *OrthogonalGrid) RenderSVG(path [][2]int, startX, startY, endX, endY int) string {
	cellW := 40
	cellH := 40
	padding := 20
	width := g.width*cellW + padding*2
	height := g.height*cellH + padding*2

	var b strings.Builder
	b.WriteString(xmlHeader(width, height))

	for y := 0; y < g.height; y++ {
		for x := 0; x < g.width; x++ {
			rx := padding + x*cellW
			ry := padding + y*cellH
			fill := "#ffffff"
			stroke := "#cccccc"
			if !g.IsWalkableAt(x, y) {
				fill = "#333333"
				stroke = "#333333"
			}
			fmt.Fprintf(&b, `<rect x="%d" y="%d" width="%d" height="%d" fill="%s" stroke="%s" stroke-width="1"/>`+"\n",
				rx, ry, cellW, cellH, fill, stroke)
		}
	}

	drawPathAndMarkers(&b, path, startX, startY, endX, endY,
		func(tx, ty int) (float64, float64) {
			return float64(padding + tx*cellW + cellW/2),
				float64(padding + ty*cellH + cellH/2)
		})

	b.WriteString("</svg>\n")
	return b.String()
}

// RenderSVG renders the hex grid and an optional path as an SVG string.

func drawPathAndMarkers(b *strings.Builder, path [][2]int, startX, startY, endX, endY int,
	centerOf func(tx, ty int) (float64, float64)) {

	if len(path) > 0 {
		pts := make([]string, len(path))
		for i, p := range path {
			cx, cy := centerOf(p[0], p[1])
			pts[i] = fmt.Sprintf("%.1f,%.1f", cx, cy)
		}
		fmt.Fprintf(b, `<polyline points="%s" fill="none" stroke="#0066cc" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>`+"\n",
			strings.Join(pts, " "))
	}

	sx, sy := centerOf(startX, startY)
	fmt.Fprintf(b, `<circle cx="%.1f" cy="%.1f" r="6" fill="#00cc44" stroke="#009933" stroke-width="2"/>`+"\n", sx, sy)

	ex, ey := centerOf(endX, endY)
	fmt.Fprintf(b, `<circle cx="%.1f" cy="%.1f" r="6" fill="#cc0000" stroke="#990000" stroke-width="2"/>`+"\n", ex, ey)
}

func xmlHeader(w, h int) string {
	return fmt.Sprintf(`<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">
`, w, h, w, h)
}
