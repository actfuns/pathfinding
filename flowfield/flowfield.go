// Package flowfield implements Flow Field pathfinding for efficient
// group movement. Instead of computing per-agent paths, it builds a
// vector field from the goal outward (one Dijkstra pass), then every
// agent looks up its direction in O(1).
//
// Single target:
//
//	ff := flowfield.New(grid, goalX, goalY)
//	for _, a := range agents {
//	    dx, dy := ff.GetDirection(a.X, a.Y)
//	    a.X += dx; a.Y += dy
//	}
//
// Multiple targets (each tile routes to the nearest goal):
//
//	ff := flowfield.NewMulti(grid, [][2]int{{3,5}, {9,7}})
//	ff.Reset(7, 3)                          // switch to single goal
//	ff.ResetMulti([][2]int{{1,1}, {9,9}})   // switch to multi goal
package flowfield

import (
	"github.com/actfuns/pathfinding/finder"
)

// Field holds the precomputed integration and vector fields.
type Field struct {
	width, height int
	grid          finder.Grid
	diagonal      finder.DiagonalMovement

	cost []float64 // integration cost to nearest goal
	dirs []dir2    // direction to cheapest neighbour

	neighborBuf []*finder.Node
	queue       []item
}

type dir2 struct{ dx, dy int }

type item struct {
	x, y int
	g    float64
}

// Option configures the flow field.
type Option func(*Field)

// WithDiagonal sets the diagonal movement rule.
// Defaults to DiagonalNever (cardinal-only).
func WithDiagonal(d finder.DiagonalMovement) Option {
	return func(f *Field) { f.diagonal = d }
}

// initField allocates + configures a Field (shared by New / NewMulti).
func initField(grid finder.Grid, opts []Option) *Field {
	w, h := grid.Width(), grid.Height()
	ff := &Field{
		width:       w,
		height:      h,
		grid:        grid,
		diagonal:    finder.DiagonalNever,
		cost:        make([]float64, w*h),
		dirs:        make([]dir2, w*h),
		neighborBuf: make([]*finder.Node, 0, 8),
		queue:       make([]item, 0, w*h),
	}
	for _, opt := range opts {
		opt(ff)
	}
	return ff
}

// resetCosts sets all costs to "unvisited" (-1).
func (ff *Field) resetCosts() {
	for i := range ff.cost {
		ff.cost[i] = -1
	}
}

// New builds a flow field toward a single goal tile.
func New(grid finder.Grid, goalX, goalY int, opts ...Option) *Field {
	ff := initField(grid, opts)
	ff.Reset(goalX, goalY)
	return ff
}

// NewMulti builds a flow field toward multiple goals.
// Each tile routes to the nearest goal automatically.
func NewMulti(grid finder.Grid, goals [][2]int, opts ...Option) *Field {
	ff := initField(grid, opts)
	ff.ResetMulti(goals)
	return ff
}

// Reset recalculates the field for a new single goal. Reuses memory.
func (ff *Field) Reset(goalX, goalY int) {
	ff.resetCosts()
	ff.integrateMulti([][2]int{{goalX, goalY}})
	ff.vectorize()
}

// ResetMulti recalculates the field for multiple goals. Reuses memory.
func (ff *Field) ResetMulti(goals [][2]int) {
	ff.resetCosts()
	ff.integrateMulti(goals)
	ff.vectorize()
}

// integrateMulti runs Dijkstra from all goals simultaneously (multi-source).
// Each tile gets the cost to the nearest reachable goal.
func (ff *Field) integrateMulti(goals [][2]int) {
	w := ff.width
	grid := ff.grid
	cost := ff.cost
	diag := ff.diagonal

	nb := ff.neighborBuf[:0]
	queue := ff.queue[:0]
	idx := func(x, y int) int { return y*w + x }

	// Push all walkable goals as initial sources
	for _, g := range goals {
		if !grid.IsInside(g[0], g[1]) || !grid.IsWalkableAt(g[0], g[1]) {
			continue
		}
		gi := idx(g[0], g[1])
		if cost[gi] < 0 {
			cost[gi] = 0
			queue = append(queue, item{x: g[0], y: g[1], g: 0})
		}
	}

	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]

		ci := idx(cur.x, cur.y)
		if cost[ci] != cur.g {
			continue
		}

		node := grid.GetNodeAt(cur.x, cur.y)
		neighbors := grid.GetNeighbors(node, diag, nb)
		nb = nb[:0]

		for _, nbNode := range neighbors {
			nx, ny := nbNode.X, nbNode.Y

			step := 1.0
			if nx-cur.x != 0 && ny-cur.y != 0 {
				step = finder.SQRT2
			}
			nd := cur.g + step*nbNode.Weight

			ni := idx(nx, ny)
			if cost[ni] < 0 || nd < cost[ni] {
				cost[ni] = nd
				queue = append(queue, item{x: nx, y: ny, g: nd})
			}
		}
	}

	ff.queue = queue[:0]
}

// vectorize builds the vector field from the integration field.
func (ff *Field) vectorize() {
	w, h := ff.width, ff.height
	grid := ff.grid
	cost := ff.cost
	diag := ff.diagonal

	nb := ff.neighborBuf[:0]
	idx := func(x, y int) int { return y*w + x }

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			ci := idx(x, y)
			if cost[ci] < 0 {
				ff.dirs[ci] = dir2{0, 0}
				continue
			}

			bestG := cost[ci]
			bestDir := dir2{0, 0}

			node := grid.GetNodeAt(x, y)
			neighbors := grid.GetNeighbors(node, diag, nb)
			nb = nb[:0]

			for _, nbNode := range neighbors {
				ni := idx(nbNode.X, nbNode.Y)
				nc := cost[ni]
				if nc < 0 {
					continue
				}
				if nc < bestG {
					bestG = nc
					bestDir = dir2{nbNode.X - x, nbNode.Y - y}
				}
			}
			ff.dirs[ci] = bestDir
		}
	}
	ff.neighborBuf = nb[:0]
}

// GetDirection returns the movement direction for tile (x, y).
// Returns (0,0) if the tile is blocked or unreachable.
func (ff *Field) GetDirection(x, y int) (int, int) {
	if x < 0 || x >= ff.width || y < 0 || y >= ff.height {
		return 0, 0
	}
	d := ff.dirs[y*ff.width+x]
	return d.dx, d.dy
}

// GetCost returns the integration cost (distance to nearest goal).
// Returns -1 for unreachable tiles.
func (ff *Field) GetCost(x, y int) float64 {
	if x < 0 || x >= ff.width || y < 0 || y >= ff.height {
		return -1
	}
	return ff.cost[y*ff.width+x]
}

// FindPath backtracks from (x, y) by following the direction chain
// to the nearest goal. Useful for debugging and visualization.
func (ff *Field) FindPath(startX, startY int) [][2]int {
	w := ff.width
	if !ff.grid.IsInside(startX, startY) {
		return nil
	}
	if ff.cost[startY*w+startX] < 0 {
		return nil
	}
	path := make([][2]int, 0, w)
	cx, cy := startX, startY
	for {
		path = append(path, [2]int{cx, cy})
		d := ff.dirs[cy*w+cx]
		if d.dx == 0 && d.dy == 0 {
			break
		}
		cx += d.dx
		cy += d.dy
	}
	return path
}
