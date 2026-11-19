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
	queue       itemHeap
}

type dir2 struct{ dx, dy int }

type item struct {
	x, y int
	g    float64
}

// itemHeap is a typed min-heap ordered by cost. A typed heap avoids the
// per-push interface boxing of container/heap, which dominates flowfield builds
// on weighted maps.
type itemHeap []item

func (h *itemHeap) push(it item) {
	*h = append(*h, it)
	i := len(*h) - 1
	for i > 0 {
		p := (i - 1) / 2
		if (*h)[p].g <= it.g {
			break
		}
		(*h)[i] = (*h)[p]
		i = p
	}
	(*h)[i] = it
}

func (h *itemHeap) pop() item {
	old := *h
	n := len(old)
	root := old[0]
	last := old[n-1]
	old = old[:n-1]
	if len(old) > 0 {
		i := 0
		for {
			l := 2*i + 1
			if l >= len(old) {
				break
			}
			c := l
			if r := l + 1; r < len(old) && old[r].g < old[l].g {
				c = r
			}
			if old[c].g >= last.g {
				break
			}
			old[i] = old[c]
			i = c
		}
		old[i] = last
	}
	*h = old
	return root
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
		queue:       make(itemHeap, 0, w*h),
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
	stepCost := stepCostFunc(grid)

	nb := ff.neighborBuf[:0]
	queue := ff.queue[:0]
	idx := func(x, y int) int { return y*w + x }

	// Cardinal-only unit-cost fields reduce to plain BFS. Keep Dijkstra for
	// weighted grids or when diagonal steps have a different cost.
	useBFS := ff.diagonal == finder.DiagonalNever && unitStepCost(grid) && allUnitWeights(grid)

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

	if useBFS {
		for head := 0; head < len(queue); head++ {
			cur := queue[head]
			ci := idx(cur.x, cur.y)
			if cost[ci] != cur.g {
				continue
			}
			node := grid.GetNodeAt(cur.x, cur.y)
			neighbors := grid.GetNeighbors(node, diag, nb)
			nb = nb[:0]
			for _, nbNode := range neighbors {
				ni := idx(nbNode.X, nbNode.Y)
				nd := cur.g + 1
				if cost[ni] < 0 || nd < cost[ni] {
					cost[ni] = nd
					queue = append(queue, item{x: nbNode.X, y: nbNode.Y, g: nd})
				}
			}
		}
		ff.queue = queue[:0]
		return
	}

	for len(queue) > 0 {
		cur := queue.pop()

		ci := idx(cur.x, cur.y)
		if cost[ci] != cur.g {
			continue
		}

		node := grid.GetNodeAt(cur.x, cur.y)
		curWeight := node.Weight
		neighbors := grid.GetNeighbors(node, diag, nb)
		nb = nb[:0]

		for _, nbNode := range neighbors {
			nx, ny := nbNode.X, nbNode.Y

			// Moving nb -> cur enters cur, so the destination-weight model
			// charges cur's weight (matching A*).
			nd := cur.g + stepCost(nx-cur.x, ny-cur.y)*curWeight

			ni := idx(nx, ny)
			if cost[ni] < 0 || nd < cost[ni] {
				cost[ni] = nd
				queue.push(item{x: nx, y: ny, g: nd})
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
	stepCost := stepCostFunc(grid)

	nb := ff.neighborBuf[:0]
	idx := func(x, y int) int { return y*w + x }

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			ci := idx(x, y)
			if cost[ci] < 0 {
				ff.dirs[ci] = dir2{0, 0}
				continue
			}

			if cost[ci] == 0 {
				ff.dirs[ci] = dir2{0, 0}
				continue
			}

			best := 0.0
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
				total := nc + stepCost(nbNode.X-x, nbNode.Y-y)*nbNode.Weight
				if bestDir == (dir2{0, 0}) || total < best {
					best = total
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

// allUnitWeights reports whether every tile uses the default movement weight.
func allUnitWeights(grid finder.Grid) bool {
	for y := 0; y < grid.Height(); y++ {
		for x := 0; x < grid.Width(); x++ {
			if grid.GetNodeAt(x, y).Weight != 1 {
				return false
			}
		}
	}
	return true
}

// unitStepCost reports whether cardinal steps cost 1.
func unitStepCost(grid finder.Grid) bool {
	if sc, ok := grid.(finder.StepCostGrid); ok {
		return sc.StepCost(1, 0) == 1 && sc.StepCost(0, 1) == 1
	}
	return true
}

// stepCostFunc resolves the grid's step-cost function once per build.
func stepCostFunc(grid finder.Grid) func(dx, dy int) float64 {
	if sc, ok := grid.(finder.StepCostGrid); ok {
		return sc.StepCost
	}
	return func(dx, dy int) float64 {
		if dx != 0 && dy != 0 {
			return finder.SQRT2
		}
		return 1
	}
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
//
// Like the finders, it separates "you asked about a tile that is not on the
// map" from "that tile cannot reach any goal": the first is finder.ErrOutOfBounds,
// the second is a nil path with a nil error.
func (ff *Field) FindPath(startX, startY int) ([][2]int, error) {
	w := ff.width
	if !ff.grid.IsInside(startX, startY) {
		return nil, finder.ErrOutOfBounds
	}
	if ff.cost[startY*w+startX] < 0 {
		return nil, nil
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
	return path, nil
}
