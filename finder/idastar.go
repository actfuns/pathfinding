package finder

import (
	"math"
	"time"
)

// IDAStarFinder is an Iterative Deepening A* pathfinder.
type IDAStarFinder struct {
	// Heuristic is the heuristic function used to estimate the cost from a node to the goal.
	Heuristic HeuristicFunc
	// Weight is the multiplier applied to the heuristic estimate (g + h*Weight).
	Weight float64
	// DiagonalMovement specifies whether diagonal moves are allowed and under what conditions.
	DiagonalMovement DiagonalMovement
	// TrackRecursion enables tracking of recursion depth for debugging or visualization purposes.
	TrackRecursion bool
	// TimeLimit is the maximum number of seconds to spend searching before giving up. A value <= 0 means no limit.
	TimeLimit float64

	customHeuristic bool

	// Reusable search scratch, so repeated FindPath calls avoid allocations.
	routeBuf     [][2]int
	neighborBufs [][]*Node
	onPath       []bool
}

// NewIDAStarFinder creates a new IDAStarFinder.
func NewIDAStarFinder(opts ...Option) *IDAStarFinder {
	opt := ApplyOptions(opts)
	f := &IDAStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
		DiagonalMovement: DiagonalNever,
		TrackRecursion:   false,
		TimeLimit:        -1,
		customHeuristic:  opt.Heuristic != nil,
	}
	if opt.DiagonalMovement != 0 {
		f.DiagonalMovement = opt.DiagonalMovement
	} else if opt.AllowDiagonal {
		if opt.DontCrossCorners {
			f.DiagonalMovement = DiagonalOnlyWhenNoObstacles
		} else {
			f.DiagonalMovement = DiagonalIfAtMostOneObstacle
		}
	}
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	}
	if opt.Weight > 0 {
		f.Weight = opt.Weight
	}
	f.TrackRecursion = opt.TrackRecursion
	f.TimeLimit = opt.TimeLimit
	if f.DiagonalMovement != DiagonalNever && opt.Heuristic == nil {
		f.Heuristic = Octile
	}
	return f
}

// FindPath finds a path using IDA*.
func (f *IDAStarFinder) FindPath(startX, startY, endX, endY int, grid Grid, dst [][2]int) ([][2]int, error) {
	if path, err := preflight(startX, startY, endX, endY, grid); path != nil || err != nil {
		return finishPath(path, dst, true), err
	}
	searchSeq := globalSearchSeq.Add(1)
	states, err := acquireStates(grid.Width(), grid.Height(), searchSeq)
	if err != nil {
		return nil, err
	}
	local := *f
	local.routeBuf = states.routeBuf[:0]
	local.neighborBufs = states.neighborBufs
	local.onPath = states.onPath
	defer func() {
		states.routeBuf = local.routeBuf
		states.neighborBufs = local.neighborBufs
		states.onPath = local.onPath
		releaseStates(states)
	}()
	p, err := local.findPath(startX, startY, endX, endY, grid, states)
	return finishPath(p, dst, true), err
}

func (f *IDAStarFinder) findPath(startX, startY, endX, endY int, grid Grid, states *searchStates) ([][2]int, error) {
	heuristic := heuristicFor(grid, f.DiagonalMovement, f.Heuristic, f.customHeuristic)
	stepCost := gridStepCost(grid)
	start := grid.GetNodeAt(startX, startY)
	end := grid.GetNodeAt(endX, endY)

	h := func(a, b *Node) float64 {
		return heuristic(float64(AbsInt(b.X-a.X)), float64(AbsInt(b.Y-a.Y)))
	}

	cost := func(a, b *Node) float64 {
		dx, dy := b.X-a.X, b.Y-a.Y
		if stepCost != nil {
			return stepCost(dx, dy) * b.Weight
		}
		if dx == 0 || dy == 0 {
			return b.Weight
		}
		return SQRT2 * b.Weight
	}

	startTime := time.Now()
	hasTimeLimit := f.TimeLimit > 0

	width, height := grid.Width(), grid.Height()
	maxDepth := width * height
	if maxDepth < 1 {
		maxDepth = 1
	}
	route := f.routeBuf[:0]
	// onPath prevents revisiting a tile within one depth-first branch. Without
	// it the search can walk cycles forever and never prove that no path exists.
	onPath := f.onPath
	if cap(onPath) < maxDepth {
		onPath = make([]bool, maxDepth)
	} else {
		onPath = onPath[:maxDepth]
		clear(onPath)
	}
	f.onPath = onPath
	startIdx := start.Y*width + start.X
	if startIdx >= 0 && startIdx < len(onPath) {
		onPath[startIdx] = true
	}

	// Each recursion depth needs its own neighbor buffer: GetNeighbors writes
	// into the buffer it is given, so sharing one across recursive calls would
	// overwrite the parent's neighbor list while it is still being iterated.
	neighborBufs := f.neighborBufs
	defer func() {
		f.routeBuf = route
		f.neighborBufs = neighborBufs
	}()
	pathEnd := -1
	var timedOut bool

	var search func(node *Node, g, cutoff float64, depth int) (float64, bool)
	search = func(node *Node, g, cutoff float64, depth int) (float64, bool) {
		if timedOut {
			return math.Inf(1), false
		}
		if hasTimeLimit && time.Since(startTime).Seconds() > f.TimeLimit {
			timedOut = true
			return math.Inf(1), false
		}

		fScore := g + h(node, end)*f.Weight
		if fScore > cutoff {
			return fScore, false
		}

		if depth >= len(route) {
			route = append(route, [2]int{})
		}
		route[depth] = [2]int{node.X, node.Y}
		if node == end {
			pathEnd = depth
			return 0, true
		}

		if depth >= len(neighborBufs) {
			neighborBufs = append(neighborBufs, make([]*Node, 0, 8))
		}
		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBufs[depth][:0])
		neighborBufs[depth] = neighbors[:0]

		min := math.Inf(1)
		for _, neighbor := range neighbors {
			idx := neighbor.Y*width + neighbor.X
			if idx < 0 || idx >= len(onPath) || onPath[idx] {
				continue
			}
			ns := states.get(neighbor)
			if f.TrackRecursion {
				ns.RetainCount++
				if !ns.Tested {
					ns.Tested = true
				}
			}

			onPath[idx] = true
			t, found := search(neighbor, g+cost(node, neighbor), cutoff, depth+1)
			onPath[idx] = false

			if found {
				return 0, true
			}

			if f.TrackRecursion {
				ns.RetainCount--
				if ns.RetainCount == 0 {
					ns.Tested = false
				}
			}

			if t < min {
				min = t
			}
		}

		return min, false
	}

	cutOff := h(start, end) * f.Weight

	for {
		pathEnd = -1
		t, found := search(start, 0, cutOff, 0)

		// Ran out of time before reaching a verdict: the search never finished,
		// so say so instead of reporting "no path" or looping forever.
		if timedOut {
			return nil, ErrTimeLimit
		}

		if found {
			path := make([][2]int, pathEnd+1)
			copy(path, route[:pathEnd+1])
			return path, nil
		}

		// All simple paths were exhausted: no route exists.
		if math.IsInf(t, 1) {
			return nil, nil
		}

		// Guarantee progress even if rounding leaves t <= cutOff.
		if t <= cutOff {
			t = cutOff + 1
		}
		cutOff = t
	}
}
