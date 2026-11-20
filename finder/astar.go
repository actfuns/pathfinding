package finder

// AStarFinder is an implementation of the A* pathfinding algorithm.
type AStarFinder struct {
	// Heuristic is the heuristic function used to estimate the cost from a node
	// to the goal. Unless the caller supplies one, it defaults to Manhattan for
	// cardinal-only movement and Octile for diagonal movement; grids can provide
	// a topology-specific default (for example, hex grids use Chebyshev).
	Heuristic HeuristicFunc

	// Weight is the multiplier applied to the heuristic estimate (H-cost)
	// to bias the search. Values > 1 make the search faster but potentially
	// suboptimal; values < 1 make it more accurate but slower. Defaults to 1.
	Weight float64

	// DiagonalMovement controls whether diagonal moves are allowed and under
	// what obstacle conditions. See DiagonalMovement values for details.
	// Defaults to DiagonalNever when not configured via Options.
	DiagonalMovement DiagonalMovement

	customHeuristic bool
	neighborBuf     []*Node
	heapSlice       []heapEntry
	pathBuf         [][2]int
}

// NewAStarFinder creates a new AStarFinder with default options.
func NewAStarFinder(opts ...Option) *AStarFinder {
	opt := ApplyOptions(opts)
	f := &AStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
		DiagonalMovement: DiagonalNever,
		customHeuristic:  opt.Heuristic != nil,
		neighborBuf:      make([]*Node, 0, 8),
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
	if f.DiagonalMovement != DiagonalNever && opt.Heuristic == nil {
		f.Heuristic = Octile
	}
	return f
}

// FindPath finds a path from (startX, startY) to (endX, endY) on the grid.
func (f *AStarFinder) FindPath(startX, startY, endX, endY int, grid Grid, dst [][2]int) ([][2]int, error) {
	if path, err := preflight(startX, startY, endX, endY, grid); path != nil || err != nil {
		return finishPath(path, dst, true), err
	}
	searchSeq := globalSearchSeq.Add(1)
	states, err := acquireStates(grid.Width(), grid.Height(), searchSeq)
	if err != nil {
		return nil, err
	}
	local := *f
	local.neighborBuf = states.neighborBuf[:0]
	local.heapSlice = states.heap[:0]
	local.pathBuf = states.pathBuf[:0]
	defer func() {
		states.neighborBuf = local.neighborBuf
		states.heap = local.heapSlice
		states.pathBuf = local.pathBuf
		releaseStates(states)
	}()
	p, err := local.findPath(startX, startY, endX, endY, grid, states)
	return finishPath(p, dst, false), err
}

func (f *AStarFinder) findPath(startX, startY, endX, endY int, grid Grid, states *searchStates) ([][2]int, error) {
	heuristic := heuristicFor(grid, f.DiagonalMovement, f.Heuristic, f.customHeuristic)
	stepCost := gridStepCost(grid)
	openList := &minHeap{nodes: f.heapSlice[:0], tieBreak: true, state: states}
	defer func() {
		clear(openList.nodes[:cap(openList.nodes)])
		f.heapSlice = openList.nodes[:0]
	}()
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	startState := states.get(startNode)
	startState.G = 0
	startState.F = 0
	openList.Push(startNode)
	startState.Opened = 1

	neighborBuf := f.neighborBuf[:0]

	for !openList.Empty() {
		node := openList.Pop()
		nodeState := states.get(node)
		nodeState.Closed = true

		if node == endNode {
			f.pathBuf = states.backtrace(endNode, f.pathBuf)
			return f.pathBuf, nil
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			ns := states.get(neighbor)
			if ns.Closed {
				continue
			}

			x, y := neighbor.X, neighbor.Y
			var ng float64
			if stepCost != nil {
				ng = nodeState.G + stepCost(x-node.X, y-node.Y)*neighbor.Weight
			} else if x-node.X == 0 || y-node.Y == 0 {
				ng = nodeState.G + neighbor.Weight
			} else {
				ng = nodeState.G + SQRT2*neighbor.Weight
			}

			if ns.Opened == 0 || ng < ns.G {
				ns.G = ng
				if ns.Opened == 0 {
					ns.H = f.Weight * heuristic(float64(AbsInt(x-endX)), float64(AbsInt(y-endY)))
				}
				ns.F = ns.G + ns.H
				ns.Parent = states.index(node)

				if ns.Opened == 0 {
					openList.Push(neighbor)
					ns.Opened = 1
				} else {
					openList.UpdateItem(neighbor)
				}
			}
		}
	}

	return nil, nil
}
