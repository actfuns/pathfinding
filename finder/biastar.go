package finder

// BiAStarFinder is a bidirectional A* pathfinder.
type BiAStarFinder struct {
	// Heuristic is the heuristic function used to estimate the cost from the
	// current node to the goal. Defaults to Manhattan (or Octile when diagonal
	// movement is enabled); topology-specific grids may override the default.
	Heuristic HeuristicFunc
	// Weight is the heuristic weight multiplier. Higher values make the
	// pathfinder more greedy. Defaults to 1.
	Weight float64
	// DiagonalMovement specifies whether diagonal moves are allowed and under
	// what conditions. Defaults to DiagonalNever.
	DiagonalMovement DiagonalMovement

	customHeuristic bool
	neighborBuf     []*Node
	startHeapSlice  []heapEntry
	endHeapSlice    []heapEntry
	pathBuf         [][2]int
}

// NewBiAStarFinder creates a new BiAStarFinder.
func NewBiAStarFinder(opts ...Option) *BiAStarFinder {
	opt := ApplyOptions(opts)
	f := &BiAStarFinder{
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

// FindPath finds a path using bidirectional A*.
func (f *BiAStarFinder) FindPath(startX, startY, endX, endY int, grid Grid) ([][2]int, error) {
	// GetNodeAt panics on an out-of-range index, so validate up front and
	// report it as a caller error instead of a crash.
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	if path, err := checkEndpoints(startX, startY, endX, endY, grid); path != nil || err != nil {
		return path, err
	}
	heuristic := heuristicFor(grid, f.DiagonalMovement, f.Heuristic, f.customHeuristic)
	stepCost := gridStepCost(grid)
	searchSeq := globalSearchSeq.Add(1)
	states := acquireStates(grid.Width(), grid.Height(), searchSeq)
	defer releaseStates(states)
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	const (
		BY_START = 1
		BY_END   = 2
	)

	startOpenList := &MinHeap{nodes: f.startHeapSlice[:0], tieBreak: true, state: states}
	endOpenList := &MinHeap{nodes: f.endHeapSlice[:0], tieBreak: true, state: states}
	defer func() {
		clear(startOpenList.nodes[:cap(startOpenList.nodes)])
		clear(endOpenList.nodes[:cap(endOpenList.nodes)])
		f.startHeapSlice = startOpenList.nodes[:0]
		f.endHeapSlice = endOpenList.nodes[:0]
	}()

	startState := states.get(startNode)
	startState.G = 0
	startState.F = 0
	startOpenList.Push(startNode)
	startState.Opened = BY_START

	endState := states.get(endNode)
	endState.G = 0
	endState.F = 0
	endOpenList.Push(endNode)
	endState.Opened = BY_END

	neighborBuf := f.neighborBuf[:0]

	for !startOpenList.Empty() && !endOpenList.Empty() {
		node := startOpenList.Pop()
		nodeState := states.get(node)
		nodeState.Closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			ns := states.get(neighbor)
			if ns.Closed {
				continue
			}
			if ns.Opened == BY_END {
				f.pathBuf = states.biBacktrace(node, neighbor, f.pathBuf)
				return f.pathBuf, nil
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
					startOpenList.Push(neighbor)
					ns.Opened = BY_START
				} else {
					startOpenList.UpdateItem(neighbor)
				}
			}
		}

		node = endOpenList.Pop()
		nodeState = states.get(node)
		nodeState.Closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			ns := states.get(neighbor)
			if ns.Closed {
				continue
			}
			if ns.Opened == BY_START {
				f.pathBuf = states.biBacktrace(neighbor, node, f.pathBuf)
				return f.pathBuf, nil
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
					ns.H = f.Weight * heuristic(float64(AbsInt(x-startX)), float64(AbsInt(y-startY)))
				}
				ns.F = ns.G + ns.H
				ns.Parent = states.index(node)

				if ns.Opened == 0 {
					endOpenList.Push(neighbor)
					ns.Opened = BY_END
				} else {
					endOpenList.UpdateItem(neighbor)
				}
			}
		}
	}

	return nil, nil
}
