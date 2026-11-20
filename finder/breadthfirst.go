package finder

// BreadthFirstFinder is a Breadth-First-Search pathfinder.
type BreadthFirstFinder struct {
	// DiagonalMovement determines whether diagonal moves are allowed and
	// under what obstacle conditions. Defaults to DiagonalNever.
	DiagonalMovement DiagonalMovement
}

// NewBreadthFirstFinder creates a new BreadthFirstFinder.
func NewBreadthFirstFinder(opts ...Option) *BreadthFirstFinder {
	opt := ApplyOptions(opts)
	f := &BreadthFirstFinder{
		DiagonalMovement: DiagonalNever,
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
	return f
}

// statelessFinder marks BreadthFirstFinder as safe to reuse after a grid clone.
func (f *BreadthFirstFinder) statelessFinder() {}

// FindPath finds a path using BFS.
func (f *BreadthFirstFinder) FindPath(startX, startY, endX, endY int32, grid Grid, dst [][2]int32) ([][2]int32, error) {
	if path, err := preflight(startX, startY, endX, endY, grid); path != nil || err != nil {
		return finishPath(path, dst, true), err
	}
	searchSeq := globalSearchSeq.Add(1)
	states, err := acquireStates(grid.Width(), grid.Height(), searchSeq)
	if err != nil {
		return nil, err
	}
	defer releaseStates(states)
	p, err := f.findPath(startX, startY, endX, endY, grid, states)
	return finishPath(p, dst, false), err
}

func (f *BreadthFirstFinder) findPath(startX, startY, endX, endY int32, grid Grid, states *searchStates) ([][2]int32, error) {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)
	startState := states.get(startNode)
	openList := states.queue[:0]
	defer func() { states.queue = openList }()
	openList = append(openList, startNode)
	startState.Opened = 1

	neighborBuf := states.neighborBuf[:0]
	defer func() { states.neighborBuf = neighborBuf }()
	head := 0

	for head < len(openList) {
		node := openList[head]
		head++
		nodeState := states.get(node)
		nodeState.Closed = true

		if node == endNode {
			states.pathBuf = states.backtrace(endNode, states.pathBuf)
			return states.pathBuf, nil
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		neighborBuf = neighbors
		for _, neighbor := range neighbors {
			ns := states.get(neighbor)
			if ns.Closed || ns.Opened != 0 {
				continue
			}
			openList = append(openList, neighbor)
			ns.Opened = 1
			ns.Parent = states.index(node)
		}
	}

	return nil, nil
}
