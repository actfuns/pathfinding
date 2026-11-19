package finder

// BreadthFirstFinder is a Breadth-First-Search pathfinder.
type BreadthFirstFinder struct {
	// DiagonalMovement determines whether diagonal moves are allowed and
	// under what obstacle conditions. Defaults to DiagonalNever.
	DiagonalMovement DiagonalMovement

	neighborBuf []*Node
	pathBuf     [][2]int
}

// NewBreadthFirstFinder creates a new BreadthFirstFinder.
func NewBreadthFirstFinder(opts ...Option) *BreadthFirstFinder {
	opt := ApplyOptions(opts)
	f := &BreadthFirstFinder{
		DiagonalMovement: DiagonalNever,
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
	return f
}

// FindPath finds a path using BFS.
func (f *BreadthFirstFinder) FindPath(startX, startY, endX, endY int, grid Grid) ([][2]int, error) {
	// GetNodeAt panics on an out-of-range index, so validate up front and
	// report it as a caller error instead of a crash.
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	if path, err := checkEndpoints(startX, startY, endX, endY, grid); path != nil || err != nil {
		return path, err
	}
	searchSeq := globalSearchSeq.Add(1)
	states := acquireStates(grid.Width(), grid.Height(), searchSeq)
	defer releaseStates(states)
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)
	startState := states.get(startNode)
	openList := make([]*Node, 0, 64)
	openList = append(openList, startNode)
	startState.Opened = 1

	neighborBuf := f.neighborBuf[:0]

	for len(openList) > 0 {
		node := openList[0]
		openList = openList[1:]
		nodeState := states.get(node)
		nodeState.Closed = true

		if node == endNode {
			f.pathBuf = states.backtrace(endNode, f.pathBuf)
			return f.pathBuf, nil
		}

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
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
