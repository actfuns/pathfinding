package finder

// BiBreadthFirstFinder is a bidirectional BFS pathfinder.
type BiBreadthFirstFinder struct {
	// DiagonalMovement specifies whether diagonal moves are allowed and
	// under what conditions (e.g., never, always, or only when obstacles
	// do not block the way).
	DiagonalMovement DiagonalMovement

	neighborBuf []*Node
	pathBuf     [][2]int
}

// NewBiBreadthFirstFinder creates a new BiBreadthFirstFinder.
func NewBiBreadthFirstFinder(opts ...Option) *BiBreadthFirstFinder {
	opt := ApplyOptions(opts)
	f := &BiBreadthFirstFinder{
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

// FindPath finds a path using bidirectional BFS.
func (f *BiBreadthFirstFinder) FindPath(startX, startY, endX, endY int, grid Grid) ([][2]int, error) {
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

	const (
		BY_START = 1
		BY_END   = 2
	)

	startOpenList := make([]*Node, 0, 64)
	endOpenList := make([]*Node, 0, 64)

	startState := states.get(startNode)
	startOpenList = append(startOpenList, startNode)
	startState.Opened = BY_START
	startState.Parent = -1

	endState := states.get(endNode)
	endOpenList = append(endOpenList, endNode)
	endState.Opened = BY_END
	endState.Parent = -1

	neighborBuf := f.neighborBuf[:0]

	for len(startOpenList) > 0 && len(endOpenList) > 0 {
		node := startOpenList[0]
		startOpenList = startOpenList[1:]
		states.get(node).Closed = true

		neighbors := grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			ns := states.get(neighbor)
			if ns.Closed {
				continue
			}
			if ns.Opened != 0 {
				if ns.Opened == BY_END {
					f.pathBuf = states.biBacktrace(node, neighbor, f.pathBuf)
					return f.pathBuf, nil
				}
				continue
			}
			startOpenList = append(startOpenList, neighbor)
			ns.Parent = states.index(node)
			ns.Opened = BY_START
		}

		node = endOpenList[0]
		endOpenList = endOpenList[1:]
		states.get(node).Closed = true

		neighbors = grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
		for _, neighbor := range neighbors {
			ns := states.get(neighbor)
			if ns.Closed {
				continue
			}
			if ns.Opened != 0 {
				if ns.Opened == BY_START {
					f.pathBuf = states.biBacktrace(neighbor, node, f.pathBuf)
					return f.pathBuf, nil
				}
				continue
			}
			endOpenList = append(endOpenList, neighbor)
			ns.Parent = states.index(node)
			ns.Opened = BY_END
		}
	}

	return nil, nil
}
