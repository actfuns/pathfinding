package finder

// BiBreadthFirstFinder is a bidirectional BFS pathfinder.
type BiBreadthFirstFinder struct {
	// DiagonalMovement specifies whether diagonal moves are allowed and
	// under what conditions (e.g., never, always, or only when obstacles
	// do not block the way).
	DiagonalMovement DiagonalMovement

	neighborBuf []*Node
	pathBuf     [][2]int32
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
func (f *BiBreadthFirstFinder) FindPath(startX, startY, endX, endY int32, grid Grid, dst [][2]int32) ([][2]int32, error) {
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
	local.pathBuf = states.pathBuf[:0]
	defer func() {
		states.neighborBuf = local.neighborBuf
		states.pathBuf = local.pathBuf
		releaseStates(states)
	}()
	p, err := local.findPath(startX, startY, endX, endY, grid, states)
	return finishPath(p, dst, false), err
}

func (f *BiBreadthFirstFinder) findPath(startX, startY, endX, endY int32, grid Grid, states *searchStates) ([][2]int32, error) {
	startNode := grid.GetNodeAt(startX, startY)
	endNode := grid.GetNodeAt(endX, endY)

	const (
		BY_START = 1
		BY_END   = 2
	)

	startOpenList := states.queue[:0]
	endOpenList := states.queueB[:0]
	defer func() { states.queue = startOpenList; states.queueB = endOpenList }()

	startState := states.get(startNode)
	startOpenList = append(startOpenList, startNode)
	startState.Opened = BY_START
	startState.Parent = -1

	endState := states.get(endNode)
	endOpenList = append(endOpenList, endNode)
	endState.Opened = BY_END
	endState.Parent = -1

	neighborBuf := f.neighborBuf[:0]
	startHead, endHead := 0, 0

	for startHead < len(startOpenList) && endHead < len(endOpenList) {
		node := startOpenList[startHead]
		startHead++
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

		node = endOpenList[endHead]
		endHead++
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
