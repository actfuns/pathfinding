package finder

// BiAStarFinder is a bidirectional A* pathfinder.
type BiAStarFinder struct {
	// Heuristic is the heuristic function used to estimate the cost from the
	// current node to the goal. Defaults to Manhattan (or Octile when diagonal
	// movement is enabled); topology-specific grids may override the default.
	Heuristic HeuristicFunc
	// Weight is the heuristic weight multiplier. Higher values make the
	// pathfinder more greedy. Defaults to 1.
	Weight float32
	// DiagonalMovement specifies whether diagonal moves are allowed and under
	// what conditions. Defaults to DiagonalNever.
	DiagonalMovement DiagonalMovement

	customHeuristic bool
}

// NewBiAStarFinder creates a new BiAStarFinder.
func NewBiAStarFinder(opts ...Option) *BiAStarFinder {
	opt := ApplyOptions(opts)
	f := &BiAStarFinder{
		Heuristic:        Manhattan,
		Weight:           1,
		DiagonalMovement: DiagonalNever,
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
	if f.DiagonalMovement != DiagonalNever && opt.Heuristic == nil {
		f.Heuristic = Octile
	}
	return f
}

// statelessFinder marks BiAStarFinder as safe to reuse after a grid clone.
func (f *BiAStarFinder) statelessFinder() {}

// FindPath finds a path using bidirectional A*.
func (f *BiAStarFinder) FindPath(startX, startY, endX, endY int32, grid Grid, dst [][2]int32) ([][2]int32, error) {
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

func (f *BiAStarFinder) findPath(startX, startY, endX, endY int32, grid Grid, states *searchStates) ([][2]int32, error) {
	if !canSearchBidirectionally(grid, startX, startY) {
		forward := AStarFinder{
			Heuristic: f.Heuristic, Weight: f.Weight, DiagonalMovement: f.DiagonalMovement,
			customHeuristic: f.customHeuristic,
		}
		return forward.findPath(startX, startY, endX, endY, grid, states)
	}

	heuristic, weight := heuristicFor(grid, f.DiagonalMovement, f.Heuristic, f.customHeuristic, f.Weight)
	stepCost := gridStepCost(grid)
	start := grid.GetNodeAt(startX, startY)
	end := grid.GetNodeAt(endX, endY)

	n := len(states.states)
	if cap(states.reverse) < n {
		states.reverse = make([]nodeState, n)
	}
	reverse := searchStates{width: states.width, height: states.height, seq: states.seq, states: states.reverse[:n]}
	forwardOpen := minHeap{nodes: states.heap[:0], tieBreak: true, state: states}
	reverseOpen := minHeap{nodes: states.heapB[:0], tieBreak: true, state: &reverse}
	neighborBuf := states.neighborBuf[:0]
	defer func() {
		states.heap = forwardOpen.nodes[:0]
		states.heapB = reverseOpen.nodes[:0]
		states.neighborBuf = neighborBuf
	}()

	directions := [...]struct {
		open         *minHeap
		other        *searchStates
		from, target *Node
		reversed     bool
	}{
		{&forwardOpen, &reverse, start, end, false},
		{&reverseOpen, states, end, start, true},
	}
	for _, side := range directions {
		index := side.open.state.index(side.from)
		state := side.open.state.at(index)
		state.Opened = 1
		state.H = weight * heuristic(float32(AbsInt(startX-endX)), float32(AbsInt(startY-endY)))
		state.F = state.H
		side.open.Push(index, state.F)
	}

	meeting := int32(-1)
	var best float32
search:
	for {
		for _, side := range directions {
			// A frontier's minimum F is a lower bound with an admissible
			// heuristic. The best complete route is an upper bound.
			if side.open.Empty() || (meeting >= 0 && side.open.nodes[0].F >= best) {
				break search
			}
			current := side.open.state
			index := side.open.Pop()
			state := &current.states[index]
			state.Closed = true
			x, y := current.xy(index)
			node := grid.GetNodeAt(x, y)
			neighborBuf = grid.GetNeighbors(node, f.DiagonalMovement, neighborBuf)
			for _, next := range neighborBuf {
				nextIndex := current.index(next)
				nextState := current.at(nextIndex)
				dx, dy := next.X-x, next.Y-y
				tileWeight := next.Weight
				if side.reversed {
					// The forward edge enters node, even when explored backwards.
					dx, dy = -dx, -dy
					tileWeight = node.Weight
				}
				cost := state.G + moveCost(stepCost, dx, dy, tileWeight)
				if nextState.Opened != 0 && cost >= nextState.G {
					continue
				}
				if nextState.Opened == 0 {
					nextState.H = weight * heuristic(float32(AbsInt(next.X-side.target.X)), float32(AbsInt(next.Y-side.target.Y)))
				}
				nextState.G = cost
				nextState.F = cost + nextState.H
				nextState.Parent = index
				if nextState.Opened == 0 || nextState.Closed {
					nextState.Opened = 1
					nextState.Closed = false
					side.open.Push(nextIndex, nextState.F)
				} else {
					side.open.UpdateItem(nextIndex, nextState.F)
				}
				if other := &side.other.states[nextIndex]; other.SearchID == states.seq && other.Opened != 0 {
					total := cost + other.G
					if meeting < 0 || total < best {
						meeting, best = nextIndex, total
					}
				}
			}
		}
	}

	if meeting < 0 {
		return nil, nil
	}
	x, y := states.xy(meeting)
	path := states.backtrace(grid.GetNodeAt(x, y), states.pathBuf)
	for i := reverse.at(meeting).Parent; i >= 0; i = reverse.at(i).Parent {
		x, y := reverse.xy(i)
		path = append(path, [2]int32{x, y})
	}
	states.pathBuf = path
	return path, nil
}
