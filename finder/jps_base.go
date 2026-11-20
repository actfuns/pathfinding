package finder

// JumpPointFinderBase is the base implementation for Jump Point Search.
//
// JPS assumes every step has the same cost (tile Weight == 1). Tile weights are
// ignored; use A* or Dijkstra for weighted grids.
//
// JPS only supports axis-aligned grids (OrthogonalGrid). Hex and staggered
// grids return ErrUnsupportedGrid.
type JumpPointFinderBase struct {
	Heuristic        HeuristicFunc
	DiagonalMovement DiagonalMovement
	Weight           float32
	TrackRecursion   bool
	jumpFn           func(s *jpsSearch, x, y, px, py int32) (int32, int32, bool)
	findNeighborsFn  func(s *jpsSearch, node *Node, buf [][2]int32) [][2]int32
}

// jpsSearch holds the per-call context and scratch for a JPS search. The
// Finder itself stays immutable so one Finder can serve concurrent searches.
type jpsSearch struct {
	*JumpPointFinderBase
	plus        *JPSPlusFinder
	states      *searchStates
	grid        Grid
	walk        walkableView
	startNode   *Node
	endNode     *Node
	endX, endY  int32
	neighborBuf []*Node
}

// statelessFinder marks the base JPS implementation as safe to reuse after a grid clone.
func (b *JumpPointFinderBase) statelessFinder() {}

// GridWithJPSSupport is optionally implemented by Grid types that support
// canonical JPS neighbor pruning (standard orthogonal grids with axis-aligned topology).
type GridWithJPSSupport interface {
	Grid
	SupportsJPSCanonicalPruning() bool
}

// supportsJPS reports whether the grid has the axis-aligned topology that JPS
// assumes. Hex and staggered grids must use A*.
func supportsJPS(grid Grid) bool {
	gs, ok := grid.(GridWithJPSSupport)
	return ok && gs.SupportsJPSCanonicalPruning()
}

// preflightJPS validates a JPS request before any search state is acquired.
func preflightJPS(startX, startY, endX, endY int32, grid Grid) (path [][2]int32, err error) {
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	if !supportsJPS(grid) {
		return nil, ErrUnsupportedGrid
	}
	return checkEndpoints(startX, startY, endX, endY, grid)
}

// NewJumpPointFinderBase creates a new JumpPointFinderBase.
func NewJumpPointFinderBase(opts ...Option) *JumpPointFinderBase {
	opt := ApplyOptions(opts)
	f := &JumpPointFinderBase{
		Heuristic:        Manhattan,
		DiagonalMovement: resolveDiagonalMovement(opt),
		Weight:           1,
		TrackRecursion:   false,
	}
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	} else if f.DiagonalMovement != DiagonalNever {
		f.Heuristic = Octile
	}
	if opt.Weight > 0 {
		f.Weight = opt.Weight
	}
	f.TrackRecursion = opt.TrackRecursion
	return f
}

// FindPath finds a path using JPS.
func (b *JumpPointFinderBase) FindPath(startX, startY, endX, endY int32, grid Grid, dst [][2]int32) ([][2]int32, error) {
	if path, err := preflightJPS(startX, startY, endX, endY, grid); path != nil || err != nil {
		return finishPath(path, dst, true), err
	}
	searchSeq := globalSearchSeq.Add(1)
	states, err := acquireStates(grid.Width(), grid.Height(), searchSeq)
	if err != nil {
		return nil, err
	}
	defer releaseStates(states)
	p, err := findJPSPath(startX, startY, endX, endY, grid, states, b, nil)
	return finishPath(p, dst, false), err
}

func findJPSPath(startX, startY, endX, endY int32, grid Grid, states *searchStates, b *JumpPointFinderBase, plus *JPSPlusFinder) ([][2]int32, error) {
	s := &states.jps
	*s = jpsSearch{
		JumpPointFinderBase: b,
		plus:                plus,
		states:              states,
		grid:                grid,
		walk:                newWalkableView(grid),
		startNode:           grid.GetNodeAt(startX, startY),
		endNode:             grid.GetNodeAt(endX, endY),
		endX:                endX,
		endY:                endY,
		neighborBuf:         states.neighborBuf[:0],
	}
	defer func() {
		states.neighborBuf = s.neighborBuf
		*s = jpsSearch{}
	}()
	openList := &minHeap{nodes: states.heap[:0], state: states}
	defer func() {
		clear(openList.nodes[:cap(openList.nodes)])
		states.heap = openList.nodes[:0]
	}()
	pathBuf := states.pathBuf[:0]
	expandBuf := states.expandBuf[:0]
	defer func() {
		states.pathBuf = pathBuf
		states.expandBuf = expandBuf
	}()

	startState := states.get(s.startNode)
	startState.G = 0
	startState.F = 0
	startIdx := states.index(s.startNode)
	endIdx := states.index(s.endNode)
	openList.Push(startIdx, startState.F)
	startState.Opened = 1

	for !openList.Empty() {
		nodeIdx := openList.Pop()
		node := grid.GetNodeAt(nodeIdx%states.width, nodeIdx/states.width)
		states.states[nodeIdx].Closed = true

		if nodeIdx == endIdx {
			pathBuf = states.backtrace(s.endNode, pathBuf)
			// Jump point paths skip intermediate tiles, so expand inline.
			// Uses expandBuf instead of ExpandPath to avoid allocation.
			expandBuf = expandBuf[:0]
			for i := 0; i < len(pathBuf)-1; i++ {
				x0, y0 := pathBuf[i][0], pathBuf[i][1]
				x1, y1 := pathBuf[i+1][0], pathBuf[i+1][1]
				dx := x1 - x0
				dy := y1 - y0
				var sx, sy int32
				if dx < 0 {
					dx = -dx
					sx = -1
				} else {
					sx = 1
				}
				if dy < 0 {
					dy = -dy
					sy = -1
				} else {
					sy = 1
				}
				err := dx - dy
				for {
					if x0 == x1 && y0 == y1 {
						break
					}
					expandBuf = append(expandBuf, [2]int32{x0, y0})
					e2 := 2 * err
					if e2 > -dy {
						err -= dy
						x0 += sx
					}
					if e2 < dx {
						err += dx
						y0 += sy
					}
				}
			}
			expandBuf = append(expandBuf, pathBuf[len(pathBuf)-1])
			return expandBuf, nil
		}

		s.identifySuccessors(node, openList)
	}

	return nil, nil
}

func (s *jpsSearch) identifySuccessors(node *Node, openList *minHeap) {
	grid := s.grid
	endX := s.endNode.X
	endY := s.endNode.Y
	x, y := node.X, node.Y
	nodeIdx := s.states.index(node)

	// FindPath guarantees supportsJPS(grid), so canonical pruning is safe.
	var heuristic HeuristicFunc
	var weight float32
	neighbors := s.states.neighborPosBuf[:0]
	if s.plus != nil {
		heuristic, weight = s.plus.Heuristic, s.plus.Weight
		neighbors = alwaysFindNeighbors(s, node, neighbors)
	} else {
		heuristic, weight = s.Heuristic, s.Weight
		neighbors = s.findNeighborsFn(s, node, neighbors)
	}
	s.states.neighborPosBuf = neighbors[:0]
	for _, neighbor := range neighbors {
		var jx, jy int32
		var ok bool
		if s.plus != nil {
			jx, jy, ok = s.plus.jumpAlways(s, neighbor[0], neighbor[1], x, y)
		} else {
			jx, jy, ok = s.jumpFn(s, neighbor[0], neighbor[1], x, y)
		}
		if !ok {
			continue
		}
		jumpNode := grid.GetNodeAt(jx, jy)
		jumpIdx := s.states.index(jumpNode)
		ns := s.states.at(jumpIdx)
		if ns.Closed {
			continue
		}

		d := Octile(float32(AbsInt(jx-x)), float32(AbsInt(jy-y)))
		ng := s.states.states[nodeIdx].G + d

		if ns.Opened == 0 || ng < ns.G {
			ns.G = ng
			if ns.Opened == 0 {
				ns.H = heuristic(float32(AbsInt(jx-endX)), float32(AbsInt(jy-endY)))
			}
			ns.F = ns.G + ns.H*weight
			ns.Parent = nodeIdx

			if ns.Opened == 0 {
				openList.Push(jumpIdx, ns.F)
				ns.Opened = 1
			} else {
				openList.UpdateItem(jumpIdx, ns.F)
			}
		}
	}
}

// NewJumpPointFinder creates a JPS finder based on the diagonal movement setting.
func NewJumpPointFinder(opts ...Option) Finder {
	switch resolveDiagonalMovement(ApplyOptions(opts)) {
	case DiagonalNever:
		return NewJPFNeverMoveDiagonally(opts...)
	case DiagonalAlways:
		return NewJPFAlwaysMoveDiagonally(opts...)
	case DiagonalOnlyWhenNoObstacles:
		return NewJPFMoveDiagonallyIfNoObstacles(opts...)
	default:
		return NewJPFMoveDiagonallyIfAtMostOneObstacle(opts...)
	}
}
