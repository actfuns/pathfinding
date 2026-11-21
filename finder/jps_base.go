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

type jpsSearchOptions struct {
	heuristic      HeuristicFunc
	weight         float32
	trackRecursion bool
	neighbors      func(*jpsSearch, *Node, [][2]int32) [][2]int32
	jump           func(*jpsSearch, int32, int32, int32, int32) (int32, int32, bool)
	table          *jumpTable
}

// jpsSearch holds the scratch and immutable configuration of one query.
type jpsSearch struct {
	jpsSearchOptions
	states      *searchStates
	grid        Grid
	walk        walkableView
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
	p, err := findJPSPath(startX, startY, endX, endY, grid, states, jpsSearchOptions{
		heuristic: b.Heuristic, weight: b.Weight, trackRecursion: b.TrackRecursion,
		neighbors: b.findNeighborsFn, jump: b.jumpFn,
	})
	return finishPath(p, dst, false), err
}

func findJPSPath(startX, startY, endX, endY int32, grid Grid, states *searchStates, options jpsSearchOptions) ([][2]int32, error) {
	s := &states.jps
	*s = jpsSearch{
		jpsSearchOptions: options,
		states:           states,
		grid:             grid,
		walk:             newWalkableView(grid),
		endX:             endX,
		endY:             endY,
		neighborBuf:      states.neighborBuf[:0],
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

	startIdx := states.index(grid.GetNodeAt(startX, startY))
	endNode := grid.GetNodeAt(endX, endY)
	endIdx := states.index(endNode)
	startState := states.at(startIdx)
	openList.Push(startIdx, startState.F)
	startState.Opened = 1

	for !openList.Empty() {
		nodeIdx := openList.Pop()
		node := grid.GetNodeAt(nodeIdx%states.width, nodeIdx/states.width)
		states.states[nodeIdx].Closed = true

		if nodeIdx == endIdx {
			pathBuf = states.backtrace(endNode, pathBuf)
			expandBuf = expandPathInto(expandBuf, pathBuf)
			return expandBuf, nil
		}

		s.identifySuccessors(node, nodeIdx, openList)
	}

	return nil, nil
}

func (s *jpsSearch) identifySuccessors(node *Node, nodeIdx int32, openList *minHeap) {
	x, y := node.X, node.Y
	nodeG := s.states.states[nodeIdx].G
	neighbors := s.neighbors(s, node, s.states.neighborPosBuf[:0])
	s.states.neighborPosBuf = neighbors[:0]
	for _, neighbor := range neighbors {
		jx, jy, ok := s.jump(s, neighbor[0], neighbor[1], x, y)
		if !ok {
			continue
		}
		jumpIdx := jy*s.states.width + jx
		ns := s.states.at(jumpIdx)
		if ns.Closed {
			continue
		}

		d := Octile(float32(AbsInt(jx-x)), float32(AbsInt(jy-y)))
		ng := nodeG + d

		if ns.Opened == 0 || ng < ns.G {
			ns.G = ng
			if ns.Opened == 0 {
				ns.H = s.heuristic(float32(AbsInt(jx-s.endX)), float32(AbsInt(jy-s.endY)))
			}
			ns.F = ns.G + ns.H*s.weight
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
