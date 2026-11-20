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
	Weight           float64
	TrackRecursion   bool
	grid             Grid
	startNode        *Node
	endNode          *Node
	neighborBuf      []*Node
	neighborPosBuf   [][2]int
	heapSlice        []heapEntry
	pathBuf          [][2]int
	expandBuf        [][2]int
	jumpFn           func(b *JumpPointFinderBase, states *searchStates, x, y, px, py int) (int, int, bool)
	findNeighborsFn  func(b *JumpPointFinderBase, states *searchStates, node *Node, buf [][2]int) [][2]int
}

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
func preflightJPS(startX, startY, endX, endY int, grid Grid) (path [][2]int, err error) {
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
		neighborBuf:      make([]*Node, 0, 8),
		neighborPosBuf:   make([][2]int, 0, 8),
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
func (b *JumpPointFinderBase) FindPath(startX, startY, endX, endY int, grid Grid, dst [][2]int) ([][2]int, error) {
	if path, err := preflightJPS(startX, startY, endX, endY, grid); path != nil || err != nil {
		return finishPath(path, dst, true), err
	}
	searchSeq := globalSearchSeq.Add(1)
	states, err := acquireStates(grid.Width(), grid.Height(), searchSeq)
	if err != nil {
		return nil, err
	}
	local := *b
	local.grid = nil
	local.startNode = nil
	local.endNode = nil
	local.neighborBuf = states.neighborBuf[:0]
	local.neighborPosBuf = states.neighborPosBuf[:0]
	local.heapSlice = states.heap[:0]
	local.pathBuf = states.pathBuf[:0]
	local.expandBuf = states.expandBuf[:0]
	defer func() {
		states.neighborBuf = local.neighborBuf
		states.neighborPosBuf = local.neighborPosBuf
		states.heap = local.heapSlice
		states.pathBuf = local.pathBuf
		states.expandBuf = local.expandBuf
		releaseStates(states)
	}()
	p, err := local.findPath(startX, startY, endX, endY, grid, states)
	return finishPath(p, dst, false), err
}

func (b *JumpPointFinderBase) findPath(startX, startY, endX, endY int, grid Grid, states *searchStates) ([][2]int, error) {
	openList := &minHeap{nodes: b.heapSlice[:0], state: states}
	defer func() {
		clear(openList.nodes[:cap(openList.nodes)])
		b.heapSlice = openList.nodes[:0]
	}()
	b.startNode = grid.GetNodeAt(startX, startY)
	b.endNode = grid.GetNodeAt(endX, endY)
	b.grid = grid
	b.neighborBuf = b.neighborBuf[:0]

	startState := states.get(b.startNode)
	startState.G = 0
	startState.F = 0
	openList.Push(b.startNode)
	startState.Opened = 1

	for !openList.Empty() {
		node := openList.Pop()
		states.get(node).Closed = true

		if node == b.endNode {
			b.pathBuf = states.backtrace(b.endNode, b.pathBuf)
			// Jump point paths skip intermediate tiles, so expand inline.
			// Uses expandBuf instead of ExpandPath to avoid allocation.
			b.expandBuf = b.expandBuf[:0]
			for i := 0; i < len(b.pathBuf)-1; i++ {
				x0, y0 := b.pathBuf[i][0], b.pathBuf[i][1]
				x1, y1 := b.pathBuf[i+1][0], b.pathBuf[i+1][1]
				dx := x1 - x0
				dy := y1 - y0
				var sx, sy int
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
					b.expandBuf = append(b.expandBuf, [2]int{x0, y0})
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
			b.expandBuf = append(b.expandBuf, b.pathBuf[len(b.pathBuf)-1])
			return b.expandBuf, nil
		}

		b.identifySuccessors(node, states, openList)
	}

	return nil, nil
}

func (b *JumpPointFinderBase) identifySuccessors(node *Node, states *searchStates, openList *minHeap) {
	grid := b.grid
	heuristic := b.Heuristic
	endX := b.endNode.X
	endY := b.endNode.Y
	x, y := node.X, node.Y

	// FindPath guarantees supportsJPS(grid), so canonical pruning is safe.
	neighbors := b.findNeighborsFn(b, states, node, b.neighborPosBuf[:0])
	b.neighborPosBuf = neighbors[:0]
	for _, neighbor := range neighbors {
		jx, jy, ok := b.jumpFn(b, states, neighbor[0], neighbor[1], x, y)
		if !ok {
			continue
		}
		jumpNode := grid.GetNodeAt(jx, jy)
		ns := states.get(jumpNode)
		if ns.Closed {
			continue
		}

		d := Octile(float64(AbsInt(jx-x)), float64(AbsInt(jy-y)))
		ng := states.get(node).G + d

		if ns.Opened == 0 || ng < ns.G {
			ns.G = ng
			if ns.Opened == 0 {
				ns.H = heuristic(float64(AbsInt(jx-endX)), float64(AbsInt(jy-endY)))
			}
			ns.F = ns.G + ns.H*b.Weight
			ns.Parent = states.index(node)

			if ns.Opened == 0 {
				openList.Push(jumpNode)
				ns.Opened = 1
			} else {
				openList.UpdateItem(jumpNode)
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
