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
	openList         *MinHeap
	neighborBuf      []*Node
	neighborPosBuf   [][2]int
	heapSlice        []*Node
	pathBuf          [][2]int
	expandBuf        [][2]int

	jumpFn          func(b *JumpPointFinderBase, x, y, px, py int) (int, int, bool)
	findNeighborsFn func(b *JumpPointFinderBase, node *Node, buf [][2]int) [][2]int
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
func (b *JumpPointFinderBase) FindPath(startX, startY, endX, endY int, grid Grid) ([][2]int, error) {
	// GetNodeAt panics on an out-of-range index, so validate up front and
	// report it as a caller error instead of a crash.
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	if !supportsJPS(grid) {
		return nil, ErrUnsupportedGrid
	}
	if path, err := checkEndpoints(startX, startY, endX, endY, grid); path != nil || err != nil {
		return path, err
	}
	searchSeq := int(globalSearchSeq.Add(1))
	b.openList = &MinHeap{nodes: b.heapSlice[:0]}
	defer func() { b.heapSlice = b.openList.nodes[:0] }()
	b.startNode = grid.GetNodeAt(startX, startY)
	b.endNode = grid.GetNodeAt(endX, endY)
	b.grid = grid
	b.neighborBuf = b.neighborBuf[:0]

	b.startNode.ResetSearch(searchSeq)
	b.endNode.ResetSearch(searchSeq)
	b.startNode.G = 0
	b.startNode.F = 0
	b.openList.Push(b.startNode)
	b.startNode.Opened = 1

	for !b.openList.Empty() {
		node := b.openList.Pop()
		node.Closed = true

		if node == b.endNode {
			b.pathBuf = b.pathBuf[:0]
			for n := b.endNode; n != nil; n = n.Parent {
				b.pathBuf = append(b.pathBuf, [2]int{n.X, n.Y})
			}
			for i, j := 0, len(b.pathBuf)-1; i < j; i, j = i+1, j-1 {
				b.pathBuf[i], b.pathBuf[j] = b.pathBuf[j], b.pathBuf[i]
			}
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

		b.identifySuccessors(node, searchSeq)
	}

	return nil, nil
}

func (b *JumpPointFinderBase) identifySuccessors(node *Node, searchSeq int) {
	grid := b.grid
	heuristic := b.Heuristic
	openList := b.openList
	endX := b.endNode.X
	endY := b.endNode.Y
	x, y := node.X, node.Y

	// FindPath guarantees supportsJPS(grid), so canonical pruning is safe.
	neighbors := b.findNeighborsFn(b, node, b.neighborPosBuf[:0])
	b.neighborPosBuf = neighbors[:0]
	for _, neighbor := range neighbors {
		jx, jy, ok := b.jumpFn(b, neighbor[0], neighbor[1], x, y)
		if !ok {
			continue
		}
		jumpNode := grid.GetNodeAt(jx, jy)
		jumpNode.ResetSearch(searchSeq)

		if jumpNode.Closed {
			continue
		}

		d := Octile(float64(AbsInt(jx-x)), float64(AbsInt(jy-y)))
		ng := node.G + d

		if jumpNode.Opened == 0 || ng < jumpNode.G {
			jumpNode.G = ng
			if jumpNode.Opened == 0 {
				jumpNode.H = heuristic(float64(AbsInt(jx-endX)), float64(AbsInt(jy-endY)))
			}
			jumpNode.F = jumpNode.G + jumpNode.H*b.Weight
			jumpNode.Parent = node

			if jumpNode.Opened == 0 {
				openList.Push(jumpNode)
				jumpNode.Opened = 1
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
