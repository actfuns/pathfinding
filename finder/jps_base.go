package finder

// JumpPointFinderBase is the base implementation for Jump Point Search.
type JumpPointFinderBase struct {
	Heuristic        HeuristicFunc
	DiagonalMovement DiagonalMovement
	TrackRecursion   bool
	grid             Grid
	startNode        *Node
	endNode          *Node
	openList         *MinHeap
	neighborBuf      []*Node
	heapSlice        []*Node
	pathBuf          [][2]int
	expandBuf        [][2]int

	searchSeq int

	jumpFn          func(b *JumpPointFinderBase, x, y, px, py int) *[2]int
	findNeighborsFn func(b *JumpPointFinderBase, node *Node) [][2]int
}

// GridWithJPSSupport is optionally implemented by Grid types that support
// canonical JPS neighbor pruning (standard orthogonal grids with axis-aligned topology).
type GridWithJPSSupport interface {
	Grid
	SupportsJPSCanonicalPruning() bool
}

// NewJumpPointFinderBase creates a new JumpPointFinderBase.
func NewJumpPointFinderBase(opts ...Option) *JumpPointFinderBase {
	opt := ApplyOptions(opts)
	f := &JumpPointFinderBase{
		Heuristic:        Manhattan,
		DiagonalMovement: opt.DiagonalMovement,
		TrackRecursion:   false,
		neighborBuf:      make([]*Node, 0, 8),
	}
	if opt.Heuristic != nil {
		f.Heuristic = opt.Heuristic
	}
	f.TrackRecursion = opt.TrackRecursion
	return f
}

// FindPath finds a path using JPS.
func (b *JumpPointFinderBase) FindPath(startX, startY, endX, endY int, grid Grid) [][2]int {
	b.searchSeq++
	b.openList = &MinHeap{nodes: b.heapSlice[:0]}
	defer func() { b.heapSlice = b.openList.nodes[:0] }()
	b.startNode = grid.GetNodeAt(startX, startY)
	b.endNode = grid.GetNodeAt(endX, endY)
	b.grid = grid
	b.neighborBuf = b.neighborBuf[:0]

	b.startNode.ResetSearch(b.searchSeq)
	b.endNode.ResetSearch(b.searchSeq)
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
			// Jump point paths skip intermediate tiles, so ExpandPath fills them in.
			// This still allocates the expanded result, which can't be cached
			// because the expansion length varies per path.
			return ExpandPath(b.pathBuf)
		}

		b.identifySuccessors(node)
	}

	return nil
}

func (b *JumpPointFinderBase) identifySuccessors(node *Node) {
	grid := b.grid
	heuristic := b.Heuristic
	openList := b.openList
	endX := b.endNode.X
	endY := b.endNode.Y
	x, y := node.X, node.Y

	// Non-orthogonal grids (staggered) need grid.GetNeighbors for correct neighbor topology.
	// JPS pruning makes axis-aligned assumptions that don't hold on staggered/hex grids.
	var neighbors [][2]int
	if gs, ok := grid.(GridWithJPSSupport); ok && gs.SupportsJPSCanonicalPruning() {
		neighbors = b.findNeighborsFn(b, node)
	} else {
		nodes := grid.GetNeighbors(node, b.DiagonalMovement, b.neighborBuf)
		neighbors = make([][2]int, len(nodes))
		for i, n := range nodes {
			neighbors[i] = [2]int{n.X, n.Y}
		}
	}
	for _, neighbor := range neighbors {
		jumpPoint := b.jumpFn(b, neighbor[0], neighbor[1], x, y)
		if jumpPoint == nil {
			continue
		}

		jx, jy := jumpPoint[0], jumpPoint[1]
		jumpNode := grid.GetNodeAt(jx, jy)
		jumpNode.ResetSearch(b.searchSeq)

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
			jumpNode.F = jumpNode.G + jumpNode.H
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
	opt := ApplyOptions(opts)
	switch opt.DiagonalMovement {
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

// appendLine appends Bresenham line tiles from (x0,y0) to (x1,y1),
// excluding the start point (already in expandBuf), without allocating.
func (b *JumpPointFinderBase) appendLine(x0, y0, x1, y1 int) {
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
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
		b.expandBuf = append(b.expandBuf, [2]int{x0, y0})
	}
}
