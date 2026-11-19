package finder

// JPFNeverMoveDiagonally is JPS with only horizontal/vertical movement.
type JPFNeverMoveDiagonally struct {
	JumpPointFinderBase
}

// NewJPFNeverMoveDiagonally creates a new JPFNeverMoveDiagonally.
func NewJPFNeverMoveDiagonally(opts ...Option) *JPFNeverMoveDiagonally {
	f := &JPFNeverMoveDiagonally{}
	f.JumpPointFinderBase = *NewJumpPointFinderBase(withDiagonal(opts, DiagonalNever)...)
	f.jumpFn = neverJump
	f.findNeighborsFn = neverFindNeighbors
	return f
}

func neverJump(b *JumpPointFinderBase, states *searchStates, x, y, px, py int) (int, int, bool) {
	grid := b.grid
	dx := x - px
	dy := y - py

	if !grid.IsWalkableAt(x, y) {
		return 0, 0, false
	}

	if b.TrackRecursion {
		states.get(grid.GetNodeAt(x, y)).Tested = true
	}

	if grid.GetNodeAt(x, y) == b.endNode {
		return x, y, true
	}

	if dx != 0 {
		if (grid.IsWalkableAt(x, y-1) && !grid.IsWalkableAt(x-dx, y-1)) ||
			(grid.IsWalkableAt(x, y+1) && !grid.IsWalkableAt(x-dx, y+1)) {
			return x, y, true
		}
	} else if dy != 0 {
		if (grid.IsWalkableAt(x-1, y) && !grid.IsWalkableAt(x-1, y-dy)) ||
			(grid.IsWalkableAt(x+1, y) && !grid.IsWalkableAt(x+1, y-dy)) {
			return x, y, true
		}
		if _, _, ok := neverJump(b, states, x+1, y, x, y); ok {
			return x, y, true
		}
		if _, _, ok := neverJump(b, states, x-1, y, x, y); ok {
			return x, y, true
		}
	} else {
		panic("Only horizontal and vertical movements are allowed")
	}

	return neverJump(b, states, x+dx, y+dy, x, y)
}

func neverFindNeighbors(b *JumpPointFinderBase, states *searchStates, node *Node, buf [][2]int) [][2]int {
	parent := states.get(node).Parent
	x, y := node.X, node.Y
	grid := b.grid

	if parent >= 0 {
		px, py := states.xy(parent)
		dx := (x - px) / MaxInt(AbsInt(x-px), 1)
		dy := (y - py) / MaxInt(AbsInt(y-py), 1)

		neighbors := buf[:0]

		if dx != 0 {
			if grid.IsWalkableAt(x, y-1) {
				neighbors = append(neighbors, [2]int{x, y - 1})
			}
			if grid.IsWalkableAt(x, y+1) {
				neighbors = append(neighbors, [2]int{x, y + 1})
			}
			if grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int{x + dx, y})
			}
		} else if dy != 0 {
			if grid.IsWalkableAt(x-1, y) {
				neighbors = append(neighbors, [2]int{x - 1, y})
			}
			if grid.IsWalkableAt(x+1, y) {
				neighbors = append(neighbors, [2]int{x + 1, y})
			}
			if grid.IsWalkableAt(x, y+dy) {
				neighbors = append(neighbors, [2]int{x, y + dy})
			}
		}
		return neighbors
	}

	neighborNodes := grid.GetNeighbors(node, DiagonalNever, b.neighborBuf)
	neighbors := buf[:0]
	for _, n := range neighborNodes {
		neighbors = append(neighbors, [2]int{n.X, n.Y})
	}
	return neighbors
}
