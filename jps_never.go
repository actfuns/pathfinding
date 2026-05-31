package pathfinding

// JPFNeverMoveDiagonally is JPS with only horizontal/vertical movement.
type JPFNeverMoveDiagonally struct {
	JumpPointFinderBase
}

// NewJPFNeverMoveDiagonally creates a new JPFNeverMoveDiagonally.
func NewJPFNeverMoveDiagonally(opt *FinderOptions) *JPFNeverMoveDiagonally {
	f := &JPFNeverMoveDiagonally{}
	f.JumpPointFinderBase = *NewJumpPointFinderBase(opt)
	f.jumpFn = neverJump
	f.findNeighborsFn = neverFindNeighbors
	return f
}

func neverJump(b *JumpPointFinderBase, x, y, px, py int) *[2]int {
	grid := b.grid
	dx := x - px
	dy := y - py

	if !grid.IsWalkableAt(x, y) {
		return nil
	}

	if b.TrackRecursion {
		grid.GetNodeAt(x, y).tested = true
	}

	if grid.GetNodeAt(x, y) == b.endNode {
		return &[2]int{x, y}
	}

	if dx != 0 {
		if (grid.IsWalkableAt(x, y-1) && !grid.IsWalkableAt(x-dx, y-1)) ||
			(grid.IsWalkableAt(x, y+1) && !grid.IsWalkableAt(x-dx, y+1)) {
			return &[2]int{x, y}
		}
	} else if dy != 0 {
		if (grid.IsWalkableAt(x-1, y) && !grid.IsWalkableAt(x-1, y-dy)) ||
			(grid.IsWalkableAt(x+1, y) && !grid.IsWalkableAt(x+1, y-dy)) {
			return &[2]int{x, y}
		}
		if neverJump(b, x+1, y, x, y) != nil || neverJump(b, x-1, y, x, y) != nil {
			return &[2]int{x, y}
		}
	} else {
		panic("Only horizontal and vertical movements are allowed")
	}

	return neverJump(b, x+dx, y+dy, x, y)
}

func neverFindNeighbors(b *JumpPointFinderBase, node *Node) [][2]int {
	parent := node.parent
	x, y := node.X, node.Y
	grid := b.grid

	if parent != nil {
		px, py := parent.X, parent.Y
		dx := (x - px) / maxInt(absInt(x-px), 1)
		dy := (y - py) / maxInt(absInt(y-py), 1)

		neighbors := make([][2]int, 0, 3)

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

	neighborNodes := grid.GetNeighbors(node, DiagonalNever)
	neighbors := make([][2]int, len(neighborNodes))
	for i, n := range neighborNodes {
		neighbors[i] = [2]int{n.X, n.Y}
	}
	return neighbors
}
