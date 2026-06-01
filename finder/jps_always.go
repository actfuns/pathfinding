package finder

// JPFAlwaysMoveDiagonally is JPS that always moves diagonally.
type JPFAlwaysMoveDiagonally struct {
	JumpPointFinderBase
}

// NewJPFAlwaysMoveDiagonally creates a new JPFAlwaysMoveDiagonally.
func NewJPFAlwaysMoveDiagonally(opt *FinderOptions) *JPFAlwaysMoveDiagonally {
	f := &JPFAlwaysMoveDiagonally{}
	f.JumpPointFinderBase = *NewJumpPointFinderBase(opt)
	f.jumpFn = alwaysJump
	f.findNeighborsFn = alwaysFindNeighbors
	return f
}

func alwaysJump(b *JumpPointFinderBase, x, y, px, py int) *[2]int {
	grid := b.grid
	dx := x - px
	dy := y - py

	if !grid.IsWalkableAt(x, y) {
		return nil
	}

	if b.TrackRecursion {
		grid.GetNodeAt(x, y).Tested = true
	}

	if grid.GetNodeAt(x, y) == b.endNode {
		return &[2]int{x, y}
	}

	if dx != 0 && dy != 0 {
		if (grid.IsWalkableAt(x-dx, y+dy) && !grid.IsWalkableAt(x-dx, y)) ||
			(grid.IsWalkableAt(x+dx, y-dy) && !grid.IsWalkableAt(x, y-dy)) {
			return &[2]int{x, y}
		}
		if alwaysJump(b, x+dx, y, x, y) != nil || alwaysJump(b, x, y+dy, x, y) != nil {
			return &[2]int{x, y}
		}
	} else {
		if dx != 0 {
			if (grid.IsWalkableAt(x+dx, y+1) && !grid.IsWalkableAt(x, y+1)) ||
				(grid.IsWalkableAt(x+dx, y-1) && !grid.IsWalkableAt(x, y-1)) {
				return &[2]int{x, y}
			}
		} else {
			if (grid.IsWalkableAt(x+1, y+dy) && !grid.IsWalkableAt(x+1, y)) ||
				(grid.IsWalkableAt(x-1, y+dy) && !grid.IsWalkableAt(x-1, y)) {
				return &[2]int{x, y}
			}
		}
	}

	return alwaysJump(b, x+dx, y+dy, x, y)
}

func alwaysFindNeighbors(b *JumpPointFinderBase, node *Node) [][2]int {
	parent := node.Parent
	x, y := node.X, node.Y
	grid := b.grid

	if parent != nil {
		px, py := parent.X, parent.Y
		dx := (x - px) / MaxInt(AbsInt(x-px), 1)
		dy := (y - py) / MaxInt(AbsInt(y-py), 1)

		neighbors := make([][2]int, 0, 5)

		if dx != 0 && dy != 0 {
			if grid.IsWalkableAt(x, y+dy) {
				neighbors = append(neighbors, [2]int{x, y + dy})
			}
			if grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int{x + dx, y})
			}
			if grid.IsWalkableAt(x+dx, y+dy) {
				neighbors = append(neighbors, [2]int{x + dx, y + dy})
			}
			if !grid.IsWalkableAt(x-dx, y) {
				neighbors = append(neighbors, [2]int{x - dx, y + dy})
			}
			if !grid.IsWalkableAt(x, y-dy) {
				neighbors = append(neighbors, [2]int{x + dx, y - dy})
			}
		} else {
			if dx == 0 {
				if grid.IsWalkableAt(x, y+dy) {
					neighbors = append(neighbors, [2]int{x, y + dy})
				}
				if !grid.IsWalkableAt(x+1, y) {
					neighbors = append(neighbors, [2]int{x + 1, y + dy})
				}
				if !grid.IsWalkableAt(x-1, y) {
					neighbors = append(neighbors, [2]int{x - 1, y + dy})
				}
			} else {
				if grid.IsWalkableAt(x+dx, y) {
					neighbors = append(neighbors, [2]int{x + dx, y})
				}
				if !grid.IsWalkableAt(x, y+1) {
					neighbors = append(neighbors, [2]int{x + dx, y + 1})
				}
				if !grid.IsWalkableAt(x, y-1) {
					neighbors = append(neighbors, [2]int{x + dx, y - 1})
				}
			}
		}
		return neighbors
	}

	neighborNodes := grid.GetNeighbors(node, DiagonalAlways, b.neighborBuf)
	neighbors := make([][2]int, len(neighborNodes))
	for i, n := range neighborNodes {
		neighbors[i] = [2]int{n.X, n.Y}
	}
	return neighbors
}
