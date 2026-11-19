package finder

// JPFMoveDiagonallyIfAtMostOneObstacle is JPS that moves diagonally when at most one obstacle exists.
type JPFMoveDiagonallyIfAtMostOneObstacle struct {
	JumpPointFinderBase
}

// NewJPFMoveDiagonallyIfAtMostOneObstacle creates a new JPFMoveDiagonallyIfAtMostOneObstacle.
func NewJPFMoveDiagonallyIfAtMostOneObstacle(opts ...Option) *JPFMoveDiagonallyIfAtMostOneObstacle {
	f := &JPFMoveDiagonallyIfAtMostOneObstacle{}
	f.JumpPointFinderBase = *NewJumpPointFinderBase(withDiagonal(opts, DiagonalIfAtMostOneObstacle)...)
	f.jumpFn = atMostOneJump
	f.findNeighborsFn = atMostOneFindNeighbors
	return f
}

func atMostOneJump(b *JumpPointFinderBase, states *searchStates, x, y, px, py int) (int, int, bool) {
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

	if dx != 0 && dy != 0 {
		if (grid.IsWalkableAt(x-dx, y+dy) && !grid.IsWalkableAt(x-dx, y)) ||
			(grid.IsWalkableAt(x+dx, y-dy) && !grid.IsWalkableAt(x, y-dy)) {
			return x, y, true
		}
		if _, _, ok := atMostOneJump(b, states, x+dx, y, x, y); ok {
			return x, y, true
		}
		if _, _, ok := atMostOneJump(b, states, x, y+dy, x, y); ok {
			return x, y, true
		}
	} else {
		if dx != 0 {
			if (grid.IsWalkableAt(x+dx, y+1) && !grid.IsWalkableAt(x, y+1)) ||
				(grid.IsWalkableAt(x+dx, y-1) && !grid.IsWalkableAt(x, y-1)) {
				return x, y, true
			}
		} else {
			if (grid.IsWalkableAt(x+1, y+dy) && !grid.IsWalkableAt(x+1, y)) ||
				(grid.IsWalkableAt(x-1, y+dy) && !grid.IsWalkableAt(x-1, y)) {
				return x, y, true
			}
		}
	}

	if grid.IsWalkableAt(x+dx, y) || grid.IsWalkableAt(x, y+dy) {
		return atMostOneJump(b, states, x+dx, y+dy, x, y)
	}
	return 0, 0, false
}

func atMostOneFindNeighbors(b *JumpPointFinderBase, states *searchStates, node *Node, buf [][2]int) [][2]int {
	parent := states.get(node).Parent
	x, y := node.X, node.Y
	grid := b.grid

	if parent >= 0 {
		px, py := states.xy(parent)
		dx := (x - px) / MaxInt(AbsInt(x-px), 1)
		dy := (y - py) / MaxInt(AbsInt(y-py), 1)

		neighbors := buf[:0]

		if dx != 0 && dy != 0 {
			if grid.IsWalkableAt(x, y+dy) {
				neighbors = append(neighbors, [2]int{x, y + dy})
			}
			if grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int{x + dx, y})
			}
			if grid.IsWalkableAt(x, y+dy) || grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int{x + dx, y + dy})
			}
			if !grid.IsWalkableAt(x-dx, y) && grid.IsWalkableAt(x, y+dy) {
				neighbors = append(neighbors, [2]int{x - dx, y + dy})
			}
			if !grid.IsWalkableAt(x, y-dy) && grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int{x + dx, y - dy})
			}
		} else {
			if dx == 0 {
				if grid.IsWalkableAt(x, y+dy) {
					neighbors = append(neighbors, [2]int{x, y + dy})
					if !grid.IsWalkableAt(x+1, y) {
						neighbors = append(neighbors, [2]int{x + 1, y + dy})
					}
					if !grid.IsWalkableAt(x-1, y) {
						neighbors = append(neighbors, [2]int{x - 1, y + dy})
					}
				}
			} else {
				if grid.IsWalkableAt(x+dx, y) {
					neighbors = append(neighbors, [2]int{x + dx, y})
					if !grid.IsWalkableAt(x, y+1) {
						neighbors = append(neighbors, [2]int{x + dx, y + 1})
					}
					if !grid.IsWalkableAt(x, y-1) {
						neighbors = append(neighbors, [2]int{x + dx, y - 1})
					}
				}
			}
		}
		return neighbors
	}

	neighborNodes := grid.GetNeighbors(node, DiagonalIfAtMostOneObstacle, b.neighborBuf)
	neighbors := buf[:0]
	for _, n := range neighborNodes {
		neighbors = append(neighbors, [2]int{n.X, n.Y})
	}
	return neighbors
}
