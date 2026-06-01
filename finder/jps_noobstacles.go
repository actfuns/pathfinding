package finder

// JPFMoveDiagonallyIfNoObstacles is JPS that only moves diagonally when there are no obstacles.
type JPFMoveDiagonallyIfNoObstacles struct {
	JumpPointFinderBase
}

// NewJPFMoveDiagonallyIfNoObstacles creates a new JPFMoveDiagonallyIfNoObstacles.
func NewJPFMoveDiagonallyIfNoObstacles(opts ...Option) *JPFMoveDiagonallyIfNoObstacles {
	f := &JPFMoveDiagonallyIfNoObstacles{}
	f.JumpPointFinderBase = *NewJumpPointFinderBase(opts...)
	f.jumpFn = noObstaclesJump
	f.findNeighborsFn = noObstaclesFindNeighbors
	return f
}

func noObstaclesJump(b *JumpPointFinderBase, x, y, px, py int) *[2]int {
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
		if noObstaclesJump(b, x+dx, y, x, y) != nil || noObstaclesJump(b, x, y+dy, x, y) != nil {
			return &[2]int{x, y}
		}
	} else {
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
		}
	}

	if grid.IsWalkableAt(x+dx, y) && grid.IsWalkableAt(x, y+dy) {
		return noObstaclesJump(b, x+dx, y+dy, x, y)
	}
	return nil
}

func noObstaclesFindNeighbors(b *JumpPointFinderBase, node *Node) [][2]int {
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
			if grid.IsWalkableAt(x, y+dy) && grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int{x + dx, y + dy})
			}
		} else {
			if dx != 0 {
				isNextWalkable := grid.IsWalkableAt(x+dx, y)
				isTopWalkable := grid.IsWalkableAt(x, y+1)
				isBottomWalkable := grid.IsWalkableAt(x, y-1)

				if isNextWalkable {
					neighbors = append(neighbors, [2]int{x + dx, y})
					if isTopWalkable {
						neighbors = append(neighbors, [2]int{x + dx, y + 1})
					}
					if isBottomWalkable {
						neighbors = append(neighbors, [2]int{x + dx, y - 1})
					}
				}
				if isTopWalkable {
					neighbors = append(neighbors, [2]int{x, y + 1})
				}
				if isBottomWalkable {
					neighbors = append(neighbors, [2]int{x, y - 1})
				}
			} else if dy != 0 {
				isNextWalkable := grid.IsWalkableAt(x, y+dy)
				isRightWalkable := grid.IsWalkableAt(x+1, y)
				isLeftWalkable := grid.IsWalkableAt(x-1, y)

				if isNextWalkable {
					neighbors = append(neighbors, [2]int{x, y + dy})
					if isRightWalkable {
						neighbors = append(neighbors, [2]int{x + 1, y + dy})
					}
					if isLeftWalkable {
						neighbors = append(neighbors, [2]int{x - 1, y + dy})
					}
				}
				if isRightWalkable {
					neighbors = append(neighbors, [2]int{x + 1, y})
				}
				if isLeftWalkable {
					neighbors = append(neighbors, [2]int{x - 1, y})
				}
			}
		}
		return neighbors
	}

	neighborNodes := grid.GetNeighbors(node, DiagonalOnlyWhenNoObstacles, b.neighborBuf)
	neighbors := make([][2]int, len(neighborNodes))
	for i, n := range neighborNodes {
		neighbors[i] = [2]int{n.X, n.Y}
	}
	return neighbors
}
