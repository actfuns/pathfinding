package finder

// JPFMoveDiagonallyIfNoObstacles is JPS that only moves diagonally when there are no obstacles.
type JPFMoveDiagonallyIfNoObstacles struct {
	JumpPointFinderBase
}

// NewJPFMoveDiagonallyIfNoObstacles creates a new JPFMoveDiagonallyIfNoObstacles.
func NewJPFMoveDiagonallyIfNoObstacles(opts ...Option) *JPFMoveDiagonallyIfNoObstacles {
	f := &JPFMoveDiagonallyIfNoObstacles{}
	f.JumpPointFinderBase = *NewJumpPointFinderBase(withDiagonal(opts, DiagonalOnlyWhenNoObstacles)...)
	f.jumpFn = noObstaclesJump
	f.findNeighborsFn = noObstaclesFindNeighbors
	return f
}

func noObstaclesJump(s *jpsSearch, x, y, px, py int32) (int32, int32, bool) {
	grid := s.grid
	dx := x - px
	dy := y - py

	if !grid.IsWalkableAt(x, y) {
		return 0, 0, false
	}

	if s.TrackRecursion {
		s.states.get(s.grid.GetNodeAt(x, y)).Tested = true
	}

	if grid.GetNodeAt(x, y) == s.endNode {
		return x, y, true
	}

	if dx != 0 && dy != 0 {
		if _, _, ok := noObstaclesJump(s, x+dx, y, x, y); ok {
			return x, y, true
		}
		if _, _, ok := noObstaclesJump(s, x, y+dy, x, y); ok {
			return x, y, true
		}
	} else {
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
		}
	}

	if grid.IsWalkableAt(x+dx, y) && grid.IsWalkableAt(x, y+dy) {
		return noObstaclesJump(s, x+dx, y+dy, x, y)
	}
	return 0, 0, false
}

func noObstaclesFindNeighbors(s *jpsSearch, node *Node, buf [][2]int32) [][2]int32 {
	parent := s.states.get(node).Parent
	x, y := node.X, node.Y
	grid := s.grid

	if parent >= 0 {
		px, py := s.states.xy(parent)
		dx := (x - px) / MaxInt(AbsInt(x-px), 1)
		dy := (y - py) / MaxInt(AbsInt(y-py), 1)

		neighbors := buf[:0]

		if dx != 0 && dy != 0 {
			if grid.IsWalkableAt(x, y+dy) {
				neighbors = append(neighbors, [2]int32{x, y + dy})
			}
			if grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int32{x + dx, y})
			}
			if grid.IsWalkableAt(x, y+dy) && grid.IsWalkableAt(x+dx, y) {
				neighbors = append(neighbors, [2]int32{x + dx, y + dy})
			}
		} else {
			if dx != 0 {
				isNextWalkable := grid.IsWalkableAt(x+dx, y)
				isTopWalkable := grid.IsWalkableAt(x, y+1)
				isBottomWalkable := grid.IsWalkableAt(x, y-1)

				if isNextWalkable {
					neighbors = append(neighbors, [2]int32{x + dx, y})
					if isTopWalkable {
						neighbors = append(neighbors, [2]int32{x + dx, y + 1})
					}
					if isBottomWalkable {
						neighbors = append(neighbors, [2]int32{x + dx, y - 1})
					}
				}
				if isTopWalkable {
					neighbors = append(neighbors, [2]int32{x, y + 1})
				}
				if isBottomWalkable {
					neighbors = append(neighbors, [2]int32{x, y - 1})
				}
			} else if dy != 0 {
				isNextWalkable := grid.IsWalkableAt(x, y+dy)
				isRightWalkable := grid.IsWalkableAt(x+1, y)
				isLeftWalkable := grid.IsWalkableAt(x-1, y)

				if isNextWalkable {
					neighbors = append(neighbors, [2]int32{x, y + dy})
					if isRightWalkable {
						neighbors = append(neighbors, [2]int32{x + 1, y + dy})
					}
					if isLeftWalkable {
						neighbors = append(neighbors, [2]int32{x - 1, y + dy})
					}
				}
				if isRightWalkable {
					neighbors = append(neighbors, [2]int32{x + 1, y})
				}
				if isLeftWalkable {
					neighbors = append(neighbors, [2]int32{x - 1, y})
				}
			}
		}
		return neighbors
	}

	neighborNodes := s.grid.GetNeighbors(node, DiagonalOnlyWhenNoObstacles, s.neighborBuf)
	s.neighborBuf = neighborNodes
	neighbors := buf[:0]
	for _, n := range neighborNodes {
		neighbors = append(neighbors, [2]int32{n.X, n.Y})
	}
	return neighbors
}
