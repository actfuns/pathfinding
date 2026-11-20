package finder

// JPFAlwaysMoveDiagonally is JPS that always moves diagonally.
type JPFAlwaysMoveDiagonally struct {
	JumpPointFinderBase
}

// NewJPFAlwaysMoveDiagonally creates a new JPFAlwaysMoveDiagonally.
func NewJPFAlwaysMoveDiagonally(opts ...Option) *JPFAlwaysMoveDiagonally {
	f := &JPFAlwaysMoveDiagonally{}
	f.JumpPointFinderBase = *NewJumpPointFinderBase(withDiagonal(opts, DiagonalAlways)...)
	f.jumpFn = alwaysJump
	f.findNeighborsFn = alwaysFindNeighbors
	return f
}

func alwaysJump(s *jpsSearch, x, y, px, py int32) (int32, int32, bool) {
	dx := x - px
	dy := y - py

	if !s.walk.at(x, y) {
		return 0, 0, false
	}

	if s.TrackRecursion {
		s.states.get(s.grid.GetNodeAt(x, y)).Tested = true
	}

	if x == s.endX && y == s.endY {
		return x, y, true
	}

	if dx != 0 && dy != 0 {
		if (s.walk.at(x-dx, y+dy) && !s.walk.at(x-dx, y)) ||
			(s.walk.at(x+dx, y-dy) && !s.walk.at(x, y-dy)) {
			return x, y, true
		}
		if _, _, ok := alwaysJump(s, x+dx, y, x, y); ok {
			return x, y, true
		}
		if _, _, ok := alwaysJump(s, x, y+dy, x, y); ok {
			return x, y, true
		}
	} else {
		if dx != 0 {
			if (s.walk.at(x+dx, y+1) && !s.walk.at(x, y+1)) ||
				(s.walk.at(x+dx, y-1) && !s.walk.at(x, y-1)) {
				return x, y, true
			}
		} else {
			if (s.walk.at(x+1, y+dy) && !s.walk.at(x+1, y)) ||
				(s.walk.at(x-1, y+dy) && !s.walk.at(x-1, y)) {
				return x, y, true
			}
		}
	}

	return alwaysJump(s, x+dx, y+dy, x, y)
}

func alwaysFindNeighbors(s *jpsSearch, node *Node, buf [][2]int32) [][2]int32 {
	parent := s.states.get(node).Parent
	x, y := node.X, node.Y

	if parent >= 0 {
		px, py := s.states.xy(parent)
		dx := (x - px) / MaxInt(AbsInt(x-px), 1)
		dy := (y - py) / MaxInt(AbsInt(y-py), 1)

		neighbors := buf[:0]

		if dx != 0 && dy != 0 {
			if s.walk.at(x, y+dy) {
				neighbors = append(neighbors, [2]int32{x, y + dy})
			}
			if s.walk.at(x+dx, y) {
				neighbors = append(neighbors, [2]int32{x + dx, y})
			}
			if s.walk.at(x+dx, y+dy) {
				neighbors = append(neighbors, [2]int32{x + dx, y + dy})
			}
			if !s.walk.at(x-dx, y) {
				neighbors = append(neighbors, [2]int32{x - dx, y + dy})
			}
			if !s.walk.at(x, y-dy) {
				neighbors = append(neighbors, [2]int32{x + dx, y - dy})
			}
		} else {
			if dx == 0 {
				if s.walk.at(x, y+dy) {
					neighbors = append(neighbors, [2]int32{x, y + dy})
				}
				if !s.walk.at(x+1, y) {
					neighbors = append(neighbors, [2]int32{x + 1, y + dy})
				}
				if !s.walk.at(x-1, y) {
					neighbors = append(neighbors, [2]int32{x - 1, y + dy})
				}
			} else {
				if s.walk.at(x+dx, y) {
					neighbors = append(neighbors, [2]int32{x + dx, y})
				}
				if !s.walk.at(x, y+1) {
					neighbors = append(neighbors, [2]int32{x + dx, y + 1})
				}
				if !s.walk.at(x, y-1) {
					neighbors = append(neighbors, [2]int32{x + dx, y - 1})
				}
			}
		}
		return neighbors
	}

	neighborNodes := s.grid.GetNeighbors(node, DiagonalAlways, s.neighborBuf)
	s.neighborBuf = neighborNodes
	neighbors := buf[:0]
	for _, n := range neighborNodes {
		neighbors = append(neighbors, [2]int32{n.X, n.Y})
	}
	return neighbors
}
