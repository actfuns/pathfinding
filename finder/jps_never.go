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

func neverJump(s *jpsSearch, x, y, px, py int32) (int32, int32, bool) {
	dx := x - px
	dy := y - py

	if !s.walk.at(x, y) {
		return 0, 0, false
	}

	if s.trackRecursion {
		s.states.get(s.grid.GetNodeAt(x, y)).Tested = true
	}

	if x == s.endX && y == s.endY {
		return x, y, true
	}

	if dx != 0 {
		if (s.walk.at(x, y-1) && !s.walk.at(x-dx, y-1)) ||
			(s.walk.at(x, y+1) && !s.walk.at(x-dx, y+1)) {
			return x, y, true
		}
	} else if dy != 0 {
		if (s.walk.at(x-1, y) && !s.walk.at(x-1, y-dy)) ||
			(s.walk.at(x+1, y) && !s.walk.at(x+1, y-dy)) {
			return x, y, true
		}
		if _, _, ok := neverJump(s, x+1, y, x, y); ok {
			return x, y, true
		}
		if _, _, ok := neverJump(s, x-1, y, x, y); ok {
			return x, y, true
		}
	} else {
		panic("Only horizontal and vertical movements are allowed")
	}

	return neverJump(s, x+dx, y+dy, x, y)
}

func neverFindNeighbors(s *jpsSearch, node *Node, buf [][2]int32) [][2]int32 {
	parent := s.states.get(node).Parent
	x, y := node.X, node.Y

	if parent >= 0 {
		px, py := s.states.xy(parent)
		dx := (x - px) / MaxInt(AbsInt(x-px), 1)
		dy := (y - py) / MaxInt(AbsInt(y-py), 1)

		neighbors := buf[:0]

		if dx != 0 {
			if s.walk.at(x, y-1) {
				neighbors = append(neighbors, [2]int32{x, y - 1})
			}
			if s.walk.at(x, y+1) {
				neighbors = append(neighbors, [2]int32{x, y + 1})
			}
			if s.walk.at(x+dx, y) {
				neighbors = append(neighbors, [2]int32{x + dx, y})
			}
		} else if dy != 0 {
			if s.walk.at(x-1, y) {
				neighbors = append(neighbors, [2]int32{x - 1, y})
			}
			if s.walk.at(x+1, y) {
				neighbors = append(neighbors, [2]int32{x + 1, y})
			}
			if s.walk.at(x, y+dy) {
				neighbors = append(neighbors, [2]int32{x, y + dy})
			}
		}
		return neighbors
	}

	neighborNodes := s.grid.GetNeighbors(node, DiagonalNever, s.neighborBuf)
	s.neighborBuf = neighborNodes
	neighbors := buf[:0]
	for _, n := range neighborNodes {
		neighbors = append(neighbors, [2]int32{n.X, n.Y})
	}
	return neighbors
}
