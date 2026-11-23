package finder

// Canonical JPS+ tables record the first goal-independent jump on each ray.
// The table is mode-specific: the static jump rules and the diagonal
// movement constraints differ between the four DiagonalMovement modes.
func (j *jumpTable) computeCanonicalDistances(grid Grid, mode DiagonalMovement) {
	walk := newWalkableView(grid)
	dirs := canonicalDirectionOrder(mode)
	for _, dir := range dirs {
		dx, dy := dirToVec(dir)
		for row := int32(0); row < j.height; row++ {
			y := row
			if dy > 0 {
				y = j.height - 1 - row
			}
			for column := int32(0); column < j.width; column++ {
				x := column
				if dx > 0 {
					x = j.width - 1 - column
				}
				nx, ny := x+dx, y+dy
				// A search may leave a blocked source tile, but every tile
				// reached along the ray must be walkable.
				if !walk.at(nx, ny) || !canonicalStepAllowed(&walk, mode, x, y, dx, dy) {
					continue
				}
				ahead := int((ny*j.width + nx) * 8)
				dist := j.getTable(ahead + dir)
				if canonicalStaticJump(&walk, mode, nx, ny, dx, dy) ||
					canonicalExtraJump(j, mode, ahead, dx, dy) {
					dist = 1
				} else if dist > 0 {
					dist++
				} else {
					dist--
				}
				j.setTable(int((y*j.width+x)*8)+dir, dist)
			}
		}
	}
}

func canonicalDirectionOrder(mode DiagonalMovement) []int {
	if mode == DiagonalNever {
		// Horizontal tables are needed by vertical static jumps.
		return []int{dirE, dirW, dirN, dirS}
	}
	return []int{dirE, dirW, dirN, dirS, dirNE, dirSE, dirSW, dirNW}
}

// canonicalStepAllowed applies the mode's one-step diagonal rule.
func canonicalStepAllowed(walk *walkableView, mode DiagonalMovement, x, y, dx, dy int32) bool {
	if dx == 0 || dy == 0 {
		return true
	}
	switch mode {
	case DiagonalNever:
		return false
	case DiagonalOnlyWhenNoObstacles:
		return walk.at(x+dx, y) && walk.at(x, y+dy)
	case DiagonalIfAtMostOneObstacle:
		return walk.at(x+dx, y) || walk.at(x, y+dy)
	default: // DiagonalAlways
		return true
	}
}

// canonicalStaticJump is the forced-neighbor test at the cell ahead on a ray.
func canonicalStaticJump(walk *walkableView, mode DiagonalMovement, x, y, dx, dy int32) bool {
	switch {
	case dy == 0:
		if mode == DiagonalNever || mode == DiagonalOnlyWhenNoObstacles {
			return (walk.at(x, y-1) && !walk.at(x-dx, y-1)) ||
				(walk.at(x, y+1) && !walk.at(x-dx, y+1))
		}
		return (walk.at(x+dx, y+1) && !walk.at(x, y+1)) ||
			(walk.at(x+dx, y-1) && !walk.at(x, y-1))
	case dx == 0:
		if mode == DiagonalNever || mode == DiagonalOnlyWhenNoObstacles {
			return (walk.at(x-1, y) && !walk.at(x-1, y-dy)) ||
				(walk.at(x+1, y) && !walk.at(x+1, y-dy))
		}
		return (walk.at(x+1, y+dy) && !walk.at(x+1, y)) ||
			(walk.at(x-1, y+dy) && !walk.at(x-1, y))
	default:
		if mode == DiagonalAlways || mode == DiagonalIfAtMostOneObstacle {
			return (!walk.at(x-dx, y) && walk.at(x-dx, y+dy)) ||
				(!walk.at(x, y-dy) && walk.at(x+dx, y-dy))
		}
		return false
	}
}

// canonicalExtraJump handles turns that are not forced neighbors: cardinal
// jumps reached from a diagonal cell, and the cardinal turns used by
// DiagonalNever's vertical ray.
func canonicalExtraJump(j *jumpTable, mode DiagonalMovement, ahead int, dx, dy int32) bool {
	if dx != 0 && dy != 0 {
		return j.getTable(ahead+jpsCardinalIdx[[2]int32{dx, 0}]) > 0 ||
			j.getTable(ahead+jpsCardinalIdx[[2]int32{0, dy}]) > 0
	}
	if mode == DiagonalNever && dy != 0 {
		return j.getTable(ahead+dirE) > 0 || j.getTable(ahead+dirW) > 0
	}
	return false
}

// tableJump resolves a goal-dependent jump using the canonical table.
func tableJump(s *jpsSearch, x, y, px, py int32) (int32, int32, bool) {
	j := s.table
	dx, dy := x-px, y-py
	rx, ry := s.endX-px, s.endY-py
	dir := jpsCardinalIdx[[2]int32{dx, dy}]
	if dx != 0 && dy != 0 {
		dir = jpsDiagonalIdx[[2]int32{dx, dy}]
	}
	dist := int32(j.getDist(px, py, dir))
	limit := AbsInt(dist)

	if dx != 0 && dy != 0 {
		// A diagonal ray can meet the goal directly or cross its row or
		// column. A cardinal ray from that intersection can then reach it.
		h, v := rx*dx, ry*dy
		steps := min(h, v)
		if steps > 0 && steps <= limit {
			jx, jy := px+steps*dx, py+steps*dy
			if h == v ||
				(h > v && h-steps <= AbsInt(int32(j.getDist(jx, jy, jpsCardinalIdx[[2]int32{dx, 0}])))) ||
				(v > h && v-steps <= AbsInt(int32(j.getDist(jx, jy, jpsCardinalIdx[[2]int32{0, dy}])))) {
				return jx, jy, true
			}
		}
	} else if dy == 0 {
		steps := rx * dx
		if steps > 0 && steps <= limit {
			if ry == 0 {
				return s.endX, s.endY, true
			}
			jx, jy := s.endX, py
			vdir := dirS
			if ry < 0 {
				vdir = dirN
			}
			if AbsInt(ry) <= AbsInt(int32(j.getDist(jx, jy, vdir))) {
				return jx, jy, true
			}
		}
	} else if dx == 0 {
		steps := ry * dy
		if steps > 0 && steps <= limit {
			if rx == 0 {
				return s.endX, s.endY, true
			}
			jx, jy := px, s.endY
			hdir := dirE
			if rx < 0 {
				hdir = dirW
			}
			if AbsInt(rx) <= AbsInt(int32(j.getDist(jx, jy, hdir))) {
				return jx, jy, true
			}
		}
	}
	if dist > 0 {
		return px + dist*dx, py + dist*dy, true
	}
	return 0, 0, false
}

// jpsPlusNeighbors selects the canonical JPS pruning function for a mode.
func jpsPlusNeighbors(mode DiagonalMovement) func(*jpsSearch, *Node, [][2]int32) [][2]int32 {
	switch mode {
	case DiagonalNever:
		return neverFindNeighbors
	case DiagonalOnlyWhenNoObstacles:
		return noObstaclesFindNeighbors
	case DiagonalIfAtMostOneObstacle:
		return atMostOneFindNeighbors
	default:
		return alwaysFindNeighbors
	}
}
