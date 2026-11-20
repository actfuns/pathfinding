package finder

// computeAlwaysDistances records the first goal-independent JPS jump on
// each ray. Negative distances record the number of walkable steps. Each
// directional pass visits the cell ahead before the current cell.
func (j *JPSPlusFinder) computeAlwaysDistances(grid Grid) {
	walk := newWalkableView(grid)
	for _, dir := range [...]int{dirN, dirE, dirS, dirW, dirNE, dirSE, dirSW, dirNW} {
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
				if !walk.at(nx, ny) {
					continue
				}
				ahead := int((ny*j.width + nx) * 8)
				dist := j.getTable(ahead + dir)
				if staticAlwaysJump(&walk, nx, ny, dx, dy) ||
					(dx != 0 && dy != 0 &&
						(j.getTable(ahead+jpsCardinalIdx[[2]int32{dx, 0}]) > 0 ||
							j.getTable(ahead+jpsCardinalIdx[[2]int32{0, dy}]) > 0)) {
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

func staticAlwaysJump(walk *walkableView, x, y, dx, dy int32) bool {
	switch {
	case dy == 0:
		return (!walk.at(x, y+1) && walk.at(x+dx, y+1)) ||
			(!walk.at(x, y-1) && walk.at(x+dx, y-1))
	case dx == 0:
		return (!walk.at(x+1, y) && walk.at(x+1, y+dy)) ||
			(!walk.at(x-1, y) && walk.at(x-1, y+dy))
	default:
		return (!walk.at(x-dx, y) && walk.at(x-dx, y+dy)) ||
			(!walk.at(x, y-dy) && walk.at(x+dx, y-dy))
	}
}

func (j *JPSPlusFinder) jumpAlways(s *jpsSearch, x, y, px, py int32) (int32, int32, bool) {
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
	} else if (dx == 0 && rx == 0) || (dy == 0 && ry == 0) {
		steps := rx*dx + ry*dy
		if steps > 0 && steps <= limit {
			return s.endX, s.endY, true
		}
	}
	if dist > 0 {
		return px + dist*dx, py + dist*dy, true
	}
	return 0, 0, false
}
