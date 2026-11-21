package finder

import "github.com/actfuns/pathfinding/internal/walkability"

// jumpTable owns the geometry metadata and packed distances of a baked map.
type jumpTable struct {
	width, height int32

	// jump8/jump16 store precomputed jump distances per cell per direction.
	// Layout: (y*width + x)*8 + dir
	// Positive = jump point at that distance; negative = open ray length without a jump point.
	// int8 is used when both dimensions are <= 128; int16 otherwise.
	jump8          []int8
	jump16         []int16
	wide           bool
	canonical      bool
	fingerprint    uint64
	hasFingerprint bool
}

// jumpTableSize validates packed ray lengths, flat indices, and byte size.
func jumpTableSize(width, height uint32, wide bool) (int, bool) {
	limit, entryBytes := uint32(128), uint64(1)
	if wide {
		limit, entryBytes = 32768, 2
	}
	if width == 0 || height == 0 || width > limit || height > limit {
		return 0, false
	}
	entries := uint64(width) * uint64(height) * 8
	const maxInt32 = 1<<31 - 1
	maxInt := uint64(^uint(0) >> 1)
	if entries > maxInt32 || entries*entryBytes+24 > maxInt {
		return 0, false
	}
	return int(entries), true
}

func (j *jumpTable) resize(size int) {
	if j.wide {
		if cap(j.jump16) < size {
			j.jump16 = make([]int16, size)
		} else {
			j.jump16 = j.jump16[:size]
		}
		j.jump8 = nil
	} else {
		if cap(j.jump8) < size {
			j.jump8 = make([]int8, size)
		} else {
			j.jump8 = j.jump8[:size]
		}
		j.jump16 = nil
	}
}

func gridWalkabilityFingerprint(grid Grid) uint64 {
	if g, ok := grid.(WalkabilityFingerprintGrid); ok {
		return g.WalkabilityFingerprint()
	}
	width, height := grid.Width(), grid.Height()
	var fingerprint uint64
	for y := int32(0); y < height; y++ {
		for x := int32(0); x < width; x++ {
			if !grid.IsWalkableAt(x, y) {
				fingerprint = walkability.Toggle(fingerprint, uint64(y)*uint64(width)+uint64(x))
			}
		}
	}
	return fingerprint
}

// detectPrimaryJumpPoints scans each walkable cell for forced neighbors.
// Returns a per-cell bitmask of directions where the cell acts as a jump point.
func (j *jumpTable) detectPrimaryJumpPoints(grid Grid) []uint8 {
	w, h := j.width, j.height
	primaryJPs := make([]uint8, int(w*h))

	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}

			for _, dir := range jpsCardinalDirs {
				nx, ny := x+dir[0], y+dir[1]
				if !grid.IsInside(nx, ny) || grid.IsWalkableAt(nx, ny) {
					continue
				}

				// Perpendicular to wall direction
				var checkDirs [2][2]int32
				if dir[0] != 0 { // horizontal direction (E/W)
					checkDirs = [2][2]int32{{0, -1}, {0, 1}} // N/S
				} else { // vertical direction (N/S)
					checkDirs = [2][2]int32{{-1, 0}, {1, 0}} // W/E
				}

				for _, cd := range checkDirs {
					// From the wall cell, check perpendicular
					wnx, wny := nx+cd[0], ny+cd[1]
					if !grid.IsInside(wnx, wny) || !grid.IsWalkableAt(wnx, wny) {
						continue
					}

					// Potential jump point from origin in perpendicular direction
					jpx, jpy := x+cd[0], y+cd[1]
					if !grid.IsInside(jpx, jpy) || !grid.IsWalkableAt(jpx, jpy) {
						continue
					}

					dirIdx := jpsCardinalIdx[cd]
					primaryJPs[int(jpy*w+jpx)] |= 1 << dirIdx
				}
			}
		}
	}
	return primaryJPs
}

// computeDistances fills the jumpTable with distances in all 8 directions.
func (j *jumpTable) computeDistances(grid Grid, primaryJPs []uint8) {
	w, h := j.width, j.height

	// Helper: get distance to obstacle or jump point in a cardinal direction
	scanCardinal := func(x, y int32, dx, dy int32, dirIdx int) int16 {
		dist := int32(0)
		for {
			nx, ny := x+dx*(dist+1), y+dy*(dist+1)
			if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
				return int16(-dist)
			}
			if primaryJPs[int(ny*w+nx)]&(1<<dirIdx) != 0 {
				return int16(dist + 1)
			}
			dist++
		}
	}

	// Part A: Cardinal directions
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := int((y*w + x) * 8)
			for _, dir := range jpsCardinalDirs {
				dirIdx := jpsCardinalIdx[dir]
				j.setTable(base+dirIdx, scanCardinal(x, y, dir[0], dir[1], dirIdx))
			}
		}
	}

	// Part B: Diagonal directions
	for y := int32(0); y < h; y++ {
		for x := int32(0); x < w; x++ {
			if !grid.IsWalkableAt(x, y) {
				continue
			}
			base := int((y*w + x) * 8)

			for _, ddir := range jpsDiagonalDirs {
				ddirIdx := jpsDiagonalIdx[ddir]
				hdx, hdy := ddir[0], int32(0) // horizontal component
				vdx, vdy := int32(0), ddir[1] // vertical component
				hDirIdx := jpsCardinalIdx[[2]int32{hdx, hdy}]
				vDirIdx := jpsCardinalIdx[[2]int32{vdx, vdy}]

				dist := int32(0)
				for {
					cx, cy := x+ddir[0]*dist, y+ddir[1]*dist
					if !grid.IsInside(cx, cy) || !grid.IsWalkableAt(cx, cy) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}

					if dist > 0 {
						// Check cardinal neighbors at this diagonal position
						cb := int((cy*w + cx) * 8)
						if j.getTable(cb+hDirIdx) > 0 || j.getTable(cb+vDirIdx) > 0 {
							j.setTable(base+ddirIdx, int16(dist))
							break
						}
					}

					// Check horizontal and vertical neighbors are walkable
					hnx, hny := cx+hdx, cy+hdy
					vnx, vny := cx+vdx, cy+vdy
					if !grid.IsInside(hnx, hny) || !grid.IsWalkableAt(hnx, hny) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}
					if !grid.IsInside(vnx, vny) || !grid.IsWalkableAt(vnx, vny) {
						j.setTable(base+ddirIdx, int16(-dist))
						break
					}

					// Next diagonal step
					nx, ny := cx+ddir[0], cy+ddir[1]
					if !grid.IsInside(nx, ny) || !grid.IsWalkableAt(nx, ny) {
						j.setTable(base+ddirIdx, int16(-(dist + 1)))
						break
					}
					dist++
				}
			}
		}
	}
}

// hasTable reports whether Precompute (or LoadPrecomputed) has run.
func (j *jumpTable) hasTable() bool { return j.jump8 != nil || j.jump16 != nil }

func (j *jumpTable) getTable(i int) int16 {
	if j.wide {
		return j.jump16[i]
	}
	return int16(j.jump8[i])
}

func (j *jumpTable) setTable(i int, v int16) {
	if j.wide {
		j.jump16[i] = v
	} else {
		j.jump8[i] = int8(v)
	}
}

// getDist returns the precomputed jump distance for cell (x,y) in direction dirIdx.
func (j *jumpTable) getDist(x, y int32, dirIdx int) int16 {
	if x < 0 || x >= j.width || y < 0 || y >= j.height {
		return 0
	}
	return j.getTable(int((y*j.width+x)*8) + dirIdx)
}
