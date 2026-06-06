package gridutil

import "math"

// HasLineOfSightBresenham reports whether two tiles can see each other —
// every tile on the Bresenham line between them is walkable.
// Only suitable for orthogonal grids. For hex and staggered grids, use
// HasLineOfSightDense instead.
func HasLineOfSightBresenham(g GridLOS, x1, y1, x2, y2 int) bool {
	dx := x2 - x1
	dy := y2 - y1
	var sx, sy int
	if dx < 0 {
		dx = -dx
		sx = -1
	} else {
		sx = 1
	}
	if dy < 0 {
		dy = -dy
		sy = -1
	} else {
		sy = 1
	}
	err := dx - dy
	x, y := x1, y1
	for {
		if !g.IsWalkableAt(x, y) {
			return false
		}
		if x == x2 && y == y2 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x += sx
		}
		if e2 < dx {
			err += dx
			y += sy
		}
	}
	return true
}

// HasLineOfSightDense reports whether two tiles can see each other using
// world-space sub-tile sampling. Suitable for all grid types.
// For orthogonal grids, HasLineOfSightBresenham is more efficient.
func HasLineOfSightDense(g GridWorld, x1, y1, x2, y2 int) bool {
	return denseLineOfSight(g, [2]int{x1, y1}, [2]int{x2, y2})
}

// SmoothenBresenham removes unnecessary waypoints from a tile path using
// Bresenham line-of-sight (greedy string-pulling).
// Only suitable for orthogonal grids. For hex and staggered grids, use
// SmoothenDense instead.
// Writes the result in-place over the input buffer (the finder's cached pathBuf).
func SmoothenBresenham(g GridLOS, path [][2]int) [][2]int {
	return smoothenPathInternal(g, path, false)
}

// SmoothenBresenhamStrict is like SmoothenBresenham but also checks corner cells
// at each diagonal step, preventing the smoothed path from cutting
// through the corner of a blocked tile.
// Only suitable for orthogonal grids.
func SmoothenBresenhamStrict(g GridLOS, path [][2]int) [][2]int {
	return smoothenPathInternal(g, path, true)
}

func smoothenPathInternal(g GridLOS, path [][2]int, strict bool) [][2]int {
	if len(path) < 2 {
		return path
	}
	writeIdx := 1
	for i := 2; i < len(path); i++ {
		if !tileLineOfSight(g, path[writeIdx-1], path[i], strict) {
			path[writeIdx] = path[i-1]
			writeIdx++
		}
	}
	if path[writeIdx-1] != path[len(path)-1] {
		path[writeIdx] = path[len(path)-1]
		writeIdx++
	}
	return path[:writeIdx]
}

// tileLineOfSight checks walkability along a Bresenham line between two tiles.
// When strict=true, also checks the two axis-aligned cells at each diagonal step
// to prevent corner-cutting.
func tileLineOfSight(g GridLOS, a, b [2]int, strict bool) bool {
	x0, y0 := a[0], a[1]
	x1, y1 := b[0], b[1]
	dx := x1 - x0
	dy := y1 - y0
	var sx, sy int
	if dx < 0 {
		dx = -dx
		sx = -1
	} else {
		sx = 1
	}
	if dy < 0 {
		dy = -dy
		sy = -1
	} else {
		sy = 1
	}
	err := dx - dy

	for {
		if !g.IsWalkableAt(x0, y0) {
			return false
		}
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		movedX, movedY := false, false
		if e2 > -dy {
			err -= dy
			x0 += sx
			movedX = true
		}
		if e2 < dx {
			err += dx
			y0 += sy
			movedY = true
		}
		// Strict mode: at each diagonal step (both axes moved),
		// also check the two axis-aligned cells the line passes through
		// the corner of. This prevents corner-cutting.
		if strict && movedX && movedY {
			if !g.IsWalkableAt(x0, y0-sy) || !g.IsWalkableAt(x0-sx, y0) {
				return false
			}
		}
	}
	return true
}

// SmoothenDense removes unnecessary waypoints from a tile path using
// world-space line-of-sight sampling. Suitable for all grid types.
// Writes the result in-place over the input buffer.
func SmoothenDense(g GridWorld, path [][2]int) [][2]int {
	if len(path) < 2 {
		return path
	}
	writeIdx := 1
	for i := 2; i < len(path); i++ {
		if !denseLineOfSight(g, path[writeIdx-1], path[i]) {
			path[writeIdx] = path[i-1]
			writeIdx++
		}
	}
	if path[writeIdx-1] != path[len(path)-1] {
		path[writeIdx] = path[len(path)-1]
		writeIdx++
	}
	return path[:writeIdx]
}

// denseLineOfSight checks walkability along a pixel-precise line between
// two tile centers. It samples the line in world space at sub-tile intervals
// and converts each sample back to tile coordinates.
func denseLineOfSight(g GridWorld, a, b [2]int) bool {
	ax, ay := g.TileToWorld(a[0], a[1])
	bx, by := g.TileToWorld(b[0], b[1])
	minDim := minInt(g.TileWidth(), g.TileHeight())
	if minDim == 0 {
		minDim = 1
	}
	steps := math.Sqrt(float64((bx-ax)*(bx-ax)+(by-ay)*(by-ay))) / float64(minDim) * 2
	if steps < 1 {
		steps = 1
	}
	for t := 0; t < int(steps); t++ {
		f := float64(t) / steps
		wx := float64(ax) + float64(bx-ax)*f
		wy := float64(ay) + float64(by-ay)*f
		tx, ty := g.WorldToTile(float32(wx), float32(wy))
		if !g.IsWalkableAt(tx, ty) {
			return false
		}
	}
	return true
}
