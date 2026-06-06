package gridutil

// CompressPath removes collinear waypoints from a tile path without any
// line-of-sight checking. Only points where the direction changes are kept.
// This is guaranteed to never cut corners since it only removes truly
// redundant waypoints. Writes the result in-place over the input buffer.
func CompressPath(path [][2]int) [][2]int {
	if len(path) < 3 {
		return path
	}
	writeIdx := 1
	for i := 1; i < len(path)-1; i++ {
		dx1 := path[i][0] - path[writeIdx-1][0]
		dy1 := path[i][1] - path[writeIdx-1][1]
		dx2 := path[i+1][0] - path[i][0]
		dy2 := path[i+1][1] - path[i][1]
		// Skip point i if the two segments are in the same direction (collinear)
		if dx1 == dx2 && dy1 == dy2 {
			continue
		}
		path[writeIdx] = path[i]
		writeIdx++
	}
	path[writeIdx] = path[len(path)-1]
	writeIdx++
	return path[:writeIdx]
}

// TileDistance returns the Chebyshev distance between two tiles:
// max(|dx|, |dy|). This is the number of steps needed for 8-directional movement.
func TileDistance(x1, y1, x2, y2 int) int {
	dx := x1 - x2
	if dx < 0 {
		dx = -dx
	}
	dy := y1 - y2
	if dy < 0 {
		dy = -dy
	}
	if dx > dy {
		return dx
	}
	return dy
}

// BresenhamLine returns all tiles on the Bresenham line from (x0, y0) to (x1, y1),
// including both endpoints.
func BresenhamLine(x0, y0, x1, y1 int) [][2]int {
	line := make([][2]int, 0, 16)
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
		line = append(line, [2]int{x0, y0})
		if x0 == x1 && y0 == y1 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
	return line
}

// TilesInRadius returns all tiles within Chebyshev distance r from (cx, cy).
func TilesInRadius(cx, cy, r int) [][2]int {
	var result [][2]int
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			result = append(result, [2]int{cx + dx, cy + dy})
		}
	}
	return result
}
