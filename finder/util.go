package finder

import "math"

// checkEndpoints handles requests that need no search. It returns a non-nil
// path for a zero-length request on a walkable tile, and ErrNotWalkable for an
// unwalkable destination. A nil path with a nil error means "run the search".
func checkEndpoints(startX, startY, endX, endY int, grid Grid) (path [][2]int, err error) {
	if !grid.IsWalkableAt(endX, endY) {
		return nil, ErrNotWalkable
	}
	if startX == endX && startY == endY {
		return [][2]int{{startX, startY}}, nil
	}
	return nil, nil
}

// PathLength computes the total length of a path.
func PathLength(path [][2]int) float64 {
	var sum float64
	for i := 1; i < len(path); i++ {
		dx := float64(path[i-1][0] - path[i][0])
		dy := float64(path[i-1][1] - path[i][1])
		sum += math.Sqrt(dx*dx + dy*dy)
	}
	return sum
}

// Interpolate returns all coordinates on the line from (x0, y0) to (x1, y1)
// using Bresenham's algorithm.
func Interpolate(x0, y0, x1, y1 int) [][2]int {
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

// ExpandPath interpolates all segments in a compressed path.
func ExpandPath(path [][2]int) [][2]int {
	if len(path) < 2 {
		return nil
	}
	expanded := make([][2]int, 0, len(path)*2)
	for i := 0; i < len(path)-1; i++ {
		interpolated := Interpolate(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
		expanded = append(expanded, interpolated[:len(interpolated)-1]...)
	}
	expanded = append(expanded, path[len(path)-1])
	return expanded
}

// CompressPath removes redundant collinear nodes from a path.
func CompressPath(path [][2]int) [][2]int {
	if len(path) < 3 {
		return path
	}
	compressed := make([][2]int, 0, len(path))
	sx, sy := path[0][0], path[0][1]
	px, py := path[1][0], path[1][1]
	dx := float64(px - sx)
	dy := float64(py - sy)
	sq := math.Sqrt(dx*dx + dy*dy)
	dx /= sq
	dy /= sq

	compressed = append(compressed, [2]int{sx, sy})

	for i := 2; i < len(path); i++ {
		lx, ly := px, py
		ldx, ldy := dx, dy
		px, py = path[i][0], path[i][1]
		dx = float64(px - lx)
		dy = float64(py - ly)
		sq = math.Sqrt(dx*dx + dy*dy)
		dx /= sq
		dy /= sq

		if dx != ldx || dy != ldy {
			compressed = append(compressed, [2]int{lx, ly})
		}
	}
	compressed = append(compressed, [2]int{px, py})
	return compressed
}
