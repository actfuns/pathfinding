package core

import "math"

// Backtrace returns the path from start to end node using parent links.
func Backtrace(node *Node) [][2]int {
	path := make([][2]int, 0, 16)
	for node != nil {
		path = append(path, [2]int{node.X, node.Y})
		node = node.Parent
	}
	// Reverse in place
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

// BiBacktrace returns the path from start to end node using bidirectional parent links.
func BiBacktrace(nodeA, nodeB *Node) [][2]int {
	pathA := make([][2]int, 0, 16)
	for nodeA != nil {
		pathA = append(pathA, [2]int{nodeA.X, nodeA.Y})
		nodeA = nodeA.Parent
	}
	pathB := make([][2]int, 0, 16)
	for nodeB != nil {
		pathB = append(pathB, [2]int{nodeB.X, nodeB.Y})
		nodeB = nodeB.Parent
	}
	// Reverse pathA, then append pathB (keep pathB order as-is since it was built from end->start)
	for i, j := 0, len(pathA)-1; i < j; i, j = i+1, j-1 {
		pathA[i], pathA[j] = pathA[j], pathA[i]
	}
	return append(pathA, pathB...)
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
		// Add all but the last point (to avoid duplicates)
		expanded = append(expanded, interpolated[:len(interpolated)-1]...)
	}
	expanded = append(expanded, path[len(path)-1])
	return expanded
}

// SmoothenPath smooths a path by removing unnecessary waypoints.
func SmoothenPath(grid *Grid, path [][2]int) [][2]int {
	if len(path) < 2 {
		return path
	}
	sx, sy := path[0][0], path[0][1]
	x1, y1 := path[len(path)-1][0], path[len(path)-1][1]
	newPath := [][2]int{{sx, sy}}

	for i := 2; i < len(path); i++ {
		ex, ey := path[i][0], path[i][1]
		line := Interpolate(sx, sy, ex, ey)
		blocked := false
		for j := 1; j < len(line); j++ {
			if !grid.IsWalkableAt(line[j][0], line[j][1]) {
				blocked = true
				break
			}
		}
		if blocked {
			lastValid := path[i-1]
			newPath = append(newPath, lastValid)
			sx, sy = lastValid[0], lastValid[1]
		}
	}
	newPath = append(newPath, [2]int{x1, y1})
	return newPath
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
