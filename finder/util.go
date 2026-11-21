package finder

import "math"

// preflight validates endpoints and handles requests that need no search.
func preflight(startX, startY, endX, endY int32, grid Grid) (path [][2]int32, err error) {
	if !grid.IsInside(startX, startY) || !grid.IsInside(endX, endY) {
		return nil, ErrOutOfBounds
	}
	return checkEndpoints(startX, startY, endX, endY, grid)
}

// checkEndpoints handles requests that need no search. It returns a non-nil
// path for a zero-length request on a walkable tile, and ErrNotWalkable for an
// unwalkable destination. A nil path with a nil error means "run the search".
func checkEndpoints(startX, startY, endX, endY int32, grid Grid) (path [][2]int32, err error) {
	if !grid.IsWalkableAt(endX, endY) {
		return nil, ErrNotWalkable
	}
	if startX == endX && startY == endY {
		return [][2]int32{{startX, startY}}, nil
	}
	return nil, nil
}

func finishPath(path, dst [][2]int32, owned bool) [][2]int32 {
	if path == nil {
		return nil
	}
	if dst != nil {
		return append(dst[:0], path...)
	}
	if owned {
		return path
	}
	return clonePath(path)
}

// PathLength computes the total length of a path.
func PathLength(path [][2]int32) float32 {
	var sum float32
	for i := 1; i < len(path); i++ {
		dx := float32(path[i-1][0]) - float32(path[i][0])
		dy := float32(path[i-1][1]) - float32(path[i][1])
		sum += float32(math.Sqrt(float64(dx*dx + dy*dy)))
	}
	return sum
}

// Interpolate returns all coordinates on the line from (x0, y0) to (x1, y1)
// using Bresenham's algorithm.
func Interpolate(x0, y0, x1, y1 int32) [][2]int32 {
	line := appendLineInterior(make([][2]int32, 0, 16), x0, y0, x1, y1)
	return append(line, [2]int32{x1, y1})
}

// appendLineInterior includes the start and omits the end, so adjacent segments share a point.
func appendLineInterior(line [][2]int32, x0, y0, x1, y1 int32) [][2]int32 {
	dx := x1 - x0
	dy := y1 - y0
	var sx, sy int32
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

	for x0 != x1 || y0 != y1 {
		line = append(line, [2]int32{x0, y0})
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
func ExpandPath(path [][2]int32) [][2]int32 {
	if len(path) < 2 {
		return nil
	}
	return expandPathInto(make([][2]int32, 0, len(path)*2), path)
}

func expandPathInto(dst, path [][2]int32) [][2]int32 {
	dst = dst[:0]
	for i := 1; i < len(path); i++ {
		from, to := path[i-1], path[i]
		dst = appendLineInterior(dst, from[0], from[1], to[0], to[1])
	}
	return append(dst, path[len(path)-1])
}

// CompressPath removes redundant collinear nodes from a path.
func CompressPath(path [][2]int32) [][2]int32 {
	if len(path) < 3 {
		return path
	}
	compressed := make([][2]int32, 0, len(path))
	sx, sy := path[0][0], path[0][1]
	px, py := path[1][0], path[1][1]
	dx := float32(px) - float32(sx)
	dy := float32(py) - float32(sy)
	sq := float32(math.Sqrt(float64(dx*dx + dy*dy)))
	dx /= sq
	dy /= sq

	compressed = append(compressed, [2]int32{sx, sy})

	for i := 2; i < len(path); i++ {
		lx, ly := px, py
		ldx, ldy := dx, dy
		px, py = path[i][0], path[i][1]
		dx = float32(px) - float32(lx)
		dy = float32(py) - float32(ly)
		sq = float32(math.Sqrt(float64(dx*dx + dy*dy)))
		dx /= sq
		dy /= sq

		if dx != ldx || dy != ldy {
			compressed = append(compressed, [2]int32{lx, ly})
		}
	}
	compressed = append(compressed, [2]int32{px, py})
	return compressed
}
