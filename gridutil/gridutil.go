// Package gridutil provides utility functions for grid-based pathfinding.
// All functions work with any grid type that implements grid.Grid.
package gridutil

import (
	"github.com/actfuns/pathfinding/grid"
)

var fastRandState uint32 = 1

func fastRand() uint32 {
	fastRandState ^= fastRandState << 13
	fastRandState ^= fastRandState >> 17
	fastRandState ^= fastRandState << 5
	return fastRandState
}

// RandomWalkable returns a random walkable tile coordinate.
func RandomWalkable(g grid.Grid) (int, int, bool) {
	w, h := g.Width(), g.Height()
	if w == 0 || h == 0 {
		return 0, 0, false
	}
	for i := 0; i < 32; i++ {
		x := int(uint32(w) * fastRand())
		y := int(uint32(h) * fastRand())
		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if g.IsWalkableAt(x, y) {
				return x, y, true
			}
		}
	}
	return 0, 0, false
}

// RandomWalkableWorld returns the world center of a random walkable tile.
func RandomWalkableWorld(g grid.Grid) (float32, float32, bool) {
	tx, ty, ok := RandomWalkable(g)
	if !ok {
		return 0, 0, false
	}
	wx, wy := g.TileToWorld(tx, ty)
	return wx, wy, true
}

// RandomWalkableInRadius returns a random walkable tile within radius tiles of (cx, cy).
func RandomWalkableInRadius(g grid.Grid, cx, cy, radius int) (int, int, bool) {
	for i := 0; i < 32; i++ {
		dx := int(uint32(2*radius+1)*fastRand()) - radius
		dy := int(uint32(2*radius+1)*fastRand()) - radius
		x, y := cx+dx, cy+dy
		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}
	// Fallback: scan all tiles in range
	for dy := -radius; dy <= radius; dy++ {
		for dx := -radius; dx <= radius; dx++ {
			x, y := cx+dx, cy+dy
			if g.IsWalkableAt(x, y) {
				return x, y, true
			}
		}
	}
	return 0, 0, false
}

// RandomWalkableInRadiusWorld returns the world center of a random walkable
// tile within radius tiles of (wx, wy).
func RandomWalkableInRadiusWorld(g grid.Grid, wx, wy float32, radius int) (float32, float32, bool) {
	tx, ty := g.WorldToTile(wx, wy)
	rtx, rty, ok := RandomWalkableInRadius(g, tx, ty, radius)
	if !ok {
		return 0, 0, false
	}
	wx2, wy2 := g.TileToWorld(rtx, rty)
	return wx2, wy2, true
}

// RandomInRing returns a random walkable tile with Chebyshev distance
// between innerR and outerR from (cx, cy). The inner exclusion zone
// is useful for spawning units around a point without landing on the spot.
func RandomInRing(g grid.Grid, cx, cy, innerR, outerR int) (int, int, bool) {
	for i := 0; i < 64; i++ {
		dx := int(uint32(2*outerR+1)*fastRand()) - outerR
		dy := int(uint32(2*outerR+1)*fastRand()) - outerR
		x, y := cx+dx, cy+dy
		dist := max(abs(dx), abs(dy))
		if dist < innerR || dist > outerR {
			continue
		}
		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}
	// Fallback: scan all tiles in the ring
	for dy := -outerR; dy <= outerR; dy++ {
		for dx := -outerR; dx <= outerR; dx++ {
			dist := max(abs(dx), abs(dy))
			if dist < innerR || dist > outerR {
				continue
			}
			x, y := cx+dx, cy+dy
			if g.IsWalkableAt(x, y) {
				return x, y, true
			}
		}
	}
	return 0, 0, false
}

// RandomInRingWorld returns the world center of a random walkable tile
// with Chebyshev distance between innerR and outerR from (wx, wy).
func RandomInRingWorld(g grid.Grid, wx, wy float32, innerR, outerR int) (float32, float32, bool) {
	tx, ty := g.WorldToTile(wx, wy)
	rtx, rty, ok := RandomInRing(g, tx, ty, innerR, outerR)
	if !ok {
		return 0, 0, false
	}
	wx2, wy2 := g.TileToWorld(rtx, rty)
	return wx2, wy2, true
}

// HasLineOfSight reports whether two tiles can see each other —
// every tile on the Bresenham line between them is walkable.
func HasLineOfSight(g grid.Grid, x1, y1, x2, y2 int) bool {
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

// HasLineOfSightWorld reports whether two world positions can see each other.
func HasLineOfSightWorld(g grid.Grid, x1, y1, x2, y2 float32) bool {
	tx1, ty1 := g.WorldToTile(x1, y1)
	tx2, ty2 := g.WorldToTile(x2, y2)
	return HasLineOfSight(g, tx1, ty1, tx2, ty2)
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

// Line returns all tiles on the Bresenham line from (x0, y0) to (x1, y1),
// including both endpoints.
func Line(x0, y0, x1, y1 int) [][2]int {
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
// The result includes (cx, cy) itself.
func TilesInRadius(cx, cy, r int) [][2]int {
	var result [][2]int
	for dy := -r; dy <= r; dy++ {
		for dx := -r; dx <= r; dx++ {
			result = append(result, [2]int{cx + dx, cy + dy})
		}
	}
	return result
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}
