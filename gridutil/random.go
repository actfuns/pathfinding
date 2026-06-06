package gridutil

import "math"

var fastRandState uint32 = 1

func fastRand() uint32 {
	fastRandState ^= fastRandState << 13
	fastRandState ^= fastRandState >> 17
	fastRandState ^= fastRandState << 5
	return fastRandState
}

// RandomWalkable returns a random walkable tile coordinate.
func RandomWalkable(g GridLOS) (int, int, bool) {
	w, h := g.Width(), g.Height()
	if w == 0 || h == 0 {
		return 0, 0, false
	}

	// Fast path: random sampling.
	for i := 0; i < 32; i++ {
		x := int(fastRand() % uint32(w))
		y := int(fastRand() % uint32(h))

		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}

	// Fallback: reservoir sampling over all walkable tiles.
	count := 0
	var chosenX, chosenY int

	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			if !g.IsWalkableAt(x, y) {
				continue
			}

			count++

			if fastRand()%uint32(count) == 0 {
				chosenX = x
				chosenY = y
			}
		}
	}

	if count == 0 {
		return 0, 0, false
	}

	return chosenX, chosenY, true
}

// RandomWalkableWorld returns the world center of a random walkable tile.
func RandomWalkableWorld(g GridWorld) (float32, float32, bool) {
	tx, ty, ok := RandomWalkable(g)
	if !ok {
		return 0, 0, false
	}
	wx, wy := g.TileToWorld(tx, ty)
	return wx, wy, true
}

// RandomWalkableInRadius returns a random walkable tile within radius tiles of (cx, cy).
func RandomWalkableInRadius(g GridLOS, cx, cy, radius int) (int, int, bool) {
	if radius < 0 {
		return 0, 0, false
	}

	for i := 0; i < 32; i++ {
		dx := int(fastRand()%uint32(2*radius+1)) - radius
		dy := int(fastRand()%uint32(2*radius+1)) - radius

		x, y := cx+dx, cy+dy
		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}

	count := 0
	var chosenX, chosenY int

	for dy := -radius; dy <= radius; dy++ {
		for dx := -radius; dx <= radius; dx++ {
			x, y := cx+dx, cy+dy

			if !g.IsWalkableAt(x, y) {
				continue
			}

			count++

			if fastRand()%uint32(count) == 0 {
				chosenX = x
				chosenY = y
			}
		}
	}

	if count == 0 {
		return 0, 0, false
	}

	return chosenX, chosenY, true
}

// RandomWalkableInRadiusWorld returns the center of a random walkable tile
// within radius world units of (wx, wy).
func RandomWalkableInRadiusWorld(g GridWorld, wx, wy float32, radius float32) (float32, float32, bool) {
	if radius <= 0 {
		return 0, 0, false
	}

	const maxRetry = 64

	// Fast path: random sampling.
	for i := 0; i < maxRetry; i++ {
		angle := float64(fastRand()) / float64(^uint32(0)) * 2 * math.Pi

		r := float64(radius) * math.Sqrt(
			float64(fastRand())/float64(^uint32(0)),
		)

		x := wx + float32(r*math.Cos(angle))
		y := wy + float32(r*math.Sin(angle))

		tx, ty := g.WorldToTile(x, y)
		if g.IsWalkableAt(tx, ty) {
			rwx, rwy := g.TileToWorld(tx, ty)
			return rwx, rwy, true
		}
	}

	// Fallback: reservoir sampling over all walkable tiles in circle.
	p1x, p1y := g.WorldToTile(wx-radius, wy-radius)
	p2x, p2y := g.WorldToTile(wx-radius, wy+radius)
	p3x, p3y := g.WorldToTile(wx+radius, wy-radius)
	p4x, p4y := g.WorldToTile(wx+radius, wy+radius)

	minTx := minInt(minInt(p1x, p2x), minInt(p3x, p4x))
	maxTx := maxInt(maxInt(p1x, p2x), maxInt(p3x, p4x))

	minTy := minInt(minInt(p1y, p2y), minInt(p3y, p4y))
	maxTy := maxInt(maxInt(p1y, p2y), maxInt(p3y, p4y))

	radiusSq := radius * radius

	count := 0
	var chosenTx, chosenTy int

	for ty := minTy; ty <= maxTy; ty++ {
		for tx := minTx; tx <= maxTx; tx++ {
			if !g.IsWalkableAt(tx, ty) {
				continue
			}

			twx, twy := g.TileToWorld(tx, ty)
			dx := twx - wx
			dy := twy - wy
			if dx*dx+dy*dy > radiusSq {
				continue
			}

			count++
			if fastRand()%uint32(count) == 0 {
				chosenTx = tx
				chosenTy = ty
			}
		}
	}

	if count == 0 {
		return 0, 0, false
	}

	rwx, rwy := g.TileToWorld(chosenTx, chosenTy)
	return rwx, rwy, true
}

// RandomInRing returns a random walkable tile with Chebyshev distance
// between innerR and outerR from (cx, cy).
func RandomInRing(g GridLOS, cx, cy, innerR, outerR int) (int, int, bool) {
	if outerR < innerR {
		innerR, outerR = outerR, innerR
	}

	if outerR < 0 {
		return 0, 0, false
	}

	if innerR < 0 {
		innerR = 0
	}

	for i := 0; i < 64; i++ {
		dx := int(fastRand()%uint32(2*outerR+1)) - outerR
		dy := int(fastRand()%uint32(2*outerR+1)) - outerR

		x, y := cx+dx, cy+dy

		dist := maxInt(abs(dx), abs(dy))
		if dist < innerR || dist > outerR {
			continue
		}

		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}

	count := 0
	var chosenX, chosenY int

	for dy := -outerR; dy <= outerR; dy++ {
		for dx := -outerR; dx <= outerR; dx++ {
			dist := maxInt(abs(dx), abs(dy))
			if dist < innerR || dist > outerR {
				continue
			}

			x, y := cx+dx, cy+dy

			if !g.IsWalkableAt(x, y) {
				continue
			}

			count++

			if fastRand()%uint32(count) == 0 {
				chosenX = x
				chosenY = y
			}
		}
	}

	if count == 0 {
		return 0, 0, false
	}

	return chosenX, chosenY, true
}

// RandomInRingWorld returns the center of a random walkable tile whose
// distance from (wx, wy) lies in [innerR, outerR].
func RandomInRingWorld(g GridWorld, wx, wy float32, innerR, outerR float32) (float32, float32, bool) {
	if outerR < innerR {
		innerR, outerR = outerR, innerR
	}

	if outerR <= 0 {
		return 0, 0, false
	}

	if innerR < 0 {
		innerR = 0
	}

	const maxRetry = 128

	innerSq := innerR * innerR
	outerSq := outerR * outerR

	// Fast path: random sampling.
	for i := 0; i < maxRetry; i++ {
		angle := float64(fastRand()) / float64(^uint32(0)) * 2 * math.Pi
		u := float64(fastRand()) / float64(^uint32(0))
		r := math.Sqrt(float64(innerSq) + u*float64(outerSq-innerSq))
		x := wx + float32(r*math.Cos(angle))
		y := wy + float32(r*math.Sin(angle))
		tx, ty := g.WorldToTile(x, y)
		if g.IsWalkableAt(tx, ty) {
			rwx, rwy := g.TileToWorld(tx, ty)
			return rwx, rwy, true
		}
	}

	// Fallback: reservoir sampling over all walkable tiles in ring.
	p1x, p1y := g.WorldToTile(wx-outerR, wy-outerR)
	p2x, p2y := g.WorldToTile(wx-outerR, wy+outerR)
	p3x, p3y := g.WorldToTile(wx+outerR, wy-outerR)
	p4x, p4y := g.WorldToTile(wx+outerR, wy+outerR)

	minTx := minInt(minInt(p1x, p2x), minInt(p3x, p4x))
	maxTx := maxInt(maxInt(p1x, p2x), maxInt(p3x, p4x))
	minTy := minInt(minInt(p1y, p2y), minInt(p3y, p4y))
	maxTy := maxInt(maxInt(p1y, p2y), maxInt(p3y, p4y))

	count := 0
	var chosenTx, chosenTy int
	for ty := minTy; ty <= maxTy; ty++ {
		for tx := minTx; tx <= maxTx; tx++ {
			if !g.IsWalkableAt(tx, ty) {
				continue
			}

			twx, twy := g.TileToWorld(tx, ty)
			dx := twx - wx
			dy := twy - wy
			distSq := dx*dx + dy*dy
			if distSq < innerSq || distSq > outerSq {
				continue
			}

			count++
			if fastRand()%uint32(count) == 0 {
				chosenTx = tx
				chosenTy = ty
			}
		}
	}

	if count == 0 {
		return 0, 0, false
	}

	rwx, rwy := g.TileToWorld(chosenTx, chosenTy)
	return rwx, rwy, true
}
