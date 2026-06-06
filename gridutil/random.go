package gridutil

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
	for i := 0; i < 32; i++ {
		dx := int(uint32(2*radius+1)*fastRand()) - radius
		dy := int(uint32(2*radius+1)*fastRand()) - radius
		x, y := cx+dx, cy+dy
		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}
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
func RandomWalkableInRadiusWorld(g GridWorld, wx, wy float32, radius int) (float32, float32, bool) {
	tx, ty := g.WorldToTile(wx, wy)
	rtx, rty, ok := RandomWalkableInRadius(g, tx, ty, radius)
	if !ok {
		return 0, 0, false
	}
	wx2, wy2 := g.TileToWorld(rtx, rty)
	return wx2, wy2, true
}

// RandomInRing returns a random walkable tile with Chebyshev distance
// between innerR and outerR from (cx, cy).
func RandomInRing(g GridLOS, cx, cy, innerR, outerR int) (int, int, bool) {
	for i := 0; i < 64; i++ {
		dx := int(uint32(2*outerR+1)*fastRand()) - outerR
		dy := int(uint32(2*outerR+1)*fastRand()) - outerR
		x, y := cx+dx, cy+dy
		dist := maxInt(abs(dx), abs(dy))
		if dist < innerR || dist > outerR {
			continue
		}
		if g.IsWalkableAt(x, y) {
			return x, y, true
		}
	}
	for dy := -outerR; dy <= outerR; dy++ {
		for dx := -outerR; dx <= outerR; dx++ {
			dist := maxInt(abs(dx), abs(dy))
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
func RandomInRingWorld(g GridWorld, wx, wy float32, innerR, outerR int) (float32, float32, bool) {
	tx, ty := g.WorldToTile(wx, wy)
	rtx, rty, ok := RandomInRing(g, tx, ty, innerR, outerR)
	if !ok {
		return 0, 0, false
	}
	wx2, wy2 := g.TileToWorld(rtx, rty)
	return wx2, wy2, true
}
