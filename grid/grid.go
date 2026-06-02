package grid

// GridType identifies the tile grid layout.
type GridType int

const (
	// Orthogonal is a standard square-tile grid (4-directional or 8-directional).
	Orthogonal GridType = iota
	// Staggered is a 45-degree isometric/staggered grid (diamond-shaped tiles).
	Staggered
	// Hexagonal is a hexagonal grid (both pointy-top and flat-top layouts).
	Hexagonal
)
