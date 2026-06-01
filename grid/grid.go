package grid

// GridType identifies the tile grid layout.
type GridType int

const (
	Orthogonal GridType = iota
	Staggered           // 45-degree isometric/staggered (diamond-shaped tiles)
	Hexagonal           // hexagonal (both pointy and flat-top)
)
