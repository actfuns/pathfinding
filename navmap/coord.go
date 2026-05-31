package navmap

// CoordConverter converts between world coordinates and tile coordinates.
//
// Different map types (orthogonal, hex, staggered) implement this interface
// to provide their specific coordinate transformation math.
type CoordConverter interface {
	// WorldToTile converts world position to tile coordinates.
	// Returns -1, -1 if the position is outside the map.
	WorldToTile(wx, wy float64) (tx, ty int)

	// TileToWorld converts tile coordinates to the world position
	// of the tile's center.
	TileToWorld(tx, ty int) (wx, wy float64)

	// TileWidth returns the world-space width of a tile.
	TileWidth() float64

	// TileHeight returns the world-space height of a tile.
	TileHeight() float64
}

// OrthogonalConverter is the standard tile-to-world converter for orthogonal maps.
type OrthogonalConverter struct {
	TileW, TileH float64
}

func NewOrthogonalConverter(tileWidth, tileHeight float64) *OrthogonalConverter {
	return &OrthogonalConverter{TileW: tileWidth, TileH: tileHeight}
}

func (c *OrthogonalConverter) WorldToTile(wx, wy float64) (int, int) {
	tx := floorDiv(wx, c.TileW)
	ty := floorDiv(wy, c.TileH)
	return tx, ty
}

func (c *OrthogonalConverter) TileToWorld(tx, ty int) (float64, float64) {
	return float64(tx)*c.TileW + c.TileW/2, float64(ty)*c.TileH + c.TileH/2
}

func (c *OrthogonalConverter) TileWidth() float64  { return c.TileW }
func (c *OrthogonalConverter) TileHeight() float64 { return c.TileH }

// HexConverter converts between world and hex tile coordinates.
// Supports both pointy-top and flat-top hexagons.
// References pmcxs-hexgrid layout math.
type HexConverter struct {
	Size     float64 // hex size (center-to-vertex)
	Pointy   bool    // true = pointy-top, false = flat-top
	TileW, TileH float64
	f0, f1, f2, f3 float64
	b0, b1, b2, b3 float64
	startAngle     float64
}

// NewPointyHexConverter creates a pointy-top hex converter.
func NewPointyHexConverter(size float64) *HexConverter {
	// Pointy orientation constants (from pmcxs-hexgrid)
	return &HexConverter{
		Size: size,
		Pointy: true,
		TileW: size * 1.5,
		TileH: size * 1.7320508075688772,
		f0: 1.7320508075688772, f1: 0.8660254037844386,
		f2: 0.0, f3: 1.5,
		b0: 0.5773502691896257, b1: -0.3333333333333333,
		b2: 0.0, b3: 0.6666666666666666,
		startAngle: 0.5,
	}
}

// NewFlatHexConverter creates a flat-top hex converter.
func NewFlatHexConverter(size float64) *HexConverter {
	return &HexConverter{
		Size: size,
		Pointy: false,
		TileW: size * 1.7320508075688772,
		TileH: size * 1.5,
		f0: 1.5, f1: 0.0,
		f2: 0.8660254037844386, f3: 1.7320508075688772,
		b0: 0.6666666666666666, b1: 0.0,
		b2: -0.3333333333333333, b3: 0.5773502691896257,
		startAngle: 0.0,
	}
}

func (c *HexConverter) WorldToTile(wx, wy float64) (int, int) {
	// Inverse hex layout math
	px := wx/c.Size - 0 // origin offset
	py := wy/c.Size - 0
	q := c.b0*px + c.b1*py
	r := c.b2*px + c.b3*py

	// Round fractional hex to integer tile coords
	// For axial coords, s = -q-r
	s := -q - r

	rq := round(q)
	rr := round(r)
	rs := round(s)

	dq := absf(float64(rq) - q)
	dr := absf(float64(rr) - r)
	ds := absf(float64(rs) - s)

	if dq > dr && dq > ds {
		rq = -rr - rs
	} else if dr > ds {
		rr = -rq - rs
	}

	return int(rq), int(rr)
}

func (c *HexConverter) TileToWorld(tx, ty int) (float64, float64) {
	x := (c.f0*float64(tx) + c.f1*float64(ty)) * c.Size
	y := (c.f2*float64(tx) + c.f3*float64(ty)) * c.Size
	return x, y
}

func (c *HexConverter) TileWidth() float64  { return c.TileW }
func (c *HexConverter) TileHeight() float64 { return c.TileH }

func round(v float64) int {
	if v < 0 {
		return int(v - 0.5)
	}
	return int(v + 0.5)
}

func absf(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}

// floorDiv returns the floor of x/y (Go's int() truncates toward zero, which
// gives wrong results for negative values).
func floorDiv(x, y float64) int {
	q := int(x / y)
	r := x - float64(q)*y
	if r < 0 {
		q--
	}
	return q
}
