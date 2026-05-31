package navmap

import (
	"testing"

	"github.com/parasol/pathfinding/core"
	"github.com/parasol/pathfinding/finder"
)

// --- Flat-top hex converter ---

func TestFlatHexConverter(t *testing.T) {
	c := NewFlatHexConverter(32)

	// Tile (0,0) center: flat-top → (0, 0)
	x, y := c.TileToWorld(0, 0)
	if x != 0 || y != 0 {
		t.Errorf("expected (0,0), got (%v,%v)", x, y)
	}

	// Roundtrip: tile → world → tile
	tx, ty := c.WorldToTile(x, y)
	if tx != 0 || ty != 0 {
		t.Errorf("expected (0,0), got (%d,%d)", tx, ty)
	}

	// Tile (1,0): flat-top → x = 1.5 * size, y = 0.866 * size
	x, y = c.TileToWorld(1, 0)
	ex := 1.5 * 32
	ey := 0.8660254037844386 * 32
	if x != ex || y != ey {
		t.Errorf("expected (%v,%v), got (%v,%v)", ex, ey, x, y)
	}

	// Roundtrip for non-zero tile
	tx, ty = c.WorldToTile(ex, ey)
	if tx != 1 || ty != 0 {
		t.Errorf("expected (1,0), got (%d,%d)", tx, ty)
	}
}

func TestFlatHexConverterRoundtrip(t *testing.T) {
	c := NewFlatHexConverter(16)
	tiles := [][2]int{
		{0, 0},
		{1, 0},
		{0, 1},
		{3, 2},
		{-1, 0},
		{0, -1},
		{-2, -3},
	}
	for _, tt := range tiles {
		wx, wy := c.TileToWorld(tt[0], tt[1])
		tx, ty := c.WorldToTile(wx, wy)
		if tx != tt[0] || ty != tt[1] {
			t.Errorf("roundtrip (%d,%d): world (%v,%v) → tile (%d,%d)", tt[0], tt[1], wx, wy, tx, ty)
		}
	}
}

func TestPointyHexConverterRoundtrip(t *testing.T) {
	c := NewPointyHexConverter(24)
	tiles := [][2]int{
		{0, 0},
		{1, 0},
		{0, 1},
		{3, 2},
		{-1, 0},
		{0, -1},
		{-2, -3},
	}
	for _, tt := range tiles {
		wx, wy := c.TileToWorld(tt[0], tt[1])
		tx, ty := c.WorldToTile(wx, wy)
		if tx != tt[0] || ty != tt[1] {
			t.Errorf("roundtrip (%d,%d): world (%v,%v) → tile (%d,%d)", tt[0], tt[1], wx, wy, tx, ty)
		}
	}
}

// --- OrthogonalConverter edge cases ---

func TestOrthogonalConverterNegative(t *testing.T) {
	c := NewOrthogonalConverter(32, 32)

	// Negative world positions should map correctly (floor division)
	tx, ty := c.WorldToTile(-10, -10)
	if tx != -1 || ty != -1 {
		t.Errorf("expected (-1,-1), got (%d,%d)", tx, ty)
	}

	tx, ty = c.WorldToTile(-32, -32)
	if tx != -1 || ty != -1 {
		t.Errorf("expected (-1,-1), got (%d,%d)", tx, ty)
	}

	tx, ty = c.WorldToTile(-33, -33)
	if tx != -2 || ty != -2 {
		t.Errorf("expected (-2,-2), got (%d,%d)", tx, ty)
	}
}

func TestOrthogonalConverterTileWidthHeight(t *testing.T) {
	c := NewOrthogonalConverter(16, 32)
	if c.TileWidth() != 16 {
		t.Errorf("expected width 16, got %v", c.TileWidth())
	}
	if c.TileHeight() != 32 {
		t.Errorf("expected height 32, got %v", c.TileHeight())
	}
}

// --- TileMapBuilder ---

func TestTileMapBuilderWithCoreGrid(t *testing.T) {
	g := core.NewGrid([][]int{
		{0, 1},
		{1, 0},
	})
	tm := NewTileMapBuilder().
		WithCoreGrid(g).
		WithOrthogonalMap(32, 32).
		WithAStar(nil).
		Build()

	if !tm.IsWalkableAt(16, 16) {
		t.Error("expected (16,16) walkable")
	}
	if tm.IsWalkableAt(48, 16) {
		t.Error("expected (48,16) unwalkable")
	}
}

func TestTileMapBuilderWithConverter(t *testing.T) {
	custom := NewOrthogonalConverter(64, 64)
	tm := NewTileMapBuilder().
		WithGrid([][]int{{0}}).
		WithConverter(custom).
		WithAStar(nil).
		Build()

	if tm.Converter.TileWidth() != 64 {
		t.Errorf("expected TileWidth 64, got %v", tm.Converter.TileWidth())
	}

	// Tile center should be (32, 32) with 64x64 tiles
	if !tm.IsWalkableAt(32, 32) {
		t.Error("expected center walkable")
	}
}

func TestTileMapBuilderWithJPF(t *testing.T) {
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{
			{0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0},
		}).
		WithJPF(&finder.FinderOptions{
			DiagonalMovement: core.DiagonalAlways,
		}).
		Build()

	path := tm.FindPath(16, 16, 4*32+16, 16)
	if path == nil {
		t.Fatal("expected path with JPF finder")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
}

func TestTileMapBuilderWithBFS(t *testing.T) {
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{
			{0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0},
		}).
		WithBFS(nil).
		Build()

	path := tm.FindPath(16, 16, 4*32+16, 16)
	if path == nil {
		t.Fatal("expected path with BFS finder")
	}
}

func TestTileMapBuilderWithDijkstra(t *testing.T) {
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{
			{0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0},
		}).
		WithDijkstra(nil).
		Build()

	path := tm.FindPath(16, 16, 4*32+16, 16)
	if path == nil {
		t.Fatal("expected path with Dijkstra finder")
	}
}

func TestTileMapBuilderWithFinder(t *testing.T) {
	custom := finder.NewAStarFinder(&finder.FinderOptions{
		Weight:        2,
		AllowDiagonal: true,
	})
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{
			{0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0},
		}).
		WithFinder(custom).
		Build()

	path := tm.FindPath(16, 16, 4*32+16, 16)
	if path == nil {
		t.Fatal("expected path with custom finder")
	}
}

func TestTileMapBuilderDefaultOrthogonal(t *testing.T) {
	// Build without WithOrthogonalMap/WithHexMap — should default to 1x1 orthogonal
	tm := NewTileMapBuilder().
		WithGrid([][]int{
			{0, 0},
			{0, 0},
		}).
		WithAStar(nil).
		Build()

	if tm.Converter.TileWidth() != 1 || tm.Converter.TileHeight() != 1 {
		t.Errorf("expected default 1x1 tiles, got %vx%v",
			tm.Converter.TileWidth(), tm.Converter.TileHeight())
	}
}

func TestTileMapNoPathDueToWalls(t *testing.T) {
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{
			{0, 1, 0},
			{0, 1, 0},
			{0, 1, 0},
		}).
		WithAStar(nil).
		Build()

	// Start tile (0,0) center = (16,16), end tile (2,2) center = (2*32+16, 2*32+16)
	path := tm.FindPath(16, 16, 2*32+16, 2*32+16)
	if path != nil {
		t.Error("expected nil for unreachable path")
	}
}

func TestTileMapOutOfBounds(t *testing.T) {
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{
			{0, 0},
			{0, 0},
		}).
		WithAStar(nil).
		Build()

	// Start out of bounds
	path := tm.FindPath(-100, -100, 16, 16)
	if path != nil {
		t.Error("expected nil when start is out of bounds")
	}

	// End out of bounds
	path = tm.FindPath(16, 16, 1000, 1000)
	if path != nil {
		t.Error("expected nil when end is out of bounds")
	}

	// IsWalkableAt out of bounds
	if tm.IsWalkableAt(-100, -100) {
		t.Error("expected false for out of bounds")
	}
}

// --- Hex converter dimensions ---

func TestHexConverterDimensions(t *testing.T) {
	c := NewPointyHexConverter(32)
	if c.TileWidth() != 32*1.5 {
		t.Errorf("pointy width: expected %v, got %v", 32*1.5, c.TileWidth())
	}

	c2 := NewFlatHexConverter(32)
	if c2.TileWidth() != 32*1.7320508075688772 {
		t.Errorf("flat width: expected %v, got %v", 32*1.7320508075688772, c2.TileWidth())
	}
}
