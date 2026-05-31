package navmap

import (
	"testing"
)

func TestOrthogonalConverter(t *testing.T) {
	c := NewOrthogonalConverter(32, 32)
	// Tile center at (0,0)
	x, y := c.TileToWorld(0, 0)
	if x != 16 || y != 16 {
		t.Errorf("expected (16,16), got (%v,%v)", x, y)
	}
	// Tile at (1,2)
	x, y = c.TileToWorld(1, 2)
	if x != 48 || y != 80 {
		t.Errorf("expected (48,80), got (%v,%v)", x, y)
	}
	// World to tile
	tx, ty := c.WorldToTile(48, 80)
	if tx != 1 || ty != 2 {
		t.Errorf("expected (1,2), got (%d,%d)", tx, ty)
	}
}

func TestOrthogonalTileMap(t *testing.T) {
	grid := [][]int{
		{0, 0, 0, 0, 0},
		{0, 1, 1, 0, 0},
		{0, 0, 0, 0, 0},
	}
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid(grid).
		WithAStar(nil).
		Build()

	path := tm.FindPath(16, 16, 4*32+16, 16)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
	// Verify start/end world coords
	start, end := path[0], path[len(path)-1]
	if start[0] != 16 || start[1] != 16 {
		t.Errorf("expected start (16,16), got (%v,%v)", start[0], start[1])
	}
	if end[0] != 4*32+16 || end[1] != 16 {
		t.Errorf("expected end (%d,16), got (%v,%v)", 4*32+16, end[0], end[1])
	}
}

func TestIsWalkableAt(t *testing.T) {
	grid := [][]int{
		{0, 1},
		{1, 0},
	}
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid(grid).
		WithAStar(nil).
		Build()

	if !tm.IsWalkableAt(16, 16) {
		t.Error("expected (16,16) to be walkable")
	}
	if tm.IsWalkableAt(48, 16) {
		t.Error("expected (48,16) to be unwalkable")
	}
	// Out of bounds
	if tm.IsWalkableAt(-100, 16) {
		t.Error("expected (-100,16) to be out of bounds")
	}
	// Out of bounds
	if tm.IsWalkableAt(1000, 16) {
		t.Error("expected (1000,16) to be out of bounds")
	}
}

func TestHexConverter(t *testing.T) {
	c := NewPointyHexConverter(32)
	// Tile (0,0) center
	x, y := c.TileToWorld(0, 0)
	if x != 0 || y != 0 {
		t.Errorf("expected (0,0), got (%v,%v)", x, y)
	}
	// Roundtrip
	tx, ty := c.WorldToTile(0, 0)
	if tx != 0 || ty != 0 {
		t.Errorf("expected (0,0), got (%d,%d)", tx, ty)
	}
}

func TestHPAEnabledTileMap(t *testing.T) {
	grid := [][]int{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	}
	tm := NewTileMapBuilder().
		WithOrthogonalMap(1, 1).
		WithGrid(grid).
		WithAStar(nil).
		WithHPA(8).
		Build()

	path := tm.FindPath(0.5, 0.5, 19.5, 3.5)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
}

func TestHexTileMap(t *testing.T) {
	grid := [][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	}
	tm := NewTileMapBuilder().
		WithHexMap(32, true).
		WithGrid(grid).
		WithAStar(nil).
		Build()

	// Find path from hex (0,0) center to hex (4,0) center
	cx, cy := tm.Converter.TileToWorld(0, 0)
	ex, ey := tm.Converter.TileToWorld(4, 0)
	path := tm.FindPath(cx, cy, ex, ey)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
	if path[0][0] != cx || path[0][1] != cy {
		t.Errorf("expected start (%v,%v), got (%v,%v)", cx, cy, path[0][0], path[0][1])
	}
	if path[len(path)-1][0] != ex || path[len(path)-1][1] != ey {
		t.Errorf("expected end (%v,%v), got (%v,%v)", ex, ey, path[len(path)-1][0], path[len(path)-1][1])
	}
}

func TestSetWalkableAt(t *testing.T) {
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{{0}}).
		WithAStar(nil).
		Build()

	if !tm.IsWalkableAt(16, 16) {
		t.Error("expected walkable initially")
	}
	tm.SetWalkableAt(16, 16, false)
	if tm.IsWalkableAt(16, 16) {
		t.Error("expected unwalkable after set")
	}
}

func TestUnreachablePath(t *testing.T) {
	tm := NewTileMapBuilder().
		WithOrthogonalMap(32, 32).
		WithGrid([][]int{
			{0, 1, 0},
			{0, 1, 0},
			{0, 1, 0},
		}).
		WithAStar(nil).
		Build()

	// Grid: 0,1,0 / 0,1,0 / 0,1,0 — wall at column 1 blocks all rows
	// Start at tile (0,0), end at tile (2,2) unreachable
	path := tm.FindPath(16, 16, 2*32+16, 2*32+16)
	if path != nil {
		t.Error("expected nil for unreachable path")
	}
}
