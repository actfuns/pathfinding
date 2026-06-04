package grid

import (
	"testing"
)

// StaggeredGrid (staggerY, odd):
//   tileW=64, tileH=32
// Expected values from Tiled's test_staggeredrenderer.cpp:
//   tileToScreenCoords: (0,0)→(0,0), (1,0)→(64,0), (0,1)→(32,16)
//   screenToTileCoords: (10,16)→(0,0), (5,5)→(-1,-1), (1,20)→(-1,1),
//                       (64,32)→(0,1), (32,-16)→(0,-2)

func TestStaggeredTileToWorld(t *testing.T) {
	g := NewStaggeredGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	}, WithStaggerTileSize(64, 32))

	// tileToScreenCoords from Tiled test
	tests := []struct {
		tx, ty int
		px, py float32
	}{
		{0, 0, 0, 0},
		{1, 0, 64, 0},
		{0, 1, 32, 16},
	}
	for _, tt := range tests {
		px, py := g.TileToWorld(tt.tx, tt.ty)
		if px != tt.px || py != tt.py {
			t.Errorf("TileToWorld(%d,%d) = (%v,%v), want (%v,%v) (Tiled ref)", tt.tx, tt.ty, px, py, tt.px, tt.py)
		}
	}
}

func TestStaggeredWorldToTile(t *testing.T) {
	g := NewStaggeredGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	}, WithStaggerTileSize(64, 32))

	// screenToTileCoords from Tiled test
	// Tiled test: QCOMPARE(floor(x), floor(expected_x)), floor(y), floor(expected_y)
	// So (10,16) → tile(0,0) means screenToTileCoords returns (0.xx, 0.xx) not exact integer
	tests := []struct {
		px, py float32
		tx, ty int
	}{
		{10, 16, 0, 0},
	}
	for _, tt := range tests {
		tx, ty := g.WorldToTile(tt.px, tt.py)
		if tx != tt.tx || ty != tt.ty {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), want (%d,%d) (Tiled ref)", tt.px, tt.py, tx, ty, tt.tx, tt.ty)
		}
	}
}

func TestStaggeredWorldToTileTiledSuite(t *testing.T) {
	g := NewStaggeredGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	}, WithStaggerTileSize(64, 32))

	// Tiled screenToTileCoords test data:
	// Tiled test compares floor(x), floor(y) — so the expected tile coords are the floored values.
	// (10, 16) → (0, 0)    — clearly in tile (0,0)
	// (5, 5)   → (-1, -1)  — top-left corner, maps outside
	// (1, 20)  → (-1, 1)   — left of tile (0,1), maps to (-1,1)
	// (64, 32) → (0, 1)    — this is tile (1,0)'s left edge? No: (64,32) is pixel right at (1,0) top.
	//                         Wait: (0,1) means row 1 (odd row shifted). In staggerY odd,
	//                         tile (0,1) screen coords = (32, 16).
	//                         (64, 32) is at the corner — in Tiled it maps to tile (0,1) floor.
	// (32, -16) → (0, -2)
	tests := []struct {
		px, py float32
		tx, ty int
	}{
		{10, 16, 0, 0},
		{5, 5, -1, -1},
		{1, 20, -1, 1},
		{64, 32, 0, 1},
		{32, -16, 0, -2},
	}
	for _, tt := range tests {
		tx, ty := g.WorldToTile(tt.px, tt.py)
		if tx != tt.tx || ty != tt.ty {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), want (%d,%d) (Tiled ref)", tt.px, tt.py, tx, ty, tt.tx, tt.ty)
		}
	}
}

// Staggered relative coordinates (topLeft, topRight, etc.) from Tiled test:
//
//	topLeft(0,0) = (-1,-1), topRight(0,0) = (0,-1), bottomLeft(0,0) = (-1,1), bottomRight(0,0) = (0,1)
//	topLeft(1,1) = (1,0),   topRight(1,1) = (2,0),  bottomLeft(1,1) = (1,2),  bottomRight(1,1) = (2,2)
func TestStaggeredRelativeCoords(t *testing.T) {
	// staggerY, odd
	tests := []struct {
		x, y   int
		fn     func(int, int, bool, bool) (int, int)
		ex, ey int
	}{
		// topLeft
		{0, 0, staggeredTopLeft, -1, -1},
		{1, 1, staggeredTopLeft, 1, 0},
		// topRight
		{0, 0, staggeredTopRight, 0, -1},
		{1, 1, staggeredTopRight, 2, 0},
		// bottomLeft
		{0, 0, staggeredBottomLeft, -1, 1},
		{1, 1, staggeredBottomLeft, 1, 2},
		// bottomRight
		{0, 0, staggeredBottomRight, 0, 1},
		{1, 1, staggeredBottomRight, 2, 2},
	}
	for _, tt := range tests {
		rx, ry := tt.fn(tt.x, tt.y, false, false)
		if rx != tt.ex || ry != tt.ey {
			t.Errorf("(%d,%d) → (%d,%d), want (%d,%d)", tt.x, tt.y, rx, ry, tt.ex, tt.ey)
		}
	}
}

// --- SVG rendering ---

func TestStaggeredSVG(t *testing.T) {
	scenarios := []svgScenario{
		{
			name: "simple 3x3", startX: 0, startY: 0, endX: 2, endY: 2,
			matrix: [][]int{
				{0, 0, 0},
				{0, 1, 0},
				{0, 0, 0},
			},
		},
		{
			name: "4x3 with obstacles", startX: 0, startY: 0, endX: 3, endY: 2,
			matrix: [][]int{
				{0, 0, 0, 0},
				{0, 1, 1, 0},
				{0, 0, 0, 0},
			},
		},
		{
			name: "all open 8x6", startX: 0, startY: 0, endX: 7, endY: 5,
			matrix: func() [][]int {
				m := make([][]int, 6)
				for i := range m {
					m[i] = make([]int, 8)
				}
				return m
			}(),
		},
		{
			name: "weighted terrain 5x5", startX: 0, startY: 0, endX: 4, endY: 4,
			matrix: NewMatrix(5, 5),
			weights: [][]float64{
				{1.0, 1.0, 1.0, 1.0, 1.0},
				{1.0, 0.3, 0.3, 0.3, 1.0},
				{1.0, 0.3, 5.0, 0.3, 1.0},
				{1.0, 0.3, 0.3, 0.3, 1.0},
				{1.0, 1.0, 1.0, 1.0, 1.0},
			},
		},
	}

	runSVGTest(t, "staggered", scenarios, func(matrix [][]int) gridForSVG {
		return NewStaggeredGrid(matrix)
	})
}

func TestStaggeredFindNearestWalkable(t *testing.T) {
	// 5x5 grid with obstacle at (2,2), default tile size 64x64
	g := NewStaggeredGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	})

	t.Run("self walkable", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(0, 0, 5, 0)
		if !ok {
			t.Fatal("expected ok")
		}
	})

	t.Run("obstacle center finds neighbor", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(128, 64, 3, 0)
		if !ok {
			t.Fatal("expected ok, obstacle center should have walkable neighbors")
		}
	})

	t.Run("all obstacles returns false", func(t *testing.T) {
		g2 := NewStaggeredGrid([][]int{{1, 1}, {1, 1}})
		_, _, ok := g2.FindNearestWalkableWorld(32, 16, 5, 0)
		if ok {
			t.Error("expected false when all obstacles")
		}
	})

	t.Run("maxRadius=0 only checks self", func(t *testing.T) {
		g2 := NewStaggeredGrid([][]int{{1, 1}, {1, 1}})
		_, _, ok := g2.FindNearestWalkableWorld(32, 16, 0, 0)
		if ok {
			t.Error("expected false when self obstacle and maxRadius=0")
		}
	})

	t.Run("maxRadius negative", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(32, 16, -1, 0)
		if ok {
			t.Error("expected false when maxRadius < 0")
		}
	})

	t.Run("outside grid returns false", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(999, 999, 5, 0)
		if ok {
			t.Error("expected false outside grid")
		}
	})
}
func TestStaggeredObstacleCount(t *testing.T) {
	g := NewStaggeredGrid([][]int{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	})
	if n := g.ObstacleCount(); n != 1 {
		t.Errorf("expected 1 obstacle, got %d", n)
	}
	g.SetWalkableAt(0, 0, false)
	if n := g.ObstacleCount(); n != 2 {
		t.Errorf("expected 2 obstacles, got %d", n)
	}
	g.SetWalkableAt(1, 1, true)
	g.SetWalkableAt(0, 0, true)
	if n := g.ObstacleCount(); n != 0 {
		t.Errorf("expected 0 obstacles, got %d", n)
	}
	g2 := g.Clone()
	if n := g2.(*StaggeredGrid).ObstacleCount(); n != 0 {
		t.Errorf("clone: expected 0 obstacles, got %d", n)
	}
}
