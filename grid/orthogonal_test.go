package grid

import (
	"strings"
	"testing"
)

func TestOrthogonalWorldToTile(t *testing.T) {
	g := NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
	}, WithOrthogonalTileSize(32, 32))

	tests := []struct {
		wx, wy float32
		tx, ty int
	}{
		{0, 0, 0, 0},
		{31, 31, 0, 0},
		{32, 0, 1, 0},
		{64, 32, 2, 1},
		{95, 63, 2, 1},
	}
	for _, tt := range tests {
		tx, ty := g.WorldToTile(tt.wx, tt.wy)
		if tx != tt.tx || ty != tt.ty {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), want (%d,%d)", tt.wx, tt.wy, tx, ty, tt.tx, tt.ty)
		}
	}
}

func TestOrthogonalTileToWorld(t *testing.T) {
	g := NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 0, 0},
	}, WithOrthogonalTileSize(32, 32))

	tests := []struct {
		tx, ty int
		wx, wy float32
	}{
		{0, 0, 16, 16},
		{1, 0, 48, 16},
		{2, 1, 80, 48},
	}
	for _, tt := range tests {
		wx, wy := g.TileToWorld(tt.tx, tt.ty)
		if wx != tt.wx || wy != tt.wy {
			t.Errorf("TileToWorld(%d,%d) = (%v,%v), want (%v,%v)", tt.tx, tt.ty, wx, wy, tt.wx, tt.wy)
		}
	}
}

func TestOrthogonalRoundTrip(t *testing.T) {
	g := NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	}, WithOrthogonalTileSize(16, 16))

	worldPositions := [][2]float32{
		{0, 0},
		{7, 7},
		{16, 16},
		{40, 24},
		{79, 47},
	}
	for _, wp := range worldPositions {
		tx, ty := g.WorldToTile(wp[0], wp[1])
		if !g.IsInside(tx, ty) {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d) outside grid", wp[0], wp[1], tx, ty)
			continue
		}
		tileLeft := float32(tx * g.tileW)
		tileTop := float32(ty * g.tileH)
		if wp[0] < tileLeft || wp[0] >= tileLeft+float32(g.tileW) {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), worldX %v not in [%v,%v)",
				wp[0], wp[1], tx, ty, wp[0], tileLeft, tileLeft+float32(g.tileW))
		}
		if wp[1] < tileTop || wp[1] >= tileTop+float32(g.tileH) {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), worldY %v not in [%v,%v)",
				wp[0], wp[1], tx, ty, wp[1], tileTop, tileTop+float32(g.tileH))
		}
	}
}

// --- SVG rendering ---

func TestOrthogonalSVG(t *testing.T) {
	scenarios := []svgScenario{
		{
			name: "simple 2x2", startX: 0, startY: 0, endX: 1, endY: 1,
			matrix: [][]int{{0, 0}, {1, 0}},
		},
		{
			name: "5x6 maze", startX: 1, startY: 1, endX: 4, endY: 4,
			matrix: [][]int{
				{0, 0, 0, 0, 0},
				{1, 0, 1, 1, 0},
				{1, 0, 1, 0, 0},
				{0, 1, 0, 0, 0},
				{1, 0, 1, 1, 0},
				{0, 0, 1, 0, 0},
			},
		},
		{
			name: "all open 10x10", startX: 0, startY: 0, endX: 9, endY: 9,
			matrix: func() [][]int {
				m := make([][]int, 10)
				for i := range m {
					m[i] = make([]int, 10)
				}
				return m
			}(),
		},
	}
	runSVGTest(t, "orthogonal", scenarios, func(matrix [][]int) gridForSVG {
		return NewOrthogonalGrid(matrix)
	})
}

func TestOrthogonalSVG_NoPath(t *testing.T) {
	g := NewOrthogonalGrid([][]int{
		{0, 1, 0},
		{0, 1, 0},
		{0, 1, 0},
	})
	svg := g.RenderSVG(nil, 0, 0, 2, 0)
	if !strings.Contains(svg, "<svg") {
		t.Error("SVG missing <svg tag")
	}
	if strings.Contains(svg, "#0066cc") {
		t.Error("SVG should not contain path when nil path given")
	}
}

func TestOrthogonalFindNearestWalkable(t *testing.T) {
	// 5x5 grid with obstacles in cross pattern, tileW=tileH=1
	g := NewOrthogonalGrid([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 1, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0},
	})

	t.Run("self walkable", func(t *testing.T) {
		x, y, ok := g.FindNearestWalkable(0, 0, 5, 0)
		if !ok {
			t.Fatal("expected ok")
		}
		if x != 0 || y != 0 {
			t.Errorf("expected (0,0), got (%v,%v)", x, y)
		}
	})

	t.Run("neighbor in ring 1 around obstacle", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkable(2, 2, 3, 0)
		if !ok {
			t.Fatal("expected ok, center obstacle has walkable neighbors")
		}
	})

	t.Run("all obstacles returns false", func(t *testing.T) {
		g2 := NewOrthogonalGrid([][]int{{1, 1}, {1, 1}})
		_, _, ok := g2.FindNearestWalkable(0, 0, 5, 0)
		if ok {
			t.Error("expected false when all obstacles")
		}
	})

	t.Run("maxRadius=0 only checks self", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkable(2, 2, 0, 0)
		if ok {
			t.Error("expected false when self obstacle and maxRadius=0")
		}
	})

	t.Run("maxRadius negative", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkable(2, 2, -1, 0)
		if ok {
			t.Error("expected false when maxRadius < 0")
		}
	})

	t.Run("walkable self with limited radius", func(t *testing.T) {
		x, y, ok := g.FindNearestWalkable(4, 4, 1, 0)
		if !ok {
			t.Fatal("expected ok")
		}
		if x != 4 || y != 4 {
			t.Errorf("expected (4,4), got (%v,%v)", x, y)
		}
	})

	t.Run("outside grid returns false", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkable(100, 100, 5, 0)
		if ok {
			t.Error("expected false outside grid")
		}
	})

	t.Run("finds single walkable cell across rings", func(t *testing.T) {
		g3 := NewOrthogonalGrid([][]int{
			{0, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
		})
		x, y, ok := g3.FindNearestWalkable(4, 4, 8, 0)
		if !ok {
			t.Fatal("expected ok")
		}
		if x != 1 || y != 1 {
			t.Errorf("expected (1,1) [edge of tile (0,0)], got (%v,%v)", x, y)
		}
	})
}

func TestOrthogonalFindNearestWalkableWorld(t *testing.T) {
	// Grid with custom tile size to test world coordinate conversion
	g := NewOrthogonalGrid([][]int{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	}, WithOrthogonalTileSize(32, 32))

	x, y, ok := g.FindNearestWalkable(48, 48, 3, 0)
	if !ok {
		t.Fatal("expected ok")
	}
	// Query point (48,48) is exactly at the center of obstacle tile (1,1).
	// All 4 cardinal neighbors are equally close. Accept any of their edge points:
	// tile (1,0): (48,32), tile (0,1): (32,48), tile (2,1): (64,48), tile (1,2): (48,64)
	if x == 48 && y == 32 {
		// top edge
	} else if x == 48 && y == 64 {
		// bottom edge
	} else if x == 32 && y == 48 {
		// left edge
	} else if x == 64 && y == 48 {
		// right edge
	} else {
		t.Errorf("unexpected result (%v,%v), expected one of cardinal edge points: (48,32), (32,48), (64,48), (48,64)", x, y)
	}
}

func TestOrthogonalObstacleCount(t *testing.T) {
	g := NewOrthogonalGrid([][]int{
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
	g.SetWalkableAt(0, 0, true)
	if n := g.ObstacleCount(); n != 0 {
		t.Errorf("expected 0 obstacles after no-op, got %d", n)
	}
	g2 := g.Clone()
	if n := g2.(*OrthogonalGrid).ObstacleCount(); n != 0 {
		t.Errorf("clone: expected 0 obstacles, got %d", n)
	}
}
