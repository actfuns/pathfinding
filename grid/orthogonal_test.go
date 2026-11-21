package grid

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func TestOrthogonalWorldToTile(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0},
		{0, 0, 0},
	}, WithTileSize(32, 32))

	tests := []struct {
		wx, wy float32
		tx, ty int32
	}{
		{0, 0, 0, 0},
		{31, 31, 0, 0},
		{32, 0, 1, 0},
		{64, 32, 2, 1},
		{95, 63, 2, 1},
		{-1, -1, -1, -1},
		{-32, -32, -1, -1},
		{-33, 0, -2, 0},
		{0, -64, 0, -2},
	}
	for _, tt := range tests {
		tx, ty := g.WorldToTile(tt.wx, tt.wy)
		if tx != tt.tx || ty != tt.ty {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), want (%d,%d)", tt.wx, tt.wy, tx, ty, tt.tx, tt.ty)
		}
	}
}

func TestOrthogonalTileToWorld(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0},
		{0, 0, 0},
	}, WithTileSize(32, 32))

	tests := []struct {
		tx, ty int32
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
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	}, WithTileSize(16, 16))

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
		tileLeft := float32(tx * g.TileWidth())
		tileTop := float32(ty * g.TileHeight())
		if wp[0] < tileLeft || wp[0] >= tileLeft+float32(g.TileWidth()) {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), worldX %v not in [%v,%v)",
				wp[0], wp[1], tx, ty, wp[0], tileLeft, tileLeft+float32(g.TileWidth()))
		}
		if wp[1] < tileTop || wp[1] >= tileTop+float32(g.TileHeight()) {
			t.Errorf("WorldToTile(%v,%v) = (%d,%d), worldY %v not in [%v,%v)",
				wp[0], wp[1], tx, ty, wp[1], tileTop, tileTop+float32(g.TileHeight()))
		}
	}
}

// --- SVG rendering ---

func TestOrthogonalSVG(t *testing.T) {
	scenarios := []svgScenario{
		{
			name: "simple 2x2", startX: 0, startY: 0, endX: 1, endY: 1,
			matrix: [][]int32{{0, 0}, {1, 0}},
		},
		{
			name: "5x6 maze", startX: 1, startY: 1, endX: 4, endY: 4,
			matrix: [][]int32{
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
			matrix: func() [][]int32 {
				m := make([][]int32, 10)
				for i := range m {
					m[i] = make([]int32, 10)
				}
				return m
			}(),
		},
		{
			name: "weighted terrain 10x8", startX: 1, startY: 0, endX: 7, endY: 7,
			matrix: NewMatrix(10, 8),
			weights: [][]float32{
				{0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3},
				{0.3, 1.0, 1.0, 1.0, 5.0, 5.0, 1.0, 1.0, 5.0, 0.3},
				{0.3, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 1.0, 5.0, 0.3},
				{0.3, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.3},
				{0.3, 1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.3},
				{0.3, 1.0, 1.0, 5.0, 5.0, 5.0, 5.0, 1.0, 5.0, 0.3},
				{0.3, 1.0, 1.0, 1.0, 5.0, 5.0, 1.0, 1.0, 5.0, 0.3},
				{0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3},
			},
		},
	}
	runSVGTest(t, "orthogonal", scenarios, func(matrix [][]int32) gridForSVG {
		return NewOrthogonalGrid(matrix)
	})
}

func TestOrthogonalFindNearestWalkable(t *testing.T) {
	// 5x5 grid with obstacles in cross pattern, tileW=tileH=1
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0, 0, 0},
		{0, 0, 1, 0, 0},
		{0, 1, 1, 1, 0},
		{0, 0, 1, 0, 0},
		{0, 0, 0, 0, 0},
	})

	t.Run("self walkable", func(t *testing.T) {
		x, y, ok := g.FindNearestWalkableWorld(0, 0, 5, 0)
		if !ok {
			t.Fatal("expected ok")
		}
		if x != 0 || y != 0 {
			t.Errorf("expected (0,0), got (%v,%v)", x, y)
		}
	})

	t.Run("neighbor in ring 1 around obstacle", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(2, 2, 3, 0)
		if !ok {
			t.Fatal("expected ok, center obstacle has walkable neighbors")
		}
	})

	t.Run("all obstacles returns false", func(t *testing.T) {
		g2 := NewOrthogonalGrid([][]int32{{1, 1}, {1, 1}})
		_, _, ok := g2.FindNearestWalkableWorld(0, 0, 5, 0)
		if ok {
			t.Error("expected false when all obstacles")
		}
	})

	t.Run("maxRadius=0 only checks self", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(2, 2, 0, 0)
		if ok {
			t.Error("expected false when self obstacle and maxRadius=0")
		}
	})

	t.Run("maxRadius negative", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(2, 2, -1, 0)
		if ok {
			t.Error("expected false when maxRadius < 0")
		}
	})

	t.Run("walkable self with limited radius", func(t *testing.T) {
		x, y, ok := g.FindNearestWalkableWorld(4, 4, 1, 0)
		if !ok {
			t.Fatal("expected ok")
		}
		if x != 4 || y != 4 {
			t.Errorf("expected (4,4), got (%v,%v)", x, y)
		}
	})

	t.Run("outside grid returns false", func(t *testing.T) {
		_, _, ok := g.FindNearestWalkableWorld(100, 100, 5, 0)
		if ok {
			t.Error("expected false outside grid")
		}
	})

	t.Run("finds single walkable cell across rings", func(t *testing.T) {
		g3 := NewOrthogonalGrid([][]int32{
			{0, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
			{1, 1, 1, 1, 1},
		})
		x, y, ok := g3.FindNearestWalkableWorld(4, 4, 8, 0)
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
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	}, WithTileSize(32, 32))

	x, y, ok := g.FindNearestWalkableWorld(48, 48, 3, 0)
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
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0},
		{0, 1, 0},
		{0, 0, 0},
	})
	if n := g.ObstacleCount(); n != 1 {
		t.Errorf("expected 1 obstacle, got %d", n)
	}
	g.(MutableGrid).SetWalkableAt(0, 0, false)
	if n := g.ObstacleCount(); n != 2 {
		t.Errorf("expected 2 obstacles, got %d", n)
	}
	g.(MutableGrid).SetWalkableAt(1, 1, true)
	g.(MutableGrid).SetWalkableAt(0, 0, true)
	if n := g.ObstacleCount(); n != 0 {
		t.Errorf("expected 0 obstacles, got %d", n)
	}
	g.(MutableGrid).SetWalkableAt(0, 0, true)
	if n := g.ObstacleCount(); n != 0 {
		t.Errorf("expected 0 obstacles after no-op, got %d", n)
	}
	g2, err := g.Clone(func(MutableGrid) {})
	if err != nil {
		t.Fatalf("Clone: %v", err)
	}
	if n := g2.(*OrthogonalGrid).ObstacleCount(); n != 0 {
		t.Errorf("clone: expected 0 obstacles, got %d", n)
	}
}

func TestOrthogonalGetNeighborsOrderAndBounds(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0},
	})
	cases := []struct {
		name   string
		x, y   int32
		mode   finder.DiagonalMovement
		expect [][2]int32
	}{
		{
			name: "corner-never", x: 0, y: 0, mode: finder.DiagonalNever,
			expect: [][2]int32{{1, 0}, {0, 1}},
		},
		{
			name: "corner-no-obstacles", x: 0, y: 0, mode: finder.DiagonalOnlyWhenNoObstacles,
			expect: [][2]int32{{1, 0}, {0, 1}, {1, 1}},
		},
		{
			name: "corner-at-most-one", x: 0, y: 0, mode: finder.DiagonalIfAtMostOneObstacle,
			expect: [][2]int32{{1, 0}, {0, 1}, {1, 1}},
		},
		{
			name: "corner-always", x: 0, y: 0, mode: finder.DiagonalAlways,
			expect: [][2]int32{{1, 0}, {0, 1}, {1, 1}},
		},
		{
			name: "edge-top", x: 1, y: 0, mode: finder.DiagonalOnlyWhenNoObstacles,
			expect: [][2]int32{{2, 0}, {1, 1}, {0, 0}, {2, 1}, {0, 1}},
		},
		{
			name: "edge-right", x: 2, y: 1, mode: finder.DiagonalAlways,
			expect: [][2]int32{{2, 0}, {2, 2}, {1, 1}, {1, 0}, {1, 2}},
		},
		{
			name: "corner-far", x: 2, y: 2, mode: finder.DiagonalAlways,
			expect: [][2]int32{{2, 1}, {1, 2}, {1, 1}},
		},
		{
			name: "center-never", x: 1, y: 1, mode: finder.DiagonalNever,
			expect: [][2]int32{{1, 0}, {2, 1}, {1, 2}, {0, 1}},
		},
		{
			name: "center-always", x: 1, y: 1, mode: finder.DiagonalAlways,
			expect: [][2]int32{{1, 0}, {2, 1}, {1, 2}, {0, 1}, {0, 0}, {2, 0}, {2, 2}, {0, 2}},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := g.GetNeighbors(&finder.Node{X: tc.x, Y: tc.y}, tc.mode, nil)
			if len(got) != len(tc.expect) {
				t.Fatalf("neighbors = %v, want %v", coordsOf(got), tc.expect)
			}
			for i, n := range got {
				if n.X != tc.expect[i][0] || n.Y != tc.expect[i][1] {
					t.Fatalf("neighbor[%d] = (%d,%d), want %v; all=%v", i, n.X, n.Y, tc.expect[i], coordsOf(got))
				}
			}
		})
	}

	for _, x := range []int32{-1, 3} {
		if got := g.GetNeighbors(&finder.Node{X: x, Y: 1}, finder.DiagonalAlways, nil); len(got) != 0 {
			t.Errorf("out-of-bounds x=%d returned %v", x, coordsOf(got))
		}
	}
	for _, y := range []int32{-1, 3} {
		if got := g.GetNeighbors(&finder.Node{X: 1, Y: y}, finder.DiagonalAlways, nil); len(got) != 0 {
			t.Errorf("out-of-bounds y=%d returned %v", y, coordsOf(got))
		}
	}
}

func TestOrthogonalGetNeighborsBlockedCardinals(t *testing.T) {
	g := NewOrthogonalGrid([][]int32{
		{0, 1, 0},
		{1, 0, 1},
		{0, 1, 0},
	})
	center := &finder.Node{X: 1, Y: 1}
	if got := g.GetNeighbors(center, finder.DiagonalOnlyWhenNoObstacles, nil); len(got) != 0 {
		t.Errorf("OnlyWhenNoObstacles returned %v, want none", coordsOf(got))
	}
	if got := g.GetNeighbors(center, finder.DiagonalIfAtMostOneObstacle, nil); len(got) != 0 {
		t.Errorf("AtMostOne returned %v, want none", coordsOf(got))
	}
	want := [][2]int32{{0, 0}, {2, 0}, {2, 2}, {0, 2}}
	got := g.GetNeighbors(center, finder.DiagonalAlways, nil)
	if len(got) != len(want) {
		t.Fatalf("Always returned %v, want %v", coordsOf(got), want)
	}
	for i, n := range got {
		if n.X != want[i][0] || n.Y != want[i][1] {
			t.Fatalf("Always neighbor[%d] = (%d,%d), want %v; all=%v", i, n.X, n.Y, want[i], coordsOf(got))
		}
	}
}

func coordsOf(nodes []*finder.Node) [][2]int32 {
	out := make([][2]int32, len(nodes))
	for i, n := range nodes {
		out[i] = [2]int32{n.X, n.Y}
	}
	return out
}
