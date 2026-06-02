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