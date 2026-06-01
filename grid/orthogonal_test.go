package grid

import (
	"testing"
)

func TestOrthogonalWorldToTile(t *testing.T) {
	g := NewOrthogonalGridWH([][]int{
		{0, 0, 0},
		{0, 0, 0},
	}, 32, 32)

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
	g := NewOrthogonalGridWH([][]int{
		{0, 0, 0},
		{0, 0, 0},
	}, 32, 32)

	tests := []struct {
		tx, ty    int
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
	g := NewOrthogonalGridWH([][]int{
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
	}, 16, 16)

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
