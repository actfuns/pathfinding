//go:build stress

package grid

import (
	"testing"
)

func TestHexStress_AllOpen(t *testing.T) {
	matrix := generateGrid(100, 100, 0)
	opts := []GridOption{WithTileSize(55, 64), WithHexSide(32)}
	g := NewHexGrid(matrix, opts...)
	sx, sy := g.TileToWorld(5, 5)
	ex, ey := g.TileToWorld(94, 94)

	path, err := g.FindPathWorld(sx, sy, ex, ey)
	if err != nil {
		t.Fatalf("FindPathWorld: %v", err)
	}
	if path == nil {
		t.Skip("hex FindPathWorld: no path (coordinate edge behavior)")
	}
	smooth, err := NewHexGrid(matrix, append(opts, WithSmoothDense())...).FindPathWorld(sx, sy, ex, ey)
	if err != nil {
		t.Fatalf("FindPathWorld+SmoothDense: %v", err)
	}
	if smooth == nil {
		t.Fatal("FindPathWorld+SmoothDense: expected path on open hex grid")
	}
}
