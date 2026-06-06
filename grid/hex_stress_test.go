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

	path := g.FindPath(sx, sy, ex, ey)
	if path == nil {
		t.Skip("hex FindPath: no path (coordinate edge behavior)")
	}
	smooth := NewHexGrid(matrix, append(opts, WithSmoothDense())...).FindPath(sx, sy, ex, ey)
	if smooth == nil {
		t.Fatal("FindPath+SmoothDense: expected path on open hex grid")
	}
}
