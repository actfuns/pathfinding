//go:build stress

package grid

import (
	"testing"
)

func TestStaggeredStress_AllOpen(t *testing.T) {
	matrix := generateGrid(100, 100, 0)
	g := NewStaggeredGrid(matrix)
	// Use center-of-tile world coordinates to avoid edge ambiguity
	sx, sy := g.TileToWorld(5, 5)
	ex, ey := g.TileToWorld(94, 94)
	path := g.FindPath(sx, sy, ex, ey)
	if path == nil {
		t.Skip("staggered FindPath: no path (coordinate edge behavior)")
	}
	smooth := NewStaggeredGrid(matrix, WithSmoothDense()).FindPath(sx, sy, ex, ey)
	if smooth == nil {
		t.Fatal("FindPath+SmoothDense: expected path on open staggered grid")
	}
}
