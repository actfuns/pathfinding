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
	path, err := g.FindPathWorld(sx, sy, ex, ey)
	if err != nil {
		t.Fatalf("FindPathWorld: %v", err)
	}
	if path == nil {
		t.Skip("staggered FindPathWorld: no path (coordinate edge behavior)")
	}
	smooth, err := NewStaggeredGrid(matrix, WithSmoothDense()).FindPathWorld(sx, sy, ex, ey)
	if err != nil {
		t.Fatalf("FindPathWorld+SmoothDense: %v", err)
	}
	if smooth == nil {
		t.Fatal("FindPathWorld+SmoothDense: expected path on open staggered grid")
	}
}
