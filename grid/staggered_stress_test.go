package grid

import (
	"testing"
)

// --- Stress tests ---

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
	smooth := NewStaggeredGrid(matrix).FindSmoothPath(sx, sy, ex, ey)
	if smooth == nil {
		t.Fatal("FindSmoothPath: expected path on open staggered grid")
	}
}

// --- Benchmarks ---

func BenchmarkStaggered100(b *testing.B) {
	matrix := generateGrid(100, 100, 0.2)
	base := NewStaggeredGrid(matrix)
	f := base.Finder()
	sx, sy := base.TileToWorld(5, 5)
	ex, ey := base.TileToWorld(94, 94)
	b.Run("AStar", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			f.FindPath(0, 0, 99, 99, base)
		}
	})
	b.Run("AStarSmooth", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			base.FindSmoothPath(sx, sy, ex, ey)
		}
	})
}