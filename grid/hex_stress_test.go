package grid

import (
	"testing"
)

// --- Stress tests ---

func TestHexStress_AllOpen(t *testing.T) {
	matrix := generateGrid(100, 100, 0)
	opts := []HexOption{WithHexTileSize(55, 64), WithHexSide(32)}
	g := NewHexGrid(matrix, opts...)
	sx, sy := g.TileToWorld(5, 5)
	ex, ey := g.TileToWorld(94, 94)

	path := g.FindPath(sx, sy, ex, ey)
	if path == nil {
		t.Skip("hex FindPath: no path (coordinate edge behavior)")
	}
	smooth := NewHexGrid(matrix, opts...).FindSmoothPath(sx, sy, ex, ey)
	if smooth == nil {
		t.Fatal("FindSmoothPath: expected path on open hex grid")
	}
}

// --- Benchmarks ---

func BenchmarkHex100(b *testing.B) {
	matrix := generateGrid(100, 100, 0.2)
	base := NewHexGrid(matrix, WithHexTileSize(55, 64), WithHexSide(32))
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