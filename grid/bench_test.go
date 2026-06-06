package grid

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/hpa"
)

// BenchmarkOrthogonalAlloc compares time and allocations across finders at multiple scales.
// Run with: go test -bench=BenchmarkOrthogonalAlloc -benchmem ./grid/
func BenchmarkOrthogonalAlloc(b *testing.B) {
	sizes := []struct {
		label  string
		w, h   int
		obs    float64
		sx, sy int
		ex, ey int
		chunk  int
	}{
		{"100x100", 100, 100, 0.2, 0, 0, 99, 99, 16},
		{"200x200", 200, 200, 0.1, 0, 0, 199, 199, 16},
		{"500x500", 500, 500, 0.1, 0, 0, 499, 499, 32},
	}

	for _, sz := range sizes {
		matrix := generateGrid(sz.w, sz.h, sz.obs)
		base := NewOrthogonalGrid(matrix)

		b.Run(sz.label+"/AStar", func(b *testing.B) {
			f := finder.NewAStarFinder()
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				f.FindPath(sz.sx, sz.sy, sz.ex, sz.ey, base)
			}
		})

		b.Run(sz.label+"/JPS", func(b *testing.B) {
			f := finder.NewJumpPointFinder()
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				f.FindPath(sz.sx, sz.sy, sz.ex, sz.ey, base)
			}
		})

		b.Run(sz.label+"/HPA", func(b *testing.B) {
			h := hpa.NewHPAFinder(hpa.WithChunkSize(sz.chunk))
			h.Build(base)
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				h.FindPath(sz.sx, sz.sy, sz.ex, sz.ey, base)
			}
		})
	}
}

func BenchmarkOrthogonal100(b *testing.B) {
	matrix := generateGrid(100, 100, 0.2)
	base := NewOrthogonalGrid(matrix)
	f := base.Finder()
	b.Run("AStar", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			f.FindPath(0, 0, 99, 99, base)
		}
	})
	b.Run("AStarSmooth", func(b *testing.B) {
		sm := NewOrthogonalGrid(matrix, WithSmoothBresenham())
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			sm.FindPathWorld(0, 0, 99, 99)
		}
	})
	b.Run("JPS", func(b *testing.B) {
		jps := NewOrthogonalGrid(matrix, WithFinder(finder.NewJumpPointFinder()))
		fj := jps.Finder()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			fj.FindPath(0, 0, 99, 99, jps)
		}
	})
	b.Run("HPA", func(b *testing.B) {
		h := hpa.NewHPAFinder(hpa.WithChunkSize(16))
		h.Build(base)
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			h.FindPath(0, 0, 99, 99, base)
		}
	})
}

func BenchmarkOrthogonal500(b *testing.B) {
	matrix := generateGrid(500, 500, 0.1)
	base := NewOrthogonalGrid(matrix)
	f := base.Finder()
	b.Run("AStar", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			f.FindPath(0, 0, 499, 499, base)
		}
	})
	b.Run("AStarSmooth", func(b *testing.B) {
		sm := NewOrthogonalGrid(matrix, WithSmoothBresenham())
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			sm.FindPathWorld(0, 0, 499, 499)
		}
	})
	b.Run("HPA", func(b *testing.B) {
		h := hpa.NewHPAFinder(hpa.WithChunkSize(16))
		h.Build(base)
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			h.FindPath(0, 0, 499, 499, base)
		}
	})
}

func BenchmarkHex100(b *testing.B) {
	matrix := generateGrid(100, 100, 0.2)
	base := NewHexGrid(matrix, WithTileSize(55, 64), WithHexSide(32))
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
		sm := NewOrthogonalGrid(matrix, WithSmoothBresenham())
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			sm.FindPathWorld(sx, sy, ex, ey)
		}
	})
}

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
		sm := NewOrthogonalGrid(matrix, WithSmoothBresenham())
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			sm.FindPathWorld(sx, sy, ex, ey)
		}
	})
}
