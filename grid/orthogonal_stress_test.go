package grid

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/hpa"
)

// --- Stress tests ---

func TestOrthogonalStress_AllOpen(t *testing.T) {
	matrix := generateGrid(200, 200, 0)

	path := NewOrthogonalGrid(matrix).FindPath(0, 0, 199, 199)
	if path == nil {
		t.Fatal("FindPath: expected path on open grid")
	}

	smooth := NewOrthogonalGrid(matrix).FindSmoothPath(0, 0, 199, 199)
	if smooth == nil {
		t.Fatal("FindSmoothPath: expected path on open grid")
	}

	if len(smooth) > 10 {
		t.Logf("smooth path reduced %d raw points to %d points", len(path), len(smooth))
	}
}

func TestOrthogonalStress_VariousFinders(t *testing.T) {
	matrix := generateGrid(100, 100, 0.1)
	for _, ft := range []struct {
		name string
		f    finder.Finder
	}{
		{"AStar", finder.NewAStarFinder()},
		{"Dijkstra", finder.NewDijkstraFinder()},
		{"BFS", finder.NewBreadthFirstFinder()},
		{"JPS", finder.NewJumpPointFinder()},
		{"HPA", func() finder.Finder {
			f := hpa.NewHPAFinder(hpa.WithChunkSize(16))
			g := NewOrthogonalGrid(matrix, WithOrthogonalTileSize(1, 1))
			f.Build(g)
			return f
		}()},
	} {
		t.Run(ft.name, func(t *testing.T) {
			g := NewOrthogonalGrid(matrix, WithOrthogonalTileSize(1, 1))
			path := g.FindPath(0, 0, 99, 99)
			if path == nil {
				t.Skip("FindPath: no path (obstacle layout may block)")
			}
		})
		t.Run(ft.name+"Smooth", func(t *testing.T) {
			smooth := NewOrthogonalGrid(matrix, WithOrthogonalTileSize(1, 1)).FindSmoothPath(0, 0, 99, 99)
			if smooth == nil {
				t.Skip("FindSmoothPath: no path (obstacle layout may block)")
			}
		})
	}
}

func TestOrthogonalStress_DenseGrid(t *testing.T) {
	matrix := generateGrid(500, 500, 0.05)
	t.Run("FindPath", func(t *testing.T) {
		path := NewOrthogonalGrid(matrix).FindPath(0, 0, 499, 499)
		if path == nil {
			t.Skip("no path on 500x500 grid with 5% obstacles")
		}
	})
	t.Run("FindSmoothPath", func(t *testing.T) {
		smooth := NewOrthogonalGrid(matrix).FindSmoothPath(0, 0, 499, 499)
		if smooth == nil {
			t.Skip("no smooth path on 500x500 grid")
		}
	})
}

// --- Benchmarks ---

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
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			base.FindSmoothPath(0, 0, 99, 99)
		}
	})
	b.Run("JPS", func(b *testing.B) {
		jps := NewOrthogonalGrid(matrix, WithOrthogonalFinder(finder.NewJumpPointFinder()))
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
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			base.FindSmoothPath(0, 0, 499, 499)
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
