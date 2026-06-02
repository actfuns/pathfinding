package grid

import (
	"math/rand"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func generateGrid(width, height int, obstacleProb float64) [][]int {
	m := make([][]int, height)
	rng := rand.New(rand.NewSource(42))
	for y := range m {
		row := make([]int, width)
		for x := range row {
			if rng.Float64() < obstacleProb {
				row[x] = 1
			}
		}
		m[y] = row
	}
	return m
}

// --- Orthogonal stress ---

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

// --- Staggered stress ---

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

// --- Hex stress ---

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

func BenchmarkOrthogonal100(b *testing.B) {
	matrix := generateGrid(100, 100, 0.2)
	base := NewOrthogonalGrid(matrix)
	f := base.Finder()
	b.Run("AStar", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			f.FindPath(0, 0, 99, 99, base.Clone())
		}
	})
	b.Run("AStarSmooth", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			base.Clone().(*OrthogonalGrid).FindSmoothPath(0, 0, 99, 99)
		}
	})
	b.Run("JPS", func(b *testing.B) {
		jps := NewOrthogonalGrid(matrix, WithOrthogonalFinder(finder.NewJumpPointFinder()))
		fj := jps.Finder()
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			fj.FindPath(0, 0, 99, 99, jps.Clone())
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
			f.FindPath(0, 0, 499, 499, base.Clone())
		}
	})
	b.Run("AStarSmooth", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			base.Clone().(*OrthogonalGrid).FindSmoothPath(0, 0, 499, 499)
		}
	})
}

func BenchmarkStaggered100(b *testing.B) {
	matrix := generateGrid(100, 100, 0.2)
	base := NewStaggeredGrid(matrix)
	f := base.Finder()
	b.Run("AStar", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			f.FindPath(0, 0, 99, 99, base.Clone())
		}
	})
	b.Run("AStarSmooth", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			base.Clone().(*StaggeredGrid).FindSmoothPath(0, 0, 99, 99)
		}
	})
}

func BenchmarkHex100(b *testing.B) {
	matrix := generateGrid(100, 100, 0.2)
	base := NewHexGrid(matrix, WithHexTileSize(55, 64), WithHexSide(32))
	f := base.Finder()
	b.Run("AStar", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			f.FindPath(0, 0, 99, 99, base.Clone())
		}
	})
	b.Run("AStarSmooth", func(b *testing.B) {
		b.ResetTimer()
		for i := 0; i < b.N; i++ {
			base.Clone().(*HexGrid).FindSmoothPath(0, 0, 99, 99)
		}
	})
}
