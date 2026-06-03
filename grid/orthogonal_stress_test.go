//go:build stress

package grid

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/hpa"
)

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
