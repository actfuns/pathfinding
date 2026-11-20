package finder_test

import (
	"sync"
	"testing"

	"github.com/actfuns/pathfinding/finder"
	"github.com/actfuns/pathfinding/grid"
)

func TestConcurrentFinders(t *testing.T) {
	m := make([][]int32, 32)
	for y := range m {
		m[y] = make([]int32, 32)
		for x := range m[y] {
			if (x*5+y*11)%23 == 0 {
				m[y][x] = 1
			}
		}
	}
	m[0][0], m[31][31] = 0, 0
	g := grid.NewOrthogonalGrid(m)

	jpsPlus := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalAlways))
	if err := jpsPlus.Precompute(g); err != nil {
		t.Fatal(err)
	}

	type entry struct {
		name string
		f    finder.Finder
	}
	entries := []entry{
		{"AStar", finder.NewAStarFinder(finder.WithDiagonal(finder.DiagonalAlways))},
		{"BiAStar", finder.NewBiAStarFinder(finder.WithDiagonal(finder.DiagonalAlways))},
		{"BFS", finder.NewBreadthFirstFinder(finder.WithDiagonal(finder.DiagonalAlways))},
		{"BiBFS", finder.NewBiBreadthFirstFinder(finder.WithDiagonal(finder.DiagonalAlways))},
		{"IDAStar", finder.NewIDAStarFinder(finder.WithDiagonal(finder.DiagonalAlways))},
		{"JPSNever", finder.NewJPFNeverMoveDiagonally()},
		{"JPSAlways", finder.NewJPFAlwaysMoveDiagonally()},
		{"JPSAtMostOne", finder.NewJPFMoveDiagonallyIfAtMostOneObstacle()},
		{"JPSNoObstacles", finder.NewJPFMoveDiagonallyIfNoObstacles()},
		{"JPSPlus", jpsPlus},
	}

	for _, e := range entries {
		t.Run(e.name, func(t *testing.T) {
			var wg sync.WaitGroup
			for i := 0; i < 8; i++ {
				wg.Add(1)
				go func() {
					defer wg.Done()
					dst := make([][2]int32, 0, 256)
					for j := 0; j < 20; j++ {
						p, err := e.f.FindPath(0, 0, 31, 31, g, dst[:0])
						if err != nil || p == nil {
							t.Errorf("path: %v %v", p, err)
							return
						}
						dst = p
					}
				}()
			}
			wg.Wait()
		})
	}
}

func TestConcurrentJPSPlusAdaptive(t *testing.T) {
	m := make([][]int32, 32)
	for y := range m {
		m[y] = make([]int32, 32)
		for x := range m[y] {
			if (x*7+y*11)%5 == 0 {
				m[y][x] = 1
			}
		}
	}
	m[0][0], m[31][31] = 0, 0
	g := grid.NewOrthogonalGrid(m)
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalAlways))
	f.Adaptive = true
	if err := f.Precompute(g); err != nil {
		t.Fatal(err)
	}
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			dst := make([][2]int32, 0, 256)
			for j := 0; j < 20; j++ {
				p, err := f.FindPath(0, 0, 31, 31, g, dst[:0])
				if err != nil || p == nil {
					t.Errorf("path: %v %v", p, err)
					return
				}
				dst = p
			}
		}()
	}
	wg.Wait()
}
