package grid

import (
	"sync"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func TestConcurrentReadOnlyFindPath(t *testing.T) {
	m := make([][]int, 48)
	for y := range m {
		m[y] = make([]int, 48)
		for x := range m[y] {
			if (x*7+y*11)%19 == 0 {
				m[y][x] = 1
			}
		}
	}
	m[0][0], m[47][47] = 0, 0
	f := finder.NewAStarFinder(finder.WithDiagonal(finder.DiagonalAlways))
	g := NewOrthogonalGrid(m, WithFinder(f), WithTileSize(16, 16))
	var wg sync.WaitGroup
	for i := 0; i < 16; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			dst := make([][2]int32, 0, 256)
			wdst := make([][2]float32, 0, 256)
			tdst := make([][2]int32, 0, 256)
			for j := 0; j < 100; j++ {
				p, err := g.FindPath(0, 0, 47, 47, nil)
				if err != nil || p == nil {
					t.Errorf("FindPath: %v %v", p, err)
					return
				}
				p, err = g.FindPath(0, 0, 47, 47, dst[:0])
				if err != nil || p == nil {
					t.Errorf("FindPath: %v %v", p, err)
					return
				}
				dst = p
				wp, err := g.FindPathWorld(0, 0, 47*16, 47*16, &PathBuff{Tile: tdst[:0], World: wdst[:0]})
				if err != nil || wp == nil {
					t.Errorf("FindPathWorld: %v %v", wp, err)
					return
				}
				wdst = wp
			}
		}()
	}
	wg.Wait()
}

func TestConcurrentJPSPlusReadOnlyFindPath(t *testing.T) {
	m := make([][]int, 48)
	for y := range m {
		m[y] = make([]int, 48)
		for x := range m[y] {
			if (x*5+y*13)%23 == 0 {
				m[y][x] = 1
			}
		}
	}
	m[0][0], m[47][47] = 0, 0
	f := finder.NewJPSPlusFinder(finder.WithDiagonal(finder.DiagonalAlways))
	g := NewOrthogonalGrid(m, WithFinder(f))
	if err := f.Precompute(g); err != nil {
		t.Fatal(err)
	}
	var wg sync.WaitGroup
	for i := 0; i < 16; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			dst := make([][2]int32, 0, 256)
			for j := 0; j < 50; j++ {
				p, err := g.FindPath(0, 0, 47, 47, dst[:0])
				if err != nil || p == nil {
					t.Errorf("FindPath: %v %v", p, err)
					return
				}
			}
		}()
	}
	wg.Wait()
}
