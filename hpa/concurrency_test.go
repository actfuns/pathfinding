package hpa

import (
	"sync"
	"testing"

	"github.com/actfuns/pathfinding/grid"
)

func TestConcurrentHPAFindPath(t *testing.T) {
	g := grid.NewOrthogonalGrid(grid.NewMatrix(48, 48))
	f := NewHPAFinder(WithChunkSize(8))
	if err := f.Build(g); err != nil {
		t.Fatal(err)
	}
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			dst := make([][2]int32, 0, 256)
			for j := 0; j < 20; j++ {
				p, err := f.FindPath(0, 0, 47, 47, g, dst[:0])
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
