package flowfield

import (
	"sync"
	"testing"

	"github.com/actfuns/pathfinding/grid"
)

func TestConcurrentFlowFieldReads(t *testing.T) {
	m := make([][]int, 48)
	for y := range m {
		m[y] = make([]int, 48)
		for x := range m[y] {
			if (x*7+y*11)%17 == 0 {
				m[y][x] = 1
			}
		}
	}
	m[0][0], m[47][47] = 0, 0
	g := grid.NewOrthogonalGrid(m)
	ff := New(g, 47, 47)

	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func(startX, startY int) {
			defer wg.Done()
			dst := make([][2]int32, 0, 256)
			for j := 0; j < 100; j++ {
				_, _ = ff.GetDirection(startX, startY)
				if ff.GetCost(startX, startY) < 0 {
					continue
				}
				p, err := ff.FindPath(startX, startY, dst[:0])
				if err != nil || p == nil {
					t.Errorf("FindPath(%d,%d): %v %v", startX, startY, p, err)
					return
				}
				dst = p
			}
		}(i%48, (i*7)%48)
	}
	wg.Wait()
}
