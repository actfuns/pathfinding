package waypoint

import (
	"sync"
	"testing"
)

func TestConcurrentWaypointFindPath(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	start := g.AddNode(0, 0)
	prev := start
	for i := 10; i <= 500; i += 10 {
		cur := g.AddNode(i, 0)
		prev.Connect(cur)
		prev = cur
	}
	grid := walkableGrid(510, 20)
	var wg sync.WaitGroup
	for i := 0; i < 8; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			dst := make([][2]int32, 0, 256)
			for j := 0; j < 20; j++ {
				p, err := f.FindPath(0, 0, 500, 0, grid, dst[:0])
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
