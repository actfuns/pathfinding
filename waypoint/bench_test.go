package waypoint

import (
	"testing"
)

func BenchmarkWaypointFindPath(b *testing.B) {

	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	prev := a
	for i := 10; i <= 1000; i += 10 {
		cur := g.AddNode(int32(i), 0)
		prev.Connect(cur)
		prev = cur
	}
	grid := walkableGrid(1010, 50)

	b.ReportAllocs()

	for b.Loop() {
		path, _ := f.FindPath(0, 0, 1000, 0, grid, nil)
		if path == nil {
			b.Fatal("path not found")
		}
	}
}
