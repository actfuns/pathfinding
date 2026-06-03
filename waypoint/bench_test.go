package waypoint

import (
	"testing"
)

func BenchmarkWaypointFindPath(b *testing.B) {
	b.StopTimer()
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	prev := a
	for i := 10; i <= 1000; i += 10 {
		cur := g.AddNode(i, 0)
		prev.Connect(cur)
		prev = cur
	}
	grid := walkableGrid(1010, 50)

	b.ReportAllocs()
	b.StartTimer()

	for i := 0; i < b.N; i++ {
		path := f.FindPath(0, 0, 1000, 0, grid)
		if path == nil {
			b.Fatal("path not found")
		}
	}
}
