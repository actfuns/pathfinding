package waypoint

import (
	"testing"
)

func TestWaypointGraphBasic(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	c := g.AddNode(10, 10)
	a.Connect(b)
	b.Connect(c)

	f := NewWaypointFinder(g)
	path := f.FindPathFloat(0, 0, 10, 10)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
	// Should go through B
	if len(path) != 3 {
		t.Errorf("expected 3 points (start=node A, node B, end=node C), got %d: %v", len(path), path)
	}
	if path[0] != [2]float64{0, 0} || path[1] != [2]float64{10, 0} || path[2] != [2]float64{10, 10} {
		t.Errorf("unexpected path: %v", path)
	}
}

func TestWaypointGraphDirectConnection(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 5)
	a.Connect(b)

	f := NewWaypointFinder(g)
	path := f.FindPathFloat(0, 0, 5, 5)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) != 2 {
		t.Errorf("expected 2 points (start=node A, end=node B), got %d: %v", len(path), path)
	}
}

func TestWaypointGraphNoPath(t *testing.T) {
	g := NewWaypointGraph()
	g.AddNode(0, 0)
	g.AddNode(10, 0)
	// No connection between a and b

	f := NewWaypointFinder(g)
	path := f.FindPathFloat(0, 0, 10, 0)
	if path != nil {
		t.Error("expected nil path for disconnected graph")
	}
}

func TestWaypointGraphEmpty(t *testing.T) {
	g := NewWaypointGraph()
	f := NewWaypointFinder(g)
	path := f.FindPathFloat(0, 0, 10, 10)
	if path != nil {
		t.Error("expected nil path for empty graph")
	}
}

func TestWaypointGraphBidirectional(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(10, 0)
	a.Connect(b)
	b.Connect(c)

	f := NewWaypointFinder(g)
	// Reverse direction should also work
	path := f.FindPathFloat(10, 0, 0, 0)
	if path == nil {
		t.Fatal("expected path in reverse direction")
	}
	// Start and end should be correct
	if path[0] != [2]float64{10, 0} {
		t.Errorf("expected start (10,0), got %v", path[0])
	}
	if path[len(path)-1] != [2]float64{0, 0} {
		t.Errorf("expected end (0,0), got %v", path[len(path)-1])
	}
}

func TestWaypointGraphFindClosest(t *testing.T) {
	g := NewWaypointGraph()
	g.AddNode(0, 0)
	g.AddNode(10, 10)
	g.AddNode(5, 5)

	n := g.FindClosest(4, 4)
	if n == nil {
		t.Fatal("expected node, got nil")
	}
	if n.X != 5 || n.Y != 5 {
		t.Errorf("expected (5,5), got (%v,%v)", n.X, n.Y)
	}
}
