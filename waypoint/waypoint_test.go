package waypoint

import (
	"testing"
)

// TestConnectOneWay verifies the directed connection method.
func TestConnectOneWay(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)

	a.ConnectOneWay(b)

	// a should have an edge to b
	if len(a.Edges) != 1 {
		t.Errorf("expected 1 edge from a, got %d", len(a.Edges))
	}
	if a.Edges[0].To != b {
		t.Error("expected a's edge to point to b")
	}

	// b should NOT have an edge back to a (directed)
	if len(b.Edges) != 0 {
		t.Errorf("expected 0 edges from b (directed), got %d", len(b.Edges))
	}

	// Cost should be Euclidean (10,0) → 10
	if a.Edges[0].Cost != 10 {
		t.Errorf("expected cost 10, got %v", a.Edges[0].Cost)
	}
}

// TestFindPathInt verifies FindPathFloat returns correct path.
func TestFindPathInt(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(10, 0)
	a.Connect(b)
	b.Connect(c)

	f := NewWaypointFinder(g)

	path := f.FindPathFloat(0.0, 0.0, 10.0, 0.0)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) < 2 {
		t.Fatal("path too short")
	}
	if path[0] != [2]float64{0, 0} || path[len(path)-1] != [2]float64{10, 0} {
		t.Errorf("wrong start/end: %v", path)
	}
}

// TestFindClosestEmpty verifies FindClosest returns nil for an empty graph.
func TestFindClosestEmpty(t *testing.T) {
	g := NewWaypointGraph()
	n := g.FindClosest(5, 5)
	if n != nil {
		t.Error("expected nil for empty graph")
	}
}

// TestConnectOneWayCost verifies cost calculation for directed connections
// with non-trivial distances.
func TestConnectOneWayCost(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(3, 4)
	a.ConnectOneWay(b)

	expected := 5.0
	if a.Edges[0].Cost != expected {
		t.Errorf("expected cost %.1f, got %v", expected, a.Edges[0].Cost)
	}

	// Zero-length edge at same position
	c := g.AddNode(5, 5)
	d := g.AddNode(5, 5)
	c.ConnectOneWay(d)
	if c.Edges[0].Cost != 0 {
		t.Errorf("expected 0 cost for same position, got %v", c.Edges[0].Cost)
	}
}

// TestFindPathWithDirectedEdges verifies that FindPath works with
// directed connections (one-way edges).
func TestFindPathWithDirectedEdges(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(10, 0)

	// One-way chain: a → b → c
	a.ConnectOneWay(b)
	b.ConnectOneWay(c)

	f := NewWaypointFinder(g)

	// Forward should work
	path := f.FindPathFloat(0.0, 0.0, 10.0, 0.0)
	if path == nil {
		t.Fatal("expected forward path with directed edges")
	}

	// Reverse should fail (no edge back)
	reverse := f.FindPathFloat(10.0, 0.0, 0.0, 0.0)
	if reverse != nil {
		t.Error("expected nil for reverse on directed graph")
	}
}
