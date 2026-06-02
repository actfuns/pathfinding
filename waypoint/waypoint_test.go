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

	if len(a.Edges) != 1 {
		t.Errorf("expected 1 edge from a, got %d", len(a.Edges))
	}
	if a.Edges[0].To != b {
		t.Error("expected a's edge to point to b")
	}
	if len(b.Edges) != 0 {
		t.Errorf("expected 0 edges from b (directed), got %d", len(b.Edges))
	}
	if a.Edges[0].Cost != 10 {
		t.Errorf("expected cost 10, got %v", a.Edges[0].Cost)
	}
}

// TestConnectOneWayCost verifies cost calculation for directed connections.
func TestConnectOneWayCost(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(3, 4)
	a.ConnectOneWay(b)

	expected := 5.0
	if a.Edges[0].Cost != expected {
		t.Errorf("expected cost %.1f, got %v", expected, a.Edges[0].Cost)
	}

	c := g.AddNode(5, 5)
	d := g.AddNode(5, 5)
	c.ConnectOneWay(d)
	if c.Edges[0].Cost != 0 {
		t.Errorf("expected 0 cost for same position, got %v", c.Edges[0].Cost)
	}
}

// TestFindPathWithDirectedEdges verifies that FindPath works with directed edges.
func TestFindPathWithDirectedEdges(t *testing.T) {
	f := NewWaypointFinder()
	a := f.AddNode(0, 0)
	b := f.AddNode(5, 0)
	c := f.AddNode(10, 0)

	f.ConnectOneWay(a, b)
	f.ConnectOneWay(b, c)

	g := walkableGrid(15, 5)
	forward := f.FindPath(0, 0, 10, 0, g)
	if forward == nil {
		t.Fatal("expected forward path with directed edges")
	}

	reverse := f.FindPath(10, 0, 0, 0, g)
	if reverse != nil {
		t.Error("expected nil for reverse on directed graph")
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
