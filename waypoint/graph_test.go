package waypoint

import "testing"

func TestWaypointGraphStructure(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	c := g.AddNode(10, 10)
	a.Connect(b)
	b.Connect(c)

	if a.X != 0 || a.Y != 0 {
		t.Errorf("expected node at (0,0), got (%v,%v)", a.X, a.Y)
	}
	if want := 1; len(a.Edges) != want {
		t.Errorf("a.Edges len = %d, want %d", len(a.Edges), want)
	}
	if want := 2; len(b.Edges) != want {
		t.Errorf("b.Edges len = %d, want %d", len(b.Edges), want)
	}
	if want := 1; len(c.Edges) != want {
		t.Errorf("c.Edges len = %d, want %d", len(c.Edges), want)
	}
}

func TestWaypointEdgeCost(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(3, 4)
	a.Connect(b)

	want := 5.0
	if a.Edges[0].Cost != want {
		t.Errorf("a->b cost = %.1f, want %.1f", a.Edges[0].Cost, want)
	}
	if b.Edges[0].Cost != want {
		t.Errorf("b->a cost = %.1f, want %.1f", b.Edges[0].Cost, want)
	}
}

func TestWaypointFindClosest(t *testing.T) {
	g := NewWaypointGraph()
	g.AddNode(0, 0)
	g.AddNode(10, 10)
	g.AddNode(5, 5)

	n := g.FindClosest(4, 4)
	if n == nil {
		t.Fatal("expected node, got nil")
	}
	if n.X != 5 || n.Y != 5 {
		t.Errorf("closest = (%d,%d), want (5,5)", n.X, n.Y)
	}
	n = g.FindClosest(0, 0)
	if n.X != 0 || n.Y != 0 {
		t.Errorf("exact match = (%d,%d), want (0,0)", n.X, n.Y)
	}
}

func TestFindClosestEmpty(t *testing.T) {
	g := NewWaypointGraph()
	if n := g.FindClosest(5, 5); n != nil {
		t.Error("expected nil for empty graph")
	}
}

func TestConnectOneWay(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	a.ConnectOneWay(b)

	if want := 1; len(a.Edges) != want {
		t.Fatalf("a.Edges len = %d, want %d", len(a.Edges), want)
	}
	if a.Edges[0].To != b {
		t.Error("expected a's edge to point to b")
	}
	if want := 0; len(b.Edges) != want {
		t.Errorf("b.Edges len = %d, want %d (directed)", len(b.Edges), want)
	}
	if a.Edges[0].Cost != 10 {
		t.Errorf("cost = %.1f, want 10", a.Edges[0].Cost)
	}
}

func TestConnectOneWayCost(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(3, 4)
	a.ConnectOneWay(b)

	if want := 5.0; a.Edges[0].Cost != want {
		t.Errorf("cost = %.1f, want %.1f (3-4-5)", a.Edges[0].Cost, want)
	}

	c := g.AddNode(5, 5)
	d := g.AddNode(5, 5)
	c.ConnectOneWay(d)
	if c.Edges[0].Cost != 0 {
		t.Errorf("same-position cost = %.1f, want 0", c.Edges[0].Cost)
	}
}

func TestEdgeStateDefault(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	a.Connect(b)

	if want := EdgeStateStatic; a.Edges[0].State != want {
		t.Errorf("default state = %d, want %d", a.Edges[0].State, want)
	}
}

func TestEdgeStateExplicit(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	a.Connect(b, EdgeStateDynamic)

	if a.Edges[0].State != EdgeStateDynamic {
		t.Errorf("state = %d, want %d", a.Edges[0].State, EdgeStateDynamic)
	}
	if b.Edges[0].State != EdgeStateDynamic {
		t.Errorf("bidirectional state = %d, want %d", b.Edges[0].State, EdgeStateDynamic)
	}
}

func TestConnectNearby(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	c := g.AddNode(10, 10)
	d := g.AddNode(100, 100) // far away

	grid := walkableGrid(110, 110)
	g.ConnectNearby(15, grid)

	// a-b: distance 10, should be connected
	if !hasEdge(a, b) {
		t.Error("a and b should be connected (dist=10 < 15)")
	}
	// b-c: distance 10, should be connected
	if !hasEdge(b, c) {
		t.Error("b and c should be connected (dist=10 < 15)")
	}
	// a-d: distance ~141, should NOT be connected
	if hasEdge(a, d) {
		t.Error("a and d should NOT be connected (dist=141 > 15)")
	}
}

func TestConnectNearbySkipsExisting(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	a.Connect(b, EdgeStateDynamic) // already connected with dynamic

	grid := walkableGrid(10, 10)
	g.ConnectNearby(10, grid)

	// Should still have exactly 1 edge (the original dynamic one)
	if want := 1; len(a.Edges) != want {
		t.Errorf("a.Edges len = %d, want %d (no duplicate)", len(a.Edges), want)
	}
	if a.Edges[0].State != EdgeStateDynamic {
		t.Error("existing edge state should not change")
	}
}

func hasEdge(a, b *WaypointNode) bool {
	for _, e := range a.Edges {
		if e.To == b {
			return true
		}
	}
	return false
}
