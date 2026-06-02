package waypoint

import (
	"math"
	"testing"
)

// TestWaypointGraphStructure verifies node/edge structure.
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
	if len(a.Edges) != 1 || len(b.Edges) != 2 || len(c.Edges) != 1 {
		t.Errorf("unexpected edge counts: a=%d b=%d c=%d", len(a.Edges), len(b.Edges), len(c.Edges))
	}
}

// TestWaypointEdgeCost verifies edge cost is Euclidean distance.
func TestWaypointEdgeCost(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(3, 4)
	a.Connect(b)

	expected := 5.0 // 3-4-5 triangle
	if a.Edges[0].Cost != expected {
		t.Errorf("expected cost %.1f (3-4-5 triangle), got %v", expected, a.Edges[0].Cost)
	}
	if b.Edges[0].Cost != expected {
		t.Errorf("expected bidirectional cost %.1f, got %v", expected, b.Edges[0].Cost)
	}
}

// TestWaypointFindClosest verifies FindClosest returns the nearest node by Euclidean distance.
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
		t.Errorf("expected closest (5,5), got (%v,%v)", n.X, n.Y)
	}

	exact := g.FindClosest(0, 0)
	if exact == nil || exact.X != 0 || exact.Y != 0 {
		t.Errorf("expected exact match (0,0), got (%v,%v)", exact.X, exact.Y)
	}
}

// TestWaypointPathCost verifies A* finds optimal (shortest) path.
func TestWaypointPathCost(t *testing.T) {
	f := NewWaypointFinder()
	a := f.AddNode(0, 0)
	b := f.AddNode(5, 0)
	c := f.AddNode(5, 5)
	d := f.AddNode(5, 10)
	e := f.AddNode(0, 10)

	f.Connect(a, b)
	f.Connect(b, c)
	f.Connect(c, d)
	f.Connect(d, e)
	f.Connect(a, c) // direct shortcut

	g := walkableGrid(15, 15)
	path := f.FindPath(0, 0, 5, 5, g)
	if path == nil {
		t.Fatal("expected path, got nil")
	}

	cost := pathCostInt(path)
	directCost := dist2Int(0, 0, 5, 5)
	if cost > directCost+0.001 {
		t.Errorf("path cost %.2f exceeds direct cost %.2f — suboptimal", cost, directCost)
	}
	t.Logf("waypoint path: %v, cost: %.2f", path, cost)
}

// TestWaypointEmptyGraph verifies FindPath returns nil for empty graph.
func TestWaypointEmptyGraph(t *testing.T) {
	f := NewWaypointFinder()
	g := walkableGrid(15, 15)
	path := f.FindPath(0, 0, 10, 10, g)
	if path != nil {
		t.Error("expected nil for empty graph")
	}
}

// TestWaypointDisconnectedGraph verifies FindPath returns nil for disconnected components.
func TestWaypointDisconnectedGraph(t *testing.T) {
	f := NewWaypointFinder()
	f.AddNode(0, 0)
	f.AddNode(10, 0)
	// No connection

	g := walkableGrid(15, 5)
	path := f.FindPath(0, 0, 10, 0, g)
	if path != nil {
		t.Error("expected nil for disconnected graph")
	}
}

// TestWaypointStartEndPositions verifies the path includes exact start and end positions.
func TestWaypointStartEndPositions(t *testing.T) {
	f := NewWaypointFinder()
	a := f.AddNode(5, 5)
	b := f.AddNode(10, 5)
	f.Connect(a, b)

	g := walkableGrid(20, 20)
	path := f.FindPath(0, 0, 15, 5, g)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) != 4 {
		t.Errorf("expected 4 points (start, node1, node2, end), got %d: %v", len(path), path)
	}
	if path[0] != [2]int{0, 0} {
		t.Errorf("expected start (0,0), got %v", path[0])
	}
	if path[len(path)-1] != [2]int{15, 5} {
		t.Errorf("expected end (15,5), got %v", path[len(path)-1])
	}
}

// TestWaypointPathIsReverseable verifies paths work in both directions.
func TestWaypointPathIsReverseable(t *testing.T) {
	f := NewWaypointFinder()
	a := f.AddNode(0, 0)
	b := f.AddNode(5, 0)
	c := f.AddNode(10, 0)
	f.Connect(a, b)
	f.Connect(b, c)

	g := walkableGrid(15, 5)
	forward := f.FindPath(0, 0, 10, 0, g)
	reverse := f.FindPath(10, 0, 0, 0, g)

	if forward == nil || reverse == nil {
		t.Fatal("expected both directions to have paths")
	}
	if forward[0] != [2]int{0, 0} || forward[len(forward)-1] != [2]int{10, 0} {
		t.Error("forward direction incorrect")
	}
	if reverse[0] != [2]int{10, 0} || reverse[len(reverse)-1] != [2]int{0, 0} {
		t.Error("reverse direction incorrect")
	}
}

// --- helpers ---

func pathCostInt(path [][2]int) float64 {
	cost := 0.0
	for i := 1; i < len(path); i++ {
		dx := path[i][0] - path[i-1][0]
		dy := path[i][1] - path[i-1][1]
		cost += math.Sqrt(float64(dx*dx + dy*dy))
	}
	return cost
}

func dist2Int(x1, y1, x2, y2 int) float64 {
	dx := x1 - x2
	dy := y1 - y2
	return math.Sqrt(float64(dx*dx + dy*dy))
}
