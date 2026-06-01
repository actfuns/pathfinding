package waypoint

import (
	"math"
	"testing"
)

// ref: epiplon-waypoints (C# Unity) — https://github.com/epiplon/Waypoints
//
// Our implementation shares the following structural properties:
//
// 1. Graph structure: nodes with positions + list of connections
// 2. Connection cost: Euclidean distance between node positions
// 3. Closest-node: find node nearest to a given position (Euclidean)
// 4. Path search: start → nearest node → ... → nearest node → end
//
// Key differences from epiplon:
//   - epiplon uses recursive DFS with heuristic-based connection sorting
//     (greedy best-first). We use proper A* with optimality guarantees.
//   - epiplon includes line-of-sight checks (Physics.Linecast) for
//     connection validation. Our graph assumes pre-validated connections.
//   - epiplon has ConnectionType (Static/Dynamic/Null). We have no type
//     system — edges are always traversable.
//   - epiplon's DFS can produce suboptimal paths. Our A* produces
//     optimal shortest paths.
//   - epiplon returns List<Connection>; we return [][2]float64.

// TestWaypointGraphStructure verifies node/edge structure matches epiplon's.
func TestWaypointGraphStructure(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	c := g.AddNode(10, 10)
	a.Connect(b)
	b.Connect(c)

	// epiplon equivalent: Node has Position and ConnectedNodes[]
	// Our WaypointNode has X,Y and Edges[]
	if a.X != 0 || a.Y != 0 {
		t.Errorf("expected node at (0,0), got (%v,%v)", a.X, a.Y)
	}
	if len(a.Edges) != 1 || len(b.Edges) != 2 || len(c.Edges) != 1 {
		t.Errorf("unexpected edge counts: a=%d b=%d c=%d", len(a.Edges), len(b.Edges), len(c.Edges))
	}
}

// TestWaypointEdgeCost verifies edge cost is Euclidean distance,
// matching epiplon's Vector3.Distance based cost.
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

// TestWaypointFindClosest verifies FindClosest matches epiplon's
// FindClosestNode behavior (nearest node by Euclidean distance).
func TestWaypointFindClosest(t *testing.T) {
	g := NewWaypointGraph()
	g.AddNode(0, 0)
	g.AddNode(10, 10)
	g.AddNode(5, 5)

	// epiplon's FindClosestNode iterates all nodes, finds closest
	n := g.FindClosest(4.5, 4.5)
	if n == nil {
		t.Fatal("expected node, got nil")
	}
	if n.X != 5 || n.Y != 5 {
		t.Errorf("expected closest (5,5), got (%v,%v)", n.X, n.Y)
	}

	// Same behavior: if at exact node position, return that node
	exact := g.FindClosest(0, 0)
	if exact == nil || exact.X != 0 || exact.Y != 0 {
		t.Errorf("expected exact match (0,0), got (%v,%v)", exact.X, exact.Y)
	}
}

// TestWaypointPathCost verifies A* finds optimal (shortest) path,
// which is a key advantage over epiplon's DFS approach.
func TestWaypointPathCost(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(5, 5)
	d := g.AddNode(5, 10)
	e := g.AddNode(0, 10)

	// Linear chain: A-B-C-D-E
	a.Connect(b)
	b.Connect(c)
	c.Connect(d)
	d.Connect(e)

	// Also direct A-C connection (shorter than A-B-C)
	a.Connect(c)

	f := NewWaypointFinder(g)

	// Path from A to C: A* should choose direct A-C not A-B-C
	// epiplon's DFS with heuristic sorting may or may not pick optimal—
	// it depends on sort order. Our A* guarantees optimal.
	path := f.FindPathFloat(0, 0, 5, 5)
	if path == nil {
		t.Fatal("expected path, got nil")
	}

	// Verify path cost is optimal
	cost := pathCost(path)
	directCost := dist2(0, 0, 5, 5)
	if cost > directCost+0.001 {
		t.Errorf("path cost %.2f exceeds direct cost %.2f — suboptimal", cost, directCost)
	}

	t.Logf("waypoint path: %v, cost: %.2f", path, cost)
}

// TestWaypointEmptyGraph verifies FindPath returns nil for empty graph,
// matching epiplon's QueryPath returning empty list.
func TestWaypointEmptyGraph(t *testing.T) {
	g := NewWaypointGraph()
	f := NewWaypointFinder(g)
	path := f.FindPathFloat(0, 0, 10, 10)
	if path != nil {
		t.Error("expected nil for empty graph")
	}
}

// TestWaypointDisconnectedGraph verifies FindPath returns nil when
// start and end are in disconnected components.
func TestWaypointDisconnectedGraph(t *testing.T) {
	g := NewWaypointGraph()
	g.AddNode(0, 0)
	g.AddNode(10, 0)
	// No connection between a and b — epiplon's DFS would also fail

	f := NewWaypointFinder(g)
	path := f.FindPathFloat(0, 0, 10, 0)
	if path != nil {
		t.Error("expected nil for disconnected graph")
	}
}

// TestWaypointStartEndPositions verifies the path includes exact
// start and end world positions (not just node positions).
func TestWaypointStartEndPositions(t *testing.T) {
	g := NewWaypointGraph()
	g.AddNode(5, 5)
	g.AddNode(10, 5)
	// Connect them
	g.Nodes[0].Connect(g.Nodes[1])

	f := NewWaypointFinder(g)

	// Start at (0,0) — closest node is (5,5)
	// End at (15,5) — closest node is (10,5)
	path := f.FindPathFloat(0, 0, 15, 5)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	// Path should be: (0,0) → (5,5) → (10,5) → (15,5)
	if len(path) != 4 {
		t.Errorf("expected 4 points (start, node1, node2, end), got %d: %v", len(path), path)
	}
	if path[0] != [2]float64{0, 0} {
		t.Errorf("expected start (0,0), got %v", path[0])
	}
	if path[len(path)-1] != [2]float64{15, 5} {
		t.Errorf("expected end (15,5), got %v", path[len(path)-1])
	}
}

// TestWaypointPathIsReverseable verifies paths work in both directions
// (epiplon's graph is undirected, same as our Connect()).
func TestWaypointPathIsReverseable(t *testing.T) {
	g := NewWaypointGraph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(10, 0)
	a.Connect(b)
	b.Connect(c)

	f := NewWaypointFinder(g)

	forward := f.FindPathFloat(0, 0, 10, 0)
	reverse := f.FindPathFloat(10, 0, 0, 0)

	if forward == nil || reverse == nil {
		t.Fatal("expected both directions to have paths")
	}

	if forward[0] != [2]float64{0, 0} || forward[len(forward)-1] != [2]float64{10, 0} {
		t.Error("forward direction incorrect")
	}
	if reverse[0] != [2]float64{10, 0} || reverse[len(reverse)-1] != [2]float64{0, 0} {
		t.Error("reverse direction incorrect")
	}
}

// --- helpers ---

func pathCost(path [][2]float64) float64 {
	cost := 0.0
	for i := 1; i < len(path); i++ {
		dx := path[i][0] - path[i-1][0]
		dy := path[i][1] - path[i-1][1]
		cost += math.Sqrt(dx*dx + dy*dy)
	}
	return cost
}

func dist2(x1, y1, x2, y2 float64) float64 {
	dx := x1 - x2
	dy := y1 - y2
	return math.Sqrt(dx*dx + dy*dy)
}
