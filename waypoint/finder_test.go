package waypoint

import (
	"math"
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

func walkableGrid(w, h int) finder.Grid {
	nodes := make([]*finder.Node, w*h)
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			n := finder.NewNode(x, y)
			n.Walkable = true
			nodes[y*w+x] = n
		}
	}
	return &testGrid{width: w, height: h, nodes: nodes}
}

type testGrid struct {
	width, height int
	nodes         []*finder.Node
}

func (g *testGrid) Width() int             { return g.width }
func (g *testGrid) Height() int            { return g.height }
func (g *testGrid) IsInside(x, y int) bool { return x >= 0 && x < g.width && y >= 0 && y < g.height }
func (g *testGrid) IsWalkableAt(x, y int) bool {
	return g.IsInside(x, y) && g.nodes[y*g.width+x].Walkable
}
func (g *testGrid) SetWalkableAt(x, y int, v bool)  { g.nodes[y*g.width+x].Walkable = v }
func (g *testGrid) GetNodeAt(x, y int) *finder.Node { return g.nodes[y*g.width+x] }
func (g *testGrid) Clone() finder.Grid {
	ng := &testGrid{width: g.width, height: g.height, nodes: make([]*finder.Node, len(g.nodes))}
	for i, n := range g.nodes {
		cp := *n
		cp.Parent = nil
		ng.nodes[i] = &cp
	}
	return ng
}
func (g *testGrid) GetNeighbors(node *finder.Node, d finder.DiagonalMovement, buf []*finder.Node) []*finder.Node {
	x, y, w := node.X, node.Y, g.width
	out := buf[:0]
	if g.IsWalkableAt(x, y-1) {
		out = append(out, g.nodes[(y-1)*w+x])
	}
	if g.IsWalkableAt(x+1, y) {
		out = append(out, g.nodes[y*w+x+1])
	}
	if g.IsWalkableAt(x, y+1) {
		out = append(out, g.nodes[(y+1)*w+x])
	}
	if g.IsWalkableAt(x-1, y) {
		out = append(out, g.nodes[y*w+x-1])
	}
	return out
}

func TestFindPathBasic(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	b := g.AddNode(10, 0)
	c := g.AddNode(10, 10)
	a.Connect(b)
	b.Connect(c)

	grid := walkableGrid(20, 20)
	path := f.FindPath(0, 0, 10, 10, grid)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if want := 3; len(path) != want {
		t.Fatalf("path len = %d, want %d", len(path), want)
	}
	if path[0] != [2]int{0, 0} || path[1] != [2]int{10, 0} || path[2] != [2]int{10, 10} {
		t.Errorf("unexpected path: %v", path)
	}
}

func TestFindPathDirect(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 5)
	a.Connect(b)

	grid := walkableGrid(10, 10)
	path := f.FindPath(0, 0, 5, 5, grid)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if want := 2; len(path) != want {
		t.Fatalf("path len = %d, want %d", len(path), want)
	}
}

func TestFindPathDisconnected(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	g.AddNode(0, 0)
	g.AddNode(10, 0)

	grid := walkableGrid(15, 5)
	if path := f.FindPath(0, 0, 10, 0, grid); path != nil {
		t.Error("expected nil for disconnected graph")
	}
}

func TestFindPathEmptyGraph(t *testing.T) {
	f := NewWaypointFinder()
	grid := walkableGrid(15, 15)
	if path := f.FindPath(0, 0, 10, 10, grid); path != nil {
		t.Error("expected nil for empty graph")
	}
}

func TestFindPathBidirectional(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(10, 0)
	a.Connect(b)
	b.Connect(c)

	grid := walkableGrid(15, 5)
	forward := f.FindPath(0, 0, 10, 0, grid)
	if forward == nil {
		t.Fatal("expected forward path")
	}
	if forward[0] != [2]int{0, 0} || forward[len(forward)-1] != [2]int{10, 0} {
		t.Errorf("forward path wrong: %v", forward)
	}

	reverse := f.FindPath(10, 0, 0, 0, grid)
	if reverse == nil {
		t.Fatal("expected reverse path")
	}
	if reverse[0] != [2]int{10, 0} || reverse[len(reverse)-1] != [2]int{0, 0} {
		t.Errorf("reverse path wrong: %v", reverse)
	}
}

func TestFindPathOptimal(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(5, 5)
	d := g.AddNode(5, 10)
	e := g.AddNode(0, 10)

	a.Connect(b)
	b.Connect(c)
	c.Connect(d)
	d.Connect(e)
	a.Connect(c) // direct shortcut

	grid := walkableGrid(15, 15)
	path := f.FindPath(0, 0, 5, 5, grid)
	if path == nil {
		t.Fatal("expected path, got nil")
	}

	cost := pathCost(path)
	direct := dist2(0, 0, 5, 5)
	if cost > direct+0.001 {
		t.Errorf("path cost %.2f > direct cost %.2f — suboptimal", cost, direct)
	}
}

func TestFindPathStartEndPositions(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(5, 5)
	b := g.AddNode(10, 5)
	a.Connect(b)

	grid := walkableGrid(20, 20)
	path := f.FindPath(0, 0, 15, 5, grid)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if want := 4; len(path) != want {
		t.Fatalf("path len = %d, want %d", len(path), want)
	}
	if path[0] != [2]int{0, 0} {
		t.Errorf("start = %v, want (0,0)", path[0])
	}
	if path[len(path)-1] != [2]int{15, 5} {
		t.Errorf("end = %v, want (15,5)", path[len(path)-1])
	}
}

func TestFindPathDirectedEdges(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(10, 0)

	a.ConnectOneWay(b)
	b.ConnectOneWay(c)

	grid := walkableGrid(15, 5)
	forward := f.FindPath(0, 0, 10, 0, grid)
	if forward == nil {
		t.Fatal("expected forward path with directed edges")
	}

	reverse := f.FindPath(10, 0, 0, 0, grid)
	if reverse != nil {
		t.Error("expected nil for reverse on directed graph")
	}
}

func TestFindPathSkipsNullEdge(t *testing.T) {
	f := NewWaypointFinder()
	g := f.Graph()
	a := g.AddNode(0, 0)
	b := g.AddNode(5, 0)
	c := g.AddNode(10, 0)
	a.Connect(b, EdgeStateStatic)
	b.Connect(c, EdgeStateNull) // blocked

	grid := walkableGrid(15, 5)
	path := f.FindPath(0, 0, 10, 0, grid)
	if path != nil {
		t.Error("expected nil when path goes through EdgeStateNull")
	}
}

// --- helpers ---

func pathCost(path [][2]int) float64 {
	cost := 0.0
	for i := 1; i < len(path); i++ {
		dx := path[i][0] - path[i-1][0]
		dy := path[i][1] - path[i-1][1]
		cost += math.Sqrt(float64(dx*dx + dy*dy))
	}
	return cost
}

func dist2(x1, y1, x2, y2 int) float64 {
	dx := x1 - x2
	dy := y1 - y2
	return math.Sqrt(float64(dx*dx + dy*dy))
}

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
