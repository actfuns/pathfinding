package waypoint

import (
	"testing"

	"github.com/actfuns/pathfinding/finder"
)

// walkableGrid returns a minimal all-walkable grid for testing pathfinding.
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

func TestWaypointGraphBasic(t *testing.T) {
	f := NewWaypointFinder()
	a := f.AddNode(0, 0)
	b := f.AddNode(10, 0)
	c := f.AddNode(10, 10)
	f.Connect(a, b)
	f.Connect(b, c)

	g := walkableGrid(20, 20)
	path := f.FindPath(0, 0, 10, 10, g)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) != 3 {
		t.Errorf("expected 3 points (start, node B, end), got %d: %v", len(path), path)
	}
	if path[0] != [2]int{0, 0} || path[1] != [2]int{10, 0} || path[2] != [2]int{10, 10} {
		t.Errorf("unexpected path: %v", path)
	}
}

func TestWaypointGraphDirectConnection(t *testing.T) {
	f := NewWaypointFinder()
	a := f.AddNode(0, 0)
	b := f.AddNode(5, 5)
	f.Connect(a, b)

	g := walkableGrid(10, 10)
	path := f.FindPath(0, 0, 5, 5, g)
	if path == nil {
		t.Fatal("expected path, got nil")
	}
	if len(path) != 2 {
		t.Errorf("expected 2 points (start, end), got %d: %v", len(path), path)
	}
}

func TestWaypointGraphNoPath(t *testing.T) {
	f := NewWaypointFinder()
	f.AddNode(0, 0)
	f.AddNode(10, 0)
	// No connection between a and b

	g := walkableGrid(15, 5)
	path := f.FindPath(0, 0, 10, 0, g)
	if path != nil {
		t.Error("expected nil path for disconnected graph")
	}
}

func TestWaypointGraphEmpty(t *testing.T) {
	f := NewWaypointFinder()
	g := walkableGrid(15, 15)
	path := f.FindPath(0, 0, 10, 10, g)
	if path != nil {
		t.Error("expected nil path for empty graph")
	}
}

func TestWaypointGraphBidirectional(t *testing.T) {
	f := NewWaypointFinder()
	a := f.AddNode(0, 0)
	b := f.AddNode(5, 0)
	c := f.AddNode(10, 0)
	f.Connect(a, b)
	f.Connect(b, c)

	grid := walkableGrid(15, 5)
	forward := f.FindPath(0, 0, 10, 0, grid)
	if forward == nil {
		t.Fatal("expected path forward")
	}
	if forward[0] != [2]int{0, 0} || forward[len(forward)-1] != [2]int{10, 0} {
		t.Errorf("forward path wrong: %v", forward)
	}

	reverse := f.FindPath(10, 0, 0, 0, grid)
	if reverse == nil {
		t.Fatal("expected path in reverse")
	}
	if reverse[0] != [2]int{10, 0} || reverse[len(reverse)-1] != [2]int{0, 0} {
		t.Errorf("reverse path wrong: %v", reverse)
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
