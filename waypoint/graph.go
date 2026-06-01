// Package waypoint provides graph-based pathfinding on arbitrary waypoint graphs.
//
// A WaypointGraph consists of nodes with positions and connections (edges) between them.
// WaypointFinder implements finder.Finder by finding the closest waypoint nodes to
// the start/end tile coordinates and running A* on the graph.
//
// Reference: Roy-T.AStar (C#) — INode/Node/Edge pattern with Connect() fluent API
package waypoint

import "math"

func sqrt(x float64) float64 {
	if x <= 0 {
		return 0
	}
	return math.Sqrt(x)
}

// WaypointNode represents a node in the waypoint graph.
type WaypointNode struct {
	ID    int
	X, Y  float64
	Edges []*WaypointEdge
}

// NewWaypointNode creates a new waypoint node.
func NewWaypointNode(id int, x, y float64) *WaypointNode {
	return &WaypointNode{
		ID:    id,
		X:     x,
		Y:     y,
		Edges: make([]*WaypointEdge, 0, 4),
	}
}

// Connect creates a bidirectional edge between this node and another.
func (n *WaypointNode) Connect(other *WaypointNode) {
	dx := n.X - other.X
	dy := n.Y - other.Y
	cost := dx*dx + dy*dy
	// sqrt for actual Euclidean distance
	if cost > 0 {
		cost = sqrt(cost)
	}
	e1 := &WaypointEdge{To: other, Cost: cost}
	e2 := &WaypointEdge{To: n, Cost: cost}
	n.Edges = append(n.Edges, e1)
	other.Edges = append(other.Edges, e2)
}

// ConnectOneWay creates a directed edge from this node to another.
func (n *WaypointNode) ConnectOneWay(other *WaypointNode) {
	dx := n.X - other.X
	dy := n.Y - other.Y
	cost := dx*dx + dy*dy
	if cost > 0 {
		cost = sqrt(cost)
	}
	n.Edges = append(n.Edges, &WaypointEdge{To: other, Cost: cost})
}

// WaypointEdge represents a directed connection between two waypoint nodes.
type WaypointEdge struct {
	To   *WaypointNode
	Cost float64
}

// WaypointGraph holds all nodes in the waypoint graph.
type WaypointGraph struct {
	Nodes []*WaypointNode
}

// NewWaypointGraph creates an empty waypoint graph.
func NewWaypointGraph() *WaypointGraph {
	return &WaypointGraph{
		Nodes: make([]*WaypointNode, 0),
	}
}

// AddNode adds a node to the graph and returns it.
func (g *WaypointGraph) AddNode(x, y float64) *WaypointNode {
	n := NewWaypointNode(len(g.Nodes), x, y)
	g.Nodes = append(g.Nodes, n)
	return n
}

// FindClosest finds the closest node to a given position.
func (g *WaypointGraph) FindClosest(x, y float64) *WaypointNode {
	if len(g.Nodes) == 0 {
		return nil
	}
	best := g.Nodes[0]
	bestD := distSq(best.X, best.Y, x, y)
	for _, n := range g.Nodes[1:] {
		d := distSq(n.X, n.Y, x, y)
		if d < bestD {
			bestD = d
			best = n
		}
	}
	return best
}

func distSq(x1, y1, x2, y2 float64) float64 {
	dx := x1 - x2
	dy := y1 - y2
	return dx*dx + dy*dy
}
