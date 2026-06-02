// Package waypoint provides graph-based pathfinding on arbitrary waypoint graphs.
//
// A WaypointGraph consists of nodes with positions and connections (edges) between them.
// WaypointFinder implements finder.Finder by finding the closest waypoint nodes to
// the start/end tile coordinates and running A* on the graph.
//
// Reference: epiplon/Waypoints (C# Unity) — Node/Connection pattern with ConnectionType
package waypoint

import "math"

func sqrt(x float64) float64 {
	if x <= 0 {
		return 0
	}
	return math.Sqrt(x)
}

// EdgeState represents the traversability state of a waypoint connection.
//
// Reference: epiplon/Waypoints ConnectionType enum.
//   - Static:  always traversable (default)
//   - Dynamic: temporarily traversable, may be blocked by moving obstacles
//   - Null:    not traversable (treated as removed)
type EdgeState int

const (
	EdgeStateStatic  EdgeState = 0 // always traversable
	EdgeStateDynamic EdgeState = 1 // can be blocked dynamically
	EdgeStateNull    EdgeState = 2 // not traversable
)

// WaypointNode represents a node in the waypoint graph.
type WaypointNode struct {
	ID    int
	X     float64
	Y     float64
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

// Connect creates a bidirectional edge between this node and another, with the given state.
// If state is not specified, defaults to EdgeStateStatic.
func (n *WaypointNode) Connect(other *WaypointNode, states ...EdgeState) {
	state := EdgeStateStatic
	if len(states) > 0 {
		state = states[0]
	}
	dx := n.X - other.X
	dy := n.Y - other.Y
	cost := dx*dx + dy*dy
	if cost > 0 {
		cost = sqrt(cost)
	}
	n.Edges = append(n.Edges, &WaypointEdge{To: other, Cost: cost, State: state})
	other.Edges = append(other.Edges, &WaypointEdge{To: n, Cost: cost, State: state})
}

// ConnectOneWay creates a directed edge from this node to another.
func (n *WaypointNode) ConnectOneWay(other *WaypointNode, states ...EdgeState) {
	state := EdgeStateStatic
	if len(states) > 0 {
		state = states[0]
	}
	dx := n.X - other.X
	dy := n.Y - other.Y
	cost := dx*dx + dy*dy
	if cost > 0 {
		cost = sqrt(cost)
	}
	n.Edges = append(n.Edges, &WaypointEdge{To: other, Cost: cost, State: state})
}

// WaypointEdge represents a directed connection between two waypoint nodes.
type WaypointEdge struct {
	To    *WaypointNode
	Cost  float64
	State EdgeState
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

// GridLOS is the interface required for LOS-based node selection.
// It matches a subset of the finder.Grid interface.
type GridLOS interface {
	IsWalkableAt(tx, ty int) bool
}

// FindClosestWithLOS finds the closest node that has a clear line of sight
// from the given position on the specified grid. If no node has LOS, falls
// back to the closest node regardless of LOS.
func (g *WaypointGraph) FindClosestWithLOS(x, y float64, grid GridLOS) *WaypointNode {
	if len(g.Nodes) == 0 {
		return nil
	}
	var best *WaypointNode
	bestD := math.MaxFloat64
	for _, n := range g.Nodes {
		if hasLineOfSight(x, y, n.X, n.Y, grid) {
			d := distSq(n.X, n.Y, x, y)
			if d < bestD {
				bestD = d
				best = n
			}
		}
	}
	if best == nil {
		return g.FindClosest(x, y)
	}
	return best
}

// hasLineOfSight checks whether the straight line from (x1,y1) to (x2,y2)
// passes only through walkable cells on the given grid.
func hasLineOfSight(x1, y1, x2, y2 float64, grid GridLOS) bool {
	ix1, iy1 := int(x1), int(y1)
	ix2, iy2 := int(x2), int(y2)

	dx := ix2 - ix1
	dy := iy2 - iy1
	var sx, sy int
	if dx < 0 {
		dx = -dx
		sx = -1
	} else {
		sx = 1
	}
	if dy < 0 {
		dy = -dy
		sy = -1
	} else {
		sy = 1
	}
	err := dx - dy

	for {
		if !grid.IsWalkableAt(ix1, iy1) {
			return false
		}
		if ix1 == ix2 && iy1 == iy2 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			ix1 += sx
		}
		if e2 < dx {
			err += dx
			iy1 += sy
		}
	}
	return true
}

func distSq(x1, y1, x2, y2 float64) float64 {
	dx := x1 - x2
	dy := y1 - y2
	return dx*dx + dy*dy
}
