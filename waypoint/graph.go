// Package waypoint provides graph-based pathfinding on arbitrary waypoint graphs.
//
// A WaypointGraph consists of nodes with positions and connections (edges) between them.
// WaypointFinder implements finder.Finder by finding the closest waypoint nodes to
// the start/end tile coordinates and running A* on the graph.
package waypoint

import "math"

func sqrt(x float32) float32 {
	if x <= 0 {
		return 0
	}
	return float32(math.Sqrt(float64(x)))
}

// EdgeState represents the traversability state of a waypoint connection.
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
	X     int32
	Y     int32
	Edges []*WaypointEdge
}

// NewWaypointNode creates a new waypoint node at the given tile coordinate.
func NewWaypointNode(id int, x, y int32) *WaypointNode {
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
	dx := float32(n.X) - float32(other.X)
	dy := float32(n.Y) - float32(other.Y)
	cost := sqrt(dx*dx + dy*dy)
	n.Edges = append(n.Edges, &WaypointEdge{To: other, Cost: cost, State: state})
	other.Edges = append(other.Edges, &WaypointEdge{To: n, Cost: cost, State: state})
}

// ConnectOneWay creates a directed edge from this node to another.
func (n *WaypointNode) ConnectOneWay(other *WaypointNode, states ...EdgeState) {
	state := EdgeStateStatic
	if len(states) > 0 {
		state = states[0]
	}
	dx := float32(n.X) - float32(other.X)
	dy := float32(n.Y) - float32(other.Y)
	cost := sqrt(dx*dx + dy*dy)
	n.Edges = append(n.Edges, &WaypointEdge{To: other, Cost: cost, State: state})
}

// WaypointEdge represents a directed connection between two waypoint nodes.
type WaypointEdge struct {
	To    *WaypointNode
	Cost  float32
	State EdgeState
}

// WaypointGraph holds all nodes in the waypoint graph.
type WaypointGraph struct {
	Nodes    []*WaypointNode
	cellSize int32                        // spatial index cell size
	cellIdx  map[[2]int32][]*WaypointNode // cell key → nodes in cell
}

// NewWaypointGraph creates an empty waypoint graph with spatial indexing.
func NewWaypointGraph() *WaypointGraph {
	return &WaypointGraph{
		Nodes:    make([]*WaypointNode, 0),
		cellSize: 64,
		cellIdx:  make(map[[2]int32][]*WaypointNode),
	}
}

// AddNode adds a node at tile coordinate (x, y) and returns it.
// The node is automatically registered in the spatial index.
func (g *WaypointGraph) AddNode(x, y int32) *WaypointNode {
	n := NewWaypointNode(len(g.Nodes), x, y)
	g.Nodes = append(g.Nodes, n)
	ck := [2]int32{x / g.cellSize, y / g.cellSize}
	g.cellIdx[ck] = append(g.cellIdx[ck], n)
	return n
}

// FindClosest finds the closest node to tile coordinate (x, y).
// Uses the spatial index for O(1)~O(k) lookup instead of O(n) scan.
func (g *WaypointGraph) FindClosest(x, y int32) *WaypointNode {
	if len(g.Nodes) == 0 {
		return nil
	}
	cx := x / g.cellSize
	cy := y / g.cellSize
	best := (*WaypointNode)(nil)
	bestD := float32(math.MaxFloat32)

	for radius := int32(0); ; radius++ {
		// bestD is squared, so compare lengths on the same scale. A node in a
		// cell at Chebyshev distance r is at least (r-1)*cellSize away.
		if radius > 0 && best != nil && float32(math.Sqrt(float64(bestD))) <= float32((radius-1)*g.cellSize) {
			break
		}

		found := false
		for dy := -radius; dy <= radius; dy++ {
			for dx := -radius; dx <= radius; dx++ {
				if radius > 0 && abs(dx) < radius && abs(dy) < radius {
					continue
				}
				ck := [2]int32{cx + dx, cy + dy}
				nodes, ok := g.cellIdx[ck]
				if !ok {
					continue
				}
				found = true
				for _, n := range nodes {
					d := distSq(n.X, n.Y, x, y)
					if d < bestD {
						bestD = d
						best = n
					}
				}
			}
		}
		if !found && best != nil {
			break
		}
		if !found && radius > 16 {
			break
		}
	}
	if best == nil {
		// Fallback: linear scan
		best = g.Nodes[0]
		bestD = distSq(best.X, best.Y, x, y)
		for _, n := range g.Nodes[1:] {
			d := distSq(n.X, n.Y, x, y)
			if d < bestD {
				bestD = d
				best = n
			}
		}
	}
	return best
}

func abs(n int32) int32 {
	if n < 0 {
		return -n
	}
	return n
}

// GridLOS is the interface required for LOS-based node selection.
// It matches a subset of the finder.Grid interface.
type GridLOS interface {
	IsWalkableAt(tx, ty int32) bool
}

// FindClosestWithLOS finds the closest node that has a clear line of sight
// from the given position on the specified grid. If no node has LOS, falls
// back to the closest node regardless of LOS.
// Uses the spatial index for efficient lookup.
func (g *WaypointGraph) FindClosestWithLOS(x, y int32, grid GridLOS) *WaypointNode {
	if len(g.Nodes) == 0 {
		return nil
	}
	cx := x / g.cellSize
	cy := y / g.cellSize
	var best *WaypointNode
	bestD := float32(math.MaxFloat32)
	var anyBest *WaypointNode
	anyBestD := float32(math.MaxFloat32)

	for radius := int32(0); ; radius++ {
		// bestD is squared, so compare lengths on the same scale. A node in a
		// cell at Chebyshev distance r is at least (r-1)*cellSize away.
		if radius > 0 && best != nil && float32(math.Sqrt(float64(bestD))) <= float32((radius-1)*g.cellSize) {
			break
		}

		found := false
		for dy := -radius; dy <= radius; dy++ {
			for dx := -radius; dx <= radius; dx++ {
				if radius > 0 && abs(dx) < radius && abs(dy) < radius {
					continue
				}
				ck := [2]int32{cx + dx, cy + dy}
				nodes, ok := g.cellIdx[ck]
				if !ok {
					continue
				}
				found = true
				for _, n := range nodes {
					d := distSq(n.X, n.Y, x, y)
					if d < anyBestD {
						anyBestD = d
						anyBest = n
					}
					if hasLineOfSight(x, y, n.X, n.Y, grid) && d < bestD {
						bestD = d
						best = n
					}
				}
			}
		}
		if !found && best != nil {
			break
		}
		if !found && radius > 16 {
			break
		}
	}
	if best == nil {
		// Nothing with line of sight was found in the search window: fall back
		// to a full scan so distant graphs are not reported as unreachable.
		bestD = float32(math.MaxFloat32)
		anyBestD = float32(math.MaxFloat32)
		for _, n := range g.Nodes {
			d := distSq(n.X, n.Y, x, y)
			if d < anyBestD {
				anyBestD = d
				anyBest = n
			}
			if hasLineOfSight(x, y, n.X, n.Y, grid) && d < bestD {
				bestD = d
				best = n
			}
		}
	}
	if best == nil {
		return anyBest
	}
	return best
}

// hasLineOfSight checks whether the straight line from (x1,y1) to (x2,y2)
// passes only through walkable cells on the given grid.
func hasLineOfSight(x1, y1, x2, y2 int32, grid GridLOS) bool {
	dx := x2 - x1
	dy := y2 - y1
	var sx, sy int32
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
		if !grid.IsWalkableAt(x1, y1) {
			return false
		}
		if x1 == x2 && y1 == y2 {
			break
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x1 += sx
		}
		if e2 < dx {
			err += dx
			y1 += sy
		}
	}
	return true
}

func distSq(x1, y1, x2, y2 int32) float32 {
	dx := float32(x1) - float32(x2)
	dy := float32(y1) - float32(y2)
	return dx*dx + dy*dy
}

// ConnectNearby auto-connects all nodes that are within maxDistance of each
// other and have a clear line of sight on the given grid. Nodes that already
// have edges are not reconnected. Uses the spatial index for O(n*k) complexity
// (k = average nodes in neighboring cells) instead of O(n^2).
func (g *WaypointGraph) ConnectNearby(maxDistance int32, grid GridLOS) {
	maxDistSq := float32(maxDistance) * float32(maxDistance)
	for _, a := range g.Nodes {
		ck := [2]int32{a.X / g.cellSize, a.Y / g.cellSize}
		for dy := int32(-1); dy <= 1; dy++ {
			for dx := int32(-1); dx <= 1; dx++ {
				nk := [2]int32{ck[0] + dx, ck[1] + dy}
				others, ok := g.cellIdx[nk]
				if !ok {
					continue
				}
				for _, b := range others {
					if a.ID >= b.ID {
						continue
					}
					dx := float32(a.X) - float32(b.X)
					dy := float32(a.Y) - float32(b.Y)
					d := dx*dx + dy*dy
					if d > maxDistSq {
						continue
					}
					if !hasLineOfSight(a.X, a.Y, b.X, b.Y, grid) {
						continue
					}
					// Only connect if not already connected
					connected := false
					for _, e := range a.Edges {
						if e.To == b {
							connected = true
							break
						}
					}
					if !connected {
						cost := sqrt(d)
						a.Edges = append(a.Edges, &WaypointEdge{To: b, Cost: cost, State: EdgeStateStatic})
						b.Edges = append(b.Edges, &WaypointEdge{To: a, Cost: cost, State: EdgeStateStatic})
					}
				}
			}
		}
	}
}
