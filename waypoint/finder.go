package waypoint

import "math"

// WaypointFinder implements A* search on a waypoint graph.
//
// It can be used directly with a WaypointGraph, or it can wrap a graph
// to implement core.Finder by mapping tile coordinates to the closest
// waypoint nodes.
type WaypointFinder struct {
	graph *WaypointGraph
}

// NewWaypointFinder creates a finder for the given graph.
func NewWaypointFinder(graph *WaypointGraph) *WaypointFinder {
	return &WaypointFinder{graph: graph}
}

// FindPath finds a path on the waypoint graph from the closest node
// to (startX, startY) to the closest node to (endX, endY).
func (f *WaypointFinder) FindPath(startX, startY, endX, endY int) [][2]float64 {
	sx, sy := float64(startX), float64(startY)
	ex, ey := float64(endX), float64(endY)
	return f.FindPathFloat(sx, sy, ex, ey)
}

// FindPathFloat finds a path between two world-coordinate positions.
func (f *WaypointFinder) FindPathFloat(startX, startY, endX, endY float64) [][2]float64 {
	startNode := f.graph.FindClosest(startX, startY)
	endNode := f.graph.FindClosest(endX, endY)
	if startNode == nil || endNode == nil {
		return nil
	}
	if startNode == endNode {
		return [][2]float64{{startX, startY}, {endX, endY}}
	}

	// A* on the graph
	open := &nodeHeap{}
	all := make(map[*WaypointNode]*aNode)
	goal := endNode

	startANode := &aNode{node: startNode, g: 0}
	startANode.h = heuristic(startNode, goal)
	startANode.f = startANode.g + startANode.h
	all[startNode] = startANode
	open.Push(startANode)

	for open.Len() > 0 {
		cur := open.Pop()
		if cur.closed {
			continue
		}
		cur.closed = true

		if cur.node == goal {
			// Reconstruct path
			var path []*WaypointNode
			for c := cur; c != nil; c = c.parent {
				path = append(path, c.node)
			}
			// Reverse
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			// Convert to float64 path, deduplicating consecutive identical points
			fpath := make([][2]float64, 0, len(path)+2)
			fpath = appendPoint(fpath, startX, startY)
			for _, n := range path {
				fpath = appendPoint(fpath, n.X, n.Y)
			}
			fpath = appendPoint(fpath, endX, endY)
			return fpath
		}

		for _, edge := range cur.node.Edges {
			ng := cur.g + edge.Cost
			existing, ok := all[edge.To]
			if !ok {
				n := &aNode{node: edge.To, g: ng, parent: cur}
				n.h = heuristic(edge.To, goal)
				n.f = n.g + n.h
				all[edge.To] = n
				open.Push(n)
			} else if !existing.closed && ng < existing.g {
				existing.g = ng
				existing.f = existing.g + existing.h
				existing.parent = cur
				open.Push(existing)
			}
		}
	}

	return nil
}

// heuristic computes Euclidean distance between two waypoint nodes.
func heuristic(a, b *WaypointNode) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(dx*dx + dy*dy)
}

// appendPoint adds (x,y) to the path only if it differs from the last point.
func appendPoint(path [][2]float64, x, y float64) [][2]float64 {
	if len(path) > 0 && path[len(path)-1][0] == x && path[len(path)-1][1] == y {
		return path
	}
	return append(path, [2]float64{x, y})
}

// aNode is an internal A* node wrapping a waypoint node.
type aNode struct {
	node   *WaypointNode
	g, h, f float64
	parent  *aNode
	closed  bool
	heapIdx int
}

// nodeHeap is a min-heap for aNode.
type nodeHeap struct {
	nodes []*aNode
}

func (h *nodeHeap) Len() int { return len(h.nodes) }

func (h *nodeHeap) Push(n *aNode) {
	n.heapIdx = len(h.nodes)
	h.nodes = append(h.nodes, n)
	h.siftUp(n.heapIdx)
}

func (h *nodeHeap) Pop() *aNode {
	n := len(h.nodes)
	last := h.nodes[n-1]
	h.nodes = h.nodes[:n-1]
	if len(h.nodes) == 0 {
		last.heapIdx = -1
		return last
	}
	root := h.nodes[0]
	h.nodes[0] = last
	last.heapIdx = 0
	root.heapIdx = -1
	h.siftDown(0)
	return root
}

func (h *nodeHeap) siftUp(idx int) {
	for idx > 0 {
		parent := (idx - 1) / 2
		if h.nodes[idx].f >= h.nodes[parent].f {
			break
		}
		h.nodes[idx], h.nodes[parent] = h.nodes[parent], h.nodes[idx]
		h.nodes[idx].heapIdx = idx
		h.nodes[parent].heapIdx = parent
		idx = parent
	}
}

func (h *nodeHeap) siftDown(idx int) {
	n := len(h.nodes)
	for {
		smallest := idx
		left := 2*idx + 1
		right := 2*idx + 2
		if left < n && h.nodes[left].f < h.nodes[smallest].f {
			smallest = left
		}
		if right < n && h.nodes[right].f < h.nodes[smallest].f {
			smallest = right
		}
		if smallest == idx {
			break
		}
		h.nodes[idx], h.nodes[smallest] = h.nodes[smallest], h.nodes[idx]
		h.nodes[idx].heapIdx = idx
		h.nodes[smallest].heapIdx = smallest
		idx = smallest
	}
}
