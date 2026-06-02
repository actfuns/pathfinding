package waypoint

import (
	"math"

	"github.com/actfuns/pathfinding/finder"
)

// WaypointFinder implements finder.Finder on a waypoint graph.
//
// It finds the closest waypoint nodes with line-of-sight to the start/end
// positions (via the grid parameter) and runs A* on the graph.
type WaypointFinder struct {
	graph *WaypointGraph
}

// NewWaypointFinder creates a finder for the given graph.
func NewWaypointFinder(graph *WaypointGraph) *WaypointFinder {
	return &WaypointFinder{graph: graph}
}

// FindPath implements finder.Finder by searching the waypoint graph.
// The grid is used for LOS-based node selection — only nodes with a clear
// line of sight from the start/end position are considered.
func (f *WaypointFinder) FindPath(startX, startY, endX, endY int, grid finder.Grid) [][2]int {
	sx, sy := float64(startX), float64(startY)
	ex, ey := float64(endX), float64(endY)

	startNode := f.graph.FindClosestWithLOS(sx, sy, grid)
	endNode := f.graph.FindClosestWithLOS(ex, ey, grid)
	if startNode == nil || endNode == nil {
		return nil
	}
	if startNode == endNode {
		return [][2]int{{startX, startY}, {endX, endY}}
	}

	path := f.findPathInternal(startNode, endNode, sx, sy, ex, ey)
	if path == nil {
		return nil
	}

	// Convert float64 path to int tile coordinates
	result := make([][2]int, len(path))
	for i, p := range path {
		result[i] = [2]int{int(p[0]), int(p[1])}
	}
	return result
}

// FindPathFloat finds a path in world-space coordinates using distance-based
// node selection (no LOS checking). Consider using FindPath (which implements
// finder.Finder) for grid-integrated pathfinding.
func (f *WaypointFinder) FindPathFloat(startX, startY, endX, endY float64) [][2]float64 {
	startNode := f.graph.FindClosest(startX, startY)
	endNode := f.graph.FindClosest(endX, endY)
	if startNode == nil || endNode == nil {
		return nil
	}
	if startNode == endNode {
		return [][2]float64{{startX, startY}, {endX, endY}}
	}
	return f.findPathInternal(startNode, endNode, startX, startY, endX, endY)
}

// findPathInternal runs A* between two graph nodes and returns the path.
func (f *WaypointFinder) findPathInternal(startNode, endNode *WaypointNode, startX, startY, endX, endY float64) [][2]float64 {
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
			var path []*WaypointNode
			for c := cur; c != nil; c = c.parent {
				path = append(path, c.node)
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			fpath := make([][2]float64, 0, len(path)+2)
			fpath = appendPoint(fpath, startX, startY)
			for _, n := range path {
				fpath = appendPoint(fpath, n.X, n.Y)
			}
			fpath = appendPoint(fpath, endX, endY)
			return fpath
		}

		for _, edge := range cur.node.Edges {
			if edge.State == EdgeStateNull {
				continue
			}
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

// appendPoint adds (x, y) to the path only if it differs from the last point.
func appendPoint(path [][2]float64, x, y float64) [][2]float64 {
	if len(path) > 0 && path[len(path)-1][0] == x && path[len(path)-1][1] == y {
		return path
	}
	return append(path, [2]float64{x, y})
}

// aNode is an internal A* node wrapping a waypoint node.
type aNode struct {
	node    *WaypointNode
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
