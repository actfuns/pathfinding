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
	startNode := f.graph.FindClosestWithLOS(startX, startY, grid)
	endNode := f.graph.FindClosestWithLOS(endX, endY, grid)
	if startNode == nil || endNode == nil {
		return nil
	}
	if startNode == endNode {
		return [][2]int{{startX, startY}, {endX, endY}}
	}

	path := f.findPathInternal(startNode, endNode, startX, startY, endX, endY)
	if path == nil {
		return nil
	}

	result := make([][2]int, len(path))
	for i, p := range path {
		result[i] = [2]int{p[0], p[1]}
	}
	return result
}

// FindPathFloat finds a path in world-space coordinates. The coordinates are
// truncated to tile coordinates for graph operations. Consider using FindPath
// (which implements finder.Finder) for grid-integrated pathfinding.
func (f *WaypointFinder) FindPathFloat(startX, startY, endX, endY float64) [][2]float64 {
	sx, sy := int(startX), int(startY)
	ex, ey := int(endX), int(endY)

	startNode := f.graph.FindClosest(sx, sy)
	endNode := f.graph.FindClosest(ex, ey)
	if startNode == nil || endNode == nil {
		return nil
	}
	if startNode == endNode {
		return [][2]float64{{startX, startY}, {endX, endY}}
	}

	path := f.findPathInternal(startNode, endNode, sx, sy, ex, ey)
	if path == nil {
		return nil
	}

	fpath := make([][2]float64, 0, len(path))
	for _, p := range path {
		fpath = append(fpath, [2]float64{float64(p[0]), float64(p[1])})
	}
	return fpath
}

// findPathInternal runs A* between two graph nodes and returns the path as tile coordinates.
func (f *WaypointFinder) findPathInternal(startNode, endNode *WaypointNode, startX, startY, endX, endY int) [][2]int {
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
			result := make([][2]int, 0, len(path))
			result = appendPointInt(result, startX, startY)
			for _, n := range path {
				result = appendPointInt(result, n.X, n.Y)
			}
			result = appendPointInt(result, endX, endY)
			return result
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
	return math.Sqrt(float64(dx*dx + dy*dy))
}

// appendPointInt adds (x, y) to the path only if it differs from the last point.
func appendPointInt(path [][2]int, x, y int) [][2]int {
	if len(path) > 0 && path[len(path)-1][0] == x && path[len(path)-1][1] == y {
		return path
	}
	return append(path, [2]int{x, y})
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
