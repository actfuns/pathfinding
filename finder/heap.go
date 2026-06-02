package finder

// MinHeap is a binary heap for A* open list, ordered by F value.
// The less function pointer is intentionally NOT used — the comparison is
// hardcoded as a.F < b.F to enable compiler inlining of the hot path.
type MinHeap struct {
	nodes []*Node
}

// NewMinHeap creates a new MinHeap with an initial capacity of 64.
func NewMinHeap() *MinHeap {
	return &MinHeap{
		nodes: make([]*Node, 0, 64),
	}
}

// Len returns the number of nodes in the heap.
func (h *MinHeap) Len() int { return len(h.nodes) }

// Empty returns true if the heap contains no nodes.
func (h *MinHeap) Empty() bool { return len(h.nodes) == 0 }

// Push inserts a node into the heap and maintains the min-heap invariant.
func (h *MinHeap) Push(n *Node) {
	n.HeapIndex = len(h.nodes)
	h.nodes = append(h.nodes, n)
	h.siftDownFrom(len(h.nodes)-1, 0)
}

// Pop removes and returns the node with the smallest F value from the heap.
// The popped node's HeapIndex is set to -1. Returns nil when the heap is empty
// (though callers should check Empty first).
func (h *MinHeap) Pop() *Node {
	n := len(h.nodes)
	last := h.nodes[n-1]
	h.nodes = h.nodes[:n-1]
	if len(h.nodes) == 0 {
		last.HeapIndex = -1
		return last
	}
	root := h.nodes[0]
	h.nodes[0] = last
	last.HeapIndex = 0
	root.HeapIndex = -1
	h.siftUpFrom(0)
	return root
}

// UpdateItem restores the heap invariant after a node's F value has changed.
// It is a no-op if the node does not belong to this heap.
func (h *MinHeap) UpdateItem(node *Node) {
	pos := node.HeapIndex
	if pos < 0 || pos >= len(h.nodes) || h.nodes[pos] != node {
		return
	}
	h.siftDownFrom(pos, 0)
	h.siftUpFrom(pos)
}

func (h *MinHeap) siftDownFrom(pos, start int) {
	for pos > start {
		parent := (pos - 1) >> 1
		if !(h.nodes[pos].F < h.nodes[parent].F) {
			break
		}
		h.swap(pos, parent)
		pos = parent
	}
}

func (h *MinHeap) siftUpFrom(pos int) {
	n := len(h.nodes)
	start := pos
	node := h.nodes[pos]
	child := 2*pos + 1
	for child < n {
		right := child + 1
		if right < n && !(h.nodes[child].F < h.nodes[right].F) {
			child = right
		}
		h.nodes[pos] = h.nodes[child]
		h.nodes[pos].HeapIndex = pos
		pos = child
		child = 2*pos + 1
	}
	h.nodes[pos] = node
	node.HeapIndex = pos
	h.siftDownFrom(pos, start)
}

func (h *MinHeap) swap(i, j int) {
	h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i]
	h.nodes[i].HeapIndex = i
	h.nodes[j].HeapIndex = j
}
