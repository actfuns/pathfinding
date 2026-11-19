package finder

// heapEntry caches the node's external state for the duration of one search so
// heap operations do not have to look it up by coordinate.
type heapEntry struct {
	node *Node
	st   *nodeState
}

// MinHeap is a binary heap for A* open list, ordered by F value. When
// tieBreak is set, equal F values are ordered by H so the search advances
// toward the goal instead of expanding an equal-cost plateau.
type MinHeap struct {
	nodes    []heapEntry
	tieBreak bool // on equal F, prefer the node closer to the goal (lower H)
	state    *searchStates
}

func (h *MinHeap) less(a, b heapEntry) bool {
	if a.st.F != b.st.F {
		return a.st.F < b.st.F
	}
	if h.tieBreak {
		return a.st.H < b.st.H
	}
	return false
}

// Len returns the number of nodes in the heap.
func (h *MinHeap) Len() int { return len(h.nodes) }

// Empty returns true if the heap contains no nodes.
func (h *MinHeap) Empty() bool { return len(h.nodes) == 0 }

// Push inserts a node into the heap and maintains the min-heap invariant.
func (h *MinHeap) Push(n *Node) {
	st := h.state.get(n)
	st.HeapIndex = int32(len(h.nodes))
	h.nodes = append(h.nodes, heapEntry{node: n, st: st})
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
		last.st.HeapIndex = -1
		return last.node
	}
	root := h.nodes[0]
	h.nodes[0] = last
	last.st.HeapIndex = 0
	root.st.HeapIndex = -1
	h.siftUpFrom(0)
	return root.node
}

// UpdateItem restores the heap invariant after a node's F value has decreased.
// In the A* algorithm this is only called when G (and thus F) strictly decreases,
// so the node can only move upward toward the root — the sift-down pass is unnecessary.
func (h *MinHeap) UpdateItem(node *Node) {
	pos := int(h.state.get(node).HeapIndex)
	if pos < 0 || pos >= len(h.nodes) || h.nodes[pos].node != node {
		return
	}
	h.siftDownFrom(pos, 0)
}

func (h *MinHeap) siftDownFrom(pos, start int) {
	for pos > start {
		parent := (pos - 1) >> 1
		if !h.less(h.nodes[pos], h.nodes[parent]) {
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
		if right < n && !h.less(h.nodes[child], h.nodes[right]) {
			child = right
		}
		h.nodes[pos] = h.nodes[child]
		h.nodes[pos].st.HeapIndex = int32(pos)
		pos = child
		child = 2*pos + 1
	}
	h.nodes[pos] = node
	h.nodes[pos].st.HeapIndex = int32(pos)
	h.siftDownFrom(pos, start)
}

func (h *MinHeap) swap(i, j int) {
	h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i]
	h.nodes[i].st.HeapIndex = int32(i)
	h.nodes[j].st.HeapIndex = int32(j)
}
